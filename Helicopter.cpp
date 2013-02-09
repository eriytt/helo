#include "Helicopter.h"

#include "Physics.h"

Rotor::Rotor(const RotorData &data, const Ogre::Vector3 &parent_pos, btRigidBody &parent_body,
	     Ogre::Root *root)
{
  revolutionsPerSecond = 0.0;
  inertia = data.inertia;
  torque = data.torque;
  maxLift = data.maxLift;
  started = false;
  pos = data.relpos;
  axis = data.axis;
  axis.normalise();
  diameter = data.diameter;
  weight = data.weight;
  swash = 0.0;
  swash_forward = 0.0;
  swash_right = 0.0;
  rotationAxis = data.rotation_axis;
  rotationAxis.normalise();
  rotation = Ogre::Radian(0.0);
  tiltSensitivity = data.tiltSensitivity;
  equilibriumLift = maxLift;
  appliedTorque = 0.0;
}

void Rotor::setEquilibriumLift(float equilift)
{
  equilibriumLift = equilift;
  equilibriumLift = HeloUtils::clamp(maxLift, 0.0f, equilibriumLift);
}

void Rotor::setSwash(float new_swash, float new_forward, float new_right)
{
  swash = new_swash;
  swash_forward = new_forward;
  swash_right = new_right;
}

void Rotor::setRotation(float radians)
{
  if (rotationAxis == Ogre::Vector3::ZERO)
    return;
  rotation = Ogre::Radian(radians) * tiltSensitivity;
}

void Rotor::applyForcesAndTorques(btRigidBody *parent_body, btScalar step)
{
  if (started)
    {
      appliedTorque = (4000.0 - fabs(revolutionsPerSecond)) * torque;
      revolutionsPerSecond += (appliedTorque / inertia) * step;
      HeloUtils::LocalApplyTorque(parent_body, btVector3(0, -appliedTorque, 0));
    }

  btScalar engine_force = (fabs(revolutionsPerSecond) / 4000.0) * equilibriumLift;
  btScalar up_force = engine_force * (1.0 + swash);
  up_force = HeloUtils::clamp(maxLift, -maxLift, up_force);

  Ogre::Vector3 rotated_axis;
  if (rotation.valueRadians() == 0.0)
    rotated_axis = axis;
  else
    rotated_axis = Ogre::Quaternion(rotation, rotationAxis) * axis ;

  btVector3 raxis(rotated_axis.x, rotated_axis.y, rotated_axis.z);
  btVector3 rpos(pos.x, pos.y, pos.z);

  HeloUtils::LocalApplyForce(parent_body, raxis * up_force, rpos);

  if (swash_forward != 0.0)
    {
      btScalar forward_force = engine_force * swash_forward;
      HeloUtils::LocalApplyForce(parent_body, raxis * forward_force,
				 rpos + btVector3(0.0, 0.0, 0.5 * -diameter));
      HeloUtils::LocalApplyForce(parent_body, raxis * -forward_force,
				 rpos + btVector3(0.0, 0.0, 0.5 * diameter));
    }
  if (swash_right != 0.0)
    {
      btScalar right_force = engine_force * swash_right;
      HeloUtils::LocalApplyForce(parent_body, raxis * right_force,
				 rpos + btVector3(0.5 * diameter, 0.0, 0.0));
      HeloUtils::LocalApplyForce(parent_body, raxis * -right_force,
				 rpos + btVector3(0.5 * -diameter, 0.0, 0.0));
    }
}



Helicopter::Helicopter(const HelicopterData &data, Ogre::Root *root)
{
  Ogre::SceneManager *mgr = root->getSceneManager("SceneManager");

  // create entity
  Ogre::Entity *ent = mgr->createEntity(data.name + "_ent", data.meshname);

  // create scene node
  node = mgr->getRootSceneNode()->createChildSceneNode(data.name + "_node");
  node->attachObject(ent);

  // create collision shape
  shape = new btBoxShape(btVector3(data.size.x / 2.0, data.size.y / 2.0, data.size.z / 2.0));

  // create fuselage rigid body
  btTransform tr;
  tr.setIdentity();
  tr.setOrigin(btVector3(data.pos.x, data.pos.y, data.pos.z));

  fuselageBody = Physics::CreateRigidBody(data.weight, tr, shape, node);

  for (Rotor::RotorDataCIter i = data.rotorData.begin(); i != data.rotorData.end(); ++i)
    {
      Rotor *r = new Rotor(*i, data.pos, *fuselageBody, root);
      r->setEquilibriumLift(data.weight  * 9.81 / static_cast<float>(data.rotorData.size()));
      rotors.push_back(r);
    }


  collective = 0.0f;
  cyclic_forward = 0.0f;
  cyclic_right = 0.0f;
  boomLength = data.boomLength;
  steer = 0.0;
  collectiveSensitivity = data.collectiveSensitivity;;
  cyclicRightSensitivity = data.cyclicRightSensitivity;
  cyclicForwardSensitivity = data.cyclicForwardSensitivity;
  steerSensitivity = data.steerSensitivity;

  cameraPosition = (Ogre::Vector3::NEGATIVE_UNIT_Z * 30.0) + (Ogre::Vector3::UNIT_Y * 5.0);
};

// TODO : honour the flag
void Helicopter::startEngines(bool start)
{
  for (RotorIter i = rotors.begin(); i != rotors.end(); ++i)
    (*i)->start();
}

void Helicopter::finishPhysicsConfiguration(Physics *phys)
{
  phys->addBody(fuselageBody);
  fuselageBody->setActivationState(DISABLE_DEACTIVATION);
}

void Helicopter::setRotorInput()
{
  rotors[0]->setSwash(collective * collectiveSensitivity,
		      cyclic_forward * cyclicForwardSensitivity,
		      cyclic_right * cyclicRightSensitivity);
}

void Helicopter::physicsUpdate(float step)
{
  setRotorInput();
  for (RotorIter i = rotors.begin(); i != rotors.end(); ++i)
    (*i)->applyForcesAndTorques(fuselageBody, step);


  if (boomLength != 0.0)
    {

      HeloUtils::LocalApplyTorque(fuselageBody,
				  btVector3(0.0,
					    rotors[0]->getAppliedTorque()
					    + (steer * steerSensitivity),
					    0.0));
    }
}


void TandemRotorHelicopter::setRotorInput()
{
  rotors[FrontRotor]->setSwash((collective * collectiveSensitivity)
			       - (cyclic_forward * cyclicForwardSensitivity), 0.0, 0.0);
  rotors[BackRotor]->setSwash((collective * collectiveSensitivity)
			      + (cyclic_forward * cyclicForwardSensitivity), 0.0, 0.0);

  rotors[FrontRotor]->setRotation((cyclic_right * cyclicRightSensitivity)
				  - (steer * steerSensitivity));
  rotors[BackRotor]->setRotation((cyclic_right * cyclicRightSensitivity)
				 + (steer * steerSensitivity));
}

Controller *Helicopter::createController(OIS::Object *dev)
{
  if (dynamic_cast<OIS::JoyStick*>(dev))
    return controller = new HelicopterJoystickController(*static_cast<OIS::JoyStick*>(dev), *this);
  else if (dynamic_cast<OIS::Mouse*>(dev))
    return NULL;
  else if (dynamic_cast<OIS::Keyboard*>(dev))
    return NULL;
  else
    return NULL;
}


bool HelicopterJoystickController::buttonPressed(const OIS::JoyStickEvent &e, int idx)
{
  if (not active)
    return true;

  switch (idx)
    {
    case 0:
      helicopter.startEngines(true);
      break;
    default:
      break;
    }
  return true;

}

bool HelicopterJoystickController::buttonReleased(const OIS::JoyStickEvent&, int)
{
  return true;
}

bool HelicopterJoystickController::axisMoved(const OIS::JoyStickEvent &e, int)
{
  if (not active)
    return true;

  float collective = -e.state.mAxes[1].abs / static_cast<float>(OIS::JoyStick::MAX_AXIS);
  float steer = -e.state.mAxes[0].abs / static_cast<float>(OIS::JoyStick::MAX_AXIS);
  float cyclic_forward = -e.state.mAxes[2].abs / static_cast<float>(OIS::JoyStick::MAX_AXIS);
  float cyclic_right = e.state.mAxes[3].abs / static_cast<float>(OIS::JoyStick::MAX_AXIS);

  helicopter.setCollective(collective);
  helicopter.setSteer(steer);
  helicopter.setCyclicForward(cyclic_forward);
  helicopter.setCyclicRight(cyclic_right);

  return true;
}

void HelicopterJoystickController::update(float timeDelta)
{
}

void HelicopterJoystickController::setActive(bool a)
{
  Controller::setActive(a);
  joystick.setEventCallback(a ? this : NULL);
}
