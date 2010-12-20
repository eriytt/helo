#include "Helicopter.h"

#include "Physics.h"

Rotor::Rotor(const RotorData &data, const Ogre::Vector3 &parent_pos, btRigidBody &parent_body,
	     Ogre::Root *root)
{
  btTransform tr;
  tr.setIdentity();
  tr.setOrigin(btVector3(parent_pos.x + data.relpos.x,
			 parent_pos.y + data.relpos.y,
			 parent_pos.z + data.relpos.z));

  body = Physics::CreateRigidBody(data.weight, tr, NULL/*No shape*/, NULL/*No scene node*/,
				  btVector3(0.1, data.inertia, 0.1));

  hinge = new btHingeConstraint(*body, parent_body,
				btVector3(0, 0, 0),
				btVector3(0, 0, 0),
				btVector3(0.0, 1.0, 0.0),
				btVector3(0.0, 1.0, 0.0));
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

void Rotor::applyForcesAndTorques(btRigidBody *parent_body)
{
  if (started)
    {
      HeloUtils::LocalApplyTorque(body, btVector3(0, torque, 0));
      HeloUtils::LocalApplyTorque(parent_body, btVector3(0, -torque, 0));
    }

  // Bullet seems to clamp this to around 94, probably the angualar damping
  btScalar engine_rps = fabs(body->getAngularVelocity().y()); // radians per second

  // btScalar engine_force = (engine_rps / 94) * lift / 4.0;
  btScalar engine_force = (engine_rps / 94) * equilibriumLift;
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

  float engine_weight = 0.0;
  for (Rotor::RotorDataCIter i = data.rotorData.begin(); i != data.rotorData.end(); ++i)
    engine_weight += (*i).weight;
  fuselageBody = Physics::CreateRigidBody(data.weight - engine_weight, tr, shape, node);

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
  for (RotorIter i = rotors.begin(); i != rotors.end(); ++i)
    {
      phys->addBody((*i)->getBody());
      phys->addConstraint((*i)->getHinge());
    }
}

void Helicopter::setRotorInput()
{
  rotors[0]->setSwash(collective * collectiveSensitivity,
		      cyclic_forward * cyclicForwardSensitivity,
		      cyclic_right * cyclicRightSensitivity);
}

void Helicopter::physicsUpdate(void)
{
  //static unsigned int updates = 0;
  //fuselageBody->applyCentralForce(btVector3(0.0, 669.0 * 9.81 * 1.1, 0.0));
  // if (not (updates % 100000)) {
  //   const btVector3 &av = engineBody->getAngularVelocity();
  //   const btVector3 &pos = engineBody->getCenterOfMassPosition();
  //   printf("Engine angular velocity %f, %f, %f\n", av.x(), av.y(), av.z());
  //   //printf("Engine postition%f, %f, %f\n", pos.x(), pos.y(), pos.z());
  // }
  //updates++;

  setRotorInput();
  for (RotorIter i = rotors.begin(); i != rotors.end(); ++i)
    (*i)->applyForcesAndTorques(fuselageBody);


  // TODO: steering force should be depending on engine_rps
  if (boomLength != 0.0)
    {
      btScalar steering_force = (313000.0 / (4000.0 / 60.0 * 6.28)) / boomLength * (1.0 + (1.5 * steer));
      HeloUtils::LocalApplyForce(fuselageBody, btVector3(-steering_force, 0.0, 0.0),
				 btVector3(0.0, 0.0, -boomLength));
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
