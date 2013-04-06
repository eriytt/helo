#include "Airplane.h"

AirplaneVehicle::AirplaneData Airplane::DataToAirplaneVehicleData(const AirplaneData &data)
{
  AirplaneVehicle::AirplaneData d;
  d.dragPolarK = data.dragPolarK;
  d.wingArea = data.wingArea;
  d.elevatorSensitivity = data.elevatorSensitivity;
  d.rudderSensitivity = data.rudderSensitivity;
  d.aileronSensitivity = data.aileronSensitivity;

  for (size_t i = 0; i < data.engineData.size(); ++i)
    {
      const Engine &e = data.engineData[i];
      AirplaneVehicle::Engine ve(HeloUtils::Ogre2BulletVector(e.position),
                                 HeloUtils::Ogre2BulletVector(e.direction),
                                 e.maxThrust);
      d.engines.push_back(ve);
    }  

  for (size_t i = 0; i < data.cl_alpha_values.size(); ++i)
    d.clAlpha.addDataPoint(HeloUtils::Deg2Rad(data.cl_alpha_values[i].first),
                           data.cl_alpha_values[i].second);
  d.clAlpha.mirrorData(HeloUtils::Deg2Rad(360.0), -1.0);

  d.pitchStability = data.pitchStability1;
  d.pitchStability2 = data.pitchStability2;
  d.yawStability = data.yawStability1;
  d.yawStability2 = data.yawStability2;
  d.rollStability = data.rollStability;

  return d;
}

Airplane::Airplane(const AirplaneData &data, Ogre::Root *root)
{
  controlData.thrust = 0.0;
  controlData.rudder = 0.0;
  controlData.elevator = 0.0;
  controlData.rudder = 0.0;
  controlData.aileron = 0.0;

  Ogre::SceneManager *mgr = root->getSceneManager("SceneManager");

  // create entity
  Ogre::Entity *ent = mgr->createEntity(data.name + "_ent", data.meshname);

  //ent->setMaterialName("shadow", "Terrain/Default");

  // create scene node
  node = mgr->getRootSceneNode()->createChildSceneNode(data.name + "_node");
  node->attachObject(ent);

  // create collision shape
  btCollisionShape* chassis_shape = new btBoxShape(btVector3(data.size.x / 2.0,
							     data.size.y / 2.0,
							     data.size.z / 2.0));
  btCompoundShape *comp = new btCompoundShape();

  // Transform of hull collision shape
  btTransform chassis_shape_trans;
  chassis_shape_trans.setIdentity();
  chassis_shape_trans.setOrigin(btVector3(0.0, 0.0, 0.0));
  comp->addChildShape(chassis_shape_trans, chassis_shape);
  shape = comp;

  // create fuselage rigid body
  btTransform tr;
  tr.setIdentity();
  tr.setOrigin(btVector3(data.position.x, data.position.y, data.position.z));
  tr.getBasis().setEulerZYX(data.rotation.x, data.rotation.y, data.rotation.z);

  body = Physics::CreateRigidBody(data.weight, tr, shape, node);
  airplane = new AirplaneVehicle(body, DataToAirplaneVehicleData(data));
  rayCastVehicle = airplane;

  
  for (unsigned int i = 0; i < data.wheelData.size(); ++i)
    {
      const WheelData &wd = data.wheelData[i];
      Ogre::Entity *tent = mgr->createEntity(data.name + "tire" + Ogre::String(1, static_cast<char>(i + 39)) + "_ent", "a10-tire.mesh");
      Ogre::SceneNode *tnode = node->createChildSceneNode(data.name + "tire" + Ogre::String(1, static_cast<char>(i + 39)) + "_node");
      tnode->attachObject(tent);
      btVector3 wheelpos(wd.relPos + (wd.direction * wd.suspensionLength));
      tnode->setPosition(Ogre::Vector3(wheelpos.x(), wheelpos.y(), wheelpos.z()));
      wheelNodes.push_back(tnode);
      rayCastVehicle->addWheel(wd);
    }
}

AirplaneVehicle::AirplaneVehicle(btRigidBody *fuselage, const AirplaneData &data) :
    CBRaycastVehicle(fuselage), d(data)
{
  controlData.thrust = 0.0;
  controlData.rudder = 0.0;
  controlData.elevator = 0.0;
  controlData.rudder = 0.0;
  controlData.aileron = 0.0;
}

void AirplaneVehicle::applyThrust(btScalar timeStep)
{
  // TODO: use apply central impulse
  //unsigned short num_engines = d.engines.size();
  btScalar thrust(0);
  for (std::vector<Engine>::const_iterator i = d.engines.begin(); i != d.engines.end(); ++i)
    {
      const Engine *e = &(*i);
      thrust += controlData.thrust * e->maxThrust;
    }

  HeloUtils::LocalApplyImpulse(chassisBody, btVector3(0.0, 0.0, thrust * timeStep),
                               btVector3(0.0, 0.0, 0.0));
}

void AirplaneVehicle::applyLift(btScalar timeStep, const btVector3 &localVelocity)
{
  btVector3 fw_vel(localVelocity);
  fw_vel.setX(0.0);

  btScalar angle_of_attack = HeloUtils::Deg2Rad(3.0);
  if (fw_vel.length2() != 0.0)
    angle_of_attack -= atan2(fw_vel.getY(), fw_vel.getZ());

  //std::cout << "Angle of attack: " <<  HeloUtils::Rad2Deg(angle_of_attack) << std::endl;
  btScalar cl = d.clAlpha[fmod(angle_of_attack, 2 * HeloUtils::PI)];
  //cl = 1.4;
  //std::cout << "Cl: " << cl << std::endl;
  btScalar q = HeloUtils::GetAirDensity(0.0) * fw_vel.length2() / 2.0;
  btScalar lift_force = cl * d.wingArea * q;

  btVector3 liftVec(fw_vel.cross(btVector3(1.0, 0.0, 0.0)));
  liftVec.normalize();

  //std::cout << "Lift force: " << lift_force << std::endl;
  HeloUtils::LocalApplyImpulse(chassisBody,
                               liftVec * lift_force * 1.2 * timeStep,
                               btVector3(0.0, 0.0, 0.0));

}

void AirplaneVehicle::applyDrag(btScalar timeStep, const btVector3 &localVelocity)
{
  // HeloUtils::LocalApplyImpulse(chassisBody,
  //                              btVector3(0.0,
  //                                        0.0,
  //                                        HeloUtils::POW2(velocityForward) * -9.0 * timeStep),
  //                              btVector3(0.0, 0.0, 0.0));
}

void AirplaneVehicle::applyRudders(btScalar timeStep, const btVector3 &localVelocity, const btVector3 &localAngularVelocity)
{
  btScalar droll(localAngularVelocity.getZ());
  btScalar dpitch(localAngularVelocity.getX());
  btScalar dyaw(localAngularVelocity.getY());

  // btScalar pitchStability(1000.0);
  // btScalar pitchStability2(2000000.0);
  // btScalar yawStability(3000.0);
  // btScalar yawStability2(3000000.0);
  // btScalar rollStability(500000.0);
    
  btScalar elevator = localVelocity.getZ() * d.elevatorSensitivity * controlData.elevator;
  btScalar pitch_correction((localVelocity.getY() * -d.pitchStability)
                            + (HeloUtils::POW2(dpitch) * d.pitchStability2
                               * (dpitch < 0.0 ? 1.0 : -1.0)));
  HeloUtils::LocalApplyTorqueImpulse(chassisBody,
                                     btVector3((elevator + pitch_correction) * timeStep, 0, 0));

  btScalar rudder(localVelocity.getZ() * d.rudderSensitivity * controlData.rudder);
  btScalar yaw_correction((localVelocity.getX() * d.yawStability)
                          + (HeloUtils::POW2(dyaw) * d.yawStability2
                             * (dyaw < 0.0 ? 1.0 : -1.0)));
  HeloUtils::LocalApplyTorqueImpulse(chassisBody,
                                     btVector3(0, (rudder + yaw_correction)* timeStep, 0));

  btScalar aileron(localVelocity.getZ() * d.aileronSensitivity * controlData.aileron);
  btScalar roll_correction(HeloUtils::POW2(droll) * d.rollStability
                           * (droll < 0.0 ? 1.0 : -1.0));
  HeloUtils::LocalApplyTorqueImpulse(chassisBody,
                                     btVector3(0, 0, (aileron + roll_correction) * timeStep));
}

void AirplaneVehicle::updateAction(btCollisionWorld* collisionWorld, btScalar timeStep)
{
  CBRaycastVehicle::updateAction(collisionWorld, timeStep);
  
  const btVector3 &vel = chassisBody->getLinearVelocity();
  const btVector3 &avel = chassisBody->getAngularVelocity();
  const btMatrix3x3 &inv_trans = chassisBody->getCenterOfMassTransform().getBasis().inverse();

  const btVector3 local_vel(inv_trans * vel);
  const btVector3 local_avel(inv_trans * avel);
  applyThrust(timeStep);
  applyLift(timeStep, local_vel);
  applyDrag(timeStep, local_vel);
  applyRudders(timeStep, local_vel, local_avel);
}

void AirplaneVehicle::setInput(const AirplaneVehicle::ControlData &cd)
{
  // Does it matter ? 
#warning Not thread safe, AirplaneVehicle lives in physics thread
  controlData = cd;
}

Controller *Airplane::createController(OIS::Object *dev)
{
  if (dynamic_cast<OIS::JoyStick*>(dev))
    return controller = new AirplaneJoystickController(*static_cast<OIS::JoyStick*>(dev), *this);
  else if (dynamic_cast<OIS::Mouse*>(dev))
    return NULL;
  else if (dynamic_cast<OIS::Keyboard*>(dev))
    return controller = new AirplaneKeyController(*static_cast<OIS::Keyboard*>(dev), *this);
  else
    return NULL;
}


bool AirplaneJoystickController::axisMoved(const OIS::JoyStickEvent &e, int)
{
  if (not active)
    return true;

  AirplaneVehicle::ControlData &cd = airplane.getControlData();
  cd.thrust = -e.state.mAxes[1].abs / static_cast<float>(OIS::JoyStick::MAX_AXIS);
  cd.aileron = e.state.mAxes[3].abs / static_cast<float>(OIS::JoyStick::MAX_AXIS);
  cd.elevator = -e.state.mAxes[2].abs / static_cast<float>(OIS::JoyStick::MAX_AXIS);
  cd.rudder = -e.state.mAxes[0].abs / static_cast<float>(OIS::JoyStick::MAX_AXIS);

  airplane.setInput();
  airplane.setSteer(cd.rudder * -0.5);
   return true;
}

void AirplaneJoystickController::setActive(bool a)
{
  Controller::setActive(a);
  joystick.setEventCallback(a ? this : NULL);
}

bool AirplaneKeyController::keyPressed(const OIS::KeyEvent& e)
{
  AirplaneVehicle::ControlData &cd = airplane.getControlData();
  if (e.key == OIS::KC_W)
    cd.thrust += 0.1;
  else if (e.key == OIS::KC_S)
    cd.thrust -= 0.1;

  if (cd.thrust > 1.0)
    cd.thrust = 1.0;
  else if (cd.thrust < -0.0)
    cd.thrust = 0.0;
  airplane.setInput();
  
  return true;
}

bool AirplaneKeyController::keyReleased(const OIS::KeyEvent& e)
{
  return true;
}

void AirplaneKeyController::update(float timeDelta)
{
  if (not active)
    return;

  AirplaneVehicle::ControlData &cd = airplane.getControlData();

  if (keyboard.isKeyDown(OIS::KC_W))
    cd.thrust = 1.0;
  else
    cd.thrust = 0.0;

  if (keyboard.isKeyDown(OIS::KC_A))
    cd.rudder = 1.0;
  else if (keyboard.isKeyDown(OIS::KC_D))
    cd.rudder = -1.0;
  else
    cd.rudder = 0.0;

  if (keyboard.isKeyDown(OIS::KC_I))
    cd.elevator = 1.0;
  else if (keyboard.isKeyDown(OIS::KC_K))
    cd.elevator = -1.0;
  else
    cd.elevator = 0.0;

  if (keyboard.isKeyDown(OIS::KC_J))
    cd.aileron = -1.0;
  else if (keyboard.isKeyDown(OIS::KC_L))
    cd.aileron = 1.0;
  else
    cd.aileron = 0.0;

  airplane.setInput();

  airplane.setSteer(cd.rudder * -0.5);
}
