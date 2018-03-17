#include "Airplane.h"

AirplaneVehicle::AirplaneData Airplane::DataToAirplaneVehicleData(const AirplaneData &data)
{
  AirplaneVehicle::AirplaneData d;
  d.dragPolarK = data.dragPolarK;
  d.dragPolarD0 = data.dragPolarD0;
  d.wingArea = data.wingArea;
  d.wingAngle = HeloUtils::Deg2Rad(d.wingAngle);
  d.elevatorSensitivity = data.elevatorSensitivity;
  d.rudderSensitivity = data.rudderSensitivity;
  d.aileronSensitivity = data.aileronSensitivity;

  for (size_t i = 0; i < data.engines.size(); ++i)
    {
      const Engine &e = data.engines[i];
      AirplaneVehicle::Engine ve(e.position, e.direction, e.maxThrust);
      d.engines.push_back(ve);
    }

  for (size_t i = 0; i < data.cl_alpha_values.size(); ++i)
    d.clAlpha.addDataPoint(HeloUtils::Deg2Rad(data.cl_alpha_values[i].first),
                           data.cl_alpha_values[i].second);
  d.clAlpha.mirrorData(HeloUtils::Deg2Rad(360.0), -1.0);

  d.pitchStability1 = data.pitchStability1;
  d.pitchStability2 = data.pitchStability2;
  d.yawStability1 = data.yawStability1;
  d.yawStability2 = data.yawStability2;
  d.rollStability = data.rollStability;

  return d;
}

Airplane::Airplane() : Car() {}

Airplane *Airplane::load(const AirplaneData &data, Ogre::Root *root)
{
  Car::load(static_cast<const CarData &>(data), root);

  AirplaneVehicle::AirplaneData d = DataToAirplaneVehicleData(data);
  static_cast<AirplaneVehicle*>(rayCasters.back())->setData(d);

  hardpoints = NULL;
  return this;
}

CBRaycastVehicle *Airplane::createRaycastVehicle(btRigidBody *b)
{
 return new AirplaneVehicle(b);
}


AirplaneVehicle::AirplaneVehicle(btRigidBody *fuselage) :
    CBRaycastVehicle(fuselage)
{
}

void AirplaneVehicle::applyLift(btScalar timeStep, const btVector3 &localVelocity)
{
  btVector3 fw_vel(localVelocity);
  fw_vel.setX(0.0);

  btScalar angle_of_attack = HeloUtils::Deg2Rad(3.0);
  if (fw_vel.length2() != 0.0)
    angle_of_attack -= atan2(fw_vel.getY(), fw_vel.getZ());

  //std::cout << "Angle of attack: " <<  HeloUtils::Rad2Deg(angle_of_attack) << std::endl;
  btScalar q = HeloUtils::GetAirDensity(0.0) * fw_vel.length2() / 2.0;
  btScalar cl = d.clAlpha[fmod(angle_of_attack + d.wingAngle, 2 * HeloUtils::PI)];
  //cl = 1.4;
  //std::cout << "Cl: " << cl << std::endl;

  btScalar lift_force = cl * d.wingArea * q;

  btVector3 liftVec(fw_vel.cross(btVector3(1.0, 0.0, 0.0)));
  liftVec.normalize();

  // std::cout << "Lift force: " << lift_force << std::endl;
  HeloUtils::LocalApplyImpulse(chassisBody,
                               liftVec * lift_force * timeStep,
                               btVector3(0.0, 0.0, 0.0));
}

void AirplaneVehicle::applyDrag(btScalar timeStep, const btVector3 &localVelocity)
{
  btVector3 fw_vel(localVelocity);
  fw_vel.setX(0.0);

  btScalar angle_of_attack = HeloUtils::Deg2Rad(3.0);
  if (fw_vel.length2() != 0.0)
    angle_of_attack -= atan2(fw_vel.getY(), fw_vel.getZ());

  btScalar q = HeloUtils::GetAirDensity(0.0) * fw_vel.length2() / 2.0;
  btScalar cl = d.clAlpha[fmod(angle_of_attack + d.wingAngle, 2 * HeloUtils::PI)];

  btScalar drag_force = (d.dragPolarD0 + (d.dragPolarK * HeloUtils::POW2(cl))) * (d.wingArea * q);

  btVector3 dragVec(-fw_vel);
  dragVec.normalize();

  HeloUtils::LocalApplyImpulse(chassisBody,
                               dragVec * drag_force * timeStep,
                               btVector3(0.0, 0.0, 0.0));
}

void AirplaneVehicle::applyRudders(btScalar timeStep, const btVector3 &localVelocity, const btVector3 &localAngularVelocity)
{
  btScalar droll(localAngularVelocity.getZ());
  btScalar dpitch(localAngularVelocity.getX());
  btScalar dyaw(localAngularVelocity.getY());

  btScalar pitch_correction((localVelocity.getY() * -d.pitchStability1)
                            + (HeloUtils::POW2(dpitch) * d.pitchStability2
                               * (dpitch < 0.0 ? 1.0 : -1.0)));
  HeloUtils::LocalApplyTorqueImpulse(chassisBody,
                                     btVector3(pitch_correction * timeStep, 0, 0));

  btScalar yaw_correction((localVelocity.getX() * d.yawStability1)
                          + (HeloUtils::POW2(dyaw) * d.yawStability2
                             * (dyaw < 0.0 ? 1.0 : -1.0)));
  HeloUtils::LocalApplyTorqueImpulse(chassisBody,
                                     btVector3(0, yaw_correction * timeStep, 0));

  btScalar roll_correction(HeloUtils::POW2(droll) * d.rollStability
                           * (droll < 0.0 ? 1.0 : -1.0));
  HeloUtils::LocalApplyTorqueImpulse(chassisBody,
                                     btVector3(0, 0, roll_correction * timeStep));
}

void AirplaneVehicle::updateAction(btCollisionWorld* collisionWorld, btScalar timeStep)
{
  CBRaycastVehicle::updateAction(collisionWorld, timeStep);

  const btVector3 &vel = chassisBody->getLinearVelocity();
  const btVector3 &avel = chassisBody->getAngularVelocity();
  const btMatrix3x3 &inv_trans = chassisBody->getCenterOfMassTransform().getBasis().inverse();

  const btVector3 local_vel(inv_trans * vel);
  const btVector3 local_avel(inv_trans * avel);
  applyLift(timeStep, local_vel);
  applyDrag(timeStep, local_vel);
  applyRudders(timeStep, local_vel, local_avel);
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

  // AirplaneVehicle::ControlData &cd = airplane.getControlData();
  // cd.thrust = -e.state.mAxes[1].abs / static_cast<float>(OIS::JoyStick::MAX_AXIS);
  // cd.aileron = e.state.mAxes[3].abs / static_cast<float>(OIS::JoyStick::MAX_AXIS);
  // cd.elevator = -e.state.mAxes[2].abs / static_cast<float>(OIS::JoyStick::MAX_AXIS);
  // cd.rudder = -e.state.mAxes[0].abs / static_cast<float>(OIS::JoyStick::MAX_AXIS);

   return true;
}

void AirplaneJoystickController::setActive(bool a)
{
  Controller::setActive(a);
  joystick.setEventCallback(a ? this : NULL);
}

bool AirplaneKeyController::keyPressed(const OIS::KeyEvent& e)
{
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

  if (keyboard.isKeyDown(OIS::KC_1))
    throttle.setValue(0.1f);
  else if (keyboard.isKeyDown(OIS::KC_2))
    throttle.setValue(0.2f);
  else if (keyboard.isKeyDown(OIS::KC_3))
    throttle.setValue(0.3f);
  else if (keyboard.isKeyDown(OIS::KC_4))
    throttle.setValue(0.4f);
  else if (keyboard.isKeyDown(OIS::KC_5))
    throttle.setValue(0.5f);
  else if (keyboard.isKeyDown(OIS::KC_6))
    throttle.setValue(0.6f);
  else if (keyboard.isKeyDown(OIS::KC_7))
    throttle.setValue(0.7f);
  else if (keyboard.isKeyDown(OIS::KC_8))
    throttle.setValue(0.8f);
  else if (keyboard.isKeyDown(OIS::KC_9))
    throttle.setValue(0.9f);
  else if (keyboard.isKeyDown(OIS::KC_0))
    throttle.setValue(1.0f);
  else if (keyboard.isKeyDown(OIS::KC_2))
    throttle.setValue(0.2f);

  if (keyboard.isKeyDown(OIS::KC_W))
    pitch.setValue(1.0);
  else if (keyboard.isKeyDown(OIS::KC_S))
    pitch.setValue(-1.0);
  else
    pitch.setValue(0.0);

  if (keyboard.isKeyDown(OIS::KC_J))
    yaw.setValue(1.0);
  else if (keyboard.isKeyDown(OIS::KC_L))
    yaw.setValue(-1.0);
  else
    yaw.setValue(0.0);

  if (keyboard.isKeyDown(OIS::KC_D))
    roll.setValue(1.0);
  else if (keyboard.isKeyDown(OIS::KC_A))
    roll.setValue(-1.0);
  else
    roll.setValue(0.0);

}
