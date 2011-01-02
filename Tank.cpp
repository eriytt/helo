#include "Tank.h"

Tank::Tank(const TankData &data, Ogre::Root *root)
{
  Ogre::SceneManager *mgr = root->getSceneManager("SceneManager");

  // create entity
  Ogre::Entity *ent = mgr->createEntity(data.name + "_ent", data.meshname);

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
  chassis_shape_trans.setOrigin(btVector3(0.0, 0.5, 0.0));
  comp->addChildShape(chassis_shape_trans, chassis_shape);
  shape = comp;

  // create fuselage rigid body
  btTransform tr;
  tr.setIdentity();
  tr.setOrigin(btVector3(data.position.x, data.position.y, data.position.z));

  body = Physics::CreateRigidBody(data.weight, tr, shape, node);
  RaycastTank *rc_tank = new RaycastTank(body);
  rayCastVehicle = rc_tank;

  for (unsigned int i = 0; i < data.wheelData.size(); ++i)
    {
      const WheelData &wd = data.wheelData[i];
      Ogre::Entity *tent = mgr->createEntity(data.name + "tire" + Ogre::String(1, static_cast<char>(i + 39)) + "_ent", "hmmwv-tire.mesh");
      Ogre::SceneNode *tnode = node->createChildSceneNode(data.name + "tire" + Ogre::String(1, static_cast<char>(i + 39)) + "_node");
      tnode->attachObject(tent);
      btVector3 wheelpos(wd.relPos + (wd.direction * wd.suspensionLength));
      tnode->setPosition(Ogre::Vector3(wheelpos.x(), wheelpos.y(), wheelpos.z()));
      wheelNodes.push_back(tnode);
      rc_tank->addWheel(wd);
    }

  for (unsigned int i = 0; i < data.driveWheelData.size(); ++i)
    {
      const DriveWheelData &wd = data.driveWheelData[i];
      Ogre::Entity *tent = mgr->createEntity(data.name + "drivetire" + Ogre::String(1, static_cast<char>(i + 39)) + "_ent", "hmmwv-tire.mesh");
      Ogre::SceneNode *tnode = node->createChildSceneNode(data.name + "drivetire" + Ogre::String(1, static_cast<char>(i + 39)) + "_node");
      tnode->attachObject(tent);
      btVector3 wheelpos(wd.realRelPos);
      tnode->setPosition(Ogre::Vector3(wheelpos.x(), wheelpos.y(), wheelpos.z()));
      driveWheelNodes.push_back(tnode);
      rc_tank->addDriveWheel(wd);
    }

  for (unsigned int i = 0; i < data.spinWheelData.size(); ++i)
    {
      const SpinWheelData &wd = data.spinWheelData[i];
      Ogre::Entity *tent = mgr->createEntity(data.name + "spintire" + Ogre::String(1, static_cast<char>(i + 39)) + "_ent", "hmmwv-tire.mesh");
      Ogre::SceneNode *tnode = node->createChildSceneNode(data.name + "spintire" + Ogre::String(1, static_cast<char>(i + 39)) + "_node");
      tnode->attachObject(tent);
      btVector3 wheelpos(wd.relPos);
      tnode->setPosition(Ogre::Vector3(wheelpos.x(), wheelpos.y(), wheelpos.z()));
      spinWheelNodes.push_back(tnode);
      rc_tank->addSpinWheel(wd);
    }
}

void Tank::finishPhysicsConfiguration(class Physics *phys)
{
  Car::finishPhysicsConfiguration(phys);

  RaycastTank *rc_tank = dynamic_cast<RaycastTank*>(rayCastVehicle);

  for (unsigned int i = 0; i != driveWheelNodes.size(); ++i)
    {
      btTransform wheel_trans(rc_tank->getDriveWheel(i)->getTransform());
      HeloMotionState *ms = new HeloMotionState(wheel_trans, driveWheelNodes[i]);
      rc_tank->getDriveWheel(i)->setMotionState(ms);
      phys->addMotionState(ms);
    }

  for (unsigned int i = 0; i != spinWheelNodes.size(); ++i)
    {
      btTransform wheel_trans(rc_tank->getSpinWheel(i)->getTransform());
      HeloMotionState *ms = new HeloMotionState(wheel_trans, spinWheelNodes[i]);
      rc_tank->getSpinWheel(i)->setMotionState(ms);
      phys->addMotionState(ms);
    }

}

RaycastTank::RaycastTank(btRigidBody *chassis) : CBRaycastVehicle(chassis)
{
  currentSteerAngle = 0.0;
  currentDriveTorque = 0.0;
  steerSensitivity = 250000.0;
}

void RaycastTank::addDriveWheel(const DriveWheelData &data)
{
  driveWheels.push_back(new DriveWheel(data));
}

void RaycastTank::addSpinWheel(const SpinWheelData &data)
{
  spinWheels.push_back(new SpinWheel(data));
}


void RaycastTank::SuspensionWheel::updateFriction(btScalar timeStep, btRigidBody *chassisBody)
{
  btScalar lateral_stick_threshold = 1.5;
  btScalar rotational_stick_threshold_squared = HeloUtils::POW2(HeloUtils::PI_4 / 10.0);

  longIdx = latIdx = 0.0f;

  // If the wheel is airborne there is nothing to do
  if (airborne)
    return;

  btScalar rotational_speed_squared = chassisBody->getAngularVelocity().length2();
  if (currentLinearVelocity.length() > lateral_stick_threshold
      or rotational_speed_squared > rotational_stick_threshold_squared)
    {
      btScalar lat_slip = currentLinearVelocity.dot(currentAxle);
      latIdx = lat_slip * -0.3;
    }
  else
    {
      if (not currentSuspensionForce == 0.0)
	latIdx = lateralEquilibrium(timeStep, chassisBody);
      assert(not std::isinf(latIdx));
    }
  longIdx = 0.0f;
}

RaycastTank::DriveWheel::DriveWheel(const DriveWheelData &data) :
  CBRaycastVehicle::Wheel(static_cast<const WheelData &>(data))
{
  realRelPos = data.realRelPos;
}

void RaycastTank::DriveWheel::updateTread(btScalar timeStep, btRigidBody *chassisBody)
{
  btScalar vel_forward = currentLinearVelocity.dot(currentForward);
  btScalar long_slip = LongitudinalSlip(vel_forward, currentAngularSpeed * d.radius);
  btScalar long_idx = long_slip * 7.0;

  assert(currentBrakeTorque >= 0);

  btScalar longitudinal_stick_threshold = 0.5;
  btScalar wheel_rotation_stick_threshold = 0.7;

  btScalar friction_torque = long_idx * currentSuspensionForce * d.radius;
  btScalar wMOI = d.momentOfInertia;

  btScalar brakeTorque = currentBrakeTorque;
  if (currentAngularSpeed < 0.0)
    brakeTorque *= btScalar(-1.0);

  // Figure out how much torque it take to stop the wheel from spinning
  // And clamp the brakeing torque to this value
  btScalar maxBrakeTorque = (currentDriveTorque - friction_torque
			     + (currentAngularSpeed * wMOI / timeStep));
  if ((brakeTorque > 0.0 and brakeTorque > maxBrakeTorque)
    or (brakeTorque < 0.0 and brakeTorque < maxBrakeTorque))
      brakeTorque = maxBrakeTorque;

  btScalar wheel_torque = currentDriveTorque - friction_torque - brakeTorque;

  // Angular acceleration = Torque / Moment of inerita
  btScalar omega = wheel_torque / wMOI;
  // Angular velocity += Angular acceleration * time
  currentAngularSpeed += omega * timeStep;
  // Angle += Angular velocity * time + (Angular  acceleration * time^2) / 2
  currentAngle += currentAngularSpeed * timeStep
    +  (omega * HeloUtils::POW2(timeStep) / 2.0);

  if (airborne)
    {
      currentAccelerationForce = 0.0;
      return;
    }

  btScalar force = 0.0;
  if (currentLinearVelocity.length() < longitudinal_stick_threshold
      and not fabs(currentAngularSpeed) < wheel_rotation_stick_threshold
      and brakeTorque)
    force = longitudinalEquilibrium(timeStep, chassisBody);
  else
    force = friction_torque / d.radius;

  currentAccelerationForce = force;
}

void RaycastTank::DriveWheel::updateMotionState()
{
  if (not motionState)
    return;

  currentTransform.setOrigin(realRelPos);
  currentTransform.setRotation(btQuaternion(d.axle, currentAngle));
  motionState->setWorldTransform(currentTransform);
}

void RaycastTank::SpinWheel::updateMotionState()
{
  if (not motionState)
    return;
  motionState->setWorldTransform(btTransform(btQuaternion(d.axle, currentAngle), d.relPos));
}

void RaycastTank::updateAction(btCollisionWorld* collisionWorld, btScalar timeStep)
{
  const btTransform &bt = chassisBody->getCenterOfMassTransform();
  const btMatrix3x3 &br = bt.getBasis();

  float right_side_force = 0.0;
  float left_side_force = 0.0;


  for (unsigned int i = 0; i < wheels.size(); ++i)
    {
      wheels[i]->updateContact(collisionWorld, chassisBody);
      wheels[i]->updateSuspension();

      if (wheels[i]->isAirborne())
	continue;

      if (i & 0x1) // left side
	left_side_force += wheels[i]->getSuspensionForce();
      else
	right_side_force += wheels[i]->getSuspensionForce();
    }

  for (unsigned int i = 0; i < driveWheels.size(); ++i)
    {
      btScalar steerTorque = currentSteerAngle * steerSensitivity;
      bool airborne = i & 1 ? (left_side_force == 0.0) : (right_side_force == 0.0);
      steerTorque *= i & 1 ? 1.0 : -1.0;

      driveWheels[i]->updateContact(collisionWorld, chassisBody);
      driveWheels[i]->setSuspensionForce(i & 1 ? left_side_force : right_side_force);
      driveWheels[i]->setTorque((currentDriveTorque * 40.0) + steerTorque);
      driveWheels[i]->setAirborne(airborne);
      driveWheels[i]->updateTread(timeStep, chassisBody);
    }


  for (unsigned int i = 0; i < driveWheels.size(); ++i)
    {
      btScalar rotation = driveWheels[i]->getAngularSpeed() * driveWheels[i]->getRadius();
      for (unsigned int j = (i & 1); j < wheels.size(); j += 2)
	{
	  wheels[j]->addRotation(rotation, rotation * timeStep);
	  wheels[j]->updateMotionState();
	}

      for (unsigned int j = (i & 1); j < spinWheels.size(); j += 2)
	{
	  spinWheels[j]->addRotation(rotation * timeStep);
	  spinWheels[j]->updateMotionState();
	}
    }

  for (unsigned int i = 0; i < wheels.size(); ++i)
    {
      if (wheels[i]->isAirborne())
	continue;

      wheels[i]->updateFriction(timeStep, chassisBody);
      btVector3 f = wheels[i]->sumForces();
      f.setZ(0.0); // No per wheel acceleration
      assert(not std::isnan(f.x()));
      assert(not std::isnan(f.y()));
      btVector3 p = wheels[i]->getContactPoint();
      chassisBody->applyImpulse(br * (f * timeStep), br *p);
    }


  for (unsigned int i = 0; i < driveWheels.size(); ++i)
    {
      btVector3 cp = driveWheels[i]->getContactPoint();
      btVector3 f = driveWheels[i]->sumForces();
      chassisBody->applyImpulse(br * (f * timeStep), br * cp);
      driveWheels[i]->updateMotionState();
    }
}

void RaycastTank::setDriveTorques(const std::vector<btScalar> &torques)
{
  currentDriveTorque = torques[0] * 2.0;
}

void RaycastTank::setSteer(btScalar radians_right)
{
  currentSteerAngle = radians_right;
}

void RaycastTank::addWheel(const WheelData &data)
{
  wheels.push_back(new SuspensionWheel(dynamic_cast<const SuspensionWheelData &>(data)));
}

RaycastTank::DriveWheel *RaycastTank::getDriveWheel(unsigned int i)
{
  assert(i < driveWheels.size());
  return driveWheels[i];
}

RaycastTank::SpinWheel *RaycastTank::getSpinWheel(unsigned int i)
{
  assert(i < spinWheels.size());
  return spinWheels[i];
}
