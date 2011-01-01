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
  rayCastVehicle = new RaycastTank(body);
  for (unsigned int i = 0; i < data.wheelData.size(); ++i)
    {
      const WheelData &wd = data.wheelData[i];
      Ogre::Entity *tent = mgr->createEntity(data.name + "tire" + Ogre::String(1, static_cast<char>(i + 39)) + "_ent", "hmmwv-tire.mesh");
      Ogre::SceneNode *tnode = node->createChildSceneNode(data.name + "tire" + Ogre::String(1, static_cast<char>(i + 39)) + "_node");
      tnode->attachObject(tent);
      btVector3 wheelpos(wd.relPos + (wd.direction * wd.suspensionLength));
      tnode->setPosition(Ogre::Vector3(wheelpos.x(), wheelpos.y(), wheelpos.z()));
      wheelNodes.push_back(tnode);
      rayCastVehicle->addWheel(wd);
    }

  for (unsigned int i = 0; i < 2; ++i)
    {
      Ogre::Entity *tent = mgr->createEntity(data.name + "drivetire" + Ogre::String(1, static_cast<char>(i + 39)) + "_ent", "hmmwv-tire.mesh");
      Ogre::SceneNode *tnode = node->createChildSceneNode(data.name + "drivetire" + Ogre::String(1, static_cast<char>(i + 39)) + "_node");
      tnode->attachObject(tent);
      btVector3 wheelpos(0.0, 0.0, 0.0);
      tnode->setPosition(Ogre::Vector3(wheelpos.x(), wheelpos.y(), wheelpos.z()));
      driveWheelNodes.push_back(tnode);
    }

    for (unsigned int i = 0; i < 2; ++i)
    {
      Ogre::Entity *tent = mgr->createEntity(data.name + "spintire" + Ogre::String(1, static_cast<char>(i + 39)) + "_ent", "hmmwv-tire.mesh");
      Ogre::SceneNode *tnode = node->createChildSceneNode(data.name + "spintire" + Ogre::String(1, static_cast<char>(i + 39)) + "_node");
      tnode->attachObject(tent);
      btVector3 wheelpos(0.0, 0.0, 4.0);
      tnode->setPosition(Ogre::Vector3(wheelpos.x(), wheelpos.y(), wheelpos.z()));
      spinWheelNodes.push_back(tnode);
    }


}

void Tank::finishPhysicsConfiguration(class Physics *phys)
{
  Car::finishPhysicsConfiguration(phys);

  RaycastTank *rc_tank = dynamic_cast<RaycastTank*>(rayCastVehicle);

  for (unsigned int i = 0; i != 2; ++i)
    {
      btTransform wheel_trans(rc_tank->getDriveWheel(i)->getTransform());
      HeloMotionState *ms = new HeloMotionState(wheel_trans, driveWheelNodes[i]);
      rc_tank->getDriveWheel(i)->setMotionState(ms);
      phys->addMotionState(ms);
    }

  for (unsigned int i = 0; i != 2; ++i)
    {
      btTransform wheel_trans(rc_tank->getSpinWheel(i)->getTransform());
      HeloMotionState *ms = new HeloMotionState(wheel_trans, spinWheelNodes[i]);
      rc_tank->getSpinWheel(i)->setMotionState(ms);
      phys->addMotionState(ms);
    }

}

RaycastTank::RaycastTank(btRigidBody *chassis) : CBRaycastVehicle(chassis)
{
  DriveWheelData d;

  d.relPos = btVector3(-1.125, 0.0, 0.0);
  d.realRelPos = btVector3(-1.125, 0.0, -4.0);
  d.suspensionLength = 50.0;
  d.maxLengthUp = 0.0;
  d.maxLengthDown = 100.0; // We always want to get a ground contact
  d.direction = btVector3(0.0, -1.0, 0.0);
  d.axle = btVector3(1.0, 0.0, 0.0);
  d.radius = 0.55;
  d.spring = 0.0;
  d.dampUp = 0.0;
  d.dampDown = 0.0;
  d.steerCoeff = 0.0;
  d.driveCoeff = 1.0;
  d.brakeCoeff = 1.0;
  d.momentOfInertia = 2000.0; // TODO: should sum the other wheels inertia and then some
  driveWheels.push_back(new DriveWheel(d));

  d.relPos = btVector3(1.125, 0.0, 0.0);
  d.realRelPos = btVector3(1.125, 0.0, -4.0);
  driveWheels.push_back(new DriveWheel(d));

  SpinWheelData sd;
  sd.relPos = btVector3(-1.125, -0.2, 4.0);
  sd.axle = btVector3(1.0, 0.0, 0.0);
  sd.radius = 0.35;
  spinWheels.push_back(new SpinWheel(sd));


  sd.relPos = btVector3(1.125, -0.2, 4.0);
  spinWheels.push_back(new SpinWheel(sd));

  currentSteerAngle = 0.0;
  currentDriveTorque = 0.0;
  steerSensitivity = 250000.0;
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
  btScalar long_idx = long_slip * 5.0;

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
  //printf("Wheel Torque: %f\n", wheel_torque);

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
    // This is the normal case when vehicle is moving.
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

  // for each wheel, update contact
  // for each wheel, update suspension
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

  driveWheels[0]->updateContact(collisionWorld, chassisBody);
  driveWheels[1]->updateContact(collisionWorld, chassisBody);

  driveWheels[0]->setSuspensionForce(right_side_force);
  driveWheels[1]->setSuspensionForce(left_side_force);

  driveWheels[0]->setTorque((currentDriveTorque * 40.0) - (currentSteerAngle * steerSensitivity));
  driveWheels[1]->setTorque((currentDriveTorque * 40.0) + (currentSteerAngle * steerSensitivity));

  driveWheels[0]->setAirborne((right_side_force == 0.0));
  driveWheels[1]->setAirborne((left_side_force == 0.0));

  driveWheels[0]->updateTread(timeStep, chassisBody);
  driveWheels[1]->updateTread(timeStep, chassisBody);


  // Set a matching rotation for all the normal wheels
  btScalar right_rotation = driveWheels[0]->getAngularSpeed() * driveWheels[0]->getRadius();
  btScalar left_rotation = driveWheels[1]->getAngularSpeed() * driveWheels[1]->getRadius();
  for (unsigned int i = 0; i < wheels.size(); ++i)
    {
      btScalar rotation = i & 1 ? left_rotation : right_rotation;
      wheels[i]->addRotation(rotation, rotation * timeStep);
      wheels[i]->updateMotionState();
    }

  for (unsigned int i = 0; i < 2; ++i)
    {
      btScalar rotation = i & 1 ? left_rotation : right_rotation;
      spinWheels[i]->addRotation(rotation * timeStep);
      spinWheels[i]->updateMotionState();
    }


  // Somehow apply sane forces to the chassis
  for (unsigned int i = 0; i < wheels.size(); ++i)
    {
      if (wheels[i]->isAirborne())
	continue;

      wheels[i]->updateFriction(timeStep, chassisBody);
      btVector3 f = wheels[i]->sumForces();
      f.setZ(0.0); // No per wheel acceleration
      //f.setX(0.0); // No per wheel acceleration
      assert(not std::isnan(f.x()));
      assert(not std::isnan(f.y()));
      //printf("Latitudal force: %f\n", f.x());
      btVector3 p = wheels[i]->getContactPoint();
      // Note the relative position in WORLD COORDINATES as the second argument
      // That is why we multiply by the rotation matrix and not the full transform
      chassisBody->applyImpulse(br * (f * timeStep), br *p);
    }
  //printf("\n");

  btVector3 rap = driveWheels[0]->getContactPoint();
  btVector3 raf = driveWheels[0]->sumForces();
  chassisBody->applyImpulse(br * (raf * timeStep), br * rap);
  //printf("Acceleration force, right side: %f\n", raf.z());

  btVector3 lap = driveWheels[1]->getContactPoint();
  btVector3 laf = driveWheels[1]->sumForces();
  chassisBody->applyImpulse(br * (laf * timeStep), br * lap);
  //printf("Acceleration force, left side: %f\n", laf.z());

  driveWheels[0]->updateMotionState();
  driveWheels[1]->updateMotionState();
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

