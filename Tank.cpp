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
}


RaycastTank::RaycastTank(btRigidBody *chassis) : CBRaycastVehicle(chassis)
{
  DriveWheelData d;

  d.relPos = btVector3(-1.125, 0.0, 0.0);
  d.suspensionLength = 50.0;
  d.maxLengthUp = 0.0;
  d.maxLengthDown = 100.0; // We always want to get a ground contact
  d.direction = btVector3(0.0, -1.0, 0.0);
  d.axle = btVector3(1.0, 0.0, 0.0);
  d.radius = 0.35;
  d.spring = 0.0;
  d.dampUp = 0.0;
  d.dampDown = 0.0;
  d.steerCoeff = 0.0;
  d.driveCoeff = 1.0;
  d.brakeCoeff = 1.0;
  d.momentOfInertia = 2000.0; // TODO: should sum the other wheels inertia and then some
  driveWheels.push_back(DriveWheel(d));

  d.relPos = btVector3(1.125, 0.0, 0.0);
  driveWheels.push_back(DriveWheel(d));

  currentSteerAngle = 0.0;
  currentDriveTorque = 0.0;
  steerSensitivity = 700000.0;
}

void RaycastTank::DriveWheel::updateTread(btScalar timeStep, btRigidBody *chassisBody)
{
  btScalar vel_forward = currentLinearVelocity.dot(currentForward);
  btScalar long_slip = LongitudinalSlip(vel_forward, currentAngularSpeed * d.radius);
  btScalar long_idx = long_slip * 10.0;

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

  // If there is no ground contact, this tire will not assert a force
  // on the car

#warning assuming tread cannot be airborne
  // if (airborne)
  //   {
  //     currentAccelerationForce = 0.0;
  //     return;
  //   }

  btScalar force = 0.0;
  if (currentLinearVelocity.length() < longitudinal_stick_threshold
      and not fabs(currentAngularSpeed) < wheel_rotation_stick_threshold
      and brakeTorque)
    // We are moving very slowly, wheel is not rolling and breaks are
    // applied, calculate equilibrium instead of using normal impulse
    // calculation.
    //if (not currentSuspensionForce == 0.0)
    force = longitudinalEquilibrium(timeStep, chassisBody);
  else
    // This is the normal case when vehicle is moving.
    force = friction_torque / d.radius;

  currentAccelerationForce = force;
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
      wheels[i].updateContact(collisionWorld, chassisBody);
      wheels[i].updateSuspension();

      if (wheels[i].isAirborne())
	continue;

      if (i & 0x1) // left side
	left_side_force += wheels[i].getSuspensionForce();
      else
	right_side_force += wheels[i].getSuspensionForce();
    }

  driveWheels[0].updateContact(collisionWorld, chassisBody);
  driveWheels[1].updateContact(collisionWorld, chassisBody);

  driveWheels[0].setSuspensionForce(right_side_force);
  driveWheels[1].setSuspensionForce(left_side_force);

  driveWheels[0].setTorque((currentDriveTorque * 40.0) - (currentSteerAngle * steerSensitivity));
  driveWheels[1].setTorque((currentDriveTorque * 40.0) + (currentSteerAngle * steerSensitivity));

  driveWheels[0].updateTread(timeStep, chassisBody);
  driveWheels[1].updateTread(timeStep, chassisBody);


  // Set a matching rotation for all the normal wheels
  btScalar right_rotation = driveWheels[0].getAngularSpeed() * driveWheels[0].getRadius();
  btScalar left_rotation = driveWheels[1].getAngularSpeed() * driveWheels[1].getRadius();
  for (int i = 0; i < wheels.size(); ++i)
    {
      btScalar rotation = i & 1 ? left_rotation : right_rotation;
      wheels[i].addRotation(rotation, rotation * timeStep);
    }


  // Somehow apply sane forces to the chassis
  for (int i = 0; i < wheels.size(); ++i)
    {
      if (wheels[i].isAirborne())
	continue;

      wheels[i].updateFriction(timeStep, chassisBody);
      btVector3 f = wheels[i].sumForces();
      f.setZ(0.0); // No per wheel acceleration
      //f.setX(0.0); // No per wheel acceleration
      assert(not std::isnan(f.x()));
      assert(not std::isnan(f.y()));
      //printf("Latitudal force: %f\n", f.x());
      btVector3 p = wheels[i].getContactPoint();
      // Note the relative position in WORLD COORDINATES as the second argument
      // That is why we multiply by the rotation matrix and not the full transform
      chassisBody->applyImpulse(br * (f * timeStep), br *p);
    }
  //printf("\n");

  btVector3 rap = driveWheels[0].getContactPoint();
  btVector3 raf = driveWheels[0].sumForces();
  chassisBody->applyImpulse(br * (raf * timeStep), br * rap);
  //printf("Acceleration force, right side: %f\n", raf.z());

  btVector3 lap = driveWheels[1].getContactPoint();
  btVector3 laf = driveWheels[1].sumForces();
  chassisBody->applyImpulse(br * (laf * timeStep), br * lap);
  //printf("Acceleration force, left side: %f\n", laf.z());
}

void RaycastTank::setDriveTorques(const std::vector<btScalar> &torques)
{
  currentDriveTorque = torques[0] * 2.0;
}

void RaycastTank::setSteer(btScalar radians_right)
{
  currentSteerAngle = radians_right;
}
