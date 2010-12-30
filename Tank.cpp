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
  rightDriveWheel.radius = 0.35;
  rightDriveWheel.currentAngularSpeed = 0.0;
  rightDriveWheel.momentOfInertia = 2000.0;
  rightDriveWheel.currentBrakeTorque = 0.0;
  rightDriveWheel.currentDriveTorque = 0.0;
  rightDriveWheel.currentAngle = 0.0;
  rightDriveWheel.currentLinearVelocity = btVector3(0.0, 0.0, 0.0);
  rightDriveWheel.currentAccelerationForce = 0.0;

  leftDriveWheel.radius = 0.35;
  leftDriveWheel.currentAngularSpeed = 0.0;
  leftDriveWheel.momentOfInertia = 2000.0;
  leftDriveWheel.currentBrakeTorque = 0.0;
  leftDriveWheel.currentDriveTorque = 0.0;
  leftDriveWheel.currentAngle = 0.0;
  leftDriveWheel.currentLinearVelocity = btVector3(0.0, 0.0, 0.0);
  leftDriveWheel.currentAccelerationForce = 0.0;
  currentSteerAngle = 0.0;
  currentDriveTorque = 0.0;
  steerSensitivity = 700000.0;
}

void RaycastTank::updateTread(bool right, btScalar suspensionForce, btScalar timestep)
{
  unsigned int idx = right ? 0 : 1;
  FakeWheel &fw = right ? rightDriveWheel : leftDriveWheel;

#warning linear velocity may be zero if wheel 0 is airborne
  btScalar vel_forward = wheels[idx].getLinearVelocity().dot(wheels[idx].getForward());
  btScalar long_slip = LongitudinalSlip(vel_forward, fw.currentAngularSpeed * fw.radius);
  btScalar long_idx = long_slip * 10.0;

  assert(fw.currentBrakeTorque >= 0);

  btScalar longitudinal_stick_threshold = 0.5;
  btScalar wheel_rotation_stick_threshold = 0.7;

  btScalar friction_torque = long_idx * suspensionForce * fw.radius;
  btScalar wMOI = fw.momentOfInertia;

  btScalar brakeTorque = fw.currentBrakeTorque;
  if (fw.currentAngularSpeed < 0.0)
    brakeTorque *= btScalar(-1.0);

  // Figure out how much torque it take to stop the wheel from spinning
  // And clamp the brakeing torque to this value
  btScalar maxBrakeTorque = (fw.currentDriveTorque - friction_torque
			     + (fw.currentAngularSpeed * wMOI / timestep));
  if ((brakeTorque > 0.0 and brakeTorque > maxBrakeTorque)
    or (brakeTorque < 0.0 and brakeTorque < maxBrakeTorque))
      brakeTorque = maxBrakeTorque;

  btScalar wheel_torque = fw.currentDriveTorque - friction_torque - brakeTorque;
  //printf("Wheel Torque: %f\n", wheel_torque);

  // Angular acceleration = Torque / Moment of inerita
  btScalar omega = wheel_torque / wMOI;
  // Angular velocity += Angular acceleration * time
  fw.currentAngularSpeed += omega * timestep;
  // Angle += Angular velocity * time + (Angular  acceleration * time^2) / 2
  fw.currentAngle += fw.currentAngularSpeed * timestep
    +  (omega * HeloUtils::POW2(timestep) / 2.0);

  // If there is no ground contact, this tire will not assert a force
  // on the car

#warning assuming tread cannot be airborne
  // if (airborne)
  //   {
  //     currentAccelerationForce = 0.0;
  //     return;
  //   }

  btScalar force = 0.0;
  if (fw.currentLinearVelocity.length() < longitudinal_stick_threshold
      and not fabs(fw.currentAngularSpeed) < wheel_rotation_stick_threshold
      and brakeTorque)
#warning equilibrium not calculated
    // We are moving very slowly, wheel is not rolling and breaks are
    // applied, calculate equilibrium instead of using normal impulse
    // calculation.
    //if (not currentSuspensionForce == 0.0)
    force = friction_torque / fw.radius; //longitudinalEquilibrium(timestep, chassisBody);
  else
    // This is the normal case when vehicle is moving.
    force = friction_torque / fw.radius;

  fw.currentAccelerationForce = force;
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

  rightDriveWheel.currentDriveTorque = (currentDriveTorque * 40.0) - (currentSteerAngle * steerSensitivity);
  leftDriveWheel.currentDriveTorque =  (currentDriveTorque * 40.0) + (currentSteerAngle * steerSensitivity);

  updateTread(true, right_side_force, timeStep);
  updateTread(false, left_side_force, timeStep);

  // for each _side_, figure out the friction index to use


  // for each _side_, update all wheels' rotations on that
  //  side to be consistent with the radius of the wheel and
  //  the friction of that side

  for (int i = 0; i < wheels.size(); ++i)
    {
      float rotation;
      if (i & 0x1) // left side
	rotation = leftDriveWheel.currentAngularSpeed * leftDriveWheel.radius;
      else
	rotation = rightDriveWheel.currentAngularSpeed * rightDriveWheel.radius;

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

#warning contact point is invalid if wheel is airborne
  btVector3 rap = wheels[0].getContactPoint();
  rap.setZ(0.0);
  btVector3 raf(0.0, 0.0, rightDriveWheel.currentAccelerationForce);
  chassisBody->applyImpulse(br * (raf * timeStep), br * rap);
  //printf("Acceleration force, right side: %f\n", rightDriveWheel.currentAccelerationForce);

  btVector3 lap = wheels[1].getContactPoint();
  lap.setZ(0.0);
  btVector3 laf(0.0, 0.0, leftDriveWheel.currentAccelerationForce);
  chassisBody->applyImpulse(br * (laf * timeStep), br * lap);
  //printf("Acceleration force, left side: %f\n", leftDriveWheel.currentAccelerationForce);
}

void RaycastTank::setDriveTorques(const std::vector<btScalar> &torques)
{
  currentDriveTorque = torques[0] * 2.0;
}

void RaycastTank::setSteer(btScalar radians_right)
{
  currentSteerAngle = radians_right;
}
