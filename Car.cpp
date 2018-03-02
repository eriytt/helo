#include "Car.h"

#include <cassert>

#include <algorithm>

#include <btBulletDynamicsCommon.h>
#include <BulletDynamics/ConstraintSolver/btContactConstraint.h>

#include "Physics.h"
#include "DriveTrain.h"
#include "Utils.h"

static HeloUtils::PieceWiseLinearFunction latMu;
static HeloUtils::PieceWiseLinearFunction longMu;

inline btScalar SlipAngle(btScalar speed, btScalar slip_speed)
{
  if (fabs(speed) < 0.01f and fabs(slip_speed) < 0.01f)
    return 0.0;
  return atan(slip_speed / speed);
}

void WheelAngleActuator::actuate(btScalar step)
{
  if (not control)
    return;

  btScalar radians_right = control->getValue() * HeloUtils::PI_4;
  for (auto w : wheels)
    w->setSteer(radians_right);
}

void WheelTorqueActuator::actuate(btScalar step)
{
  if (not control)
    return;

  btScalar throttleValue = control->getValue();
  for (auto w : wheels)
    w->setTorque(throttleValue * 1000);
}


void HingeActuator::actuate(btScalar step)
{
  if (not control)
    return;

  btScalar radians_right = control->getValue() * HeloUtils::PI_4;
  hinge->setMotorTarget(-radians_right / 2.0, 0.1);
}


CBRaycastVehicle::Wheel::Wheel(const SuspensionWheelData &data)
{
  d = data;
  airborne = true; // makes contactPoint and contactNormal undefined
  currentSuspensionLength = d.suspensionLength;
  currentLinearVelocity = btVector3(0.0, 0.0, 0.0);
  currentSuspensionForce = 0.0;
  currentAxle = data.axle;
#warning hardcoded forward vector
  currentForward = btVector3(0.0, 0.0, 1.0);
  currentAngularSpeed = 0.0;
  currentAngle = 0.0;
  // longIdx and latIdx are undefined when airborne
  currentAccelerationForce = 0.0;
  currentGround = NULL;
  currentDriveTorque = 0.0;
  currentBrakeTorque = 0.0;
  currentSteerAngle = 0.0;
  motionState = NULL;
}

void CBRaycastVehicle::Wheel::setSteer(btScalar right_radians)
{
  if (d.steerCoeff)
    {
      currentSteerAngle = right_radians * d.steerCoeff;
      currentAxle = btVector3(btMatrix3x3(btQuaternion(d.direction, currentSteerAngle)) * d.axle);
    }
}

void CBRaycastVehicle::Wheel::setTorque(btScalar torque)
{
  if (d.driveCoeff)
    currentDriveTorque = d.driveCoeff * torque;
}

const btTransform &CBRaycastVehicle::Wheel::getTransform() const
{
  return currentTransform;
}

void CBRaycastVehicle::Wheel::updateContact(btCollisionWorld *world, btRigidBody *chassisBody)
{
  btVehicleRaycaster::btVehicleRaycasterResult rayResults;
  const btTransform &chassisTransform = chassisBody->getCenterOfMassTransform();
  btVector3 rayvector = d.direction * (d.suspensionLength + d.radius);

  // Collision testing is done in world coordinates
  btVector3 source = chassisTransform * d.relPos;
  btVector3 target = chassisTransform * (d.relPos + rayvector);
  btCollisionWorld::ClosestRayResultCallback rayCallback(source, target);
  // TODO: How to make sure the ray does not hit ourselves?
  world->rayTest(source, target, rayCallback);


  const btRigidBody* hit = rayCallback.m_collisionObject ? btRigidBody::upcast(rayCallback.m_collisionObject) : 0;

  if ((not hit) or (not hit->hasContactResponse()))
    {
      airborne = true;
      currentGround = NULL;
      //put wheel info as in rest position
      currentSuspensionLength = d.suspensionLength;
      currentLinearVelocity = btVector3(0, 0, 0);
    }
  else
    {
      btTransform invChassisTransform = chassisTransform.inverse();
      const btMatrix3x3 &inv_rot = invChassisTransform.getBasis();

      airborne = false;
      currentGround = const_cast<btRigidBody*>(hit); // Hmm, dangerous
						     // cast, how to
						     // do this?
      contactPoint = invChassisTransform * rayCallback.m_hitPointWorld;
      contactNormal = inv_rot * rayCallback.m_hitNormalWorld;
      // TODO: is it really necessary to normalize the normal?
      contactNormal.normalize();

      currentSuspensionLength = (contactPoint - d.relPos).length() - d.radius;
      if (currentSuspensionLength < d.maxLengthUp)
	  currentSuspensionLength = d.maxLengthUp;
      if (currentSuspensionLength > d.maxLengthDown)
	  currentSuspensionLength = d.maxLengthDown;

      btVector3 rel_pos(rayCallback.m_hitPointWorld - chassisBody->getCenterOfMassPosition());
      currentLinearVelocity = (inv_rot * chassisBody->getVelocityInLocalPoint(rel_pos));
    }
}

void CBRaycastVehicle::Wheel::updateSuspension()
{
  if (airborne)
    {
      currentSuspensionForce = 0.0;
      return;
    }

  // Spring
  btScalar length_diff = d.suspensionLength - currentSuspensionLength;
  btScalar force = d.spring * length_diff;

  // Damper
  btVector3 suspension_vector = contactPoint - d.relPos;
  suspension_vector.normalize();
  btScalar suspension_speed = currentLinearVelocity.dot(suspension_vector);
  btScalar suspension_damping = suspension_speed >= 0 ? d.dampDown : d.dampUp;
  force += suspension_damping * suspension_speed;

  if (force < 0.0)
    force = 0.0;
  currentSuspensionForce = force;
  return;
}

void CBRaycastVehicle::Wheel::updateFriction(btScalar timeStep, btRigidBody *chassisBody)
{
  //btScalar lateral_stick_threshold = 3.0;
  btScalar lateral_stick_threshold = 1.5;
  btScalar rotational_stick_threshold_squared = HeloUtils::POW2(HeloUtils::PI_4 / 10.0);

  longIdx = latIdx = 0.0f;

  // If the wheel is airborne there is nothing to do
  if (airborne)
    return;

  btScalar vel_forward = currentLinearVelocity.dot(currentForward);
  btScalar rotational_speed_squared = chassisBody->getAngularVelocity().length2();


  btScalar slip_angle;
  if (currentLinearVelocity.length() > lateral_stick_threshold
      or rotational_speed_squared > rotational_stick_threshold_squared)
    {
      //std::cout << "Wheel is unstable" << std::endl;
      // Calculate lateral slip.
      // Lateral slip is the speed of the car in the contact point
      // projected on the axle of the tire
      btScalar lat_slip = currentLinearVelocity.dot(currentAxle);
      slip_angle = SlipAngle(fabs(vel_forward), lat_slip);
      // The slip angle needs to have a (semi) realistic upper bound or
      // a tank like vehicle will not be able to turn when standing still.
      // This does not affect normal car-like vehicles much as they typically
      // do not turn their steering wheels that much. It only takes it longer
      // to stop when spinning out of control.
      slip_angle = HeloUtils::clamp(HeloUtils::PI_4, -HeloUtils::PI_4, slip_angle);
      //printf("Slip Angle: %f\n", slip_angle);
      latIdx = -slip_angle * 3.0f;
      assert(not std::isinf(latIdx));
    }
  else
    {
      //std::cout << "Wheel is stable" << std::endl;
      if (not (currentSuspensionForce == 0.0))
	latIdx = lateralEquilibrium(timeStep, chassisBody);
      assert(not std::isinf(latIdx));
    }

  // Get longitudinal slip.
  btScalar tire_speed = currentAngularSpeed * d.radius;
  btScalar long_slip = LongitudinalSlip(vel_forward, tire_speed);
  //printf("vel forward: %f, vel tire: %f\n", vel_forward, vel_tire);

  //printf("long slip: %f\n", long_slip);

  // These two should be be calculated based on the above slips
  longIdx = long_slip * 2.3f;
  //longIdx = 0.0f;
  //latIdx = clamp(-slip_angle * 2.3f, -1.0f, 1.0f);

  //std::cout << "longIdx: " << longIdx << " latIdx: " << latIdx << std::endl;
  // Multiply the lateral slip with a saturation factor that is zero
  // when the car is not moving
  //if (latIdx > 0.8)
  //printf("latIdx: %f, velocity: %f\n", latIdx, wheelState.velocity.length());
  //latIdx *= clamp(wheelState.velocity.length(), 0.00f, 5.0f) * 1.0f/5.0f ;
  //printf("clamped latIdx: %f\n", latIdx);
}

void CBRaycastVehicle::Wheel::updateRotation(btScalar timeStep, btRigidBody *chassisBody)
{
  assert(currentBrakeTorque >= 0);
  btScalar longitudinal_stick_threshold = 0.5;
  btScalar wheel_rotation_stick_threshold = 0.7;
  //btScalar longitudinal_stick_threshold = 1.1;
  //btScalar wheel_rotation_stick_threshold = 1.4;
  // If wheel is NOT in contact with ground, the suspension force is 0
  btScalar friction_torque = longIdx * currentSuspensionForce * d.radius;
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
  if (airborne)
    {
      currentAccelerationForce = 0.0;
      return;
    }

  btScalar force = 0.0;
  if (currentLinearVelocity.length() < longitudinal_stick_threshold
      and not (fabs(currentAngularSpeed) < wheel_rotation_stick_threshold)
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

btVector3 CBRaycastVehicle::Wheel::sumForces()
{
#warning Redo the coordinates correctly
  // TODO: The suspension force should be projected on the ground normal,
  //       otherwise a vehicle that is not perfectly balanced will get a
  //       net force in the "leaning" direction, probably forwards or backwards.
  return btVector3(latIdx * currentSuspensionForce, currentSuspensionForce, currentAccelerationForce);
}

void CBRaycastVehicle::Wheel::updateMotionState()
{
  if (not motionState)
    return;
  //currentTransform.setIdentity();
  currentTransform.setOrigin(btVector3(d.relPos + (d.direction * currentSuspensionLength)));
  currentTransform.setRotation(btQuaternion(d.direction, currentSteerAngle)
  			       * btQuaternion(d.axle, currentAngle));
  motionState->setWorldTransform(currentTransform);
}

const btVector3 &CBRaycastVehicle::Wheel::getContactPoint()
{
  assert(not isAirborne());
  return contactPoint;
}

CBRaycastVehicle::CBRaycastVehicle(btRigidBody* chassis)
{
	chassisBody = chassis;
	indexRightAxis = 0;
	indexUpAxis = 2;
	indexForwardAxis = 1;
}

void CBRaycastVehicle::setCoordinateSystem(int rightIndex,int upIndex,int forwardIndex)
{
  indexRightAxis = rightIndex;
  indexUpAxis = upIndex;
  indexForwardAxis = forwardIndex;
}

btWheelInfo& CBRaycastVehicle::getWheelInfo(int index)
{
  btAssert((index >= 0) && (index < getNumWheels()));
  return m_wheelInfo[index];
}

void CBRaycastVehicle::setDriveTorques(const std::vector<btScalar> &torques)
{
  for (unsigned int i = 0; i < torques.size(); i++)
    if (i < wheels.size())
      wheels[i]->setTorque(torques[i]);
}

Actuator *CBRaycastVehicle::getActuator(const std::string &name)
{
  auto a = actuators.find(name);
  return a != actuators.end() ? a->second : nullptr;
}


void CBRaycastVehicle::setAccelerationTorque(unsigned short wheel_index,
					     btScalar acceleration_torque)
{
  if (accelerationTorque.size() <= wheel_index)
    return;
  accelerationTorque[wheel_index] = acceleration_torque;
}

CBRaycastVehicle::Wheel *CBRaycastVehicle::addWheel(const SuspensionWheelData &data)
{
  wheels.push_back(new Wheel(data));
  return wheels.back();
}

void CBRaycastVehicle::updateWheelTransformsWS(btWheelInfo& wheel, bool interpolatedTransform)
{
  wheel.m_raycastInfo.m_isInContact = false;

  btTransform chassisTrans = getChassisWorldTransform();
  if (interpolatedTransform && (chassisBody->getMotionState()))
    chassisBody->getMotionState()->getWorldTransform(chassisTrans);

  wheel.m_raycastInfo.m_hardPointWS = chassisTrans( wheel.m_chassisConnectionPointCS );
  wheel.m_raycastInfo.m_wheelDirectionWS = chassisTrans.getBasis() *  wheel.m_wheelDirectionCS ;
  wheel.m_raycastInfo.m_wheelAxleWS = chassisTrans.getBasis() * wheel.m_wheelAxleCS;
}

/*
 * Calculate the longitudinal impulse that makes sure that the
 * tire does not move forward or backwards at all.
 */
btScalar CBRaycastVehicle::Wheel::longitudinalEquilibrium(btScalar timeStep, btRigidBody *chassisBody)
{
  if (timeStep == btScalar(0.0f) or airborne)
    return btScalar(0.0f);

  btScalar longImpulse;
  const btTransform &bt = chassisBody->getCenterOfMassTransform();
  const btMatrix3x3 &br = bt.getBasis();
  btVector3 forward(btMatrix3x3(btQuaternion(d.direction, HeloUtils::PI_2)) * currentAxle);

  resolveSingleBilateral(*chassisBody,
  			 bt * contactPoint,
  			 *currentGround,
  			 bt * contactPoint,
  			 btScalar(0.0f),
  			 br * forward,
  			 longImpulse,
  			 timeStep);
  return longImpulse / timeStep;
}



/*
 * Calculate the lateral friction index that makes sure that the tire
 * don't slip sideways at all.
 */
btScalar CBRaycastVehicle::Wheel::lateralEquilibrium(btScalar timeStep, btRigidBody *chassisBody)
{
  if (timeStep == btScalar(0.0f) or airborne)
    return btScalar(0.0f);

  btScalar sideImpulse;
  const btTransform &bt = chassisBody->getCenterOfMassTransform();
  const btMatrix3x3 &br = bt.getBasis();
  resolveSingleBilateral(*chassisBody,
  			 bt * contactPoint,
  			 *currentGround,
  			 bt * contactPoint,
  			 btScalar(0.0f),
  			 br * currentAxle,
  			 sideImpulse,
  			 timeStep);
  assert(not std::isnan(sideImpulse));
  assert(not std::isinf(sideImpulse));
  return sideImpulse / timeStep / currentSuspensionForce;
}

btScalar CBRaycastVehicle::getWheelRotationSpeed(unsigned short wheelIndex)
{
  return m_wheelInfo[wheelIndex].m_deltaRotation;
}


// void CBRaycastVehicle::WheelState::calculate(btWheelInfo &wi, int wheel,
// 					     CBRaycastVehicle &vehicle)
// {
//   if (not wi.m_raycastInfo.m_isInContact)
//     return; // Note: this makes all of 'this' invalid
//   const btTransform &wt = vehicle.getWheelTransformWS(wheel);
//   btMatrix3x3 base_vectors = wt.getBasis();
//   // which index to choose for right is freakin' magic
//   right[0] = -base_vectors[0][1];
//   right[1] = -base_vectors[1][1];
//   right[2] = -base_vectors[2][1];

//   forward = wi.m_raycastInfo.m_contactNormalWS.cross(right);
//   forward.normalize();

//   // TODO: Is the wheel's position really at the contact point? Tire radius?
//   relativePosition = (wi.m_raycastInfo.m_contactPointWS -
// 		       vehicle.getRigidBody()->getCenterOfMassPosition());

//   velocity = vehicle.getRigidBody()->getVelocityInLocalPoint(relativePosition);
//   valid = true;
// }

void CBRaycastVehicle::updateAction(btCollisionWorld* collisionWorld, btScalar timeStep)
{
  for (auto a: actuators)
    a.second->actuate(timeStep);

  std::vector<btVector3> forces(wheels.size());

  const btTransform &bt = chassisBody->getCenterOfMassTransform();
  const btMatrix3x3 &br = bt.getBasis();
  for (unsigned int i = 0; i < wheels.size(); ++i)
    {
      wheels[i]->updateContact(collisionWorld, chassisBody);
      wheels[i]->updateSuspension();
      wheels[i]->updateFriction(timeStep, chassisBody);
      wheels[i]->updateRotation(timeStep, chassisBody);
      btVector3 f = wheels[i]->sumForces();
      forces[i] = f;
    }

  for (unsigned int i = 0; i < wheels.size(); ++i)
    {
      btVector3 f = forces[i];
      if (f.length()) // TODO: better to check if wheel is airborne
	{
	  assert(not std::isnan(f.x()));
	  assert(not std::isnan(f.y()));
	  assert(not std::isnan(f.z()));

	  btVector3 p = wheels[i]->getContactPoint();
	  // Note the relative position in WORLD COORDINATES as the second argument
	  // That is why we multiply by the rotation matrix and not the full transform
	  chassisBody->applyImpulse(br * (f * timeStep), br *p);
	}
      wheels[i]->updateMotionState();
    }
}

Ogre::SceneNode *Car::createWheel(const std::string &prefix,
                                  const SuspensionWheelData &wd,
                                  const btVector3 &globalTranslation,
                                  const btVector3 &globalRotation,
                                  Ogre::SceneManager *mgr,
                                  Ogre::SceneNode *parent,
                                  RayCaster &rayCaster)
{
  Ogre::Entity *tent = mgr->createEntity(prefix + wd.name  + "_ent", wd.meshname);
  Ogre::SceneNode *tnode = parent->createChildSceneNode(prefix + wd.name + "_node");
  tnode->attachObject(tent);
  btVector3 wheelpos(wd.relPos + (wd.direction * wd.suspensionLength));
  tnode->setPosition(Ogre::Vector3(wheelpos.x(), wheelpos.y(), wheelpos.z()));
  wheelNodes.insert(std::make_pair(rayCaster.addWheel(wd), tnode));
  return tnode;
}

CBRaycastVehicle *Car::createRaycastVehicle(btRigidBody *b)
{
 return new CBRaycastVehicle(b);
}

btTypedConstraint *Car::createConstraint(const std::string &prefix,
                                         Car::Constraint *c,
                                         btRigidBody *parent,
                                         btRigidBody *child)
{

  btVector3 pt(HeloUtils::Ogre2BulletVector(c->relativePositionParent));
  btVector3 ct(HeloUtils::Ogre2BulletVector(c->relativePositionChild));

  Car::HingeConstraint *hc;
  if ((hc = dynamic_cast<Car::HingeConstraint*>(c)) != nullptr)
    {
      auto btc = new btHingeConstraint(*parent, // A
                                       *child,  // B
                                       pt, // pivotInA
                                       ct, // pivotInB,
                                       HeloUtils::Ogre2BulletVector(hc->parentAxis),    // axisInA,
                                       HeloUtils::Ogre2BulletVector(hc->childAxis),    // axisInB,
                                       true); // useReferenceFrameA (parent)
      if (hc->upperLimit != 0.0 or hc->lowerLimit != 0.0)
        btc->setLimit(HeloUtils::Deg2Rad(hc->lowerLimit),
                      HeloUtils::Deg2Rad(hc->upperLimit));
      if (hc->motorMaxImpulse)
        {
          auto act = new HingeActuator(nullptr, btc);
          rayCasters.back()->addActuator(prefix.substr(prefix.find('.') + 1) + c->name, act);
          btc->setMaxMotorImpulse(hc->motorMaxImpulse);
          btc->setMotorTarget(hc->motorInitialTarget, hc->motorDT);
          btc->enableMotor(true);
        }
      return btc;
    }
  else
    {
      btTransform pTrans(btMatrix3x3(1.0, 0.0, 0.0,
                                      0.0, 1.0, 0.0,
                                      0.0, 0.0, 1.0),
                         pt);
      btTransform cTrans(btMatrix3x3(1.0, 0.0, 0.0,
                                     0.0, 1.0, 0.0,
                                     0.0, 0.0, 1.0),
                         ct);
      return new btFixedConstraint(*parent, *child, pTrans, cTrans);
    }
}

Ogre::SceneNode *Car::createBodiesAndWheels(const std::string &prefix,
                                            const BodyData &data,
                                            const btVector3 &globalTranslation,
                                            const btVector3 &globalRotation,
                                            Ogre::SceneManager *mgr,
                                            Ogre::SceneNode *parent,
                                            btRigidBody *bParent,
                                            RayCaster *rayCaster)
{
  if (not parent)
    parent = mgr->getRootSceneNode();

  const std::string thisName(prefix + data.name);

  Ogre::Entity *ent = mgr->createEntity(thisName + "_ent", data.meshname);

  Ogre::SceneNode *n = mgr->getRootSceneNode()->createChildSceneNode(thisName + "_node");
  n->attachObject(ent);

  btCollisionShape* chassis_shape = new btBoxShape(btVector3(data.size.x / 2.0,
         						     data.size.y / 2.0,
                                                             data.size.z / 2.0));

  btCompoundShape *comp = new btCompoundShape();

  // Transform of hull collision shape
  btTransform chassis_shape_trans;
  chassis_shape_trans.setIdentity();
  chassis_shape_trans.setOrigin(HeloUtils::Ogre2BulletVector(data.colShapeOffset));
  comp->addChildShape(chassis_shape_trans, chassis_shape);
  shape = comp;

  btVector3 localTranslation(globalTranslation + HeloUtils::Ogre2BulletVector(data.relativePosition));
  // create fuselage rigid body
  btTransform tr;
  tr.setIdentity();
  tr.setOrigin(localTranslation);
  tr.getBasis().setEulerZYX(globalRotation.x(), globalRotation.y(), globalRotation.z());

  auto b = Physics::CreateRigidBody(data.mass, tr, shape, n);
  bodies.push_back(b);

  if (data.isRaycaster)
    {
      rayCaster = createRaycastVehicle(b);
      rayCasters.push_back(rayCaster);
    }

  if (bParent and data.constraint)
    {
      auto c = createConstraint(thisName + ".",
                                data.constraint,
                                bParent,
                                b);
      delete data.constraint;
      constraints.push_back(c);
      pivot = static_cast<btHingeConstraint*>(c);
    }

  for (auto wd : data.suspensionWheels)
    createWheel(thisName + ".",
                wd,
                globalTranslation,
                globalRotation,
                mgr,
                n,
                *rayCaster);

  for (auto wd : data.wheels)
    createSpinWheel(thisName + ".",
                    static_cast<const WheelData &>(wd),
                    globalTranslation,
                    globalRotation,
                    mgr,
                    n,
                    *rayCaster);

  for (auto wd : data.driveWheels)
    this->createDriveWheel(thisName + ".",
                           static_cast<const DriveWheelData &>(wd),
                           globalTranslation,
                           globalRotation,
                           mgr,
                           n,
                           *rayCaster);

  for (auto a : data.actuators)
    {
      if (a.type == "wheeltorque")
        {
          auto wta = new WheelTorqueActuator(rayCaster->getWheels());
          rayCaster->addActuator(thisName.substr(thisName.find('.') + 1) + "." + a.name, wta);
        }
    }


  for (auto cd : data.children)
    createBodiesAndWheels(prefix + data.name + ".",
                          cd,
                          localTranslation,
                          globalRotation,
                          mgr,
                          n,
                          b,
                          rayCaster);

  return n;
}

Car::Car() {}

Car *Car::load(const CarData &data, Ogre::Root *root)
{
  Ogre::SceneManager *mgr = root->getSceneManager("SceneManager");
  for (auto b : data.bodies)
    node = createBodiesAndWheels(data.name + ".",
                                 b,
                                 HeloUtils::Ogre2BulletVector(data.position),
                                 HeloUtils::Ogre2BulletVector(data.rotation),
                                 mgr);

  /*init driveline variables*/
  // engine = new Engine(TORQUE_WHEEL_THICKNESS
  // 		      * TORQUE_WHEEL_DENSITY
  // 		      * pow(TORQUE_WHEEL_RADIUS, 4)/ 20.0);
  // engine->addTorqueDataPoint(300, 70, 30);
  // engine->addTorqueDataPoint(600, 100, 0);
  // engine->addTorqueDataPoint(1000, 400, -30);
  // engine->addTorqueDataPoint(1500, 450, -50);
  // engine->addTorqueDataPoint(2000, 500, -75);
  // engine->addTorqueDataPoint(3500, 525, -130);
  // engine->addTorqueDataPoint(5500, 425, -175);
  // engine->addTorqueDataPoint(9500, 100, -250);
  // engine->addTorqueDataPoint(1300, 30, -350);
  // engine->addTorqueDataPoint(MOTOR_RPM_MAX, 0, -450);

  // clutchObj = new Clutch(engine);
  // gearbox = new Gearbox(clutchObj);
  // gearbox->addGear(3.584);
  // gearbox->addGear(2.022);
  // gearbox->addGear(1.384);
  // gearbox->addGear(1.0);
  // gearbox->addGear(0.861);

  // differential = new Differential(gearbox, DIFFERENTIAL_RATIO * 2.0);
  // engine->setRPM(500);

  //clutch_force = CLUTCH_MAX_FORCE;
  //current_gear=0;
  //motor_rpm=1;

  //axis_rpm = 0.0;
  //wheel_angle = 0.0;
  //max_wheel_angle = M_PI_4;

#ifdef AUTO_CLUTCH
  //clutch_counter=AUTO_CLUTCH_TIME;
#endif

  // float long_scale = 0.00;
  // float lat_scale = 0.00;

  // longMu.addDataPoint(0.0, 0.0);
  // longMu.addDataPoint(0.15, 0.9 * long_scale);
  // longMu.addDataPoint(3.0, 0.0);

  // latMu.addDataPoint( 0.0, 0.0);
  // latMu.addDataPoint(10.0, 0.7 * lat_scale);
  // latMu.addDataPoint(15.0, 0.9 * lat_scale);
  // latMu.addDataPoint(20.0, 1.0 * lat_scale);
  // latMu.addDataPoint(25.0, 0.9 * lat_scale);
  // latMu.addDataPoint(35.0, 0.8 * lat_scale);
  // latMu.addDataPoint(90.0, 0.6 * lat_scale);
  return this;
}

void Car::finishPhysicsConfiguration(Physics *phys)
{
  for (auto b : bodies)
    phys->addBody(b);

  for (auto c : constraints)
    phys->addConstraint(c, true);

  for (auto rc : rayCasters)
    phys->addAction(dynamic_cast<btActionInterface*>(rc));


  for (auto wn : wheelNodes)
    {
      auto wheel = wn.first;
      auto node = wn.second;
      btTransform wheel_trans(wheel->getTransform());
      HeloMotionState *ms = new HeloMotionState(wheel_trans, node);
      wheel->setMotionState(ms);
      phys->addMotionState(ms);
    }

  //const Ogre::Vector3 position; // TODO: make this the real position
  //wheel_distance_x = 3.0; // length
  //wheel_distance_y = 2.6; // width
  //wheel_distance_z = -0.2;
  //hull_offset_z = 1.5;

  //float tire_width = 0.6;


  /* Bullet stuff*/

  /*
   * The wheel collision shape is needed to keep tires from falling
   * through the ground if the car tips over
   */

  /* TODO: free these */
  // btCollisionShape* chassisShape = new btBoxShape(btVector3(CAR_LENGTH,
  // 							    CAR_WIDTH,
  // 							    1.5f));
  // btCompoundShape* compound = new btCompoundShape();

  // This shape is used as flip-over protection and collision shapes
  // for the wheels when the car rolls
  // btCollisionShape* wheelsShape = new btBoxShape(btVector3(wheel_distance_x
  // 							   + (2 * TIRE_RADIUS),
  // 							   wheel_distance_y
  // 							   + tire_width,
  // 							   0.1f));


  // Transform of hull collision shape
  // btTransform flip_prot_trans;
  // flip_prot_trans.setIdentity();
  // flip_prot_trans.setOrigin(btVector3(0.0, 0.0, wheel_distance_z));
  // compound->addChildShape(flip_prot_trans, wheelsShape);

  // btTransform hull_trans;
  // hull_trans.setIdentity();
  // hull_trans.setOrigin(btVector3(0.0, 0.0, hull_offset_z + wheel_distance_z));
  // compound->addChildShape(hull_trans, chassisShape);


  // btTransform car_position;
  // car_position.setIdentity();
  // car_position.setOrigin(btVector3(position.x, position.y, position.z));
  //  btRigidBody *car_chassis = Physics::CreateRigidBody(CAR_MASS, car_position, compound, NULL);

  //rayCastVehicle = new CBRaycastVehicle(car_chassis, rayCaster);

  //car_chassis->setActivationState(DISABLE_DEACTIVATION);
  //rayCastVehicle->setCoordinateSystem(1, 2, 0);   // Right handed coordinate system with z up and y forward
  //phys->getWorld()->addVehicle(rayCastVehicle);

  /* Wheels */
  // btVector3 wheelDirection(0.0, 0.0, -1.0);
  // btVector3 wheelAxle(0.0, 1.0, 0.0);
  // float	suspensionStiffness = 7.0f;
  // float	suspensionDamping = 1.0f;
  // float	suspensionCompression = 1.0f;
  // float	rollInfluence = 0.1f;            // TODO: Suspicious, is this used?
  // float	wheelFriction = 1000;            // TODO: Suspicious, is this used?
  // float suspension_length = 0.9;
  //IMesh *simple_tire_mesh = loadMesh("TireSimple.tmsh");

//   for (int i = 0; i < 4; ++i)
//     {
//       btScalar wheelOffsetX(wheel_distance_x);
//       btScalar wheelOffsetY(wheel_distance_y);
//       btScalar wheelOffsetZ(wheel_distance_z);
//       bool isFrontWheel = true;
//       switch(i)
// 	{
// 	case Car::FRONT_RIGHT:
// 	  wheelOffsetY *= -1.0;
// 	  break;
// 	case Car::FRONT_LEFT:
// 	  break;
// 	case Car::BACK_RIGHT:
// 	  wheelOffsetX *= -1.0;
// 	  wheelOffsetY *= -1.0;
// 	  isFrontWheel = false;
// 	  break;
// 	case Car::BACK_LEFT:
// 	  wheelOffsetX *= -1.0;
// 	  isFrontWheel = false;
// 	  break;
// 	}

//       btVector3 connectionPoint(wheelOffsetX, wheelOffsetY, wheelOffsetZ);
// #warning compose a valid WheelData
//       CBRaycastVehicle::WheelData wd;
//       rayCastVehicle->addWheel(wd);

//       btWheelInfo& wheel = rayCastVehicle->getWheelInfo(i);
//       wheel.m_suspensionStiffness = suspensionStiffness;
//       wheel.m_wheelsDampingRelaxation = suspensionDamping;
//       wheel.m_wheelsDampingCompression = suspensionCompression;
//       wheel.m_frictionSlip = wheelFriction;
//       wheel.m_rollInfluence = rollInfluence;
//     }

  /*
   * A note on suspension:
   * suspensionRestLenght above is the suspension length at the
   * equilibrium point. Same as m_suspensionRestLength1 in
   * btWheelInfo struct.
   *
   * m_suspensionStiffness is the spring constant.
   *
   * m_maxSuspensionTravelCm is how much the suspension can move
   * in CENTIMETERS, totally fucking not obvious.
   *
   * m_clippedInvContactDotSuspension is the inverse of the
   * wheel "down vector" projected on the ground normal,
   * clamped to < 10.0.
   * It is multiplied in with the calculation to determine
   * the force of the suspension.
   * ?????? Probably read only...
   *
   * m_suspensionRelativeVelocity: vehicle velocity in the
   * suspension direction at the wheel contact point.
   *
   * m_wheelsDampingCompression: the suspentions damping
   * factor when the the suspension is pushed together
   *
   * m_wheelsDampingRelaxation:  the suspentions damping
   * factor when the the suspension is pulled apart
   *
   * The resulting force is multiplied with the mass
   * of the vehicle.
   */
}

CBRaycastVehicle::Wheel *CBRaycastVehicle::getWheel(unsigned int i)
{
  assert(i < wheels.size());
  return wheels[i];
}


Ogre::Vector3 Car::getPosition()
{
  btRigidBody *b = rayCasters[0]->getRigidBody();
  btVector3 pos(b->getCenterOfMassPosition());
  return Ogre::Vector3(pos[0], pos[1], pos[2]);
}

Ogre::Vector3 Car::getVelocity()
{
  btRigidBody *b = rayCasters[0]->getRigidBody();
  btVector3 vel(b->getLinearVelocity());
  return Ogre::Vector3(vel[0], vel[1], vel[2]);
}

void Car::setSteer(Ogre::Real radians_right)
{
  // for (auto rc : rayCasters)
  //   rc->setSteer(btScalar(radians_right));
  // pivot->setMotorTarget(-radians_right / 2.0, 0.1);
}

void Car::setThrottle(Ogre::Real fraction)
{
  // for (auto rc : rayCasters)
  //   {
  //     std::vector<btScalar> torques;
  //     for (unsigned int i = 0; i < rc->getNumWheels(); ++i)
  //       torques.push_back(fraction * 1000);
  //     rc->setDriveTorques(torques);
  //   }
}

void Car::update(void)
{
}


void Car::setInput(control_data_t &cdata)
{
}

static float long_mu_max = 1.0;
static float lat_mu_max = 1.0;

static void ClampToFrictionEllipse(float &long_mu, float &lat_mu)
{
  if (not long_mu)
    {
      lat_mu = std::min(1.0f, lat_mu);
      return;
    }
  if (not lat_mu)
    {
      long_mu = std::min(1.0f, long_mu);
      return;
    }

  float y_inv = sqrt((lat_mu / (long_mu * lat_mu_max)) + (1.0 / lat_mu_max));
  if ((1.0 / y_inv) < long_mu_max)
    return;

  long_mu = 1.0 / y_inv;
  lat_mu = lat_mu / (long_mu * y_inv);
  return;
}

static void GetMus(float ratio, float angle, float &long_mu, float &lat_mu)
{
  long_mu = longMu[ratio];
  lat_mu = latMu[angle];
  return;
}

car_data_t Car::getCarData()
{
  car_data_t info;
  // info.wheel_angle = wheel_angle;
  // info.gear = gearbox->getCurrentGear();
  // info.rpm = engine->getRPM();
  // info.velocity = getVelocity();
  //Info.car_angle = car_angle;
  //info.pos = pos;
  return info;
}

float Car::getRPM()
{
  //return motor_rpm;
  return 0;
}
float Car::getSpeed()
{
  return static_cast<float>(bodies[0]->getLinearVelocity().length());
}

::Actuator *Car::getActuator(const std::string &id)
{

  for (auto rc : rayCasters)
    {
      auto a = rc->getActuator(id);
      if (a)
        return a;
    }
  return nullptr;
}

Controller *Car::createController(OIS::Object *dev)
{
  if (dynamic_cast<OIS::JoyStick*>(dev))
    return controller = new CarJoystickController(*static_cast<OIS::JoyStick*>(dev), *this);
  else if (dynamic_cast<OIS::Mouse*>(dev))
    return NULL;
  else if (dynamic_cast<OIS::Keyboard*>(dev))
    {
      controller = new CarKeyController(*static_cast<OIS::Keyboard*>(dev), *this);
      return controller;
    }
  else
    return NULL;
}

bool CarKeyController::keyPressed(const OIS::KeyEvent& e)
{
  return true;
}

bool CarKeyController::keyReleased(const OIS::KeyEvent& e)
{
  return true;
}

void CarKeyController::update(float timeDelta)
{
  if (not active)
    return;

  if (keyboard.isKeyDown(OIS::KC_A))
    //car.setSteer(Ogre::Real(-HeloUtils::PI_4));
    steer.setValue(-1.0);
  else if (keyboard.isKeyDown(OIS::KC_D))
    //car.setSteer(Ogre::Real(HeloUtils::PI_4));
    steer.setValue(1.0);
  else
    //car.setSteer(Ogre::Real(0.0));
    steer.setValue(0.0);

  if (keyboard.isKeyDown(OIS::KC_W))
    //car.setThrottle(HeloUtils::Fraction(1, 1));
    accel.setValue(1.0);
  else
    //car.setThrottle(HeloUtils::Fraction(0, 1));
    accel.setValue(0.0);

  // if (mKeyboard->isKeyDown(OIS::KC_S))
  //   car->setThrottle(Ogre::Fraction(0, 1));

}

bool CarJoystickController::axisMoved(const OIS::JoyStickEvent &e, int)
{
  if (not active)
    return true;

   car.setSteer(e.state.mAxes[3].abs / static_cast<float>(OIS::JoyStick::MAX_AXIS) * 0.5);
   car.setThrottle(-e.state.mAxes[1].abs / static_cast<float>(OIS::JoyStick::MAX_AXIS));
   return true;
}

void CarJoystickController::setActive(bool a)
{
  Controller::setActive(a);
  joystick.setEventCallback(a ? this : NULL);
}
