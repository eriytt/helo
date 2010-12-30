#ifndef TANK_H
#define TANK_H

#include "Car.h"

class RaycastTank : public CBRaycastVehicle
{
public:
  typedef struct
  {
    float radius;
    float currentAngularSpeed;
    float momentOfInertia;
    float currentBrakeTorque;
    float currentDriveTorque;
    float currentAngle;
    btVector3 currentLinearVelocity;
    btScalar currentAccelerationForce;
  } FakeWheel;

  FakeWheel rightDriveWheel;
  FakeWheel leftDriveWheel;

  btScalar currentSteerAngle;
  btScalar currentDriveTorque;
  btScalar steerSensitivity; // Hmm, perhaps reuse any of the wheel's sensitivity...

  RaycastTank(btRigidBody *chassis);
  virtual void updateAction(btCollisionWorld* collisionWorld, btScalar timeStep);
  void updateTread(bool right, btScalar suspensionForce, btScalar timestep);
  void setDriveTorques(const std::vector<btScalar> &torques);
  void setSteer(btScalar radians_right);
};


class Tank : public Car
{
public:
  typedef Car::CarData TankData;

public:
  Tank(const TankData &data, Ogre::Root *root);
};

#endif // TANK_H
