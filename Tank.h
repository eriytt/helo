#ifndef TANK_H
#define TANK_H

#include "Car.h"

class RaycastTank : public CBRaycastVehicle
{
public:
  typedef CBRaycastVehicle::WheelData DriveWheelData;

protected:
  class DriveWheel : public CBRaycastVehicle::Wheel
  {
  public:
    DriveWheel(const DriveWheelData &data) : CBRaycastVehicle::Wheel(static_cast<const WheelData &>(data)) {}
    void updateTread(btScalar timeStep, btRigidBody *chassisBody);
    void setSuspensionForce(btScalar force) {currentSuspensionForce = force;}
    btVector3 sumForces() {return currentForward * currentAccelerationForce;};
  };

  std::vector<DriveWheel> driveWheels;

  btScalar currentSteerAngle;
  btScalar currentDriveTorque;
  btScalar steerSensitivity; // Hmm, perhaps reuse any of the wheel's sensitivity...

public:
  RaycastTank(btRigidBody *chassis);
  virtual void updateAction(btCollisionWorld* collisionWorld, btScalar timeStep);
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
