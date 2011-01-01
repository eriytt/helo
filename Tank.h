#ifndef TANK_H
#define TANK_H

#include "Car.h"

class RaycastTank : public CBRaycastVehicle
{
public:
  class DriveWheelData : public CBRaycastVehicle::WheelData
  {
  public:
    btVector3 realRelPos;
  };

  typedef CBRaycastVehicle::WheelData SuspensionWheelData;

  class SpinWheelData
  {
  public:
    btVector3 relPos;
    btVector3 axle;
    btScalar radius;
  };

protected:
  class DriveWheel : public CBRaycastVehicle::Wheel
  {
  protected:
    btVector3 realRelPos;

  public:
    DriveWheel(const DriveWheelData &data);
    void updateTread(btScalar timeStep, btRigidBody *chassisBody);
    void updateMotionState();
    void setSuspensionForce(btScalar force) {currentSuspensionForce = force;}
    btVector3 sumForces() {return currentForward * currentAccelerationForce;};
    void setAirborne(bool isAirborne) {airborne = isAirborne;}
  };

  class SuspensionWheel : public CBRaycastVehicle::Wheel
  {
  public:
    SuspensionWheel(const SuspensionWheelData &data) : CBRaycastVehicle::Wheel(static_cast<const WheelData &>(data)) {}
    void updateFriction(btScalar timeStep, btRigidBody *chassisBody);
  };

  class SpinWheel
  {
  protected:
    SpinWheelData d;
    btScalar currentAngle;
    btMotionState *motionState;

  public:
    SpinWheel(const SpinWheelData &data): d(data), currentAngle(0.0), motionState(NULL) {}
    const btTransform getTransform() {return btTransform(btTransform(btQuaternion(d.axle, currentAngle), d.relPos));}
    void addRotation(btScalar rotation) {currentAngle += (rotation / d.radius);}
    void setMotionState(btMotionState *ms) {motionState = ms;}
    void updateMotionState();
  };

  protected:
  std::vector<DriveWheel*> driveWheels;
  std::vector<SpinWheel*> spinWheels;

  btScalar currentSteerAngle;
  btScalar currentDriveTorque;
  btScalar steerSensitivity; // Hmm, perhaps reuse any of the wheel's sensitivity...

public:
  RaycastTank(btRigidBody *chassis);
  virtual void updateAction(btCollisionWorld* collisionWorld, btScalar timeStep);
  void setDriveTorques(const std::vector<btScalar> &torques);
  void setSteer(btScalar radians_right);
  void addWheel(const WheelData &data);
  DriveWheel *getDriveWheel(unsigned int);
  SpinWheel *getSpinWheel(unsigned int);
};


class Tank : public Car
{
public:
  typedef Car::CarData TankData;

protected:
  std::vector<Ogre::SceneNode*> driveWheelNodes;
  std::vector<Ogre::SceneNode*> spinWheelNodes;

public:
  Tank(const TankData &data, Ogre::Root *root);
  void finishPhysicsConfiguration(class Physics *phys);
};

#endif // TANK_H
