#ifndef TANK_H
#define TANK_H

#include "Car.h"

class RaycastTank : public CBRaycastVehicle
{
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
    SuspensionWheel(const SuspensionWheelData &data) : CBRaycastVehicle::Wheel(data) {}
    void updateFriction(btScalar timeStep, btRigidBody *chassisBody);
  };

  class SpinWheel
  {
  protected:
    WheelData d;
    btScalar currentAngle;
    btMotionState *motionState;

  public:
    SpinWheel(const WheelData &data): d(data), currentAngle(0.0), motionState(NULL) {}
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
  Wheel *addWheel(const SuspensionWheelData &data);
  void addDriveWheel(const DriveWheelData &data);
  void addSpinWheel(const WheelData &data);
  DriveWheel *getDriveWheel(unsigned int);
  SpinWheel *getSpinWheel(unsigned int);
};


class Tank : public Car
{
protected:
  virtual CBRaycastVehicle *createRaycastVehicle(btRigidBody *b);

  virtual Ogre::SceneNode *createSpinWheel(const std::string &prefix,
                                           const WheelData &wd,
                                           const btVector3 &globalTranslation,
                                           const btVector3 &globalRotation,
                                           Ogre::SceneManager *mgr,
                                           Ogre::SceneNode *parent,
                                           RayCaster &rayCaster);

  virtual Ogre::SceneNode *createDriveWheel(const std::string &prefix,
                                            const DriveWheelData &wd,
                                            const btVector3 &globalTranslation,
                                            const btVector3 &globalRotation,
                                            Ogre::SceneManager *mgr,
                                            Ogre::SceneNode *parent,
                                            RayCaster &rayCaster);

public:
  class TankData : public Car::CarData
  {
  public:
    Ogre::Vector3 turretPosition;
    Ogre::Vector3 barrelPosition;
    //std::vector<DriveWheelData> driveWheelData;
    //std::vector<WheelData> spinWheelData;
  };

protected:
  std::vector<Ogre::SceneNode*> driveWheelNodes;
  std::vector<Ogre::SceneNode*> spinWheelNodes;

public:
  Tank();
  virtual Tank *load(const TankData &data, Ogre::Root *root);
  void finishPhysicsConfiguration(class Physics *phys);
};

#endif // TANK_H
