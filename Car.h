#ifndef CAR_H
#define CAR_H


#include <math.h>
#include <vector>
#include <Ogre.h>
#include "BulletDynamics/Vehicle/btRaycastVehicle.h"

#include "Physics.h"
#include "Utils.h"
#include "Controllers.h"
#include "Vehicle.h"

#define BRAKE_MAX 32767
#define CAR_LENGTH 4.923
#define WHEEL_BASE 2.819
#define CAR_WIDTH 1.885
#define GEARS 6
#define CLUTCH_DYNAMIC_FRICTION_INDEX 0.35
#define CLUTCH_STATIC_FRICTION_INDEX 0.5
#define TORQUE_WHEEL_RADIUS 0.2
#define TORQUE_WHEEL_THICKNESS 0.03
#define TORQUE_WHEEL_DENSITY 7800
#define TRANSMISSION_EFFICIENCY 0.7
#define MOTOR_RPM_MAX 20000.0
#define DIFFERENTIAL_RATIO 3.5
#define TIRE_RADIUS 1.6519
#define CAR_MASS 1635.0
#define AUTO_CLUTCH_TIME 3.0
#define BRAKE_MAX_FORCE 10000

typedef struct torque_data{
  int rpm;
  int max_throttle;
  int min_throttle;
} torque_data_t;


typedef struct {
  float wheel_angle;             /* angle of the steering wheel. 0 is straight ahead. unit: radians*/
  int gear;                      /* the current gear.*/
  float rpm;                     /* revolves per minute of the engine*/
  Ogre::Vector3 velocity;        /* a vector describing the car's velocity in global coordinates. unit: m/s*/
  float car_angle;               /* angle of the car body in global coordinates. starts at PI/2. unit: radians*/
  //  point_t pos;               /* the cars position in global coordinates. starts at 0,0. unit:m */
} car_data_t;

typedef struct {
  float throttle_power;
  float steer_angle;
  float clutch;
  float brake;
  int shift_gear;
} control_data_t;



template <int T>
class MovingAverageVector
{
protected:
  std::vector<btVector3> values;

public:
  inline MovingAverageVector()
  {
    for (int i = 0; i < T; ++i)
      values.push_back(btVector3(0.0, 0.0, 0.0));
  }

  inline void addValue(btVector3 v)
  {
    values.erase(values.begin());
    values.push_back(v);
  }

  inline btVector3 getAverage()
  {
    btVector3 res(0.0, 0.0, 0.0);
    for (int i = 0; i < T; ++i)
      {
	res[0] += values[i][0];
	res[1] += values[i][1];
	res[2] += values[i][2];
      }

    res[0] /= T;
    res[1] /= T;
    res[2] /= T;
    return res;
  }
};

/* Note that a positive slip is an accelerating slip */
inline btScalar LongitudinalSlip(btScalar speed, btScalar tire_speed)
{
  btScalar diff = speed - tire_speed;

  if (fabs(diff) < 0.00001f) // not slipping
    {
      //printf("diff so small that we dont regard this as slipping: %f\n", diff);
      return 0.0;
    }

  if (diff >= 0.0) // Use   v - Romega / v equation, v > Romega
    { // Wheel is spinning slower than the ground, reverse the car
      if (speed >= 0.0)
	  // car going forward but wheels spinning backwards
	  // or car is moving forward and the wheel too but not as fast
	return -HeloUtils::unit_clamp(diff / speed);
      else
	// Car is in reveresing and the and the tire reverses faster
	// ie accelerating in reverse
	return HeloUtils::unit_clamp(diff / speed);
    }
  else  // Use   Romega - v / Romega equation, v < Romega
    {   // Wheel is spinning faster than ground, accelerate car
      if (tire_speed >= 0.0)
	// Normal acceleration
	return -HeloUtils::unit_clamp(diff / tire_speed);
      else
	// Sliding in reverse trying to go forward ?
	return HeloUtils::unit_clamp(diff / tire_speed);
    }
}


class CBRaycastVehicle : public btActionInterface
{
public:
  struct WheelData
  {
    std::string name;
    std::string meshname;
    btVector3 relPos;
    btVector3 axle;
    btScalar radius;
  };

  struct SuspensionWheelData : WheelData
  {
    btScalar suspensionLength;
    btScalar maxLengthUp;
    btScalar maxLengthDown;
    btVector3 direction;
    btScalar spring;
    btScalar dampUp;
    btScalar dampDown;
    btScalar steerCoeff;
    btScalar driveCoeff;
    btScalar brakeCoeff;
    btScalar momentOfInertia;
  };

  struct DriveWheelData : SuspensionWheelData
  {
    btVector3 realRelPos;
  };

protected:
  class Wheel
  {
  protected:
    SuspensionWheelData d;

  protected:
    // State variables, updated every time step
    bool airborne;
    btVector3 contactPoint;
    btVector3 contactNormal;
    btScalar currentSuspensionLength;
    btVector3 currentLinearVelocity;
    btScalar currentSuspensionForce;
    btVector3 currentAxle;
    btVector3 currentForward;
    btScalar currentAngularSpeed; // Rotation speed
    btScalar currentAngle;        // Rotaion angle
    btScalar longIdx, latIdx;     // current longitudinal and latitudal friction indices
    btScalar currentAccelerationForce;
    btRigidBody *currentGround;

    // input parameters
    btScalar currentDriveTorque;
    btScalar currentBrakeTorque;
    btScalar currentSteerAngle;
    btTransform currentTransform; // TODO: maybe not needed
    btMotionState *motionState;

  public:
    Wheel(const SuspensionWheelData &data);
    void updateContact(btCollisionWorld *world, btRigidBody *chassisBody);
    void updateSuspension();
    virtual void updateFriction(btScalar timeStep, btRigidBody *chassisBody);
    void updateRotation(btScalar timeStep, btRigidBody *chassisBody);
    btScalar lateralEquilibrium(btScalar timeStep, btRigidBody *chassisBody);
    btScalar longitudinalEquilibrium(btScalar timeStep,btRigidBody *chassisBody);
    virtual btVector3 sumForces();
    virtual void updateMotionState();
    void setSteer(btScalar right_radians);
    void setTorque(btScalar torque);
    const btVector3 &getContactPoint();
    const btTransform &getTransform() const;
    void setMotionState(btMotionState *ms) {motionState = ms;}
    btScalar getAngularSpeed() {return currentAngularSpeed;}
    btScalar getRadius() {return d.radius;}
    bool isAirborne() {return airborne;}
    btScalar getSuspensionForce() {return currentSuspensionForce;}
    void addRotation(btScalar rotationSpeed, btScalar rotation)
    {
      currentAngularSpeed = rotationSpeed / d.radius;
      currentAngle += (rotation / d.radius);
    }
  };


protected:
  btRigidBody *chassisBody;
  std::vector<Wheel*> wheels;

  int indexRightAxis;
  int indexUpAxis;
  int indexForwardAxis;
  btAlignedObjectArray<btWheelInfo> m_wheelInfo;

  std::vector<btScalar> accelerationTorque;
  btScalar brakingTorque;

public:
  CBRaycastVehicle(btRigidBody* chassis);

  virtual void addWheel(const SuspensionWheelData &data);
  virtual void setAccelerationTorque(unsigned short wheel_index, btScalar acceleration_torque);
  virtual void setBrakeTorque(btScalar brake_torque) {brakingTorque = brake_torque;}
  virtual btScalar getWheelRotationSpeed(unsigned short wheelIndex);
  virtual btRigidBody* getRigidBody() {return chassisBody;}
  virtual void updateAction(btCollisionWorld* collisionWorld, btScalar step);
  virtual void debugDraw(btIDebugDraw* debugDrawer) {}
  virtual void setCoordinateSystem(int rightIndex,int upIndex,int forwardIndex);
  virtual btWheelInfo& getWheelInfo(int index);
  virtual const btTransform& getChassisWorldTransform() const {return chassisBody->getCenterOfMassTransform();}
  virtual void setSteer(btScalar radians_right);
  virtual void setDriveTorques(const std::vector<btScalar> &torques);
  virtual Wheel *getWheel(unsigned int);

protected:
  virtual int getNumWheels() const { return int (m_wheelInfo.size());}
  virtual void updateWheelTransformsWS(btWheelInfo& wheel, bool interpolatedTransform);
};

class Car : public PhysicsObject, public Controllable, public HeloUtils::Trackable, public Vehicle
{
public:
  typedef CBRaycastVehicle::WheelData WheelData;
  typedef CBRaycastVehicle::SuspensionWheelData SuspensionWheelData;
  typedef CBRaycastVehicle::DriveWheelData DriveWheelData;

  struct BodyData {
    std::string name;
    std::string meshname;
    float mass;
    Ogre::Vector3 relativePosition;
    Ogre::Vector3 size;
    Ogre::Vector3 colShapeOffset;
    std::vector<BodyData> children;
    std::vector<WheelData> wheels;
    std::vector<SuspensionWheelData> suspensionWheels;
    std::vector<DriveWheelData> driveWheels;
  };

  struct CarData
  {
    Ogre::String name;
    Ogre::Vector3 position;
    Ogre::Vector3 rotation;
    std::vector<BodyData> bodies;
  };

  // typedef struct
  // {
  //   Ogre::String name;
  //   Ogre::String meshname;
  //   Ogre::Vector3 position;
  //   Ogre::Vector3 rotation;
  //   std::vector<Configuration::BodyData> bodies;
  //   Ogre::Vector3 size;
  //   Ogre::Real weight;
  //   std::vector<WheelData> wheelData;
  // } CarData;

 protected:
  CBRaycastVehicle *rayCastVehicle;

  // Ogre
  Ogre::SceneNode *node;

  // Bullet
  btCollisionShape *shape;
  std::vector<btRigidBody*> bodies;
  std::vector<Ogre::SceneNode*> wheelNodes;
  std::vector<btTypedConstraint *> constraints;


protected:
  Ogre::SceneNode *createWheel(const std::string &prefix,
                               const SuspensionWheelData &wd,
                               const btVector3 &globalTranslation,
                               const btVector3 &globalRotation,
                               Ogre::SceneManager *mgr,
                               Ogre::SceneNode *parent);

  virtual CBRaycastVehicle *createRaycastVehicle(btRigidBody *b);

  virtual Ogre::SceneNode *createSpinWheel(const std::string &prefix,
                                           const WheelData &wd,
                                           const btVector3 &globalTranslation,
                                           const btVector3 &globalRotation,
                                           Ogre::SceneManager *mgr,
                                           Ogre::SceneNode *parent) {return nullptr;}

  virtual Ogre::SceneNode *createDriveWheel(const std::string &prefix,
                                            const DriveWheelData &wd,
                                            const btVector3 &globalTranslation,
                                            const btVector3 &globalRotation,
                                            Ogre::SceneManager *mgr,
                                            Ogre::SceneNode *parent) { return nullptr; }

  Ogre::SceneNode *createBodiesAndWheels(const std::string &prefix,
                                         const BodyData &data,
                                         const btVector3 &globalTranslation,
                                         const btVector3 &globalRotation,
                                         Ogre::SceneManager *mgr,
                                         Ogre::SceneNode *parent = nullptr,
                                         btRigidBody *bParent = nullptr);

public:
  Car();
  virtual Car *load(const CarData &data, Ogre::Root *root);
  virtual void finishPhysicsConfiguration(Physics *phys);
  virtual Ogre::SceneNode *getSceneNode() const {return node;}
  virtual void setSteer(Ogre::Real radians_right);
  virtual void setThrottle(Ogre::Real fraction);
  virtual Ogre::Vector3 getPosition();
  virtual Ogre::Vector3 getVelocity();
  virtual float getRPM();
  virtual float getSpeed();
  virtual car_data_t getCarData();
  virtual void setInput(control_data_t &cdata);
  virtual void update(void);
  virtual void physicsUpdate(float step) {}
  Controller *createController(OIS::Object *dev);
};

#endif /*CAR_H*/
