#ifndef CAR_H
#define CAR_H


#include <math.h>
#include <vector>
#include <OGRE/Ogre.h>
#include "BulletDynamics/Vehicle/btRaycastVehicle.h"

#include "Physics.h"

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

#define FORCE_AVERAGE_COUNT 2 // Over how many iterations will forces
			      // affecting the vehicle be averaged. 1
			      // means no averaging effect at
			      // all. This reduces jitter due to
			      // numeric instability significantly
class CBRaycastVehicle : public btActionInterface
{
public:
  // Structure with enough information to attach a wheel to the car
  typedef struct
  {
    btVector3 relPos;
    btScalar suspensionLength;
    btScalar maxLengthUp;
    btScalar maxLengthDown;
    btVector3 direction;
    btVector3 axle;
    btScalar radius;
    btScalar spring;
    btScalar dampUp;
    btScalar dampDown;
    btScalar steerCoeff;
    btScalar driveCoeff;
    btScalar brakeCoeff;
    btScalar momentOfInertia;
  } WheelData;

protected:
  class Wheel
  {
  protected:
    WheelData d;

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
    Wheel(const WheelData &data);
    void updateContact(btCollisionWorld *world, btRigidBody *chassisBody);
    void updateSuspension();
    void updateFriction(btScalar timeStep, btRigidBody *chassisBody);
    void updateRotation(btScalar timeStep, btRigidBody *chassisBody);
    btScalar lateralEquilibrium(btScalar timeStep, btRigidBody *chassisBody);
    btScalar longitudinalEquilibrium(btScalar timeStep,btRigidBody *chassisBody);
    btVector3 sumForces();
    void setSteer(btScalar right_radians);
    void setTorque(btScalar torque);
    const btVector3 &getContactPoint();
    const btTransform &getTransform() const;
    void setMotionState(btMotionState *ms) {motionState = ms;}
  };


protected:
  btRigidBody *chassisBody;
  std::vector<Wheel> wheels;

  int indexRightAxis;
  int indexUpAxis;
  int indexForwardAxis;
  btAlignedObjectArray<btWheelInfo> m_wheelInfo;

  MovingAverageVector<FORCE_AVERAGE_COUNT> wheelForce[4];
  std::vector<btScalar> accelerationTorque;
  btScalar brakingTorque;

public:
  CBRaycastVehicle(btRigidBody* chassis);

  void addWheel(const WheelData &data);
  void setAccelerationTorque(unsigned short wheel_index, btScalar acceleration_torque);
  void setBrakeTorque(btScalar brake_torque) {brakingTorque = brake_torque;}
  btScalar getWheelRotationSpeed(unsigned short wheelIndex);
  const btTransform& getWheelTransformWS(int wheelIndex) const;
  btRigidBody* getRigidBody() {return chassisBody;}
  void updateAction(btCollisionWorld* collisionWorld, btScalar step);
  void debugDraw(btIDebugDraw* debugDrawer) {}
  void setCoordinateSystem(int rightIndex,int upIndex,int forwardIndex);
  btWheelInfo& getWheelInfo(int index);
  const btTransform& getChassisWorldTransform() const {return chassisBody->getCenterOfMassTransform();}
  void setSteer(btScalar radians_right);
  void setDriveTorques(const std::vector<btScalar> &torques);
  Wheel &getWheel(unsigned int);

protected:
  int getNumWheels() const { return int (m_wheelInfo.size());}
  void updateWheelTransformsWS(btWheelInfo& wheel, bool interpolatedTransform);
};

class Car : public PhysicsObject
{
public:
  typedef CBRaycastVehicle::WheelData WheelData;

  typedef struct
  {
    Ogre::String name;
    Ogre::String meshname;
    Ogre::Vector3 position;
    Ogre::Vector3 size;
    Ogre::Real weight;
    std::vector<WheelData> wheelData;
  } CarData;

 protected:
  CBRaycastVehicle *rayCastVehicle;

  // Ogre
  Ogre::SceneNode *node;

  // Bullet
  btCollisionShape *shape;
  btRigidBody *body;
  std::vector<Ogre::SceneNode*> wheelNodes;

  // This is how where the tires are attached relative to the rigid
  // body

  /***********Drivetrain variables**************/
  // int clutch_force;
  // int current_gear;
  // float motor_rpm;
  // float speed;
public:
  // float axis_rpm;

protected:

  // class Engine *engine;
  // class Clutch *clutchObj;
  // class Gearbox *gearbox;
  // class Differential *differential;
  /**********************/

public:
  // typedef enum {
  //   FRONT_RIGHT,
  //   FRONT_LEFT,
  //   BACK_RIGHT,
  //   BACK_LEFT
  // } wheelPosition_t;

 public:
  Car(const CarData &data, Ogre::Root *root);
  virtual void finishPhysicsConfiguration(Physics *phys);
  virtual Ogre::SceneNode *getSceneNode() {return node;}
  virtual void setSteer(Ogre::Real radians_right);
  virtual void setThrottle(Ogre::Real fraction);
  virtual Ogre::Vector3 getPosition();
  virtual Ogre::Vector3 getVelocity();
  virtual float getRPM();
  virtual float getSpeed();
  virtual car_data_t getCarData();
  virtual void setInput(control_data_t &cdata);
  virtual void update(void);
  virtual void physicsUpdate() {}

protected:
  void SetAccel(int direction);

  /*Driveline functions*/
  //virtual int GetTorqueWheelEnergy(int rpm);
  //virtual int GetTorqueWheelRPM(int energy);
  //virtual float GetMotorTorque(float rpm, int throttle);
  //virtual float CalculateDriveForce(void);
  //virtual void GetWheelFriction(void);
  //virtual void DetermineCalculationModel(void);
};

#endif /*CAR_H*/
