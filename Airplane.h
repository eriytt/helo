#ifndef AIRPLANE_H
#define AIRPLANE_H

#include "Car.h"

class AirplaneVehicle : public CBRaycastVehicle
{
public:
  class Engine
  {
  public:
    btVector3 position;
    btVector3 direction;
    btScalar maxThrust;

  public:
    Engine(const btVector3 &pos, const btVector3 &dir, const btScalar thrust) :
      position(pos), direction(dir), maxThrust(thrust) {}
  };

  typedef struct {
    HeloUtils::PieceWiseLinearFunction clAlpha;
    btScalar dragPolarK;
    btScalar dragPolarD0;
    std::vector<Engine> engines;
    btScalar wingArea;
    btScalar wingAngle;
    btScalar elevatorSensitivity;
    btScalar rudderSensitivity;
    btScalar aileronSensitivity;
    btScalar pitchStability;
    btScalar pitchStability2;
    btScalar yawStability;
    btScalar yawStability2;
    btScalar rollStability;
  } AirplaneData;

public:
  typedef struct {
    float thrust;
    float rudder;
    float elevator;
    float aileron;
  } ControlData;


protected:
  AirplaneData d;
  ControlData controlData;

protected:
  virtual void applyThrust(btScalar timeStep);
  virtual void applyLift(btScalar timeStep, const btVector3 &localVelocity);
  virtual void applyDrag(btScalar timeStep, const btVector3 &localVelocity);
  virtual void applyRudders(btScalar timeStep, const btVector3 &localVelocity,
                            const btVector3 &localAngularVelocity);

public:
  AirplaneVehicle(btRigidBody *fuselage, const AirplaneData &data);
  virtual void updateAction(btCollisionWorld* collisionWorld, btScalar step);
  void setInput(const ControlData &cd);
};


class Airplane : public Car
{
public:
  class Engine
  {
  public:
    Ogre::Vector3 position;
    Ogre::Vector3 direction;
    Ogre::Real maxThrust;

  public:
    Engine() {}
    Engine(const btVector3 &pos, const btVector3 &dir, const btScalar thrust) :
      position(pos), direction(dir), maxThrust(thrust) {}
  };

public:
  typedef struct
  {
    Ogre::String name;
    Ogre::String meshname;
    Ogre::Vector3 position;
    Ogre::Vector3 rotation;
    Ogre::Vector3 size;
    Ogre::Real weight;
    std::vector<WheelData> wheelData;
    std::vector<Engine> engineData;
    std::vector<std::pair<Ogre::Real, Ogre::Real> > cl_alpha_values;
    Ogre::Real dragPolarK;
    Ogre::Real dragPolarD0;
    Ogre::Real wingArea;
    Ogre::Real wingAngle;
    Ogre::Real aileronSensitivity;
    Ogre::Real elevatorSensitivity;
    Ogre::Real rudderSensitivity;
    Ogre::Real pitchStability1;
    Ogre::Real pitchStability2;
    Ogre::Real yawStability1;
    Ogre::Real yawStability2;
    Ogre::Real rollStability;
  } AirplaneData;

private:
  static AirplaneVehicle::AirplaneData DataToAirplaneVehicleData(const AirplaneData &data);

protected:
  AirplaneVehicle *airplane;
  AirplaneVehicle::ControlData controlData;

public:
  Airplane(const AirplaneData &data, Ogre::Root *root);
  //  void setThrottle(Ogre::Real fraction);
  virtual Controller *createController(OIS::Object *dev);
  void setInput() {airplane->setInput(controlData);}
  AirplaneVehicle::ControlData &getControlData() {return controlData;}
};

#endif // AIRPLANE_H
