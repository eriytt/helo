#ifndef AIRPLANE_H
#define AIRPLANE_H

#include "Car.h"
#include "HardPoints.h"

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

  struct AirplaneData : Car::CarData
  {
    HeloUtils::PieceWiseLinearFunction clAlpha;
    btScalar dragPolarK;
    btScalar dragPolarD0;
    std::vector<Engine> engines;
    btScalar wingArea;
    btScalar wingAngle;
    btScalar elevatorSensitivity;
    btScalar rudderSensitivity;
    btScalar aileronSensitivity;
    btScalar pitchStability1;
    btScalar pitchStability2;
    btScalar yawStability1;
    btScalar yawStability2;
    btScalar rollStability;
  };

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
  typedef AirplaneVehicle::Engine Engine
;
  struct AirplaneData: AirplaneVehicle::AirplaneData
  {
    std::vector<Engine> engineData;
    std::vector<std::pair<Ogre::Real, Ogre::Real> > cl_alpha_values;
  };

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
