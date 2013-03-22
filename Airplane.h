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
    std::vector<Engine> engines;
  } AirplaneData;

public:
  typedef struct {
    float thrust;
    float steer_angle;
    float clutch;
    float brake;
    int shift_gear;
  } ControlData;


protected:
  AirplaneData d;
  ControlData controlData;

protected:
  virtual void applyThrust(btScalar timeStep);
  virtual void applyLift(btScalar timeStep, btScalar velocityForward);
  virtual void applyDrag(btScalar timeStep, btScalar velocityForward);
  virtual void applyRudders(btScalar timeStep, btScalar velocityForward);

public:
  AirplaneVehicle(btRigidBody *fuselage, const AirplaneData &data) :
    CBRaycastVehicle(fuselage), d(data) {}
  virtual void updateAction(btCollisionWorld* collisionWorld, btScalar step);
  void setInput(const ControlData &cd);
};


class Airplane : public Car
{
public:
  typedef struct
  {
    Ogre::String name;
    Ogre::String meshname;
    Ogre::Vector3 position;
    Ogre::Vector3 size;
    Ogre::Real weight;
    std::vector<WheelData> wheelData;
    std::vector<std::pair<Ogre::Real, Ogre::Real> > cl_alpha_values;
    Ogre::Real dragPolarK;
  } AirplaneData;

private:
  static AirplaneVehicle::AirplaneData DataToAirplaneVehicleData(const AirplaneData &data);

protected:
  AirplaneVehicle *airplane;
  AirplaneVehicle::ControlData controlData;

public:
  Airplane(const AirplaneData &data, Ogre::Root *root);
  void setThrottle(Ogre::Real fraction);
};

#endif // AIRPLANE_H
