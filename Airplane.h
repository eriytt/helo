#ifndef AIRPLANE_H
#define AIRPLANE_H

#include "Car.h"

class AirplaneVehicle : public CBRaycastVehicle
{
protected:
  btScalar thrust;

public:
  AirplaneVehicle(btRigidBody *fuselage) : CBRaycastVehicle(fuselage), thrust(0.0) {}
  virtual void updateAction(btCollisionWorld* collisionWorld, btScalar step);
  void setThrust(btScalar thrust);
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

  } AirplaneData;

// private:
//   static Car::CarData AirplaneToCarData(const AirplaneData &data);

protected:
  AirplaneVehicle *airplane;

public:
  Airplane(const AirplaneData &data, Ogre::Root *root);
  void setThrottle(Ogre::Real fraction);
};

#endif // AIRPLANE_H
