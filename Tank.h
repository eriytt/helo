#ifndef TANK_H
#define TANK_H

#include "Car.h"

class Tank : public Car
{
public:
  typedef Car::CarData TankData;

  Tank(const TankData &data, Ogre::Root *root) : Car(static_cast<const Car::CarData &>(data), root) {}
};

#endif // TANK_h
