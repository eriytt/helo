#ifndef AIRPLANE_H
#define AIRPLANE_H

#include "Car.h"

class Airplane : public Car
{
  typedef struct
  {
    Ogre::String name;
    Ogre::String meshname;
    Ogre::Vector3 position;
    Ogre::Vector3 size;
    Ogre::Real weight;
    std::vector<WheelData> wheelData;

  } AirplaneData;
};

#endif // AIRPLANE_H
