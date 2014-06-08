#ifndef VEHICLE_H
#define VEHICLE_H

#include "Armament.h"
#include "HardPoints.h"

class Vehicle
{
protected:
  const Hardpoint::Config *hardpoints;
  std::map<int, const Armament*> armaments;
  
 public:
  virtual void addArmament(Armament *armament, int hardpointID) {armaments[hardpointID] = armament;}
  void setHardpointConfig(const Hardpoint::Config &conf) {hardpoints = &conf;}
};

#endif // VEHICLE_H
