#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "Utils.h"

class DriveTrainComponent {
protected:
  class DriveTrainComponent *parent;

 public:
  DriveTrainComponent() : parent(NULL) {}
  DriveTrainComponent(class DriveTrainComponent *p) : parent(p) {}
  void setParent(class DriveTrainComponent *p) {parent = p;}
  virtual void setRPM(float rpm) {if (parent) parent->setRPM(rpm);}
  virtual float getRPM() {return parent != NULL ? parent->getRPM() : 0;}
  virtual float getMOI() {return parent != NULL ? parent->getMOI() : 0;}
  virtual float getTorque() {return parent != NULL ? parent->getTorque() : 0;}
  virtual float getRatio(float ratio) {return parent != NULL ? parent->getRatio(ratio): 0;}
};


class Engine : public DriveTrainComponent {
protected:
  float throttle;
  float RPM;
  float moi;
  HeloUtils::PieceWiseLinearFunction torque_max_throttle;
  HeloUtils::PieceWiseLinearFunction torque_min_throttle;

public:
  Engine(float moment_of_inertia);
  void setRPM(float rpm);
  float getTorque();

public:
  void setThrottle(float t);
  void addTorqueDataPoint(float rpm, float torque_max, float torque_min);
  float getMOI();
  float getRPM();
  float getRatio(float ratio);
};


#define CLUTCH_MAX 1.0
#define CLUTCH_MAX_FORCE 10250

class Clutch : public DriveTrainComponent {
protected:
  float currentClutch;
  bool locked;
  float torqueFrictionIndexSlipping;
  float torqueFrictionIndexLocked;
  float inRPM, outRPM, lastOutRPM;
  float outTorque;

public:
  Clutch(DriveTrainComponent *motor);
  void setClutch(float clutch);

  void setRPM(float rpm);
  float getTorque();
};

class Gearbox : public DriveTrainComponent {
protected:
  unsigned short currentGear;
  std::vector<float> gearRatio;

public:
  Gearbox(DriveTrainComponent *clutch);
  void addGear(float gear_ratio);
  void shift(int gear);
  void shiftUp();
  void shiftDown();

  void setRPM(float rpm);
  float getTorque();
  float getRatio(float ratio);
  int getCurrentGear();
};

class Differential : public DriveTrainComponent {
protected:
  float differentialRatio;
  float transmissionEfficiency;

public:
  Differential(DriveTrainComponent *gearbox, float ratio);
  virtual void setRPM(float rpm);
  virtual float getTorque();
  virtual float getRatio(float ratio);
  virtual void setTransmissionEfficinecy(float efficiency);
};

#endif // DRIVETRAIN_H
