#include "DriveTrain.h"

#include <iostream>

Engine::Engine(float moment_of_inertia) : throttle(0),
					  RPM(0),
					  moi(moment_of_inertia)

{
  addTorqueDataPoint(0, 0, 0);
}

void Engine::addTorqueDataPoint(float rpm, float torque_max,
				float torque_min)
{
  torque_max_throttle.addDataPoint(rpm, torque_max);
  torque_min_throttle.addDataPoint(rpm, torque_min);
}

float Engine::getTorque()
{
  float min_torque = torque_min_throttle[RPM];
  return min_torque + throttle * (torque_max_throttle[RPM] - min_torque);
}

void Engine::setRPM(float rpm)
{
  //printf("Setting rpm: %f\n", rpm);
  RPM = rpm;
}

float Engine::getMOI()
{
  return moi;
}

float Engine::getRPM()
{
  return RPM;
}

void Engine::setThrottle(float t)
{
  //printf("Setting throttle: %f\n", t);
  throttle = t;
}

float Engine::getRatio(float ratio)
{
  return ratio;
}


Clutch::Clutch(DriveTrainComponent *motor) : DriveTrainComponent(motor),
					     currentClutch(1.0),
					     locked(true),
					     inRPM(0),
					     outRPM(0),
					     lastOutRPM(0)
{
  torqueFrictionIndexSlipping = 0.08;
  torqueFrictionIndexLocked = 1.0;
  outRPM = 0.0;
  outTorque = 0.0;
}

void Clutch::setClutch(float clutch)
{
  //printf("Settng clutch: %f\n", clutch);
  currentClutch = clutch;
}

void Clutch::setRPM(float rpm)
{
  float clutchForce = currentClutch * CLUTCH_MAX_FORCE;
  lastOutRPM = outRPM;
  outRPM = rpm;

  //printf("Setting clutch rpm: %f\n", rpm);
  if (locked)
    {
      //printf("Clutch is locked\n");
      inRPM = outRPM;
      parent->setRPM(inRPM);
      outTorque = parent->getTorque();
      if (fabs(parent->getTorque())
	  > fabs(clutchForce * torqueFrictionIndexLocked))
	{
	  locked = false;
	  outTorque = 0.0;
	  printf("Unlocked clutch\n");
	}
    }
  else
    {
      //printf("Clutch is slipping\n");
      if ((lastOutRPM > inRPM && outRPM <= inRPM)
	  || (lastOutRPM < inRPM && outRPM >= inRPM))
	if (fabs(parent->getTorque())
	    < fabs(clutchForce * torqueFrictionIndexSlipping))
	  {
	    std::cout << "Locking clutch (new rpm: " << rpm
		      << ", in rpm: " << inRPM << ", out rpm: " << outRPM
		      << ", last out rpm: " << lastOutRPM << ")" << std::endl;
	    locked = true;
	    inRPM = outRPM;
	    parent->setRPM(inRPM);
	    outTorque = parent->getTorque();
	    return;
	  }

      float motor_torque = parent->getTorque();
      float counter_torque = clutchForce * torqueFrictionIndexSlipping;


      /* Car is going faster than motor, accelarate motor*/
      if (outRPM > inRPM)
	counter_torque *= -1.0;

      float motor_rpm_delta = ((motor_torque - counter_torque)
			       / parent->getMOI()
			       /** g.dtime*/); // TODO: how to get time delta here?
      parent->setRPM(parent->getRPM() + motor_rpm_delta);
      inRPM = parent->getRPM();
      outTorque = counter_torque;
    }
}

float Clutch::getTorque()
{
//   if (locked)
//     return parent->getTorque();

//   return currentClutch * CLUTCH_MAX_FORCE *
//   torqueFrictionIndexSlipping;
  return outTorque;
}

Gearbox::Gearbox(DriveTrainComponent *clutch) : DriveTrainComponent(clutch),
						currentGear(0)
{
  gearRatio.push_back(0);
}

void Gearbox::addGear(float gear_ratio)
{
  gearRatio.push_back(gear_ratio);
}

void Gearbox::shift(int gear)
{
  printf("Shifting to %d\n", gear);
  currentGear = gear;
}

void Gearbox::shiftUp()
{
  if (currentGear == gearRatio.size() - 1)
    return;
  shift(currentGear + 1);
}

void Gearbox::shiftDown()
{
  if (currentGear == 0)
    return;
  shift(currentGear - 1);
}


void Gearbox::setRPM(float rpm)
{
  assert(parent);
  float currentGearRatio = gearRatio[currentGear];
  if (currentGearRatio)
    {
      parent->setRPM(rpm * gearRatio[currentGear]);
      return;
    }

  float torque = parent->getTorque();
  float rpm_delta = (torque / parent->getMOI() /** g.dtime*/); // TODO: how to get time delta here

  //printf("Clutch MOI was %f\n", parent->getMOI());
  parent->setRPM(parent->getRPM() + rpm_delta);
}

float Gearbox::getTorque()
{
  assert(parent);
  return parent->getTorque() * gearRatio[currentGear];
}

float Gearbox::getRatio(float ratio)
{
  if (currentGear)
    return parent->getRatio(ratio) * gearRatio[currentGear];
  return parent->getRatio(ratio);
}
int Gearbox::getCurrentGear()
{
  return currentGear;
}
Differential::Differential(DriveTrainComponent *gearbox, float ratio_) :
  DriveTrainComponent(gearbox), differentialRatio(ratio_),
  transmissionEfficiency(1.0) {}

void Differential::setTransmissionEfficinecy(float efficiency)
{
  transmissionEfficiency = efficiency;
}

void Differential::setRPM(float rpm)
{
  assert(parent);
  parent->setRPM(rpm * differentialRatio);
}

float Differential::getTorque()
{
  assert(parent);
  return parent->getTorque() * differentialRatio * transmissionEfficiency;
}

float Differential::getRatio(float ratio)
{
  return parent->getRatio(ratio) * differentialRatio;
}
