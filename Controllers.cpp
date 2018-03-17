#include "Controllers.h"

#include <stdexcept>

#include "Car.h"
#include "Airplane.h"

SimpleCarAutoController::SimpleCarAutoController(Vehicle *v) : targetSpeed(0.0)
{
  car = dynamic_cast<Car*>(v);
  if (not car)
    throw std::runtime_error("SimpleCarAutoController: Tried to initialize"
			     " with Vehicle that is not a Car");
}

void SimpleCarAutoController::update(float timeDelta)
{
  float s = car->getSpeed();

  // if (s < targetSpeed)
  //   car->setThrottle(0.3);
  // else
  //   car->setThrottle(0.0);
}

AirplaneKeyController::AirplaneKeyController(OIS::Keyboard &kb, Airplane &ap)
  : keyboard(kb), airplane(ap)
{
  Actuator *act = ap.getActuator("Fuselage.Throttle");
  if (act)
    act->setControl(&throttle);

  act = ap.getActuator("Fuselage.Yaw");
  if (act)
    act->setControl(&yaw);

  act = ap.getActuator("Fuselage.Pitch");
  if (act)
    act->setControl(&pitch);

  act = ap.getActuator("Fuselage.Roll");
  if (act)
    act->setControl(&roll);

}

CarKeyController::CarKeyController(OIS::Keyboard &kb, Car &c)
  : keyboard(kb), car(c)
{
  Actuator *act = car.getActuator("Back.Front.steer");
  if (act)
    act->setControl(&steer);

  act = car.getActuator("Back.BackWheelDrive");
  if (act)
    act->setControl(&accel);

  act = car.getActuator("Back.Front.FrontWheelDrive");
  if (act)
    act->setControl(&accel);

  act = car.getActuator("Body.Drive");
  if (act)
    act->setControl(&accel);

  act = car.getActuator("Body.Steer");
  if (act)
    act->setControl(&steer);


  act = car.getActuator("Back.Front.Hoe.hoe");
  if (act)
    act->setControl(&hoe);

  act = car.getActuator("Back.Front.Hoe.Bucket.bucket");
  if (act)
    act->setControl(&bucket);

}
