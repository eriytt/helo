#include "Controllers.h"

#include <stdexcept>

#include "Car.h"

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

  if (s < targetSpeed)
    car->setThrottle(0.3);
  else
    car->setThrottle(0.0);
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
}
