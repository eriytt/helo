#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include <OIS/OIS.h>

class Control
{
  float val;

public:
  Control(): val(0.0f) {}
  virtual float getValue() {return val;};
  virtual void setValue(float value) {val = value;}
};


class Controller
{
 protected:
  bool active;

 public:
  Controller() : active(false) {}
  virtual ~Controller() {}
  virtual void setActive(bool a) {active = a;}
  virtual void update(float timeDelta) = 0;
};

class Controllable
{
protected:
  Controller *controller;

public:
  Controllable() : controller(NULL) {}
  Controller *getController() {return controller;};
  void setController(Controller *c) {controller = c;}
  virtual Controller *createController(OIS::Object *dev) = 0;
};

class Vehicle;
class Car;

class CarKeyController : public Controller, public OIS::KeyListener
{
protected:
  OIS::Keyboard &keyboard;
  Car &car;
  Control steer, accel, hoe, bucket;

public:
  CarKeyController(OIS::Keyboard &kb, Car &c);
  bool keyPressed(const OIS::KeyEvent&);
  bool keyReleased(const OIS::KeyEvent&);
  void update(float timeDelta);
};

class CarJoystickController : public Controller, public OIS::JoyStickListener
{
protected:
  OIS::JoyStick &joystick;
  Car &car;

public:
  CarJoystickController(OIS::JoyStick &js, Car &c) : joystick(js), car(c) {}
  bool buttonPressed(const OIS::JoyStickEvent&, int) {return true;}
  bool buttonReleased(const OIS::JoyStickEvent&, int) {return true;}
  bool axisMoved(const OIS::JoyStickEvent&, int);
  void update(float timeDelta) {}
  void setActive(bool a);
};

class SimpleCarAutoController : public Controller
{
 protected:
  Car *car;
  float targetSpeed;

 public:
  SimpleCarAutoController(Car *c) : car(c), targetSpeed(0.0) {}
  SimpleCarAutoController(Vehicle *v);
  virtual void update(float timeDelta);
  void setSpeed(float s) {targetSpeed = s;}
  float getSpeed() {return targetSpeed;}
};

class TankKeyController : public CarKeyController
{
public:
  bool keyPressed(const OIS::KeyEvent&);
  bool keyReleased(const OIS::KeyEvent&);
};

class TankJoystickController : public CarJoystickController
{
public:
  bool buttonPressed(const OIS::JoyStickEvent&, int);
  bool buttonReleased(const OIS::JoyStickEvent&, int);
  bool axisMoved(const OIS::JoyStickEvent&, int);
};

class Helicopter;

class HelicopterKeyController : public Controller, public OIS::KeyListener
{
public:
  bool keyPressed(const OIS::KeyEvent&);
  bool keyReleased(const OIS::KeyEvent&);
  void update(float timeDelta);
};

class HelicopterJoystickController : public Controller, public OIS::JoyStickListener
{
protected:
  OIS::JoyStick &joystick;
  Helicopter &helicopter;

public:
  HelicopterJoystickController(OIS::JoyStick &js, Helicopter &h) : joystick(js), helicopter(h) {}
  bool buttonPressed(const OIS::JoyStickEvent&, int);
  bool buttonReleased(const OIS::JoyStickEvent&, int);
  bool axisMoved(const OIS::JoyStickEvent&, int);
  void update(float timeDelta);
  void setActive(bool a);
};

class CharacterKeyController : public Controller, public OIS::KeyListener
{
public:
  bool keyPressed(const OIS::KeyEvent&);
  bool keyReleased(const OIS::KeyEvent&);
};

class CharacterJoystickController : public Controller, public OIS::JoyStickListener
{
public:
  bool buttonPressed(const OIS::JoyStickEvent&, int);
  bool buttonReleased(const OIS::JoyStickEvent&, int);
  bool axisMoved(const OIS::JoyStickEvent&, int);
};


class Airplane;

class AirplaneJoystickController : public Controller, public OIS::JoyStickListener
{
protected:
  OIS::JoyStick &joystick;
  Airplane &airplane;

public:
  AirplaneJoystickController(OIS::JoyStick &js, Airplane &ap) : joystick(js), airplane(ap) {}
  bool buttonPressed(const OIS::JoyStickEvent&, int) {return true;}
  bool buttonReleased(const OIS::JoyStickEvent&, int) {return true;}
  bool axisMoved(const OIS::JoyStickEvent&, int);
  void update(float timeDelta) {}
  void setActive(bool a);
};

class AirplaneKeyController : public Controller, public OIS::KeyListener
{
protected:
  OIS::Keyboard &keyboard;
  Airplane &airplane;
  Control throttle, yaw, pitch, roll;

public:
  AirplaneKeyController(OIS::Keyboard &kb, Airplane &ap);
  bool keyPressed(const OIS::KeyEvent&);
  bool keyReleased(const OIS::KeyEvent&);
  void update(float timeDelta);
};


#endif
