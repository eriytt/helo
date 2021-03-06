#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include <OIS/OIS.h>

class Controller
{
 protected:
  bool active;

 public:
  Controller() : active(false) {}
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
  virtual Controller *createController(OIS::Object *dev) = 0;
};

class Car;

class CarKeyController : public Controller, public OIS::KeyListener
{
protected:
  OIS::Keyboard &keyboard;
  Car &car;

public:
  CarKeyController(OIS::Keyboard &kb, Car &c) : keyboard(kb), car(c) {}
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

public:
  AirplaneKeyController(OIS::Keyboard &kb, Airplane &ap) : keyboard(kb), airplane(ap) {}
  bool keyPressed(const OIS::KeyEvent&);
  bool keyReleased(const OIS::KeyEvent&);
  void update(float timeDelta);
};


#endif
