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
public:
  bool buttonPressed(const OIS::JoyStickEvent&, int);
  bool buttonReleased(const OIS::JoyStickEvent&, int);
  bool axisMoved(const OIS::JoyStickEvent&, int);
  void update(float timeDelta);
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

class HelicopterKeyController : public Controller, public OIS::KeyListener
{
public:
  bool keyPressed(const OIS::KeyEvent&);
  bool keyReleased(const OIS::KeyEvent&);
  void update(float timeDelta);
};

class HelicopterJoystickController : public Controller, public OIS::JoyStickListener
{
public:
  bool buttonPressed(const OIS::JoyStickEvent&, int);
  bool buttonReleased(const OIS::JoyStickEvent&, int);
  bool axisMoved(const OIS::JoyStickEvent&, int);
  void update(float timeDelta);
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

#endif
