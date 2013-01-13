#ifndef INPUTHANDLER_H
#define INPUTHANDLER_H

#include <OIS/OIS.h>

class InputHandler
{
protected:
  OIS::InputManager *mInputManager;

  std::vector<OIS::Keyboard*> keyboards;
  std::vector<OIS::Mouse*> mice;
  std::vector<OIS::JoyStick*> joysticks;

protected:
  void initKeyboards();
  void initMice();
  void initJoysticks();

public:
  InputHandler(size_t windowHnd);
  unsigned int getNumKeyboards() {return keyboards.size();}
  unsigned int getNumMice() {return mice.size();}
  unsigned int getNumJoysticks() {return joysticks.size();}
  OIS::Keyboard *getKeyboard(unsigned int idx);
  OIS::Mouse *getMouse(unsigned int idx);
  OIS::JoyStick *getJoystick(unsigned int idx);
  void update(void);
};

#endif // INPUTHANDLER_H
