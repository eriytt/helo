#ifndef INPUTHANDLER_H
#define INPUTHANDLER_H

#include <OIS/OIS.h>

class InputHandler
{
protected:
  class KeyboardProxy : public OIS::KeyListener
  {
  protected:
    std::vector<OIS::KeyListener*> listeners;

  public:
    void addListener(OIS::KeyListener *l) {listeners.push_back(l);}
    void delListener(OIS::KeyListener *l);
    bool keyPressed(const OIS::KeyEvent& e);
    bool keyReleased(const OIS::KeyEvent& e);
  };

protected:
  OIS::InputManager *mInputManager;

  std::vector<OIS::Keyboard*> keyboards;
  std::vector<OIS::Mouse*> mice;
  std::vector<OIS::JoyStick*> joysticks;

  KeyboardProxy kproxy;

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
  void addKeyboardListener(OIS::KeyListener *l) {kproxy.addListener(l);}
  void delKeyboardListener(OIS::KeyListener *l) {kproxy.delListener(l);}
  
  void update(void);
};

#endif // INPUTHANDLER_H
