#include "InputHandler.h"

#include <sstream>

InputHandler::InputHandler(size_t windowHnd)
{
  OIS::ParamList pl;
  std::ostringstream windowHndStr;

  windowHndStr << windowHnd;
  pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));
  pl.insert(std::make_pair(std::string("XAutoRepeatOn"), "true"));
  pl.insert(std::make_pair(std::string("x11_keyboard_grab"), "false"));
  pl.insert(std::make_pair(std::string("x11_mouse_grab"), "false"));
  pl.insert(std::make_pair(std::string("x11_mouse_hide"), "false"));
  mInputManager = OIS::InputManager::createInputSystem(pl);

  initKeyboards();
  initMice();
  initJoysticks();

  if (getNumKeyboards())
    {
      getKeyboard(0)->setBuffered(true);
      getKeyboard(0)->setEventCallback(&kproxy);
    }
  // TODO: else raise exception?
}

void InputHandler::initKeyboards()
{
  try
    {
      OIS::Keyboard *k = static_cast<OIS::Keyboard*>(mInputManager->createInputObject(OIS::OISKeyboard, true));
      keyboards.push_back(k);
    }
  catch (const OIS::Exception &e)
    {
      return;
    }
  return;
}

void InputHandler::initMice()
{
  try
    {
      OIS::Mouse *m = static_cast<OIS::Mouse*>(mInputManager->createInputObject(OIS::OISMouse, false));
      mice.push_back(m);
    }
  catch (const OIS::Exception &e)
    {
      return;
    }
  return;

}

void InputHandler::initJoysticks()
{
  try
    {
      OIS::JoyStick *j = static_cast<OIS::JoyStick*>(mInputManager->createInputObject(OIS::OISJoyStick, true));
      joysticks.push_back(j);
    }
  catch (const OIS::Exception &e)
    {
      return;
    }
  return;

}

OIS::Keyboard *InputHandler::getKeyboard(unsigned int idx)
{
  if (idx < getNumKeyboards())
    return keyboards[idx];
  return NULL;
}

OIS::Mouse *InputHandler::getMouse(unsigned int idx)
{
  if (idx < getNumMice())
    return mice[idx];
  return NULL;
}

OIS::JoyStick *InputHandler::getJoystick(unsigned int idx)
{
  if (idx < getNumJoysticks())
    return joysticks[idx];
  return NULL;
}

void InputHandler::update(void)
{
  for (std::vector<OIS::Keyboard*>::iterator i = keyboards.begin(); i != keyboards.end(); ++i)
    //    if ((*i)->getEventCallback())
      (*i)->capture();

  for (std::vector<OIS::Mouse*>::iterator i = mice.begin(); i != mice.end(); ++i)
    if ((*i)->getEventCallback())
      (*i)->capture();

  for (std::vector<OIS::JoyStick*>::iterator i = joysticks.begin(); i != joysticks.end(); ++i)
    if ((*i)->getEventCallback())
      (*i)->capture();
}

void InputHandler::KeyboardProxy::delListener(OIS::KeyListener *l)
{
  for (std::vector<OIS::KeyListener*>::const_iterator i = listeners.begin();
       i != listeners.end(); ++i)
    if ((*i) == l)
      {
        //i->erase();
        return;
      }
}

bool InputHandler::KeyboardProxy::keyPressed(const OIS::KeyEvent& e)
{
  if (not listeners.size())
    return true;

  for (std::vector<OIS::KeyListener*>::const_iterator i = listeners.begin();
       i != listeners.end(); ++i)
    (*i)->keyPressed(e);

  return true;
}

bool InputHandler::KeyboardProxy::keyReleased(const OIS::KeyEvent& e)
{
  if (not listeners.size())
    return true;

  for (std::vector<OIS::KeyListener*>::const_iterator i = listeners.begin();
       i != listeners.end(); ++i)
    (*i)->keyReleased(e);

  return true;
}
