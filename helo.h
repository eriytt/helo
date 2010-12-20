#ifndef HELO_H
#define HELO_H

#include <OGRE/Ogre.h>
#include <OIS/OIS.h>

// Forward declarations
class Physics;
class Terrain;
class btRigidBody;

class heloApp : public Ogre::FrameListener, public OIS::KeyListener, public OIS::JoyStickListener
{
  bool mExit;

  // OGRE stuff
protected:
  Ogre::Root *mRoot;
  Ogre::Camera *cam;
  OIS::Keyboard *mKeyboard;
  OIS::JoyStick *mJoy;
  OIS::InputManager *mInputManager;

  void initOGRE();
  void createRoot();
  void defineResources();
  void setupRenderSystem();
  void createRenderWindow();
  void initializeResourceGroups();
  void setupScene();
  void setupInputSystem();
  void setupCEGUI();
  void createFrameListener();
  bool doOgreUpdate();
  bool frameStarted(const Ogre::FrameEvent& evt);
  void handleInput();

  // OIS event listener
protected:
  // keyboard
  bool keyPressed(const OIS::KeyEvent&);
  bool keyReleased(const OIS::KeyEvent&);

  // joystick
  bool buttonPressed(const OIS::JoyStickEvent&, int);
  bool buttonReleased(const OIS::JoyStickEvent&, int);
  bool axisMoved(const OIS::JoyStickEvent&, int);

protected:
  Terrain *terrain;
  Physics *physics;

  // Testing
protected:
  void setSpherePosition(const Ogre::Vector3 &newPos);
  btRigidBody *sphere;
  Ogre::SceneNode *sphere_node;

protected:
  class Helicopter *current_vehicle;
  class Car *current_car;

public:
  heloApp();
  virtual ~heloApp();
  int main(int argc, char *argv[]);

public:
  static heloApp *theApp;
};
#endif // HELO_H
