#ifndef HELO_H
#define HELO_H

#include <Ogre.h>

#include "InputHandler.h"
#include "Configuration.h"

// Forward declarations
class Physics;
class Terrain;
class btRigidBody;
class Controllable;

class heloApp : public Ogre::FrameListener, public OIS::KeyListener//, public OIS::JoyStickListener
{

protected:
  static Ogre::String DefaultTerrainResourceGroup;

  // OGRE stuff
protected:
  Ogre::Root *mRoot;
  Ogre::Camera *cam;
  Ogre::Timer * timer;
  InputHandler *inputHandler;

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
  void handleInput(Ogre::Real delta);


protected:
  Configuration *conf;
  Terrain *terrain;
  Physics *physics;
  unsigned long lastFrameTime_us;
  bool mExit;

  // Testing
protected:
  void setSpherePosition(const Ogre::Vector3 &newPos);
  btRigidBody *sphere;
  Ogre::SceneNode *sphere_node;

protected:
  unsigned int currentControllable;
  void cycleControllable();

protected:
  Controllable *create_HMMWV(const Ogre::String name, const Ogre::Vector3 &position, Ogre::Root *root, Physics &physics);
  Controllable *create_Defender(const Ogre::String name, const Ogre::Vector3 &position, Ogre::Root *root, Physics &physics);
  Controllable *create_Chinook(const Ogre::String name, const Ogre::Vector3 &position, Ogre::Root *root, Physics &physics);
  Controllable *create_M93A1(const Ogre::String name, const Ogre::Vector3 &position, Ogre::Root *root, Physics &physics);
  Controllable *create_M1Abrams(const Ogre::String name, const Ogre::Vector3 &position, Ogre::Root *root, Physics &physics);
  Controllable *create_Soldier(const Ogre::String name, const Ogre::Vector3 &position, Ogre::Root *root, Physics &physics);


public:
  heloApp();
  virtual ~heloApp();
  int main(int argc, char *argv[]);
  bool keyPressed(const OIS::KeyEvent &e);
  bool keyReleased(const OIS::KeyEvent &e);

public:
  static heloApp *theApp;
};
#endif // HELO_H
