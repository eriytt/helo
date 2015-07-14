#include <string>
#include <iostream>

#include "helo.h"
#include "Terrain.h"
#include "Camera.h"
#include "Physics.h"

// TODO: hopefully not needed here indefinately
#include "Helicopter.h"
#include "Car.h"
#include "Tank.h"
#include "Character.h"
//#include "Configuration.h"
#include "ScriptEngine.h"
#include "Python.h"
#include "Lua.h"

class Error {
public:
  static void ErrMessage(const std::string err_string)
  {
    std::cerr << err_string << std::endl;
  }
};


Ogre::String heloApp::DefaultTerrainResourceGroup("Terrain/Default");
heloApp *heloApp::theApp = 0;

heloApp::heloApp(): mRoot(0), timer(0), conf(0), terrain(0), camera(0),
		    physics(0), scripter(0), lastFrameTime_us(0), mExit(false)
{
  if (theApp)
    throw HeloException("Singleton instance already initialized");

  theApp = this;
}

heloApp::~heloApp()
{
  if (scripter)
    delete scripter;
  if (physics)
    delete physics;

  if (terrain)
    delete terrain;

  if (mRoot)
    delete mRoot;
}

// Controllable *heloApp::create_Soldier(const Ogre::String name, const Ogre::Vector3 &position, Ogre::Root *root, Physics &physics)
// {
//   Character *soldier = new Character(root);
//   return soldier;
// }

extern "C" int luaopen_helo(lua_State* L); // defined in the lua wrapper

int heloApp::main(int argc, char *argv[])
{
  initOGRE();

  terrain = new ::Terrain(mRoot, DefaultTerrainResourceGroup);
  camera->setTerrain(terrain);

  conf = new Configuration(mRoot);
  try
    {
      conf->loadConfig();
    }
  catch (Configuration::ConfigurationError e)
    {
      Error::ErrMessage(e.what());
      exit(-1);
    }

  if (conf->usePython())
    scripter = new Python(argv[0], true, conf->xtermPath());
  else if (conf->useLua())
    {
      Lua *lua = new Lua(argv[0], true, conf->xtermPath());
      lua->openLib(luaopen_helo);
      scripter = lua;
    }

  // TODO: maybe there should be some space above the highest top of the terrain?
  physics = new Physics(terrain->getBounds(), conf->physicsInThread());
  conf->setPhysics(physics);
  conf->setResourceBase("./resources/");
  physics->addBody(terrain->createBody());

  if (scripter)
    for (const std::string &script : conf->getPreScripts())
      scripter->runFile(script);

  if (conf->getStartMission() != "")
    {
      try
	{
	  conf->loadMission(conf->getStartMission());
	}
      catch (Configuration::ConfigurationError e)
	{
	  Error::ErrMessage(e.what());
	  exit(-1);
	}
    }


  Ogre::SceneManager *mgr = mRoot->getSceneManager("SceneManager");
  Ogre::Entity *ent = mgr->createEntity("Sphere", "Sphere.mesh");
  //ent->setCastShadows(true);
  sphere_node = mgr->getRootSceneNode()->createChildSceneNode("SphereNode");
  sphere_node->setPosition(Ogre::Vector3(1683 / 1.2, 50, 2116 / 1.2));
  sphere_node->attachObject(ent);
  sphere = physics->testSphere(Ogre::Vector3(1683 / 1.1, 50, 2116 / 1.1), 1.0, sphere_node);
  physics->addBody(sphere);

  //Controllable *c;

  // c = create_Soldier("soldier0", Ogre::Vector3(1880.96, 1.0, 1650.3), mRoot, *physics);
  // controllables.push_back(c);

  OIS::Object *dev = NULL;
  if (inputHandler->getNumJoysticks())
    dev = inputHandler->getJoystick(0);
  else
    dev = inputHandler->getKeyboard(0);

  std::cout << "Joysticks: " << inputHandler->getNumJoysticks() << std::endl;

  const std::vector<Controllable*> &controllables = conf->getControllables();

  for (std::vector<Controllable*>::const_iterator i = controllables.begin(); i != controllables.end(); ++i)
    {
      Controllable *c = *i;
      c->createController(dev);
    }

  if (controllables.size())
    {
      currentControllable = controllables.size() - 1;
      cycleControllable();

      HeloUtils::Trackable *t = dynamic_cast<HeloUtils::Trackable*>(controllables[currentControllable]);
      camera->setTrackable(t);
    }

  physics->finishConfiguration();

  if (scripter)
    for (const std::string &script : conf->getPostScripts())
      scripter->runFile(script);

  using ul_t = unsigned long;
  using EQ_t = EventQueue<ul_t>;
  using LE_t = EQ_t::LambdaEvent;
  using eid_t = EQ_t::EventID;

  queue.postEvent(1 * 1000000,
		  new LE_t([](ul_t at, ul_t et, eid_t id) -> void {
		      std::cout << "Event (explicit lambda) after 1 seconds" << std::endl;
		    }));

  queue.postEvent(2 * 1000000, [](ul_t at, ul_t et, eid_t id){
      std::cout << "Event (lambda) after 2 seconds" << std::endl;});

  queue.postEvent(1 * 1000000,
   		  new PrintEvent<unsigned long>("Event (derived) after 1 seconds"));

  queue.postEvent(3 * 1000000, [](ul_t at, ul_t et, eid_t id){
      std::cout << "Event (lambda) after 3 seconds" << std::endl;});

  mainLoop();

  return 0;
}

void heloApp::mainLoop()
{
  const std::vector<Controllable*> &controllables = conf->getControllables();
  timer->reset();
  lastFrameTime_us = timer->getMicroseconds();
  while (not mExit) {
    unsigned long frame_time = timer->getMicroseconds();
    Ogre::Real tdelta = (frame_time - lastFrameTime_us) / Ogre::Real(1000000);
    lastFrameTime_us = frame_time;
    physics->step();
    physics->sync();

    if (scripter and scripter->needsToRun())
      {
	// If physics doesn't run in a thread the stop/resume should
	// be nops
	physics->stop();
	scripter->run();
	physics->resume();
      }

    queue.advance(lastFrameTime_us);

    inputHandler->update();
    handleInput(tdelta);
    camera->update(tdelta);
    doOgreUpdate();
    //usleep(50000);
  }
}

void heloApp::cycleControllable()
{
  const std::vector<Controllable*> &controllables = conf->getControllables();
  if (not controllables.size())
    return;

  Controller *c = controllables[currentControllable]->getController();
  if (c)
    c->setActive(false);

  unsigned int retries = controllables.size();
  int i = 1;
  while (retries--)
    {
      unsigned int cidx = (currentControllable + i) % controllables.size();
      Controller *c = controllables[cidx]->getController();
      if (c)
	{
	  c->setActive(true);
	  currentControllable = cidx;
	  break;
	}
      ++i;
    }
  /* Raise exception if retries is 0 ? */
}

bool heloApp::keyPressed(const OIS::KeyEvent &e)
{
  switch (e.key)
    {
    case OIS::KC_ESCAPE:
      mExit = true;
      break;
    case OIS::KC_SPACE:
      cycleControllable();
      break;
    default:
      return true;
    }

  return true;
}

bool heloApp::keyReleased(const OIS::KeyEvent &e)
{
  return true;
}

void heloApp::handleInput(Ogre::Real delta)
{
  const std::vector<Controllable*> &controllables = conf->getControllables();

  if (not controllables.size())
    return;

  Controller *c = controllables[currentControllable]->getController();
  if (c)
    c->update(static_cast<float>(delta));

}

void heloApp::setSpherePosition(const Ogre::Vector3 &newPos)
{
  btTransform tr;
  tr.setIdentity();
  tr.setOrigin(btVector3(newPos.x, newPos.y, newPos.z));
  std::cout << "New position: " << newPos.x << " "  << newPos.y << " "  << newPos.z <<  std::endl;
  //btMotionState *mt = sphere->getMotionState();
  sphere->setCenterOfMassTransform(tr);
  sphere->setLinearVelocity(btVector3(0, 0, 0));
  sphere->setAngularVelocity(btVector3(0, 0, 0));
  sphere->activate();
}


int main(int argc, char *argv[])
{
  heloApp app;
  return app.main(argc, argv);
}
