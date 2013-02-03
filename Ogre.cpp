#include "helo.h"

using namespace Ogre;

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "TinyXMLResourceManager.h"

void heloApp::createRoot()
{
  fs::path pc = "plugins.cfg";
  std::cout << "plugins.cfg: " << pc << std::endl;
  mRoot = new Root(String(pc.string()));
}

void heloApp::defineResources()
{
  ResourceGroupManager &rgm = ResourceGroupManager::getSingleton();
  rgm.addResourceLocation(".", "FileSystem");
  rgm.addResourceLocation("resources", "FileSystem");
}

void heloApp::setupRenderSystem()
{
  RenderSystem *rs = mRoot->getRenderSystemByName("OpenGL Rendering Subsystem");
  mRoot->setRenderSystem(rs);
  rs->setConfigOption("Full Screen", "No");
  rs->setConfigOption("Video Mode", "640 x 480 @ 32-bit colour");
}

void heloApp::createRenderWindow()
{
  mRoot->initialise(true, "Tutorial Render Window");
}

void heloApp::initializeResourceGroups()
{
  // TODO: save reference for deletion?
  OGRE_NEW TinyXMLResourceManager();

  TextureManager::getSingleton().setDefaultNumMipmaps(5);

  ResourceGroupManager &mgr = ResourceGroupManager::getSingleton();
  mgr.createResourceGroup(DefaultTerrainResourceGroup, false);
  mgr.createResourceGroup("helo", false);
  mgr.initialiseAllResourceGroups();
}

void heloApp::setupScene()
{
  SceneManager *mgr = mRoot->createSceneManager("DefaultSceneManager", "SceneManager");
  cam = mgr->createCamera("Camera");
  Viewport* vp = mRoot->getAutoCreatedWindow()->addViewport(cam, 0 , 0.0, 0.0, 1.0, 1.0);
  vp->setBackgroundColour(ColourValue::Blue);

  cam->setPosition(Vector3(1783, 2, 1600));
  cam->lookAt(Vector3(1757.96, 14.59526, 1625.3));
  cam->setNearClipDistance(0.1);
  cam->setFarClipDistance(50000);
  if (mRoot->getRenderSystem()->getCapabilities()->hasCapability(RSC_INFINITE_FAR_PLANE))
    cam->setFarClipDistance(0);   // enable infinite far clip distance if we can


  Vector3 lightdir(0.55, -0.3, 0.75);
  lightdir.normalise();
  Light *l = mgr->createLight("tstLight");
  l->setType(Light::LT_DIRECTIONAL);
  l->setDirection(lightdir);
  l->setDiffuseColour(ColourValue::White);
  l->setSpecularColour(ColourValue(0.4, 0.4, 0.4));

  mgr->setAmbientLight(ColourValue(0.2, 0.2, 0.2));

  //mgr->setAmbientLight(ColourValue(0.5, 0.5, 0.5));
}

void heloApp::setupInputSystem()
{
  size_t windowHnd = 0;
  RenderWindow *win = mRoot->getAutoCreatedWindow();
  win->getCustomAttribute("WINDOW", &windowHnd);

  inputHandler = new InputHandler(windowHnd);

  // OIS::Keyboard *k = inputHandler->getKeyboard(0);
  // k->setEventCallback(this);

  // OIS::JoyStick *j = inputHandler->getJoystick(0);
  // j->setEventCallback(this);
}


void heloApp::setupCEGUI()
{
}

void heloApp::createFrameListener()
{
  mRoot->addFrameListener(this);
  timer = new Ogre::Timer();
}


void heloApp::initOGRE()
{
  createRoot();
  defineResources();
  setupRenderSystem();
  createRenderWindow();
  initializeResourceGroups();
  setupScene();
  setupInputSystem();
  setupCEGUI();
  createFrameListener();
}

bool heloApp::frameStarted(const FrameEvent& evt)
{
  mExit = inputHandler->getKeyboard(0)->isKeyDown(OIS::KC_ESCAPE);
  return not mExit;
}

bool heloApp::doOgreUpdate()
{
  //std::cout << "doOgreUpdate" << std::endl;
  WindowEventUtilities::messagePump();
  return mRoot->renderOneFrame();
}
