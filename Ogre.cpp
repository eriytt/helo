#include "helo.h"

using namespace Ogre;

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

void heloApp::createRoot()
{
  fs::path pc = "plugins.cfg";
  std::cout << "plugins.cfg: " << pc << std::endl;
  mRoot = new Root(String(pc.file_string()));
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
  TextureManager::getSingleton().setDefaultNumMipmaps(5);
  ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
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
  std::ostringstream windowHndStr;
  OIS::ParamList pl;
  RenderWindow *win = mRoot->getAutoCreatedWindow();

  win->getCustomAttribute("WINDOW", &windowHnd);
  windowHndStr << windowHnd;
  pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));
  pl.insert(std::make_pair(std::string("XAutoRepeatOn"), "true"));
  pl.insert(std::make_pair(std::string("x11_keyboard_grab"), "false"));
  pl.insert(std::make_pair(std::string("x11_mouse_grab"), "false"));
  pl.insert(std::make_pair(std::string("x11_mouse_hide"), "false"));
  mInputManager = OIS::InputManager::createInputSystem(pl);

  try
    {
      mKeyboard = static_cast<OIS::Keyboard*>(mInputManager->createInputObject(OIS::OISKeyboard, true));
      //mMouse = static_cast<OIS::Mouse*>(mInputManager->createInputObject(OIS::OISMouse, false));
      mJoy = static_cast<OIS::JoyStick*>(mInputManager->createInputObject(OIS::OISJoyStick, true));
    }
  catch (const OIS::Exception &e)
    {
      throw Exception(42, e.eText, "Application::setupInputSystem");
    }

  mKeyboard->setEventCallback(this);
  mJoy->setEventCallback(this);
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
  mKeyboard->capture();
  mJoy->capture();
  mExit = mKeyboard->isKeyDown(OIS::KC_ESCAPE);

  handleInput();
  return not mExit;
}

bool heloApp::doOgreUpdate()
{
  //std::cout << "doOgreUpdate" << std::endl;
  WindowEventUtilities::messagePump();
  return mRoot->renderOneFrame();
}
