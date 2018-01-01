%module helo

%include "std_vector.i"
%include "std_string.i"

%{
#include "helo.h"
#include "Terrain.h"
#include "Utils.h"
#include "Camera.h"
#include "Controllers.h"
#include "Physics.h"
#include "Vehicle.h"
#include "Lua.h"
%}

%nodefault Ogre;
namespace Ogre
{
  typedef float Real;

  class Vector3
  {
    public:
    Real x, y, z;
    Vector3( const Real fX, const Real fY, const Real fZ );
  };

  class AxisAlignedBox
  {
  public:
    const Vector3& getMinimum(void) const;
    const Vector3& getMaximum(void) const;
  };

  %nodefault SceneNode;
  class SceneNode
  {
  public:
    virtual const Vector3 &getPosition (void) const;
  };
};

%nodefault Terrain;
class Terrain
{
public:
  Ogre::Real getHeight(Ogre::Real x, Ogre::Real y);
  Ogre::AxisAlignedBox getBounds();
};

namespace HeloUtils
{
  %nodefault Trackable;
  class Trackable
  {
    virtual Ogre::SceneNode *getSceneNode() = 0;
  };
};

%nodefault Camera;
class Camera
{
public:
  void setTrackable(HeloUtils::Trackable *t);
  HeloUtils::Trackable *getTrackable();
  void setTerrain(Terrain *t);
  void setPosition(const Ogre::Vector3 &newpos);
  void setLookAt(const Ogre::Vector3 &newlook);
  const Ogre::Vector3 &getPosition() const;
  Ogre::Vector3 getDirection() const;
};

%nodefault Controllable;
class Controllable
{
 public:
  Controller *getController();
  void setController(Controller *c);

  %extend {
    HeloUtils::Trackable *toTrackable() {return dynamic_cast<HeloUtils::Trackable*>($self);}
  }
};

class SimpleCarAutoController : public Controller
{
 public:
  SimpleCarAutoController(Vehicle *v);
  void setSpeed(float s);
  float getSpeed();
};


%template(ControllableVector) std::vector<Controllable*>;

%nodefault PhysicsObject;
class PhysicsObject
{
public:
  virtual void finishPhysicsConfiguration(class Physics *phys) = 0;
};


%nodefault Vehicle;
class Vehicle
{
 public:
  %extend {
    PhysicsObject *toPhysicsObject() {return dynamic_cast<PhysicsObject*>($self);}
    Controllable *toControllable() {return dynamic_cast<Controllable*>($self);}
  }
};


%nodefault Configuration;
class Configuration
{
public:
  const std::vector<Controllable*> &getControllables();
  Vehicle *Configuration::loadVehicle(const std::string &type, const std::string &name, const Ogre::Vector3 &position, const Ogre::Vector3 &rotation);
};

%header %{
typedef EventQueue<unsigned long> ulEventQueue;
%}


%nodefault ulEventQueue;
class ulEventQueue
{
};

%wrapper %{
static int postEventToQueue(lua_State* L) {
  EventQueue<unsigned long> *queue = (EventQueue<unsigned long> *) 0;
  unsigned long at = 0;
  bool arg3_is_lua_function = false;
  int arg3_ref = LUA_NOREF;
  ulEventQueue::EventID eid = -1;

  SWIG_check_num_args("postEventToQueue", 3, 3)

  if(!SWIG_isptrtype(L,1)) SWIG_fail_arg("postEventToQueue",1,"ulEventQueue *");
  if(!lua_isnumber(L,2)) SWIG_fail_arg("postEventToQueue",2,"unsigned long");

  // Checking of th closure argument
  arg3_is_lua_function = static_cast<bool>(lua_isfunction(L, 3) && !lua_iscfunction(L, 3));
  if(!arg3_is_lua_function)
      SWIG_fail_arg("postEventToQueue",3,"Lua function");

  if (!SWIG_IsOK(SWIG_ConvertPtr(L, 1, reinterpret_cast<void**>(&queue), SWIGTYPE_p_ulEventQueue, 0)))
    {
      SWIG_fail_ptr("postEventToQueue",1,SWIGTYPE_p_ulEventQueue);
    }

  SWIG_contract_assert((lua_tonumber(L, 2) >= 0), "number must not be negative")
  at = (unsigned long)lua_tonumber(L, 2);

  arg3_ref = luaL_ref(L, LUA_REGISTRYINDEX);

  eid = queue->postEvent(at, new LuaEventCallback<unsigned long>(L, arg3_ref));

  // clear the stack
  lua_settop(L, 0);

  // push return value on the stack
  lua_pushinteger(L, eid);

  // One value returned
  return 1;

fail:
  lua_error(L);
  return 0;
}
%}


%native(postEventToQueue) unsigned int postEventToQueue(unsigned long at, void *c);


class heloApp : public Ogre::FrameListener, public OIS::KeyListener//, public OIS::JoyStickListener
{
public:
  static heloApp *theApp;
  Terrain *getTerrain();
  Camera *getCamera();
  Physics *getPhysics();
  Configuration *getConfiguration();
  ulEventQueue *getEventQueue();
};
