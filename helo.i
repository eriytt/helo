%module helo

%include "std_vector.i"

%{
#include "helo.h"
#include "Terrain.h"
#include "Utils.h"
#include "Camera.h"
#include "Controllers.h"
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
  %extend {
    HeloUtils::Trackable *toTrackable() {return dynamic_cast<HeloUtils::Trackable*>($self);}
  }
};

%template(ControllableVector) std::vector<Controllable*>;

%nodefault Configuration;
class Configuration
{
public:
  const std::vector<Controllable*> &getControllables();
};

class heloApp : public Ogre::FrameListener, public OIS::KeyListener//, public OIS::JoyStickListener
{
public:
  static heloApp *theApp;
  Terrain *getTerrain();
  Camera *getCamera();
  Configuration *getConfiguration();
};
