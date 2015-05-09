#ifndef CAMERA_H
#define CAMERA_H

#include "Utils.h"

class Terrain;

class Camera
{
 protected:
  Ogre::Camera *cam;
  HeloUtils::Trackable *track;
  Terrain *terrain;

public:
  Camera(Ogre::Camera *c) : cam(c), track(0), terrain(0) {}
  void setTrackable(HeloUtils::Trackable *t);
  HeloUtils::Trackable *getTrackable() {return track;}
  void setTerrain(Terrain *t) {terrain = t;}
  void update(Ogre::Real tdelta);
  void setPosition(const Ogre::Vector3 &newpos);
  void setLookAt(const Ogre::Vector3 &newlook);
  const Ogre::Vector3 &getPosition() const;
  Ogre::Vector3 getDirection() const;
};

#endif // CAMERA_H
