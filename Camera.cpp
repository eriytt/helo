#include "Camera.h"

#include "Terrain.h"

void Camera::setTrackable(HeloUtils::Trackable *t)
{
  track = t;

  if (not t) {
    cam->setAutoTracking(false);
    return;
  }

  Ogre::SceneNode *sn = t->getSceneNode();
  cam->setAutoTracking(true, sn, t->getTrackOffset());
  cam->setFixedYawAxis(true, t->getCameraUp());
}

void Camera::update(Ogre::Real tdelta)
{
  if (not (track and track->cameraFollow()))
    return;

  Ogre::SceneNode *sn = track->getSceneNode();
  Ogre::Vector3 current = cam->getPosition();
  Ogre::Vector3 desired = sn->convertLocalToWorldPosition(track->getCameraPosition());
  Ogre::Vector3 error = desired - current;
  HeloUtils::Trackable::CameraParams params = track->getCameraParameters();
  Ogre::Vector3 newpos = current + (error * params.p * tdelta);

  if (terrain)
    {
      // TODO: compensate for ground level softly
      Ogre::Real terrain_height = terrain->getHeight(newpos.x, newpos.z);
      if (newpos.y < terrain_height + params.minGroundOffset)
	newpos.y = terrain_height + params.minGroundOffset;
    }

  cam->setPosition(newpos);
}

void Camera::setPosition(const Ogre::Vector3 &newpos)
{
  if (track)
    return;

  cam->setPosition(newpos);
}

void Camera::setLookAt(const Ogre::Vector3 &newlook)
{
  if (track)
    return;

  cam->lookAt(newlook);
}

const Ogre::Vector3 &Camera::getPosition() const
{
  return cam->getPosition();
}

Ogre::Vector3 Camera::getDirection() const
{
  return cam->getDirection();
}
