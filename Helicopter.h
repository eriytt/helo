#ifndef HELICOPTER_H
#define HELICOPTER_H

#include <Ogre.h>
#include <btBulletDynamicsCommon.h>

#include "Physics.h"
#include "Utils.h"
#include "Controllers.h"

class Rotor
{
public:
  typedef struct
  {
    float weight;
    float diameter;
    Ogre::Vector3 relpos;
    Ogre::Vector3 axis;
    Ogre::Vector3 rotation_axis;
    float torque; // TODO: replace with torque function of rps
    float inertia;
    float maxLift;
    float tiltSensitivity;
  } RotorData;

  typedef std::vector<RotorData> RotorDataList;
  typedef std::vector<RotorData>::iterator RotorDataIter;
  typedef std::vector<RotorData>::const_iterator RotorDataCIter;

protected:
  float swash;
  float swash_right;
  float swash_forward;
  Ogre::Vector3 axis;
  Ogre::Vector3 rotationAxis;
  Ogre::Radian rotation;
  btScalar tiltSensitivity;
  Ogre::Vector3 pos;
  float diameter;
  float torque;
  float revolutionsPerSecond;
  float inertia;
  float maxLift;
  float equilibriumLift;
  float weight;
  float appliedTorque;
  bool started;

public:
  Rotor(const RotorData &data, const Ogre::Vector3 &parent_pos, btRigidBody &parent_body, Ogre::Root *root);
  void applyForcesAndTorques(btRigidBody *parent_body, float step);
  float getWeight() {return weight;}
  void start() {started = true;}
  void setSwash(float new_swash, float new_forward, float new_right);
  void setRotation(float radians);
  void setEquilibriumLift(float equilift);
  float getAppliedTorque() {return appliedTorque;}
};


class Helicopter : public PhysicsObject, public Controllable, public HeloUtils::Trackable
{
public:
  typedef struct
  {
    Ogre::String name;
    Ogre::String meshname;
    Ogre::Vector3 size;
    Ogre::Vector3 pos;
    btScalar boomLength;
    btScalar weight;
    Rotor::RotorDataList rotorData;
    btScalar collectiveSensitivity;
    btScalar cyclicRightSensitivity;
    btScalar cyclicForwardSensitivity;
    btScalar steerSensitivity;
  } HelicopterData;

  typedef std::vector<Rotor*> RotorList;
  typedef std::vector<Rotor*>::iterator RotorIter;

protected:
  Ogre::SceneNode *node;
  btCollisionShape *shape;
  btRigidBody *fuselageBody;


  RotorList rotors;

  float collective;
  float cyclic_forward;
  float cyclic_right;
  float collectiveSensitivity;
  float cyclicRightSensitivity;
  float cyclicForwardSensitivity;
  float steerSensitivity;

  btScalar boomLength;
  float steer;

public:
  Helicopter(const HelicopterData &data, Ogre::Root *root);
  void finishPhysicsConfiguration(Physics *phys);
  void physicsUpdate(float step);
  void startEngines(bool start);
  void setCollective(float val) {collective = HeloUtils::unit_clamp(val);} // up/down
  void setCyclic(float forward, float right) // tilt
  {
    cyclic_forward = HeloUtils::unit_clamp(forward);
    cyclic_right = HeloUtils::unit_clamp(right);
  }
  void setCyclicForward(float forward) {cyclic_forward = HeloUtils::unit_clamp(forward);}
  void setCyclicRight(float right) {cyclic_right = HeloUtils::unit_clamp(right);}
  void setSteer(float val) {steer = HeloUtils::unit_clamp(val);}
  Ogre::SceneNode *getSceneNode() {return node;}
  Controller *createController(OIS::Object *dev);
  CameraParams getCameraParameters()
    {
      CameraParams params;
      params.p = 5.0;
      return params;
    }


protected:
  virtual void setRotorInput();
};

class TandemRotorHelicopter : public Helicopter
{
public:
  typedef enum
    {
      FrontRotor = 0,
      BackRotor
    } RotorPosition;

public:
  TandemRotorHelicopter(const HelicopterData &data, Ogre::Root *root) : Helicopter(data, root)
  {
    cameraPosition = (Ogre::Vector3::NEGATIVE_UNIT_Z * 40.0) + (Ogre::Vector3::UNIT_Y * 7.0);
  }

protected:
  virtual void setRotorInput();
};

#endif // HELICOPTER_H
