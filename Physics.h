#ifndef PHYSICS_H
#define PHYSICS_H

#include <vector>

#include <Ogre.h>

#include <btBulletDynamicsCommon.h>

#include "Thread.h"

class HeloMotionState : public btMotionState
{
protected:
  btTransform worldTrans;
  btTransform offset;
  Ogre::SceneNode *snode;
  bool dirty;

public:
  HeloMotionState(const btTransform& startTrans = btTransform::getIdentity(),
		  Ogre::SceneNode *node = 0,
		  const btTransform& centerOfMassOffset = btTransform::getIdentity())
    : worldTrans(startTrans),
      offset(centerOfMassOffset),
      snode(node),
      dirty(false) {}

  virtual ~HeloMotionState() {}

  void setNode(Ogre::SceneNode *node) {
    snode = node;
  }

  virtual void getWorldTransform(btTransform &trans) const {
    trans = worldTrans;
  }

  virtual void setWorldTransform(const btTransform &trans) {
    if(snode == 0)
      return;
    worldTrans = trans;
    dirty = true;
  }

  virtual void updateSceneNode()
  {
    if (not (snode && dirty))
      return;
    btQuaternion rot = worldTrans.getRotation();
    btVector3 pos = worldTrans.getOrigin();
    snode->setOrientation(rot.w(), rot.x(), rot.y(), rot.z());
    snode->setPosition(pos.x(), pos.y(), pos.z());
    dirty = false;
  }
};


class PhysicsObject
{
public:
  virtual void finishPhysicsConfiguration(class Physics *phys) = 0;
  virtual void physicsUpdate(float step) = 0;
};

typedef std::vector < PhysicsObject* > PhysObjVector;
typedef std::vector < PhysicsObject* > ::iterator PhysObjIter;


class Physics : public ThreadWorker
{
protected:
  typedef std::vector<btRigidBody*> BodyVector;
  typedef std::vector<btRigidBody*>::iterator BodyIter;

  typedef std::vector<HeloMotionState*> MSVector;
  typedef std::vector<HeloMotionState*>::iterator MSIter;

protected:
  Thread *physicsThread;

protected:
  btDiscreteDynamicsWorld *world;
  btDefaultCollisionConfiguration *collisionConfiguration;
  btCollisionDispatcher *dispatcher;
  //btAxisSweep3 *overlappingPairCache;
  btDbvtBroadphase *overlappingPairCache;
  btSequentialImpulseConstraintSolver *constraintSolver;
  btClock clock;
  BodyVector bodies;
  MSVector motionStates;
  bool runInThread;

  PhysObjVector pobjects;

protected:
  void internalStep(float timeSlice);

public:
  Physics(Ogre::AxisAlignedBox bbox, bool inThread);
  virtual ~Physics();
  // TODO: add BodyVector version
  void addBody(btRigidBody *body);
  void addMotionState(HeloMotionState *ms);
  void addConstraint(btTypedConstraint* constraint, bool disableCollisionsBetweenLinkedBodies = false);
  void addAction(btActionInterface *action);
  void addObject(PhysicsObject *obj);
  void finishConfiguration();
  btRigidBody *testSphere(const Ogre::Vector3 &pos, Ogre::Real size, Ogre::SceneNode *node);
  void step();
  void sync();
  void work();
  btDynamicsWorld *getWorld() {return world;}

  static btRigidBody* CreateRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape,
				      Ogre::SceneNode *node = 0, btVector3 localInertia = btVector3(0, 0, 0));
};

#endif // PHYSICS_H
