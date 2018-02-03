#ifndef PHYSICS_H
#define PHYSICS_H

#include <immintrin.h>

#include <vector>
#include <atomic>

#include <Ogre.h>

#include <btBulletDynamicsCommon.h>

#include "Thread.h"

void print_aborts();

class HeloMotionState : public btMotionState
{
public:
  class bad_align : public std::bad_alloc
  {
    std::string msg;
  public:
    bad_align(const std::string &msg) : msg(msg) {}
    virtual const char* what() const throw() {return msg.c_str();}
  };

protected:
  static const unsigned int TRANSACTION_RETRY_COUNT = 3;
  // TODO: allow this constant to be configurable
  static const size_t CACHE_LINE_SIZE = 64;

#ifdef USE_TSX
private:
  void *bufptr;

protected:
  bool dirty;
  std::atomic<bool> committed;
  std::atomic<bool> updating;
  alignas(CACHE_LINE_SIZE) btTransform committedTrans;

public:
  void *operator new(size_t size);
  void operator delete(void *ptr);
#else
protected:
  bool dirty;
#endif

protected:
  btTransform worldTrans;

  btTransform offset;
  Ogre::SceneNode *snode;

public:
  HeloMotionState(const btTransform& startTrans = btTransform::getIdentity(),
		  Ogre::SceneNode *node = 0,
		  const btTransform& centerOfMassOffset = btTransform::getIdentity())
    : dirty(false),
#ifdef USE_TSX
      committed(false),
      updating(false),
      committedTrans(startTrans),
#endif
      worldTrans(startTrans),
      offset(centerOfMassOffset),
      snode(node) {}
  virtual ~HeloMotionState() {}
    
  void setNode(Ogre::SceneNode *node) {snode = node;}
  Ogre::SceneNode *getNode() const {return snode;}
  virtual void getWorldTransform(btTransform &trans) const {trans = worldTrans;}
  virtual void setWorldTransform(const btTransform &trans)
  {
    if(snode == 0)
      return;

    worldTrans = trans;
    dirty = true;
  }

  virtual void commitTransform()
  {
    if (not dirty)
      return;

#ifdef USE_TSX
    updating = true;
    committedTrans = worldTrans;
    committed = true;
    updating = false;
#endif
    dirty = false;
  }

#ifdef USE_TSX
  bool tsxGetTransform(btTransform &t);
#endif
  bool getTransform(btTransform &t) const;
  void updateSceneNode();
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
public:
  class Listener {
  public:
    virtual void motionStateAdded(const HeloMotionState &ms) = 0;
  };

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

  std::vector<Listener*> listeners;

protected:
  void internalStep(float timeSlice);

public:
  Physics(Ogre::AxisAlignedBox bbox, bool inThread);
  virtual ~Physics();
  // TODO: add BodyVector version
  void addBody(btRigidBody *body);
  void addMotionState(HeloMotionState *ms);
  void addListener(Listener *l) {listeners.push_back(l);}
  void addConstraint(btTypedConstraint* constraint, bool disableCollisionsBetweenLinkedBodies = false);
  void addAction(btActionInterface *action);
  void addObject(PhysicsObject *obj);
  void finishConfiguration();
  btRigidBody *testSphere(const Ogre::Vector3 &pos, Ogre::Real size, Ogre::SceneNode *node);
  void step();
  void sync();
  void work();
  void stop();
  void resume();
  btDynamicsWorld *getWorld() {return world;}

  static btRigidBody* CreateRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape,
				      Ogre::SceneNode *node = 0, btVector3 localInertia = btVector3(0, 0, 0));
};

#endif // PHYSICS_H
