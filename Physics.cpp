#include "Physics.h"
using namespace std;
using namespace Ogre;

btRigidBody* Physics::CreateRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape,
				      SceneNode *node, btVector3 localInertia)
{
  bool isDynamic = (mass != 0.f);

  if (isDynamic and shape != NULL)
    shape->calculateLocalInertia(mass, localInertia);

  // TODO: Bullet seems to die if there is no shape.
  //       This empty shape should perhaps be freed somewhere
  //       Make sure not to calculate its inertia tensor, because it asserts
  if (shape == NULL)
    shape = new btEmptyShape();

  btMotionState *ms = new HeloMotionState(startTransform, node);

  btRigidBody::btRigidBodyConstructionInfo cInfo(mass, ms, shape, localInertia);

  btRigidBody* body = new btRigidBody(cInfo);
  body->setContactProcessingThreshold(BT_LARGE_FLOAT);

  return body;
}

Physics::Physics(Ogre::AxisAlignedBox bbox, bool inThread) : runInThread(inThread)
{

  btDefaultCollisionConfiguration *collisionConfiguration = new btDefaultCollisionConfiguration();
  btCollisionDispatcher *dispatcher = new btCollisionDispatcher(collisionConfiguration);

  /*const Vector3 &min = bbox.getMinimum();
  const Vector3 &max = bbox.getMaximum();
  btVector3 worldMin(min.x, min.y, min.z);
  btVector3 worldMax(max.x, max.y, max.z);
  btAxisSweep3 *overlappingPairCache = new btAxisSweep3(worldMin,worldMax);*/
  overlappingPairCache = new btDbvtBroadphase();

  btSequentialImpulseConstraintSolver *constraintSolver = new btSequentialImpulseConstraintSolver();
  world = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, constraintSolver, collisionConfiguration);

  world->setGravity(btVector3(0.0f, -9.81f, 0.0f));

  clock.reset();

  if (runInThread)
    physicsThread = new Thread(this);
}

Physics::~Physics()
{
  if (runInThread)
    delete physicsThread;

  for (BodyIter b = bodies.begin(); b != bodies.end(); ++b)
    delete *b;

  delete world;

  // Apparently, Bullet frees these when the world i deleted
  //delete collisionConfiguration;
  //delete dispatcher;
  //delete overlappingPairCache;

  // TODO: Examining the bullet source code, we should be able to free this
  //delete constraintSolver;
}

void Physics::addBody(btRigidBody *body)
{
  bodies.push_back(body);
  motionStates.push_back(dynamic_cast<HeloMotionState*>(body->getMotionState()));
  // TODO: stop simulation before adding a body
  world->addRigidBody(body);
}

void Physics::addMotionState(HeloMotionState *ms)
{
  motionStates.push_back(ms);
}

void Physics::addConstraint(btTypedConstraint* constraint, bool disableCollisionsBetweenLinkedBodies)
{
  // TODO: stop simulation before adding a body
  world->addConstraint(constraint, disableCollisionsBetweenLinkedBodies);
}

void Physics::addAction(btActionInterface *action)
{
  world->addAction(action);
}

void Physics::addObject(PhysicsObject *obj)
{
  pobjects.push_back(obj);
}

void Physics::finishConfiguration()
{
  for (PhysObjIter i = pobjects.begin(); i != pobjects.end(); ++i)
    (*i)->finishPhysicsConfiguration(this);
}

btRigidBody* sphere;

btRigidBody *Physics::testSphere(const Ogre::Vector3 &pos, Ogre::Real size, Ogre::SceneNode *node)
{
  btCollisionShape* shape = new btSphereShape(size);
  btTransform tr;
  tr.setIdentity();
  tr.setOrigin(btVector3(pos.x, pos.y, pos.z));

  sphere = CreateRigidBody(100, tr, shape, node);
  return sphere;
}

void Physics::step()
{
  if (runInThread)
    {
      physicsThread->resume();
      return;
    }
  internalStep(0.01);
}

void Physics::sync()
{
  bool thread_was_running = false;
  if (runInThread)
    thread_was_running = physicsThread->isRunning();

  if (thread_was_running)
    physicsThread->stop();

  for (MSIter i = motionStates.begin(); i != motionStates.end(); ++i)
    (*i)->updateSceneNode();

  if (thread_was_running)
    physicsThread->start();
}

void Physics::internalStep(float timeSlice)
{
  world->stepSimulation(timeSlice, 10);
  for (PhysObjIter i = pobjects.begin(); i != pobjects.end(); ++i)
    (*i)->physicsUpdate();

  //sphere->applyCentralForce(btVector3(300.0, 900.0, 0.0));
}

void Physics::work()
{
  float udt = clock.getTimeMicroseconds();
  clock.reset();
  internalStep(udt * 0.000001f);
  //  figure out how long the timestep should be
  //  internalStep(timestep);
}
