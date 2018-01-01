#include "Physics.h"

#include <map>

using namespace std;
using namespace Ogre;

#ifdef USE_TSX

#define _XABORT_TOTAL 0x4711
#define _XABORT_SUCCESS 0x4712
#define _SYNC_SUCCESS 0x4713
#define _SYNC_TRIES 0X4714
#define _SYNC_FAIL 0x4715


static std::map<unsigned int, unsigned int> aborts = {
  {_XABORT_CONFLICT , 0},
  {_XABORT_RETRY , 0},
  {_XABORT_EXPLICIT , 0},
  {_XABORT_CAPACITY , 0},
  {_XABORT_DEBUG , 0},
  {_XABORT_NESTED , 0},
  {_XABORT_SUCCESS , 0},
  {_XABORT_TOTAL , 0},
  {_SYNC_TRIES , 0},
  {_SYNC_SUCCESS , 0},
  {_SYNC_FAIL , 0},
};

static void update_aborts(unsigned int stat)
{
  if(stat & _XABORT_CONFLICT)
    aborts[_XABORT_CONFLICT]++;
  if(stat & _XABORT_RETRY)
    aborts[_XABORT_RETRY]++;
  if(stat & _XABORT_EXPLICIT)
    aborts[_XABORT_EXPLICIT]++;
  if(stat & _XABORT_CAPACITY)
    aborts[_XABORT_CAPACITY]++;
  if(stat & _XABORT_DEBUG)
    aborts[_XABORT_DEBUG]++;
  if(stat & _XABORT_NESTED)
    aborts[_XABORT_NESTED]++;

  aborts[_XABORT_TOTAL]++;
}

static void clear_aborts()
{
  aborts[_XABORT_CONFLICT] = 0;
  aborts[_XABORT_RETRY] = 0;
  aborts[_XABORT_EXPLICIT] = 0;
  aborts[_XABORT_CAPACITY] = 0;
  aborts[_XABORT_DEBUG] = 0;
  aborts[_XABORT_NESTED] = 0;
  aborts[_XABORT_TOTAL] = 0;
  aborts[_XABORT_SUCCESS] = 0;
  aborts[_SYNC_TRIES] = 0;
  aborts[_SYNC_FAIL] = 0;
  aborts[_SYNC_SUCCESS] = 0;
}

void print_aborts()
{
  std::cout << "Conflict: " <<   aborts[_XABORT_CONFLICT] << std::endl
            << "Retry:    " <<   aborts[_XABORT_RETRY] << std::endl
            << "Explicit: " <<   aborts[_XABORT_EXPLICIT] << std::endl
	    << "Capacity: " <<   aborts[_XABORT_CAPACITY] << std::endl
	    << "Debug:    " <<   aborts[_XABORT_DEBUG] << std::endl
	    << "Nested:   " <<   aborts[_XABORT_NESTED] << std::endl
	    << "Total   : " <<   aborts[_XABORT_TOTAL] << std::endl
  	    << "Success : " <<   aborts[_XABORT_SUCCESS] << std::endl
	    << "Sync attempts: " <<   aborts[_SYNC_TRIES] << std::endl
    	    << "Sync fail    : " <<   aborts[_SYNC_FAIL] << std::endl
    	    << "Sync success : " <<   aborts[_SYNC_SUCCESS] << std::endl;
  clear_aborts();
}

void *HeloMotionState::operator new(size_t size)
  {
    size_t bufsize = sizeof(HeloMotionState) + CACHE_LINE_SIZE;;
    void *buf = ::operator new(bufsize);
    void *objptr = std::align(CACHE_LINE_SIZE,
			      sizeof(HeloMotionState),
			      buf, bufsize);

    if (objptr == nullptr)
      throw bad_align("Alignment of motion state data failed");

    void *transform_ptr = &(static_cast<HeloMotionState*>(objptr))->committedTrans;
    size_t transform_addr = reinterpret_cast<size_t>(transform_ptr);
    if (transform_addr & (CACHE_LINE_SIZE - 1))
      throw bad_align("Transform of motion state not aligned to cache line");
    
    static_cast<HeloMotionState*>(objptr)->bufptr = buf;

    return objptr;
  }

void HeloMotionState::operator delete(void *ptr)
{
  void *buf = static_cast<HeloMotionState*>(ptr)->bufptr;
  ::operator delete(buf);
}

#include "Serialize.hh"
std::ofstream serstream("serialize.txt");
ByteStream serializer(serstream);


void HeloMotionState::updateSceneNode()
{
  if (not (snode && committed))
    return;
  
  btTransform t;
  unsigned int retries = HeloMotionState::TRANSACTION_RETRY_COUNT;
  aborts[_SYNC_TRIES]++;
 transaction_retry:
  unsigned int xstat = _xbegin();

  // In transaction context:
  if (xstat == _XBEGIN_STARTED)
    {
      if (updating)
	_xabort(1);
      //t = committedTrans;
      t = committedTrans;
      committed = false;
      _xend();
    }
  else
    {
      update_aborts(xstat);
      
      if ((xstat & _XABORT_CONFLICT
	   or xstat & _XABORT_RETRY
	   or xstat & _XABORT_EXPLICIT)
	  and retries)
	{
	  --retries;
	  goto transaction_retry;
	}

      //std::cout << "Skipping synchronization" << std::endl;
      aborts[_SYNC_FAIL]++;
      // On fatal abort or max retry count reached, skip the update:
      return;
    }
      

  // After successful commit, proceed to update:
  //std::cout << "Successful synchronization" << std::endl;
  aborts[_XABORT_SUCCESS]++;
  aborts[_SYNC_SUCCESS]++;
  btQuaternion rot = t.getRotation();
  btVector3 pos = t.getOrigin();
  snode->setOrientation(rot.w(), rot.x(), rot.y(), rot.z());
  snode->setPosition(pos.x(), pos.y(), pos.z());
  serializer << reinterpret_cast<void*>(this) << snode->_getFullTransform();
  
}
#else
void HeloMotionState::updateSceneNode()
{
  if (not (snode && dirty))
    return;
  btQuaternion rot = worldTrans.getRotation();
  btVector3 pos = worldTrans.getOrigin();
  snode->setOrientation(rot.w(), rot.x(), rot.y(), rot.z());
  snode->setPosition(pos.x(), pos.y(), pos.z());
  dirty = false;
}
#endif // USE_TSX

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
  // No need to stop physics thread with tsx.
#ifndef USE_TSX
  bool thread_was_running = false;
  if (runInThread)
    thread_was_running = physicsThread->isRunning();

  if (thread_was_running)
    physicsThread->stop();
#endif

  for (MSIter i = motionStates.begin(); i != motionStates.end(); ++i)
    (*i)->updateSceneNode();

#ifndef USE_TSX
  if (thread_was_running)
    physicsThread->start();
#endif
}

void Physics::internalStep(float timeSlice)
{
  for (PhysObjIter i = pobjects.begin(); i != pobjects.end(); ++i)
    (*i)->physicsUpdate(timeSlice);
  world->stepSimulation(timeSlice, 10);

  //  std::cout << "Committing all transforms" << std::endl;

  for (auto ms : motionStates)
    ms->commitTransform();
}

void Physics::work()
{
  float udt = clock.getTimeMicroseconds();
  clock.reset();
  internalStep(udt * 0.000001f);
  //  figure out how long the timestep should be
  //  internalStep(timestep);
}


void Physics::stop()
{
  if (runInThread and physicsThread->isRunning())
    physicsThread->stop();
}

void Physics::resume()
{
  if (runInThread and (not physicsThread->isRunning()))
      physicsThread->resume();
}
