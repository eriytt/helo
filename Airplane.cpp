#include "Airplane.h"

AirplaneVehicle::AirplaneData Airplane::DataToAirplaneVehicleData(const AirplaneData &data)
{
  AirplaneVehicle::AirplaneData d;
  d.dragPolarK = data.dragPolarK;

  for (size_t i = 0; i < data.engineData.size(); ++i)
    {
      const Engine &e = data.engineData[i];
      AirplaneVehicle::Engine ve(HeloUtils::Ogre2BulletVector(e.position),
                                 HeloUtils::Ogre2BulletVector(e.direction),
                                 e.maxThrust);
      d.engines.push_back(ve);
    }  

  for (size_t i = 0; i < data.cl_alpha_values.size(); ++i)
    d.clAlpha.addDataPoint(data.cl_alpha_values[i].first,
                           data.cl_alpha_values[i].second);

  return d;
}

Airplane::Airplane(const AirplaneData &data, Ogre::Root *root)
{
  // TODO: init control data
  Ogre::SceneManager *mgr = root->getSceneManager("SceneManager");

  // create entity
  Ogre::Entity *ent = mgr->createEntity(data.name + "_ent", data.meshname);

  //ent->setMaterialName("shadow", "Terrain/Default");

  // create scene node
  node = mgr->getRootSceneNode()->createChildSceneNode(data.name + "_node");
  node->attachObject(ent);

  // create collision shape
  btCollisionShape* chassis_shape = new btBoxShape(btVector3(data.size.x / 2.0,
							     data.size.y / 2.0,
							     data.size.z / 2.0));
  btCompoundShape *comp = new btCompoundShape();

  // Transform of hull collision shape
  btTransform chassis_shape_trans;
  chassis_shape_trans.setIdentity();
  chassis_shape_trans.setOrigin(btVector3(0.0, 0.0, 0.0));
  comp->addChildShape(chassis_shape_trans, chassis_shape);
  shape = comp;

  // create fuselage rigid body
  btTransform tr;
  tr.setIdentity();
  tr.setOrigin(btVector3(data.position.x, data.position.y, data.position.z));

  body = Physics::CreateRigidBody(data.weight, tr, shape, node);
  airplane = new AirplaneVehicle(body, DataToAirplaneVehicleData(data));
  rayCastVehicle = airplane;

  
  for (unsigned int i = 0; i < data.wheelData.size(); ++i)
    {
      const WheelData &wd = data.wheelData[i];
      Ogre::Entity *tent = mgr->createEntity(data.name + "tire" + Ogre::String(1, static_cast<char>(i + 39)) + "_ent", "a10-tire.mesh");
      Ogre::SceneNode *tnode = node->createChildSceneNode(data.name + "tire" + Ogre::String(1, static_cast<char>(i + 39)) + "_node");
      tnode->attachObject(tent);
      btVector3 wheelpos(wd.relPos + (wd.direction * wd.suspensionLength));
      tnode->setPosition(Ogre::Vector3(wheelpos.x(), wheelpos.y(), wheelpos.z()));
      wheelNodes.push_back(tnode);
      rayCastVehicle->addWheel(wd);
    }
}

void AirplaneVehicle::applyThrust(btScalar timeStep)
{
  // TODO: use apply central impulse
  //unsigned short num_engines = d.engines.size();
  btScalar thrust(0);
  for (std::vector<Engine>::const_iterator i = d.engines.begin(); i != d.engines.end(); ++i)
    {
      const Engine *e = &(*i);
      thrust += controlData.thrust * e->maxThrust;
    }

  HeloUtils::LocalApplyImpulse(chassisBody, btVector3(0.0, 0.0, thrust * timeStep),
                               btVector3(0.0, 0.0, 0.0));
}

void AirplaneVehicle::applyLift(btScalar timeStep, btScalar velocityForward)
{
  HeloUtils::LocalApplyImpulse(chassisBody,
                               btVector3(0.0,
                                         velocityForward * 1700.0 * timeStep,
                                         0.0),
                               btVector3(0.0, 0.0, 0.0));
}

void AirplaneVehicle::applyDrag(btScalar timeStep, btScalar velocityForward)
{
  HeloUtils::LocalApplyImpulse(chassisBody,
                               btVector3(0.0,
                                         0.0,
                                         HeloUtils::POW2(velocityForward) * -9.0 * timeStep),
                               btVector3(0.0, 0.0, 0.0));
}

void AirplaneVehicle::applyRudders(btScalar timeStep, btScalar velocityForward)
{
}

void AirplaneVehicle::updateAction(btCollisionWorld* collisionWorld, btScalar timeStep)
{
  CBRaycastVehicle::updateAction(collisionWorld, timeStep);
  
  // btScalar wing_span(18);
  // btScalar wing_area(36.0);
  // btScalar aspect_ratio(HeloUtils::POW2(wing_span) / wing_area);
  // btScalar airplane_efficiency_factor(0.7);
  // btScalar K(1.0/ (HeloUtils::PI * aspect_ratio * airplane_efficiency_factor));
  


  const btVector3 &vel = chassisBody->getLinearVelocity();
  const btMatrix3x3 &inv_trans = chassisBody->getCenterOfMassTransform().getBasis().inverse();
  // This is the component of the velocity that goes in the forward direction
  const btScalar vel_forward = (inv_trans * vel).dot(btVector3(0.0, 0.0, 1.0));
  applyThrust(timeStep);
  applyLift(timeStep, vel_forward);
  applyDrag(timeStep, vel_forward);
  applyRudders(timeStep, vel_forward);
}

void AirplaneVehicle::setInput(const AirplaneVehicle::ControlData &cd)
{
  // Does it matter ? 
#warning Not thread safe, AirplaneVehicle lives in physics thread
  controlData = cd;
}

void Airplane::setThrottle(Ogre::Real fraction)
{
  controlData.thrust = fraction;
  airplane->setInput(controlData);
}

