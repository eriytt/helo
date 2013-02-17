#include "Airplane.h"

// Car::CarData Airplane::AirplaneToCarData(const AirplaneData &data)
// {
//   Car::CarData cd;
//   cd.name = data.name;
//   cd.meshname = data.meshname;
//   cd.position = data.position;
//   cd.size = data.size;
//   cd.weight = data.weight;
  
//   cd.wheelData.resize(data.wheelData.size());

//   for (size_t i = 0; i < data.wheelData.size(); ++i)
//     cd.wheelData[i] = data.wheelData[i];
  
//   return cd;
// }

Airplane::Airplane(const AirplaneData &data, Ogre::Root *root)
{
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
  airplane = new AirplaneVehicle(body);
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

void Airplane::setThrottle(Ogre::Real fraction)
{
  airplane->setThrust(fraction * 100000.0);
}


void AirplaneVehicle::updateAction(btCollisionWorld* collisionWorld, btScalar timeStep)
{
  CBRaycastVehicle::updateAction(collisionWorld, timeStep);

  // TODO: use apply central impulse
  HeloUtils::LocalApplyImpulse(chassisBody, btVector3(0.0, 0.0, thrust * timeStep),
                               btVector3(0.0, 0.0, 0.0));

  const btVector3 &vel = chassisBody->getLinearVelocity();
  const btMatrix3x3 &inv_trans = chassisBody->getCenterOfMassTransform().getBasis().inverse();
  // This is the component of the velocity that goes in the forward direction
  const btScalar vel_forward = (inv_trans * vel).dot(btVector3(0.0, 0.0, 1.0));
  HeloUtils::LocalApplyImpulse(chassisBody,
                               btVector3(0.0,
                                         vel_forward * 1700.0 * timeStep,
                                         0.0),
                               btVector3(0.0, 0.0, 0.0));

  HeloUtils::LocalApplyImpulse(chassisBody,
                               btVector3(0.0,
                                         0.0,
                                         HeloUtils::POW2(vel_forward) * -9.0 * timeStep),
                               btVector3(0.0, 0.0, 0.0));

}

void AirplaneVehicle::setThrust(btScalar t)
{
  thrust = t;
}
