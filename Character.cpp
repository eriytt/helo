#include "Character.h"

AnimAction::AnimAction(std::string action_name)
{
  name = action_name;
  trans = Ogre::Vector3::ZERO;
}

void AnimAction::update(Ogre::Real tdelta, Ogre::SceneNode *node)
{
  node->translate(trans * tdelta);
}

AnimQueue::AnimQueue(Ogre::Entity *entity, AnimAction &start_action) :
  ent(entity), currentAnim(*(ent->getAnimationState(start_action.getName())), start_action)
{
  currentAnim.state.setEnabled(true);
  currentAnim.state.setLoop(false);
}

void AnimQueue::push(AnimAction &action)
{
  Ogre::AnimationState *astate = ent->getAnimationState(action.getName());
  queue.push(Anim(*astate, action));
}

void AnimQueue::update(Ogre::Real tdelta, Ogre::SceneNode *node)
{
  if (currentAnim.state.hasEnded())
    if (not queue.empty())
      {
	// Stop the old animation
	currentAnim.state.setEnabled(false);

	currentAnim = queue.front();
	queue.pop();

	// Start the new animation
	currentAnim.state.setTimePosition(0.0);
	currentAnim.state.setLoop(false);
	currentAnim.state.setEnabled(true);
      }

  float rate = 0.1 * 10;
  currentAnim.state.addTime(tdelta * rate);
  currentAnim.action.update(tdelta * rate, node);
}

Character::Character(Ogre::Root *root)
{
  Ogre::SceneManager *mgr = root->getSceneManager("SceneManager");

  ent = mgr->createEntity("Soldier_ent", "Soldier.mesh");
  node = mgr->getRootSceneNode()->createChildSceneNode("Soldier_node");
  node->setPosition(1785.0, 0.0, 1600.0);
  node->attachObject(ent);


  Ogre::Entity *went = mgr->createEntity("SoldierWeapon_ent", "M4_M203.mesh");
  ent->attachObjectToBone("Bone.002_L.004", went);


  AnimAction *pi = new AnimAction("PatrolIdle");
  AnimAction *pv = new AnimAction("PatrolVigilant");
  AnimAction *pvw = new AnimAction("PatrolVigilantWalk");
  AnimAction *pvwen = new AnimAction("PatrolVigilantWalkEnt");
  AnimAction *pvwex = new AnimAction("PatrolVigilantWalkEx");

  pi->addSuccessor(pv);

  pv->setTranslation(Ogre::Vector3(0.0, 0.0, 0.4));
  pv->addSuccessor(pvwen);

  pvwen->setTranslation(Ogre::Vector3(0.0, 0.0, 1.2));
  pvwen->addSuccessor(pvw);

  pvw->setTranslation(Ogre::Vector3(0.0, 0.0, 1.6));
  pvw->addSuccessor(pvw);
  pvw->addSuccessor(pvwex);

  pvwex->setTranslation(Ogre::Vector3(0.0, 0.0, 1.4));


  animQueue = new AnimQueue(ent, *pi);
  animQueue->push(*pv);
  animQueue->push(*pvwen);
  animQueue->push(*pvw);
  animQueue->push(*pvw);
  animQueue->push(*pvw);
  animQueue->push(*pvw);
  animQueue->push(*pvw);
  animQueue->push(*pvw);
  animQueue->push(*pvw);
  animQueue->push(*pvw);
  animQueue->push(*pvw);
  animQueue->push(*pvw);
  animQueue->push(*pvw);
  animQueue->push(*pvw);
  animQueue->push(*pvwex);

  animQueue->push(*pvwen);
  animQueue->push(*pvw);
  animQueue->push(*pvw);
  animQueue->push(*pvw);
  animQueue->push(*pvw);
  animQueue->push(*pvw);
  animQueue->push(*pvwex);

  animQueue->push(*pvwen);
  animQueue->push(*pvw);
  animQueue->push(*pvw);
  animQueue->push(*pvw);
  animQueue->push(*pvw);
  animQueue->push(*pvw);
  animQueue->push(*pvwex);
}


void Character::update(Ogre::Real tdelta)
{
  animQueue->update(tdelta, node);
}
