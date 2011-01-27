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

AnimQueue::AnimQueue(Ogre::Entity *entity, AnimQueue::Source &s) : source(s), ent(entity)
{
  reset();
}

Ogre::Real AnimQueue::AnimChannel::push(AnimAction &action, Ogre::Real starttime, Ogre::Entity *ent)
{
  Ogre::AnimationState *astate = ent->getAnimationState(action.getName());
  float stoptime = starttime + astate->getLength();
  queue.push(Anim(*astate, action, starttime, stoptime));
  std::cout << "Pushing animation: " << action.getName() << " at " << starttime << " - " << stoptime << std::endl;
  return stoptime;
}

bool AnimQueue::AnimChannel::isFree(float time)
{
  if (queue.empty())
    return true;
  return queue.back().stop <= time;
}

void AnimQueue::AnimChannel::runChannel(Ogre::Real tdelta, Ogre::Real abstime, Ogre::SceneNode *node)
{
  if (queue.empty())
    return;

  float rate = 0.1 * 10.0;
  abstime *= rate;

  if (queue.front().start > abstime)
    return;

  if (not queue.front().state.getEnabled())
    {
      std::cout << "Starting animation: " << queue.front().action.getName() << ":" << queue.front().start
		<< "  abstime:" << abstime << std::endl;
      queue.front().state.setEnabled(true);
      queue.front().state.setLoop(false);
      queue.front().state.setTimePosition(0.0);
    }

  if (queue.front().state.hasEnded())
    {
      queue.front().state.setEnabled(false);
      if (queue.size() == 1)
	source.channelEmpty(queue.front().action, queue.front().stop);

      queue.pop();
      runChannel(tdelta, abstime, node);
      return;
    }

  queue.front().state.addTime(tdelta * rate);
  queue.front().action.update(tdelta * rate, node);
}

Ogre::Real AnimQueue::push(AnimAction &action)
{
  return push(action, timer.getMicroseconds() / Ogre::Real(1000000));
}

Ogre::Real AnimQueue::push(AnimAction &action, Ogre::Real starttime)
{
  // Find a free queue or create and push action on that queue
  for (size_t c = 0; c < channels.size(); ++c)
    if (channels[c].isFree(starttime))
	return channels[c].push(action, starttime, ent);


  // No available queue, create a new
  channels.push_back(AnimChannel(source));
  return channels.back().push(action, starttime, ent);
}

void AnimQueue::update(Ogre::SceneNode *node)
{
  unsigned long frame_time = timer.getMicroseconds();
  Ogre::Real tdelta = (frame_time - lastFrameTime_us) / Ogre::Real(1000000);
  lastFrameTime_us = frame_time;
  Ogre::Real abstime = frame_time / Ogre::Real(1000000);

  for (size_t c = 0; c < channels.size(); ++c)
    channels[c].runChannel(tdelta, abstime, node);
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


  animQueue = new AnimQueue(ent, *this);
  Ogre::Real stoptime = animQueue->push(*pi);
  stoptime = animQueue->push(*pv, stoptime);
  stoptime = animQueue->push(*pvwen, stoptime);
  stoptime = animQueue->push(*pvw, stoptime);
  stoptime = animQueue->push(*pvw, stoptime);
  stoptime = animQueue->push(*pvw, stoptime);
  animQueue->push(*pvw, stoptime);
  animQueue->push(*pi, stoptime);
  // animQueue->push(*pvw);
  // animQueue->push(*pvw);
  // animQueue->push(*pvw);
  // animQueue->push(*pvw);
  // animQueue->push(*pvw);
  // animQueue->push(*pvw);
  // animQueue->push(*pvw);
  // animQueue->push(*pvw);
  // animQueue->push(*pvw);
  // animQueue->push(*pvw);
  // animQueue->push(*pvw);
  // animQueue->push(*pvwex);

  // animQueue->push(*pvwen);
  // animQueue->push(*pvw);
  // animQueue->push(*pvw);
  // animQueue->push(*pvw);
  // animQueue->push(*pvw);
  // animQueue->push(*pvw);
  // animQueue->push(*pvwex);

  // animQueue->push(*pvwen);
  // animQueue->push(*pvw);
  // animQueue->push(*pvw);
  // animQueue->push(*pvw);
  // animQueue->push(*pvw);
  // animQueue->push(*pvw);
  // animQueue->push(*pvwex);
}


void Character::update(Ogre::Real tdelta)
{
  animQueue->update(node);
}

void Character::channelEmpty(const AnimAction &last_action, Ogre::Real stoptime)
{
  std::cout << "Some channel emptied, last action played was " << last_action.getName() << std::endl;
}
