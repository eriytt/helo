#include "Character.h"

AnimAction::AnimAction(std::string action_name, PlayType t)
  : trans(Ogre::Vector3::ZERO), name(action_name), type(t)
{}

AnimQueue::Anim::Anim(Ogre::AnimationState &s, const AnimAction &a, float starttime, float stoptime) :
  state(s), action(const_cast<AnimAction&>(a)), start(starttime), stop(stoptime)
{
  if (action.getType() != AnimAction::Play)
    stop = start + 0.2;
}

void AnimQueue::Anim::update(Ogre::Real tdelta, Ogre::SceneNode *node, std::map<AnimAction::Modifier::ModType, Ogre::Real> &modifiers)
{
  const std::vector<const AnimAction::Modifier*> &mods = action.getModifiers();
  Ogre::Vector3 translation = action.getTranslation();
  Ogre::Quaternion rotation(Ogre::Quaternion::IDENTITY);

  for (unsigned int i = 0; i < mods.size(); ++i)
    {
      AnimAction::Modifier::ModType mt = mods[i]->getType();
      std::map<AnimAction::Modifier::ModType, Ogre::Real>::iterator iter = modifiers.find(mt);
      if (iter != modifiers.end())
	mods[i]->modify(translation, rotation, node, 0.0, tdelta, (*iter).second);
    }

  if (action.getType() == AnimAction::Play)
    state.addTime(tdelta);
  node->translate(translation * tdelta, Ogre::SceneNode::TS_LOCAL);
  node->rotate(rotation * tdelta);
}

void AnimQueue::Anim::startAnim()
{
  state.setEnabled(true);
  state.setLoop(false);

  if (action.getType() == AnimAction::EndPose)
    state.setTimePosition(state.getLength());
  else
    state.setTimePosition(0.0);
}

void AnimQueue::Anim::stopAnim()
{
  state.setEnabled(false);
}

bool AnimQueue::Anim::isPlaying(Ogre::Real abstime)
{
  return state.getEnabled();
}

bool AnimQueue::Anim::hasEnded(Ogre::Real abstime)
{
  if (action.getType() == AnimAction::Play)
    return state.hasEnded();
  return abstime >= stop;
}


Ogre::Real AnimQueue::AnimChannel::push(const AnimAction &action, Ogre::Real starttime, Ogre::Entity *ent)
{
  Ogre::AnimationState *astate = ent->getAnimationState(action.getName());
  float stoptime = starttime + astate->getLength();
  queue.push(Anim(*astate, action, starttime, stoptime));
  //std::cout << "Pushing animation: " << action.getName() << " at " << starttime << " - " << stoptime << std::endl;
  return stoptime;
}

bool AnimQueue::AnimChannel::isFree(float time)
{
  if (queue.empty())
    return true;
  return queue.back().stop <= time;
}

void AnimQueue::AnimChannel::runChannel(Ogre::Real tdelta, Ogre::Real abstime, Ogre::SceneNode *node, std::map<AnimAction::Modifier::ModType, Ogre::Real> &modifiers)
{
  if (queue.empty())
    {
      source.channelEmpty(NULL, queue.front().stop);
      return;
    }

  float rate = 0.1 * 10.0;
  abstime *= rate; // TODO: is this really correct, it assumes a constant rate...

  if (queue.front().getStartTime() > abstime)
    return;

  if (not queue.front().isPlaying(abstime))
    {
      //std::cout << "Starting animation: " << queue.front().action.getName() << ":" << queue.front().start
      //	<< "  abstime:" << abstime << std::endl;
      queue.front().startAnim();
    }

  if (queue.front().hasEnded(abstime))
    {
      queue.front().stopAnim();
      if (queue.size() == 1)
	source.channelEmpty(&queue.front().action, queue.front().stop);

      queue.pop();
      runChannel(tdelta, abstime, node, modifiers);
      return;
    }

  queue.front().update(tdelta * rate, node, modifiers);
}

AnimQueue::AnimQueue(Ogre::Entity *entity, AnimQueue::Source &s) : source(s), ent(entity)
{
  channels.push_back(AnimChannel(source));
  reset();
  source.channelEmpty(NULL, timer.getMicroseconds() / Ogre::Real(1000000));
}


Ogre::Real AnimQueue::push(const AnimAction &action)
{
  return push(action, timer.getMicroseconds() / Ogre::Real(1000000));
}

Ogre::Real AnimQueue::push(const AnimAction &action, Ogre::Real starttime)
{
  // Find a free queue or create and push action on that queue
  for (size_t c = 0; c < channels.size(); ++c)
    if (channels[c].isFree(starttime))
	return channels[c].push(action, starttime, ent);


  // No available queue, create a new
  channels.push_back(AnimChannel(source));
  return channels.back().push(action, starttime, ent);
}

void AnimQueue::update(Ogre::SceneNode *node, std::map<AnimAction::Modifier::ModType, Ogre::Real> &modifiers)
{
  unsigned long frame_time = timer.getMicroseconds();
  Ogre::Real tdelta = (frame_time - lastFrameTime_us) / Ogre::Real(1000000);
  lastFrameTime_us = frame_time;
  Ogre::Real abstime = frame_time / Ogre::Real(1000000);

  for (size_t c = 0; c < channels.size(); ++c)
    channels[c].runChannel(tdelta, abstime, node, modifiers);
}


AnimScheduler::AnimScheduler(Ogre::Entity *entity) : ent(entity), queue(entity, *this)
{
  state[AnimAction::Movement] = AnimAction::Idle;
  state[AnimAction::Head] = AnimAction::Look;
  state[AnimAction::Pose] = AnimAction::Stand;

  nextState[AnimAction::Movement] = AnimAction::Idle;
  nextState[AnimAction::Head] = AnimAction::Look;
  nextState[AnimAction::Pose] = AnimAction::Stand;
}

void AnimScheduler::update(Ogre::SceneNode *node)
{
  queue.update(node, modifiers);
}

void AnimScheduler::channelEmpty(const AnimAction *last_action, Ogre::Real stoptime)
{
  std::string la = last_action ? last_action->getName() : "None";
  //std::cout << "Some channel emptied, last action played was "
  //	    << la << std::endl;

  std::vector<const AnimAction*> todo;

  graph.breadthFirstSearch(state.key(), nextState.key(), &todo);
  if (todo.size() == 0)
    {
      std::cout << "ERROR: could not find state transition!!!!!" << std::endl;
      return;
    }

  for (unsigned int i = 0; i < todo.size(); ++i)
    stoptime = queue.push(*todo[i], stoptime);
  state = nextState;
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


  AnimAction *pv = new AnimAction("PatrolVigilant");
  pv->setStartState(AnimAction::Movement, AnimAction::Idle);
  pv->setStartState(AnimAction::Head, AnimAction::Look);
  pv->setStartState(AnimAction::Pose, AnimAction::Stand);
  pv->setEndState(AnimAction::Movement, AnimAction::Idle);
  pv->setEndState(AnimAction::Head, AnimAction::Aim);
  pv->setEndState(AnimAction::Pose, AnimAction::Stand);

  AnimAction *pvpose = new AnimAction("PatrolVigilant", AnimAction::EndPose);
  pvpose->setStartState(AnimAction::Movement, AnimAction::Idle);
  pvpose->setStartState(AnimAction::Head, AnimAction::Aim);
  pvpose->setStartState(AnimAction::Pose, AnimAction::Stand);
  pvpose->setEndState(AnimAction::Movement, AnimAction::Idle);
  pvpose->setEndState(AnimAction::Head, AnimAction::Aim);
  pvpose->setEndState(AnimAction::Pose, AnimAction::Stand);

  AnimAction *pvw = new AnimAction("PatrolVigilantWalk");
  pvw->setStartState(AnimAction::Movement, AnimAction::Walk);
  pvw->setStartState(AnimAction::Head, AnimAction::Aim);
  pvw->setStartState(AnimAction::Pose, AnimAction::Stand);
  pvw->setEndState(AnimAction::Movement, AnimAction::Walk);
  pvw->setEndState(AnimAction::Head, AnimAction::Aim);
  pvw->setEndState(AnimAction::Pose, AnimAction::Stand);
  pvw->addModifier(new AnimAction::DirectionMod(1.0));
  pvw->addModifier(new AnimAction::RotationMod(1.0));

  AnimAction *pvwen = new AnimAction("PatrolVigilantWalkEnt");
  pvwen->setStartState(AnimAction::Movement, AnimAction::Idle);
  pvwen->setStartState(AnimAction::Head, AnimAction::Aim);
  pvwen->setStartState(AnimAction::Pose, AnimAction::Stand);
  pvwen->setEndState(AnimAction::Movement, AnimAction::Walk);
  pvwen->setEndState(AnimAction::Head, AnimAction::Aim);
  pvwen->setEndState(AnimAction::Pose, AnimAction::Stand);

  AnimAction *pvwex = new AnimAction("PatrolVigilantWalkEx");
  pvwex->setStartState(AnimAction::Movement, AnimAction::Walk);
  pvwex->setStartState(AnimAction::Head, AnimAction::Aim);
  pvwex->setStartState(AnimAction::Pose, AnimAction::Stand);
  pvwex->setEndState(AnimAction::Movement, AnimAction::Idle);
  pvwex->setEndState(AnimAction::Head, AnimAction::Aim);
  pvwex->setEndState(AnimAction::Pose, AnimAction::Stand);


  pv->setTranslation(Ogre::Vector3(0.0, 0.0, 0.4));
  pvwen->setTranslation(Ogre::Vector3(0.0, 0.0, 1.2));
  pvw->setTranslation(Ogre::Vector3(0.0, 0.0, 1.5));
  pvwex->setTranslation(Ogre::Vector3(0.0, 0.0, 1.4));


  animSched = new AnimScheduler(ent);
  animSched->registerAction(*pv);
  animSched->registerAction(*pvpose);
  animSched->registerAction(*pvwen);
  animSched->registerAction(*pvw);
  animSched->registerAction(*pvwex);

  animSched->setState(AnimAction::Head, AnimAction::Aim);
}


void Character::update(Ogre::Real tdelta)
{
  animSched->update(node);
}
