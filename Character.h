#ifndef CHARACTER_H
#define CHARACTER_H

#include <OGRE/Ogre.h>


class AnimAction
{
protected:
  Ogre::Vector3 trans;
  std::vector<class AnimAction*> successors;
  std::string name;

public:
  AnimAction(std::string action_name);
  void addSuccessor(class AnimAction *action) {successors.push_back(action);}
  void setTranslation(const Ogre::Vector3 &translation) {trans = translation;}
  void update(Ogre::Real tdelta, Ogre::SceneNode *node);
  const std::string &getName() const {return name;}
};

class AnimQueue
{
public:
  class Source
  {
  public:
    virtual void channelEmpty(const AnimAction &last_action, Ogre::Real stoptime) = 0;
  };

protected:
  Source &source;


protected:
  class Anim
  {
  public:
    Anim(Ogre::AnimationState &s, AnimAction &a, float starttime, float stoptime) :
      state(s), action(a), start(starttime), stop(stoptime) {}
    void operator=(const AnimQueue::Anim& other)
    {
      state = other.state;
      action = other.action;
      start = other.start;
      stop = other.stop;
    }
    Ogre::AnimationState &state;
    AnimAction &action;
    float start;
    float stop;
  };

  class AnimChannel
  {
  protected:
    std::queue<Anim> queue;
    AnimQueue::Source &source;

  public:
    AnimChannel(AnimQueue::Source &s) : source(s) {}
    void operator=(AnimChannel &other) {source = other.source;}
    void runChannel(Ogre::Real tdelta, Ogre::Real abstime, Ogre::SceneNode *node);
    Ogre::Real push(AnimAction &action, Ogre::Real starttime, Ogre::Entity *ent);
    bool isFree(Ogre::Real starttime);
  };

protected:
  std::vector<AnimChannel> channels;



protected:
  Ogre::Entity *ent;
  Ogre::Timer timer;
  unsigned long lastFrameTime_us;


public:
  AnimQueue(Ogre::Entity *entity, Source &s);
  Ogre::Real push(AnimAction &action);                        // Action should start playing now
  Ogre::Real push(AnimAction &action, Ogre::Real starttime);  // Action should start playing at starttime
  void update(Ogre::SceneNode *node);
  void reset() {lastFrameTime_us = timer.getMicroseconds(); timer.reset();}

protected:
  AnimChannel &findFreeChannel(Ogre::Real starttime);
};



class Character : public AnimQueue::Source
{
protected:
  Ogre::SceneNode *node;
  Ogre::Entity *ent;
  AnimQueue *animQueue;

public:
  Character(Ogre::Root *root);
  void update(Ogre::Real tdelta);
  Ogre::SceneNode *getSceneNode() {return node;}
  void channelEmpty(const AnimAction &last_action, Ogre::Real stoptime);
};

#endif // CHARACTER_h
