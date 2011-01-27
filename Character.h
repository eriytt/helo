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
protected:
  class Anim
  {
  public:
    Anim(Ogre::AnimationState &s, AnimAction &a) : state(s), action(a) {}
    void operator=(const AnimQueue::Anim& other) {state = other.state; action = other.action;}
    Ogre::AnimationState &state;
    AnimAction &action;
  };

protected:
  Ogre::Entity *ent;
  Anim currentAnim;
  std::queue<Anim> queue;

public:
  AnimQueue(Ogre::Entity *entity, AnimAction &start_action);
  void push(AnimAction &action);
  void update(Ogre::Real tdelta, Ogre::SceneNode *node);
};

class Character
{
protected:
  Ogre::SceneNode *node;
  Ogre::Entity *ent;
  AnimQueue *animQueue;

public:
  Character(Ogre::Root *root);
  void update(Ogre::Real tdelta);
  Ogre::SceneNode *getSceneNode() {return node;}
};

#endif // CHARACTER_h
