#ifndef CHARACTER_H
#define CHARACTER_H


#include <OGRE/Ogre.h>

class AnimAction
{
public:
  typedef enum
    {
      Movement,
      Head,
      Pose,
      LastMode
    } AnimMode;

  typedef enum
    {
      Idle, Run, Walk, Back, TurnRight, TurnLeft,
      Look, Aim,
      Stand, Crouch, Lay,
      LastState
    } AnimState;

  class State
  {
  protected:
    std::vector<AnimState> state;
  public:
    State()
    {
      state.resize(AnimAction::LastMode);
      for (unsigned int i = 0; i < state.size(); ++i)
	state[i] = static_cast<AnimState>(0);
    }
    AnimAction::AnimState &operator[](const AnimMode &m)  {return state[m];}
    unsigned long long key() const
    {
      unsigned long long m = state[Movement];
      unsigned long long h = state[Head];
      unsigned long long p = state[Pose];
      return m + ( h << 16) + (p << 32);
    }; // TODO: fixme, and don't forget to mask
    bool operator==(const State &other)
    {
      for (unsigned int i = 0; i < AnimAction::LastMode; ++i)
	if (state[i] != other.state[i])
	  return false;
      return true;
    }
  };

  typedef enum
    {
      Play,
      StartPose,
      EndPose
    } PlayType;

protected:
  Ogre::Vector3 trans;
  std::string name;
  State startState;
  State endState;
  PlayType type;

public:
  AnimAction(std::string action_name, PlayType t = Play);
  void setTranslation(const Ogre::Vector3 &translation) {trans = translation;}
  const Ogre::Vector3 &getTranslation() {return trans;}
  PlayType getType() {return type;}
  void setStartState(AnimMode mode, AnimState state) {startState[mode] = state;}
  void setEndState(AnimMode mode, AnimState state) {endState[mode] = state;}
  const State &getStartState() const {return startState;}
  const State &getEndState() const {return endState;}
  const std::string &getName() const {return name;}
};

class Graph
{
public:
  typedef unsigned long long KeyType;

  class EdgeObj : public AnimAction
  {
  public:
    KeyType getStartState() const {return AnimAction::getStartState().key();}
    KeyType getEndState() const {return AnimAction::getEndState().key();}
  };

  class Node;

  class Edge
  {
  protected:
    class Graph::Node *node;
    EdgeObj &obj;
  public:
    Edge(const EdgeObj &edgeobj, class Graph::Node *n) : node(n), obj(const_cast<EdgeObj&>(edgeobj)) {}
    void operator=(const Edge &other) {node = other.node; obj = other.obj;}
    const EdgeObj &getEdgeObject() {return obj;}
    class Node *getNode() {return node;}
    bool cmp_node(class Node *n) {return node == n;}
  };

  class Node
  {
  protected:
    Node *parent;
    std::vector<class Edge> edges;

  public:
    void setParent(Node *p) {parent = p;}
    Node *getParent() {return parent;}
    void addEdge(const Node &node, const class Graph::EdgeObj& edgeobj)
    {
      edges.push_back(Edge(edgeobj, const_cast<Node*>(&node)));
    }
    std::vector<Edge> &getEdges() {return edges;}
  };
  typedef Node NodeType;
  typedef NodeType *NodePtrType;


protected:
  std::map<KeyType, Node> nodes;
  std::set<Node*> visited;
  std::list<Node*> queue;

protected:
  void constructPath(Node *startnode, Node *endnode, std::vector<const AnimAction*> *v)
  {
    if (v == NULL)
      return;

    std::vector<Node*> node_sequence;
    Node *n = endnode;
    while (n != NULL)
      {
  	node_sequence.insert(node_sequence.begin(), n);
  	n = n->getParent();
      }

    // Cannot follow parent if startnode == endnode
    if (startnode == endnode)
      node_sequence.push_back(endnode);

    for (unsigned int i = 1; i < node_sequence.size(); ++i)
      {
  	Node *n0 = node_sequence[i - 1];
  	Node *n1 = node_sequence[i];

  	// Find the edge that takes us from n0 to n1
  	std::vector<Edge> &edges = n0->getEdges();
	for (unsigned int e = 0; e < edges.size(); ++e)
	  if (edges[e].cmp_node(n1))
	    {
	      v->push_back(dynamic_cast<const AnimAction*>(&edges[e].getEdgeObject()));
	      break;
	    }
      }
  }

public:
  void addEdge(const EdgeObj &obj)
  {
    KeyType start_state = obj.getStartState();
    KeyType end_state = obj.getEndState();

    // add start and end state nodes if they are not already present
    Node sn = Node();
    Node en = Node();
    nodes.insert(std::pair<KeyType, Node>(start_state, sn));
    nodes.insert(std::pair<KeyType, Node>(end_state, en));


    // add an Edge to the start state
    nodes[start_state].addEdge(nodes[end_state], obj);
    Node *nsn = &nodes[start_state];
    Node *nen = &nodes[start_state];
    nsn->addEdge(*nen, obj);
  }

  bool breadthFirstSearch(KeyType start_state, KeyType end_state, std::vector<const AnimAction*> *edgelist)
  {
    Node* startnode = &nodes[start_state];
    if (not startnode)
      return false;

    Node* endnode = &nodes[end_state];
    if (not endnode)
      return false;

    visited.clear();
    // queue is already empty or we would not be here

    queue.push_back(startnode);
    startnode->setParent(NULL);
    while (not queue.empty())
      {
    	Node* n = queue.front();
	queue.erase(queue.begin());
    	if (n == endnode)
    	  {
    	    constructPath(startnode, endnode, edgelist);
    	    return true;
    	  }
    	visited.insert(n);

	std::vector<Edge> &edges = n->getEdges();
    	for (unsigned int i = 0; i < edges.size(); ++i)
    	  {
	    Node* neighbor_node = edges[i].getNode();
    	    if (std::find_if(visited.begin(), visited.end(),
			     std::bind2nd(std::equal_to<Node*>(), neighbor_node)) == visited.end()
		and std::find_if(queue.begin(), queue.end(),
				 std::bind2nd(std::equal_to<Node*>(), neighbor_node)) == queue.end())
    	      {
    		neighbor_node->setParent(n);
    		queue.push_back(neighbor_node);
    	      }

    	  }
      }
    return false;
  }
};

class AnimQueue
{
public:
  class Source
  {
  public:
    virtual void channelEmpty(const AnimAction *last_action, Ogre::Real stoptime) = 0;
  };

protected:
  Source &source;


protected:
  class Anim
  {
  public:
    Ogre::AnimationState &state;
    AnimAction &action;
    float start;
    float stop;

  public:
    Anim(Ogre::AnimationState &s, const AnimAction &a, float starttime, float stoptime);
    void operator=(const AnimQueue::Anim& other)
    {
      state = other.state;
      action = other.action;
      start = other.start;
      stop = other.stop;
    }
    void startAnim();
    void stopAnim();
    float getStartTime() {return start;}
    float getStopTime() {return stop;}
    bool isPlaying(Ogre::Real abstime);
    bool hasEnded(Ogre::Real abstime);
    void update(Ogre::Real tdelta, Ogre::SceneNode *node);
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
    Ogre::Real push(const AnimAction &action, Ogre::Real starttime, Ogre::Entity *ent);
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
  Ogre::Real push(const AnimAction &action);                        // Action should start playing now
  Ogre::Real push(const AnimAction &action, Ogre::Real starttime);  // Action should start playing at starttime
  void update(Ogre::SceneNode *node);
  void reset() {lastFrameTime_us = timer.getMicroseconds(); timer.reset();}

protected:
  AnimChannel &findFreeChannel(Ogre::Real starttime);
};

class AnimScheduler : public AnimQueue::Source
{
protected:
  Graph graph;

protected:
  Ogre::Entity *ent;
  AnimAction::State state;
  AnimAction::State nextState;
  AnimQueue queue;
  std::list<AnimAction> actions;

public:
  AnimScheduler(Ogre::Entity *entity);
  void registerAction(AnimAction &action) {actions.push_back(action); graph.addEdge(static_cast<Graph::EdgeObj&>(action));}
  void setState(AnimAction::AnimMode mode, AnimAction::AnimState state) {nextState[mode] = state;}
  void update(Ogre::SceneNode * node);
  void channelEmpty(const AnimAction *last_action, Ogre::Real stoptime);
};


class Character
{
protected:
  Ogre::SceneNode *node;
  Ogre::Entity *ent;
  AnimScheduler *animSched;

public:
  Character(Ogre::Root *root);
  void update(Ogre::Real tdelta);
  Ogre::SceneNode *getSceneNode() {return node;}
  void setForward(bool fw) {animSched->setState(AnimAction::Movement, fw ? AnimAction::Walk : AnimAction::Idle);}
};

#endif // CHARACTER_h
