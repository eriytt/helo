#ifndef EVENTQUEUE_H
#define EVENTQUEUE_H

#include "Utils.h"

template <typename T>
class EventQueue
{
public:
  typedef unsigned int EventID;

public:
  class Event
  {
  public:
    virtual void operator()(T time, EventID id) const {};
    virtual void dispose(bool handled) const {delete this;};
    virtual ~Event() {}
  };

  template <class L>
  class LambdaEvent : public Event
  { 
  protected:
    std::function<L> f;
  public:
    LambdaEvent(std::function<L> lambda) : f(lambda) {}
    virtual void operator()(T time, EventID id) const {f();}
  };


protected:
  struct EventQueueEntry
  {
    T time;
    EventID id;
    const Event *e;

    EventQueueEntry(T time, EventID id, const Event *e) : time(time), id(id), e(e) {}
    bool operator<(const EventQueueEntry &other) const {return time < other.time;}
  };
  
 protected:
  HeloUtils::SortedList<EventQueueEntry> queue;
  EventID ectr;

public:
  EventQueue() : ectr(0) {}

  void advance(T until)
  {
    T etime = 0;
    while (etime <= until)
      {
	if (queue.empty())
	  break;
	
	const EventQueueEntry &e = queue.front();
	if (e.time > until)
	  break;

	(*e.e)(e.time, e.id);
	etime = e.time;
	e.e->dispose(true);
	queue.pop_front();
      }
  }

  void advanceNoTrigger(T until)
  {
    T etime = 0;
    while (etime <= until)
      {
	if (queue.empty())
	  break;
	    
	const EventQueueEntry &e = queue.front();
	if (e.time > until)
	  break;

	etime = e.time;
	e.e->dispose(false);
      }
  }

  EventID postEvent(T at, const Event *e)
  {
    EventID new_id = ectr++;
    queue.insert(EventQueueEntry(at, new_id, e));
    return new_id;
  }

  template <class L>
  EventID postEvent(T at, std::function<L> lambda)
  {
    EventID new_id = ectr++;
    queue.insert(EventQueueEntry(at, new_id, new LambdaEvent<L>(lambda)));
    return new_id;
  }

  bool cancelEvent(EventID id)
  {
    bool found;
    queue.remove_if([id, &found](const EventQueueEntry &e)
		    {
		      if (e.id == id)
			{
			  e.e->dispose(false);
			  return (found = true);
			}
		      return false;
		    });
    return found;
  }
};

/* Example of deriving event */
template <typename T>
class PrintEvent : public EventQueue<T>::Event
{
protected:
  std::string s;
public:
  PrintEvent(const std::string &s) : s(s) {}
  virtual void operator()(T time, typename EventQueue<T>::EventID id) const {std::cout << "PrintEvent triggered: " << s << std::endl ;}
};
#endif // EVENTQUEUE_H
