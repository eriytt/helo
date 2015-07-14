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
    virtual void operator()(T actual_time, T event_time, EventID id) const = 0;
    virtual void dispose(bool handled) const {delete this;};
    virtual ~Event() {}
  };

  class LambdaEvent : public Event
  {
  protected:
    std::function<void(T, T, EventID)> f;
  public:
    LambdaEvent(std::function<void(T, T, EventID)> lambda) : f(lambda) {}
    virtual void operator()(T actual_time, T event_time, EventID id) const {f(actual_time, event_time, id);}
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

	(*e.e)(until, e.time, e.id);
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

  EventID postEvent(T at, std::function<void(T, T, EventID)> lambda)
  {
    EventID new_id = ectr++;
    queue.insert(EventQueueEntry(at, new_id, new LambdaEvent(lambda)));
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
  virtual void operator()(T actual_time, T event_time, typename EventQueue<T>::EventID id) const {std::cout << "PrintEvent triggered: " << s << std::endl ;}
};
#endif // EVENTQUEUE_H
