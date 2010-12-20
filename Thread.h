#ifndef THREAD_H
#define THREAD_H

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

class ThreadWorker
{
  friend class Thread;
protected:
  // Override to do the thread work
  virtual void work() =0;
};


class Thread : public ThreadWorker
{
protected:
  class ThreadTrampoline
  {
  protected:
    Thread *thread;
  public:
    ThreadTrampoline(Thread *t) : thread(t) {}
    void operator()()
    {
      thread->entry();
    }
  };

 protected:
  boost::mutex run_lock;
  boost::thread thread;
  ThreadWorker *threadWorker;
  float timeout;
  bool cont;
  bool exit;

protected:
  void entry()
  {
    while (true)
      {
	run_lock.lock();
	if (exit)
	  return;
	main();
      }
  }

  void main()
  {
    run_lock.unlock();
    while (cont)
      if (threadWorker)
	threadWorker->work();
      else
	work();
    return;
  }

  virtual void work() {}

  void commonInit()
  {
    run_lock.lock();
    thread = boost::thread(ThreadTrampoline(this));
  }

public:
  Thread(float joinTimeout = 0.0) : threadWorker(0), timeout(joinTimeout), cont(false), exit(false)
  {
    commonInit();
  }

  Thread(ThreadWorker *worker, float joinTimeout = 0.0) : threadWorker(worker), timeout(joinTimeout),
							  cont(false), exit(false)
  {
    commonInit();
  }

  virtual ~Thread()
  {
    exit = true;
    cont = false;

    if (not isRunning())
      run_lock.unlock();

    if (thread.joinable())
      {
	if (timeout > 0.0)
	  thread.timed_join(boost::posix_time::milliseconds(timeout));
	else
	  thread.join();
      }
  }

  void start() {resume();}
  void resume() {if (not isRunning()) {cont = true; run_lock.unlock();}}
  void stop() {run_lock.lock(); cont = false;}
  bool isRunning() {return cont == true;}
};

#endif // THREAD_H
