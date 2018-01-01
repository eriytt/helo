#include <iostream>
#include <memory>
#include <cstddef>
//#include <stdexcept>

#include <btBulletDynamicsCommon.h>

#define CACHE_LINE_SIZE static_cast<size_t>(64)

class TransferrableMotionState: public btMotionState
{
  class bad_align : public std::bad_alloc
  {
    std::string msg;
  public:
    bad_align(const std::string &msg) : msg(msg) {}
    virtual const char* what() const throw() {return msg.c_str();}
  };

private:
  void *bufptr;

public:
  alignas(CACHE_LINE_SIZE) btTransform t;
public:
  void *operator new(size_t size)
  {
    size_t bufsize = sizeof(TransferrableMotionState) + CACHE_LINE_SIZE;;
    std::cout << "Allocating " << bufsize << " bytes" << std::endl;
    void *buf = ::operator new(bufsize);
    // void *origbuf = buf;
    void *objptr = std::align(CACHE_LINE_SIZE,
			      sizeof(TransferrableMotionState),
			      buf, bufsize);

    if (objptr == nullptr)
      throw bad_align("Alignment of motion state data failed");

    void *transform_ptr = &(static_cast<TransferrableMotionState*>(objptr))->t;
    size_t transform_addr = reinterpret_cast<size_t>(transform_ptr);
    if (transform_addr & (CACHE_LINE_SIZE - 1))
      throw bad_align("Transform of motion state not aligned to cache line");
    
    // std::cout << "Allocating " << bufsize << " bytes @"
    // 	      << origbuf << ", aligning to " << buf
    // 	      << std::endl;
    // std::cout << "Wasting "
    // 	      << sizeof(TransferrableMotionState) + CACHE_LINE_SIZE - bufsize
    // 	      << " bytes in alignment" << std::endl;
    // std::cout << "Max alignment " << alignof(std::max_align_t) << std::endl;

    static_cast<TransferrableMotionState*>(objptr)->bufptr = buf;

    return objptr;
  }

  void operator delete(void *ptr)
  {
    void *buf = static_cast<TransferrableMotionState*>(ptr)->bufptr;
    ::operator delete(buf);
  }

public:
  TransferrableMotionState(const btTransform& startTrans = btTransform::getIdentity())
    : t(startTrans) {}

  virtual ~TransferrableMotionState() {}

  virtual void getWorldTransform(btTransform &trans) const {}

  virtual void setWorldTransform(const btTransform &trans) {}
};

void print_alignment(TransferrableMotionState *ms)
{
  printf("size: %ld\n", sizeof(TransferrableMotionState));
  printf("ms @ %p\n", ms);
  printf("t  @ %p\n", &ms->t);
  printf("size of t: %ld\n", sizeof(ms->t));
  unsigned long long ptr1 = ((unsigned long long)(&ms->t));
  printf("unaligned: %lld\n\n", ptr1 % CACHE_LINE_SIZE);
}

TransferrableMotionState *alloc_motionstate()
{
  TransferrableMotionState *ms = new TransferrableMotionState();
  return ms;
}

int main(void)
{
  //void *data = std::aligned_alloc(CACHE_LINE_SIZE,
  //sizeof(TransferrableMotionState))

  TransferrableMotionState ms;
  int foo = 4711;
  TransferrableMotionState ms2;


  printf("%d\n", foo);
  print_alignment(&ms);
  print_alignment(&ms2);
  print_alignment(alloc_motionstate());
  new int;
  new int;
  print_alignment(alloc_motionstate());
  print_alignment(alloc_motionstate());
  new int;
  print_alignment(alloc_motionstate());
  return 0;
}


