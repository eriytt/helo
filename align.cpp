#include <iostream>

#include <btBulletDynamicsCommon.h>

#define CACHE_LINE_SIZE 64ull

class TransferrableMotionState: public btMotionState
{
public:
  alignas(CACHE_LINE_SIZE) btTransform t;

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
  printf("unaligned: %llu\n\n", reinterpret_cast<unsigned long long>(&ms->t) % CACHE_LINE_SIZE);

  // std::cout << "unaligned: " << ((unsigned long long)(&ms->t)) % CACHE_LINE_SIZE << std::endl;
  //   std::cout << "unaligned: " <<  ptr1 % CACHE_LINE_SIZE << std::endl;

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


