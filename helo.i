%module helo

%{
#include "helo.h"
%}

class heloApp : public Ogre::FrameListener, public OIS::KeyListener//, public OIS::JoyStickListener
{
public:
  static heloApp *theApp;
};
