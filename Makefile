
DEBUG_FLAGS = -g
OBJECTS = helo.o Ogre.o Terrain.o Physics.o Helicopter.o Car.o DriveTrain.o Tank.o Character.o
OGRE = /usr/ogre
BULLET_DEBUG = /usr/bullet-debug
BULLET_OPT = /usr/bullet

ifeq ($(OPTIMIZE), yes)
  BULLET = $(BULLET_OPT)
  OPT_FLAGS = -O2
else
  BULLET = $(BULLET_DEBUG)
endif

OGRE_INCLUDE = -I$(OGRE)/include -I$(OGRE)/include/OGRE
BULLET_INCLUDE = -I$(BULLET)/include/bullet
CXXFLAGS = -Wall $(OPT_FLAGS) $(DEBUG_FLAGS) $(OGRE_INCLUDE) $(BULLET_INCLUDE)

OGRE_LIBS = -L$(OGRE)/lib -lOgreMain -lOgreTerrain -lOgrePaging
BULLET_LIBS = -L$(BULLET)/lib -lBulletDynamics -lBulletCollision  -lLinearMath
OIS_LIBS = -lOIS
BOOST_LIBS = -lboost_system
LDFLAGS = $(BULLET_LIBS) $(OGRE_LIBS) $(OIS_LIBS) $(BOOST_LIBS) 

helo: $(OBJECTS)
	g++ -o $@ $(OBJECTS) $(LDFLAGS)

clean:
	rm $(OBJECTS)
