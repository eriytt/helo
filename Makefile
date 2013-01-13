
DEBUG_FLAGS = -g
OBJECTS = helo.o Ogre.o Terrain.o Physics.o Helicopter.o Car.o DriveTrain.o Tank.o Character.o InputHandler.o TerrainMaterial.o
BULLET_DEBUG = /usr/bullet-debug
BULLET_OPT = /usr/bullet

ifeq ($(OPTIMIZE), yes)
  OGRE = /usr
  BULLET = $(BULLET_OPT)
  OPT_FLAGS = -O2 $(DEBUG_FLAGS)
  OIS = /usr
else
  OGRE = /usr/ogre-debug
  OGRE_DEBUG_SUFFIX = _d
  BULLET = $(BULLET_DEBUG)
  OIS = /usr/ois-debug
  OPT_FLAGS = 
endif

OGRE_CXXFLAGS = -I$(OGRE)/include/OGRE
BULLET_CXXFLAGS = -I$(BULLET)/include/bullet
OIS_CXXFLAGS = -I${OIS}/include
CXXFLAGS = -Wall $(OPT_FLAGS) $(DEBUG_FLAGS) $(OGRE_CXXFLAGS) $(BULLET_CXXFLAGS) ${OIS_CXXFLAGS}

OGRE_LDFLAGS = -L$(OGRE)/lib -lOgreMain${OGRE_DEBUG_SUFFIX} -lOgreTerrain${OGRE_DEBUG_SUFFIX} -lOgrePaging${OGRE_DEBUG_SUFFIX}
BULLET_LDFLAGS = -L$(BULLET)/lib -lBulletDynamics -lBulletCollision  -lLinearMath
OIS_LDFLAGS = -L${OIS}/lib -lOIS
BOOST_LDFLAGS = -lboost_system -lboost_thread -lboost_filesystem
LDFLAGS = $(BULLET_LDFLAGS) $(OGRE_LDFLAGS) $(OIS_LDFLAGS) $(BOOST_LDFLAGS) 

all: helo

helo: $(OBJECTS)
	g++ -o $@ $(OBJECTS) $(LDFLAGS)

clean:
	rm -f $(OBJECTS) helo
