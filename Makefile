HOSTNAME = $(shell hostname --fqdn)

PATHS = paths.$(HOSTNAME)
include $(PATHS)

OBJECTS = helo.o Ogre.o Terrain.o Physics.o Helicopter.o Car.o DriveTrain.o Tank.o Character.o InputHandler.o TerrainMaterial.o TinyXMLResource.o TinyXMLResourceManager.o Configuration.o Airplane.o HardPoints.o

ifeq ($(OPTIMIZE), yes)
  DEBUG_FLAGS = -O2
else
  DEBUG_FLAGS = -g
endif

# BOOST_THREAD_VERSION=4 must be defined to 4 in ogre 1.9
# https://ogre3d.atlassian.net/browse/OGRE-398
OGRE_XTRA_CXXFLAGS = -DBOOST_THREAD_VERSION=4
OGRE_CXXFLAGS = -I$(OGRE)/include/OGRE $(OGRE_XTRA_CXXFLAGS)

BULLET_CXXFLAGS = -I$(BULLET)/include/bullet

OIS_CXXFLAGS = -I${OIS}/include

TINYXML_CXXFLAGS = -I${TINYXML}/include

CXXFLAGS = -Wall $(OPT_FLAGS) $(DEBUG_FLAGS) $(OGRE_CXXFLAGS) $(BULLET_CXXFLAGS) ${OIS_CXXFLAGS} ${TINYXML_CXXFLAGS}

OGRE_LDFLAGS = -L$(OGRE)/lib -lOgreMain${OGRE_DEBUG_SUFFIX} -lOgreTerrain${OGRE_DEBUG_SUFFIX} -lOgrePaging${OGRE_DEBUG_SUFFIX}
BULLET_LDFLAGS = -L$(BULLET)/lib -lBulletDynamics -lBulletCollision  -lLinearMath
OIS_LDFLAGS = -L${OIS}/lib -lOIS
BOOST_LDFLAGS = -lboost_system -lboost_thread -lboost_filesystem -lboost_chrono
ZZIP_LDFLAGS = -L${ZZIP}/lib -lzzip
TINYXML_LDFLAGS = -L${TINYXML}/lib -ltinyxml
LDFLAGS = $(BULLET_LDFLAGS) $(OGRE_LDFLAGS) $(OIS_LDFLAGS) $(BOOST_LDFLAGS) ${ZZIP_LDFLAGS} ${TINYXML_LDFLAGS}

all: helo

helo: $(OBJECTS)
	g++ -o $@ $(OBJECTS) $(LDFLAGS)

clean:
	rm -f $(OBJECTS) helo
