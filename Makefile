HOSTNAME = $(shell hostname --fqdn)

PATHS = paths.$(HOSTNAME)
include $(PATHS)

OBJECTS = helo.o Ogre.o Terrain.o Physics.o Helicopter.o Car.o DriveTrain.o Tank.o Character.o InputHandler.o TerrainMaterial.o TinyXMLResource.o TinyXMLResourceManager.o Configuration.o Airplane.o HardPoints.o Python.o Readline.o ExtConsole.o Lua.o lua_wrap.o Camera.o

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

PYTHON_CXXFLAGS = -I/usr/include/python2.7

LUA_CXXFLAGS = -I/usr/include/lua5.1

CXXFLAGS = -Wall -std=c++11 -MMD $(OPT_FLAGS) $(DEBUG_FLAGS) $(OGRE_CXXFLAGS) $(BULLET_CXXFLAGS) ${OIS_CXXFLAGS} ${TINYXML_CXXFLAGS} ${PYTHON_CXXFLAGS} \
	${LUA_CXXFLAGS}

OGRE_LDFLAGS = -L$(OGRE)/lib -lOgreMain${OGRE_DEBUG_SUFFIX} -lOgreTerrain${OGRE_DEBUG_SUFFIX} -lOgrePaging${OGRE_DEBUG_SUFFIX}
BULLET_LDFLAGS = -L$(BULLET)/lib -lBulletDynamics -lBulletCollision  -lLinearMath
OIS_LDFLAGS = -L${OIS}/lib -lOIS
BOOST_LDFLAGS = -lboost_system -lboost_thread -lboost_filesystem -lboost_chrono
ZZIP_LDFLAGS = -L${ZZIP}/lib -lzzip
TINYXML_LDFLAGS = -L${TINYXML}/lib -ltinyxml
READLINE_LDFLAGS = -lreadline
PYTHON_LDFLAGS = -L/usr/lib/x86_64-linux-gnu -lpython2.7
LUA_LDFLAGS = -llua5.1

LDFLAGS = $(BULLET_LDFLAGS) $(OGRE_LDFLAGS) $(OIS_LDFLAGS) $(BOOST_LDFLAGS) ${ZZIP_LDFLAGS} ${TINYXML_LDFLAGS} ${READLINE_LDFLAGS} ${PYTHON_LDFLAGS} \
	${LUA_LDFLAGS}

all: helo datapkg

helo: $(OBJECTS)
	g++ -o $@ $(OBJECTS) $(LDFLAGS)

DATADIRS = resources/Armaments resources/Missions/ resources/Vehicles/
DATAFILES = $(shell find ${DATADIRS} -type f ! -name '*.xml' ! -name '*~' ) resources/terrain.png resources/terrain_detail.jpg resources/Sphere.mesh
DATAPKG_FILE = helodata.tgz

datapkg: ${DATAPKG_FILE}

${DATAPKG_FILE}: ${DATAFILES}
	tar -czf ${DATAPKG_FILE} $^

clean:
	rm -f $(OBJECTS) helo ${LUA_WRAPPER}

datapkg-clean:
	rm -f ${DATAPKG_FILE}

# Lua wrapping
LUA_WRAPPER = lua_wrap.cpp 
${LUA_WRAPPER}: helo.i
	swig -c++ -lua -o $@ $< 


-include $(OBJECTS:%.o=%.d)
