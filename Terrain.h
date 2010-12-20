#ifndef TERRAIN_H
#define TERRAIN_H

#include <OGRE/Terrain/OgreTerrain.h>
#include <OGRE/Terrain/OgreTerrainGroup.h>

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>

class Terrain
{
 protected:
  Ogre::TerrainGlobalOptions* mTerrainGlobals;
  Ogre::TerrainGroup* mTerrainGroup;
  bool mTerrainsImported;
  Ogre::Real heightScale;
  btHeightfieldTerrainShape *terrain_shape;

public:
  Terrain(Ogre::Root *root);
  virtual ~Terrain();
  btRigidBody *createBody();
  Ogre::AxisAlignedBox getBounds() {return  mTerrainGroup->getTerrain (0, 0)->getWorldAABB();}

protected:
  void defineTerrain(long x, long y, bool flat = false);
  void getTerrainImage(bool flipX, bool flipY, Ogre::Image& img);
  void initBlendMaps(Ogre::Terrain* terrain);
  void configureTerrainDefaults(Ogre::Light* l, Ogre::SceneManager *mgr);
  void setupContent(Ogre::SceneManager *mgr);
  void saveTerrains(bool onlyIfModified);
};

#define TERRAIN_WORLD_SIZE 12000.0f
#define TERRAIN_SIZE 513
#define TERRAIN_FILE_PREFIX String("testTerrain")
#define TERRAIN_FILE_SUFFIX String("dat")

#define TERRAIN_PAGE_MIN_X 0
#define TERRAIN_PAGE_MIN_Y 0
#define TERRAIN_PAGE_MAX_X 0
#define TERRAIN_PAGE_MAX_Y 0

#endif // TERRAIN_H
