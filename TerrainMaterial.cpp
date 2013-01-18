#include "TerrainMaterial.h"

#include "Utils.h"

TerrainMaterial::TerrainMaterial(const Ogre::String &resourceGroupName)
{
  mProfiles.push_back(OGRE_NEW Profile(this, "SimpleMaterial", "Profile for rendering shaded terrain material", resourceGroupName));
    setActiveProfile("SimpleMaterial");
}


TerrainMaterial::Profile::Profile(Ogre::TerrainMaterialGenerator* parent, const Ogre::String& name, const Ogre::String& desc,
				  const Ogre::String& resourceGroupName)
  : Ogre::TerrainMaterialGenerator::Profile(parent, name, desc), resourceGroup(resourceGroupName)
{
}

TerrainMaterial::Profile::~Profile()
{
}

Ogre::MaterialPtr TerrainMaterial::Profile::generate(const Ogre::Terrain* terrain)
{
  //TerrainMaterial* parent = (TerrainMaterial*)getParent();
  const Ogre::String shadow_name("shadow");

  // Set Ogre material
  Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create("shaded_terrain", resourceGroup, false);
  Ogre::Technique *tech = mat->getTechnique(0);

  Ogre::Pass *pass = tech->getPass(0);
  pass->setLightingEnabled(false);
  pass->setSceneBlending(Ogre::SBT_REPLACE);
  pass->setDepthWriteEnabled(true);
  pass->removeAllTextureUnitStates();

  Ogre::uint32 size = terrain->getSize() - 1;
  Ogre::TexturePtr shadow_tex = Ogre::TextureManager::getSingleton().createManual(shadow_name, resourceGroup, Ogre::TEX_TYPE_2D, size, size, /*num_mips*/ 1, Ogre::PF_X8R8G8B8);
  Ogre::HardwarePixelBufferSharedPtr buf = shadow_tex->getBuffer();

  unsigned char *data = static_cast<unsigned char*>(buf->lock(Ogre::HardwareBuffer::HBL_DISCARD));
  for (Ogre::uint32 i = 0; i < HeloUtils::POW2(size) * 4; i += 4)
    {
      data[i + 0] = 0;
      data[i + 1] = 0;
      data[i + 2] = (char)i;
      data[i + 3] = 255;
    }
  buf->unlock();


  Ogre::TextureUnitState *tex = pass->createTextureUnitState();
  tex->setTextureName(shadow_tex->getName());

  return mat;
}

Ogre::MaterialPtr TerrainMaterial::Profile::generateForCompositeMap(const Ogre::Terrain* terrain)
{
  return terrain->_getCompositeMapMaterial();
}

Ogre::uint8 TerrainMaterial::Profile::getMaxLayers(const Ogre::Terrain* terrain) const
{
    return 0;
}

void TerrainMaterial::Profile::updateParams(const Ogre::MaterialPtr& mat, const Ogre::Terrain* terrain)
{
}

void TerrainMaterial::Profile::updateParamsForCompositeMap(const Ogre::MaterialPtr& mat, const Ogre::Terrain* terrain)
{
}

void TerrainMaterial::Profile::requestOptions(Ogre::Terrain* terrain)
{
    terrain->_setMorphRequired(false);
    terrain->_setNormalMapRequired(false);
    terrain->_setLightMapRequired(false);
    terrain->_setCompositeMapRequired(false);
}

