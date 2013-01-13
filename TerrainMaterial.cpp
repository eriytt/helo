#include "TerrainMaterial.h"

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

  // Set Ogre material
  Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create("shaded_terrain", resourceGroup, false);
  Ogre::Technique *tech = mat->getTechnique(0);

  Ogre::Pass *pass = tech->getPass(0);
  pass->setLightingEnabled(false);
  pass->setSceneBlending(Ogre::SBT_REPLACE);
  pass->setDepthWriteEnabled(true);
  pass->removeAllTextureUnitStates();

  // Ogre::TextureUnitState *text = pass->createTextureUnitState();
  //     text->setColourOperationEx(Ogre::LayerBlendOperationEx::LBX_SOURCE1, Ogre::LayerBlendSource::LBS_MANUAL,
  //        Ogre::LayerBlendSource::LBS_CURRENT, ColourValue(0.3, 0.7, 0.3));
  //     text->setAlphaOperation(Ogre::LayerBlendOperationEx::LBX_SOURCE1, Ogre::LayerBlendSource::LBS_MANUAL,
  //        Ogre::LayerBlendSource::LBS_CURRENT, 0.5);

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

