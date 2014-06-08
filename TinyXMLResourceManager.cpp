// TinyXMLResourceManager.cpp class for Ogre
// Author: xadhoom
// License: Do whatever you want with it.
 
#include "TinyXMLResourceManager.h"
 
using namespace Ogre;
 
 
template<> TinyXMLResourceManager* Singleton<TinyXMLResourceManager>::msSingleton = 0;
TinyXMLResourceManager* TinyXMLResourceManager::getSingletonPtr()
{
   return msSingleton;
}
 
 
TinyXMLResourceManager& TinyXMLResourceManager::getSingleton()
{
   assert( msSingleton );  return ( *msSingleton );
}
 
 
TinyXMLResourceManager::TinyXMLResourceManager()
{
   // Resource type
   mResourceType = "TinyXML";
 
   // Register with resource group manager
   ResourceGroupManager::getSingleton()._registerResourceManager(mResourceType, this);
}
 
 
TinyXMLResourceManager::~TinyXMLResourceManager()
{
   // Resources cleared by superclass
   // Unregister with resource group manager
   ResourceGroupManager::getSingleton()._unregisterResourceManager(mResourceType);
}
 
 
Resource* TinyXMLResourceManager::createImpl(const String& name, ResourceHandle handle,
                                             const String& group, bool isManual, ManualResourceLoader* loader,
                                             const NameValuePairList* params)
{
   return OGRE_NEW TinyXMLResource(this, name, handle, group, isManual, loader);
}
