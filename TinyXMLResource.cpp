// TinyXMLResource.cpp class for Ogre
// Author: xadhoom
// License: Do whatever you want with it.


#include "TinyXMLResource.h"
#include "TinyXMLResourceManager.h"

using namespace Ogre;


TinyXMLResource::TinyXMLResource(ResourceManager* creator, const String& name, ResourceHandle handle,
                                 const String& group, bool isManual, ManualResourceLoader* loader)
                                 :Resource(creator, name, handle, group, isManual, loader)
{
}


TinyXMLResource::~TinyXMLResource()
{
   // have to call this here rather than in Resource destructor
   // since calling virtual methods in base destructors causes crash
   unload();
}


TinyXMLResource& TinyXMLResource::operator=(const TinyXMLResource& rhs)
{
   mName = rhs.mName;
   mGroup = rhs.mGroup;
   mCreator = rhs.mCreator;
   mIsManual = rhs.mIsManual;
   mLoader = rhs.mLoader;
   mHandle = rhs.mHandle;
   mSize = rhs.mSize;
 
   mLoadingState = rhs.mLoadingState;
   mIsBackgroundLoaded = rhs.mIsBackgroundLoaded;
 
   // clear previous structure
   mTinyXMLDoc.Clear();
 
   const TiXmlElement* node;
   for(node = rhs.mTinyXMLDoc.FirstChildElement(); node; node = node->NextSiblingElement())
      mTinyXMLDoc.InsertEndChild(*node);
 
   // copy error id if occured
   if(rhs.mTinyXMLDoc.Error())
     mTinyXMLDoc.SetError(rhs.mTinyXMLDoc.ErrorId(), NULL, NULL, TIXML_ENCODING_UNKNOWN);
 
   // mIsLoaded state should still be the same as the original TinyXMLResource
   assert(isLoaded() == rhs.isLoaded());
 
   return *this;
}
 
 
void TinyXMLResource::prepareImpl()
{
   // Load from specified 'name'
   if (getCreator()->getVerbose())
      LogManager::getSingleton().logMessage("TinyXMLResource: Loading " + mName + ".");
 
   mFreshFromDisk = ResourceGroupManager::getSingleton().openResource(mName, mGroup, true, this);
 
   // fully prebuffer into host RAM
   mFreshFromDisk = DataStreamPtr(OGRE_NEW MemoryDataStream(mName,mFreshFromDisk));
}
 
 
 
void TinyXMLResource::unprepareImpl()
{
   mFreshFromDisk.setNull();
}
 
 
void TinyXMLResource::loadImpl()
{
   // Load from specified 'name'
   if (getCreator()->getVerbose())
      LogManager::getSingleton().logMessage("TinyXMLResource: Loading XML data from " + mName + ".");
 
   // If the only copy is local on the stack, it will be cleaned
   // up reliably in case of exceptions, etc
   DataStreamPtr data(mFreshFromDisk);
   mFreshFromDisk.setNull();
 
   if (data.isNull()) 
   {
      OGRE_EXCEPT(Exception::ERR_INVALID_STATE,
         "Data doesn't appear to have been prepared in " + mName,
         "TinyXMLResource::loadImpl()");
   }
 
   // clear possible previous errors
   mTinyXMLDoc.ClearError();
 
   mTinyXMLDoc.Parse(data->getAsString().c_str());
   if(mTinyXMLDoc.Error())
   {
      LogManager::getSingleton().logMessage("TinyXMLResource: An Error occured while loading XML data from " + mName + ".");
   }
}
 
 
void TinyXMLResource::unloadImpl()
{
   mTinyXMLDoc.Clear();
}
