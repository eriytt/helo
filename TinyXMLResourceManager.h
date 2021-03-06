// TinyXMLResourceManager.h class for Ogre
// Author: xadhoom
// License: Do whatever you want with it.
 
#ifndef TINYXMLRESOURCEMANAGER_H
#define TINYXMLRESOURCEMANAGER_H_
 
#include "Ogre.h"
#include "OgreSingleton.h"
#include "OgreResourceManager.h"
#include "TinyXMLResource.h"
 
 
class TinyXMLResourceManager : public Ogre::ResourceManager, public Ogre::Singleton<TinyXMLResourceManager>
{
protected:
 
   // Overridden from ResourceManager
   Ogre::Resource* createImpl(const Ogre::String& name, Ogre::ResourceHandle handle, 
      const Ogre::String& group, bool isManual, Ogre::ManualResourceLoader* loader,
      const Ogre::NameValuePairList* params);
 
public:
   /** Default constructor.
   */
   TinyXMLResourceManager();
 
   /** Default destructor.
   */
   virtual ~TinyXMLResourceManager();
 
   /** Override standard Singleton retrieval.
   @par
   This method just delegates to the template version anyway,
   but the implementation stays in this single compilation unit,
   preventing link errors.
   */
   static TinyXMLResourceManager& getSingleton();
 
   /** Override standard Singleton retrieval.
   @par
   This method just delegates to the template version anyway,
   but the implementation stays in this single compilation unit,
   preventing link errors.
   */
   static TinyXMLResourceManager* getSingletonPtr();
};
 
#endif  // TINYXMLRESOURCEMANAGER_H
