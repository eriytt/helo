// TinyXMLResourceManager.cpp class for Ogre
// Author: xadhoom
// License: Do whatever you want with it.
 
#ifndef TINYXMLRESOURCE_H
#define TINYXMLRESOURCE_H

#include <tinyxml.h>
 
#include "Ogre.h"


class TinyXMLResource : public Ogre::Resource
{
public:

   /** Constructor - use resource manager's create method rather than this.
   */
   TinyXMLResource(Ogre::ResourceManager* creator, const Ogre::String& name, Ogre::ResourceHandle handle,
      const Ogre::String& group, bool isManual = false, Ogre::ManualResourceLoader* loader = 0);

   ~TinyXMLResource();

   /** Assignment operator to allow easy copying between TinyXMLResources.
   */
   TinyXMLResource& operator=( const TinyXMLResource& rhs );

   /** Returns the actual TinyXML document.
   @remarks
   The return value is upcasted to an TiXmlNode to have a consistent
   interface without the file input/output functionality of TinyXML.
   */
   TiXmlNode* getXMLData() { return &mTinyXMLDoc; }
 
   /**  If, an error occurs during parsing, Error() will return true.
   */
   bool getError() const { return mTinyXMLDoc.Error(); }
 
   /**  Returns a textual (english) description of the error if one occured.
   */
   const char * getErrorDesc() const { return mTinyXMLDoc.ErrorDesc(); }
 
   /** Generally, you probably want the error string ( ErrorDesc() ). But if you
   prefer the ErrorId, this function will fetch it.
   */
   const int getErrorId() const { return mTinyXMLDoc.ErrorId(); }
 
protected:
 
   /** Overridden from Resource.
   */
   void prepareImpl();
 
   /** Overridden from Resource.
   */
   void unprepareImpl();
 
   /** Overridden from Resource.
   */
   void loadImpl();
 
   /** Unloads the TinyXMLResource, frees resources etc.
   @see
   Resource
   */
   void unloadImpl();
 
   size_t calculateSize() const { return 0; } // TODO 
 
   Ogre::DataStreamPtr  mFreshFromDisk;
 
   TiXmlDocument        mTinyXMLDoc;
};

typedef Ogre::SharedPtr<TinyXMLResource> TinyXMLPtr;

#endif  // TINYXMLRESORCE_H
