#include "Configuration.h"

#include <Ogre.h>

#include "TinyXMLResourceManager.h"
#include "XML.h"
#include "Car.h"
void Configuration::loadConfig()
{
  TinyXMLPtr xml = TinyXMLResourceManager::getSingleton().create("helo.xml", "helo");
 
  // actually load our xml file
  xml->load();
 
  if(xml->getError())
    {
      Ogre::LogManager::getSingletonPtr()
        ->logMessage("An error occured in file " + xml->getName() + 
                     " : " + Ogre::String(xml->getErrorDesc()));
      throw ConfigurationError(xml->getName(), xml->getErrorDesc());
    }
  else
    {
      try
        {
          TiXmlNode* node = xml->getXMLData();
          node = node->FirstChild();
 
          TiXmlNode *child = NULL;
          while((child = node->IterateChildren("Missions", child)))
            readMissions(child);
     
          // child = NULL;
          // while((child = node->IterateChildren("Vehicles", child)))
          //   readVehicles(child);
        }
      catch (XMLUtils::XMLError e)
        {
          throw ConfigurationError(std::string("Error loading configuration: ") + e.what());
        }
     }
}

void Configuration::readMissions(TiXmlNode *parent)
{
  TiXmlNode *child = NULL;
  while((child = parent->IterateChildren(child)))
    {
      Mission mission;
      mission.name = XMLUtils::GetAttribute<std::string>("name", child);
      mission.path = XMLUtils::GetAttribute<std::string>("path", child);
      missions.push_back(mission);
    }

  if (not missions.size())
    throw ConfigurationError(parent->GetDocument()->ValueStr(),
                             "No mission definitions found'");
}

void Configuration::readVehicles(TiXmlNode *parent)
{
  TiXmlNode *child = NULL;
  while((child = parent->IterateChildren("Vehicle", child)))
    {
      Vehicle v;
      v.name = XMLUtils::GetAttribute<std::string>("name", child);
      v.path = XMLUtils::GetAttribute<std::string>("path", child);
      const std::string type(XMLUtils::GetAttribute<std::string>("type", child));
      Ogre::Vector3 pos(XMLUtils::GetVectorParam<Ogre::Vector3>("position", child));

      //std::cout << "New vehicle: name = " << v.name << ", path = " << v.path << std::endl;

      loadVehicle(type, v.name, pos);
      vehicles.push_back(v);
    }
}

void Configuration::loadVehicle(const std::string &type, const std::string &name, const Ogre::Vector3 &position)
{
  Ogre::ResourceGroupManager &rgm = Ogre::ResourceGroupManager::getSingleton();
  if (not rgm.resourceGroupExists("Vehicles/" + type))
    {
      rgm.createResourceGroup(type, false);
      rgm.addResourceLocation(resource_base_path +"Vehicles/" + type, "FileSystem", type);
      rgm.initialiseResourceGroup(type);
      TinyXMLPtr xml = TinyXMLResourceManager::getSingleton().create(type + ".xml", type);
      xml->load();
    }

  TinyXMLPtr xml = TinyXMLResourceManager::getSingleton().createOrRetrieve(type + ".xml", type).first;

  if(xml->getError())
    {
      Ogre::LogManager::getSingletonPtr()
        ->logMessage("An error occured in file " + xml->getName() + 
                     " : " + Ogre::String(xml->getErrorDesc()));
      throw ConfigurationError(xml->getName(), xml->getErrorDesc());
    }
  else
    {
      TiXmlNode* node = xml->getXMLData();
      TiXmlNode* n = node->FirstChild();


      Car::CarData cd; // Hmm, only if it is a car
      cd.position = position;
      cd.name = name;

      cd.meshname = XMLUtils::GetAttribute<std::string>("meshname", n);;
      cd.weight = XMLUtils::GetAttribute<float>("weight", n);
      cd.size = XMLUtils::GetVectorParam<Ogre::Vector3>("size", n);

      TiXmlNode *wsn = n->IterateChildren("wheels", NULL);
      if (not wsn)
        throw ConfigurationError(n->GetDocument()->ValueStr(),
                                 n->ValueStr() + " has no child child named 'wheels'");

      cd.wheelData.resize(XMLUtils::GetNumChildren(wsn, "wheel"));
      
      TiXmlNode *wn = NULL;
      for (int i = 0; (wn = wsn->IterateChildren(std::string("wheel"), wn)); ++i)
        {
          cd.wheelData[i].relPos = HeloUtils::Ogre2BulletVector(XMLUtils::GetVectorParam<Ogre::Vector3>("relativePosition", wn));
          cd.wheelData[i].suspensionLength = XMLUtils::GetAttribute<float>("suspensionLength", wn);
          cd.wheelData[i].maxLengthUp = XMLUtils::GetAttribute<float>("maxLengthUp", wn);
          cd.wheelData[i].maxLengthDown = XMLUtils::GetAttribute<float>("maxLengthDown", wn);
          cd.wheelData[i].direction = HeloUtils::Ogre2BulletVector(XMLUtils::GetVectorParam<Ogre::Vector3>("direction", wn));
          cd.wheelData[i].axle = HeloUtils::Ogre2BulletVector(XMLUtils::GetVectorParam<Ogre::Vector3>("axle", wn));
          cd.wheelData[i].radius = XMLUtils::GetAttribute<float>("radius", wn);
          cd.wheelData[i].spring = XMLUtils::GetAttribute<float>("spring", wn);
          cd.wheelData[i].dampUp = XMLUtils::GetAttribute<float>("dampUp", wn);
          cd.wheelData[i].dampDown = XMLUtils::GetAttribute<float>("dampDown", wn);
          cd.wheelData[i].steerCoeff = XMLUtils::GetAttribute<float>("steerCoeff", wn);
          cd.wheelData[i].driveCoeff = XMLUtils::GetAttribute<float>("driveCoeff", wn);
          cd.wheelData[i].brakeCoeff = XMLUtils::GetAttribute<float>("brakeCoeff", wn);;
          cd.wheelData[i].momentOfInertia = XMLUtils::GetAttribute<float>("momentOfInertia", wn);
        }
      Car *car = new Car(cd, root);
      physics->addObject(car);
      controllables.push_back(car);
    }
}

void Configuration::loadMission(std::string mission_name)
{
  try
    {
      Mission *m = NULL;

      for (unsigned int i = 0; i < missions.size(); ++i)
        if (missions[i].name == mission_name)
          m = &missions[i];

      Ogre::ResourceGroupManager &rgm = Ogre::ResourceGroupManager::getSingleton();
      rgm.createResourceGroup(m->name, false);
      rgm.addResourceLocation(resource_base_path + m->path, "FileSystem", m->name);
      rgm.initialiseResourceGroup(m->name);
      // Ogre::StringVectorPtr p = rgm.listResourceLocations(m->name);
      // for (Ogre::StringVector::iterator i = p->begin(); i != p->end(); ++i)
      //   std::cout << *i << std::endl;

      TinyXMLPtr xml = TinyXMLResourceManager::getSingleton().create( m->name + ".xml", m->name);
 
      // actually load our xml file
      xml->load();
 
      if(xml->getError())
        {
          Ogre::LogManager::getSingletonPtr()
            ->logMessage("An error occured in file " + xml->getName() + 
                         " : " + Ogre::String(xml->getErrorDesc()));
          throw ConfigurationError(xml->getName(), xml->getErrorDesc());
        }
      else
        {
          TiXmlNode* node = xml->getXMLData();
          node = node->FirstChild();
     
          TiXmlNode *child = NULL;
          while((child = node->IterateChildren("Vehicles", child)))
            {
              TiXmlNode *v = NULL;
              while((v = child->IterateChildren("Vehicle", v)))
                {
                  const std::string type(XMLUtils::GetAttribute<std::string>("type", v));
                  const std::string name(XMLUtils::GetAttribute<std::string>("name", v));
                  const Ogre::Vector3 pos = XMLUtils::GetVectorParam<Ogre::Vector3>("position", v);
                  loadVehicle(type, name, pos);
                }
            }
        }
    } 
  catch (XMLUtils::XMLError e)
    {
      throw ConfigurationError("Error loading mission '" + mission_name + "': " + e.what());
    }

}
