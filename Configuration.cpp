#include "Configuration.h"

#include <Ogre.h>

#include "TinyXMLResourceManager.h"
#include "XML.h"
#include "Car.h"
#include "Helicopter.h"
#include "Tank.h"
#include "Airplane.h"

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

	  TiXmlNode *sn = node->IterateChildren("Settings", NULL);
	  if (not sn)
	    throw ConfigurationError(node->GetDocument()->ValueStr(),
			     node->ValueStr() + " has no child child named 'Settings'");

	  runPhysicsInThread = XMLUtils::GetAttribute<bool>("physicsInThread", sn);


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

void Configuration::loadCar(TiXmlNode *n, const std::string &name, const Ogre::Vector3 &position)
{
  Car::CarData cd;
  cd.position = position;
  cd.name = name;

  cd.meshname = XMLUtils::GetAttribute<std::string>("meshname", n);
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

void Configuration::loadHelicopter(TiXmlNode *n, const std::string &name, const Ogre::Vector3 &position)
{
  Helicopter::HelicopterData hd;
  hd.name = name;
  hd.pos = position;

  hd.meshname = XMLUtils::GetAttribute<std::string>("meshname", n);
  hd.size = XMLUtils::GetVectorParam<Ogre::Vector3>("size", n);
  hd.weight = XMLUtils::GetAttribute<float>("weight", n);
  hd.boomLength = XMLUtils::GetAttribute<float>("boomLength", n);
  hd.collectiveSensitivity = XMLUtils::GetAttribute<float>("collectiveSensitivity", n);
  hd.cyclicRightSensitivity = XMLUtils::GetAttribute<float>("cyclicRightSensitivity", n);
  hd.cyclicForwardSensitivity = XMLUtils::GetAttribute<float>("cyclicForwardSensitivity", n);
  hd.steerSensitivity = XMLUtils::GetAttribute<float>("steerSensitivity", n);


  TiXmlNode *rsn = n->IterateChildren("rotors", NULL);
  if (not rsn)
    throw ConfigurationError(n->GetDocument()->ValueStr(),
			     n->ValueStr() + " has no child child named 'rotors'");

  hd.rotorData.resize(XMLUtils::GetNumChildren(rsn, "rotor"));

  TiXmlNode *rn = NULL;
  for (int i = 0; (rn = rsn->IterateChildren(std::string("rotor"), rn)); ++i)
    {
      hd.rotorData[i].weight = XMLUtils::GetAttribute<float>("weight", rn);
      hd.rotorData[i].diameter = XMLUtils::GetAttribute<float>("diameter", rn);
      hd.rotorData[i].relpos = XMLUtils::GetVectorParam<Ogre::Vector3>("relativePosition", rn);
      hd.rotorData[i].axis = XMLUtils::GetVectorParam<Ogre::Vector3>("axis", rn);
      hd.rotorData[i].rotation_axis = XMLUtils::GetVectorParam<Ogre::Vector3>("rotationAxis", rn);
      hd.rotorData[i].torque = XMLUtils::GetAttribute<float>("torque", rn);
      hd.rotorData[i].inertia = XMLUtils::GetAttribute<float>("inertia", rn);
      hd.rotorData[i].maxLift = XMLUtils::GetAttribute<float>("maxLift", rn);
      hd.rotorData[i].tiltSensitivity = XMLUtils::GetAttribute<float>("tiltSensitivity", rn);
    }

  if (hd.rotorData.size() > 2)
    throw ConfigurationError(n->GetDocument()->ValueStr(), "A maximum 2 rotors are supported");

  Helicopter *helicopter = NULL;
  if (hd.rotorData.size() == 1)
    helicopter = new Helicopter(hd, root);
  else
    helicopter = new TandemRotorHelicopter(hd, root);

  physics->addObject(helicopter);
  controllables.push_back(helicopter);
}

void Configuration::loadTank(TiXmlNode *n, const std::string &name, const Ogre::Vector3 &position)
{
  Tank::TankData td;
  td.position = position;
  td.name = name;

  td.meshname = XMLUtils::GetAttribute<std::string>("meshname", n);
  td.weight = XMLUtils::GetAttribute<float>("weight", n);
  td.size = XMLUtils::GetVectorParam<Ogre::Vector3>("size", n);
  td.turretPosition = XMLUtils::GetVectorParam<Ogre::Vector3>("turretPosition", n);
  td.barrelPosition = XMLUtils::GetVectorParam<Ogre::Vector3>("barrelPosition", n);

  TiXmlNode *wsn = n->IterateChildren("wheels", NULL);
  if (not wsn)
    throw ConfigurationError(n->GetDocument()->ValueStr(),
			     n->ValueStr() + " has no child child named 'wheels'");

  td.wheelData.resize(XMLUtils::GetNumChildren(wsn, "wheel"));

  TiXmlNode *wn = NULL;
  for (int i = 0; (wn = wsn->IterateChildren(std::string("wheel"), wn)); ++i)
    {
      td.wheelData[i].relPos = HeloUtils::Ogre2BulletVector(XMLUtils::GetVectorParam<Ogre::Vector3>("relativePosition", wn));
      td.wheelData[i].suspensionLength = XMLUtils::GetAttribute<float>("suspensionLength", wn);
      td.wheelData[i].maxLengthUp = XMLUtils::GetAttribute<float>("maxLengthUp", wn);
      td.wheelData[i].maxLengthDown = XMLUtils::GetAttribute<float>("maxLengthDown", wn);
      td.wheelData[i].direction = HeloUtils::Ogre2BulletVector(XMLUtils::GetVectorParam<Ogre::Vector3>("direction", wn));
      td.wheelData[i].axle = HeloUtils::Ogre2BulletVector(XMLUtils::GetVectorParam<Ogre::Vector3>("axle", wn));
      td.wheelData[i].radius = XMLUtils::GetAttribute<float>("radius", wn);
      td.wheelData[i].spring = XMLUtils::GetAttribute<float>("spring", wn);
      td.wheelData[i].dampUp = XMLUtils::GetAttribute<float>("dampUp", wn);
      td.wheelData[i].dampDown = XMLUtils::GetAttribute<float>("dampDown", wn);
      td.wheelData[i].steerCoeff = XMLUtils::GetAttribute<float>("steerCoeff", wn);
      td.wheelData[i].driveCoeff = XMLUtils::GetAttribute<float>("driveCoeff", wn);
      td.wheelData[i].brakeCoeff = XMLUtils::GetAttribute<float>("brakeCoeff", wn);;
      td.wheelData[i].momentOfInertia = XMLUtils::GetAttribute<float>("momentOfInertia", wn);
    }


  TiXmlNode *dwsn = n->IterateChildren("driveWheels", NULL);
  if (not dwsn)
    throw ConfigurationError(n->GetDocument()->ValueStr(),
			     n->ValueStr() + " has no child child named 'driveWheels'");

  td.driveWheelData.resize(XMLUtils::GetNumChildren(dwsn, "wheel"));

  wn = NULL;
  for (int i = 0; (wn = dwsn->IterateChildren(std::string("wheel"), wn)); ++i)
    {
      td.driveWheelData[i].relPos = HeloUtils::Ogre2BulletVector(XMLUtils::GetVectorParam<Ogre::Vector3>("relativePosition", wn));
      td.driveWheelData[i].realRelPos = HeloUtils::Ogre2BulletVector(XMLUtils::GetVectorParam<Ogre::Vector3>("realRelativePosition", wn));
      td.driveWheelData[i].suspensionLength = XMLUtils::GetAttribute<float>("suspensionLength", wn);
      td.driveWheelData[i].maxLengthUp = XMLUtils::GetAttribute<float>("maxLengthUp", wn);
      td.driveWheelData[i].maxLengthDown = XMLUtils::GetAttribute<float>("maxLengthDown", wn);
      td.driveWheelData[i].direction = HeloUtils::Ogre2BulletVector(XMLUtils::GetVectorParam<Ogre::Vector3>("direction", wn));
      td.driveWheelData[i].axle = HeloUtils::Ogre2BulletVector(XMLUtils::GetVectorParam<Ogre::Vector3>("axle", wn));
      td.driveWheelData[i].radius = XMLUtils::GetAttribute<float>("radius", wn);
      td.driveWheelData[i].spring = XMLUtils::GetAttribute<float>("spring", wn);
      td.driveWheelData[i].dampUp = XMLUtils::GetAttribute<float>("dampUp", wn);
      td.driveWheelData[i].dampDown = XMLUtils::GetAttribute<float>("dampDown", wn);
      td.driveWheelData[i].steerCoeff = XMLUtils::GetAttribute<float>("steerCoeff", wn);
      td.driveWheelData[i].driveCoeff = XMLUtils::GetAttribute<float>("driveCoeff", wn);
      td.driveWheelData[i].brakeCoeff = XMLUtils::GetAttribute<float>("brakeCoeff", wn);;
      td.driveWheelData[i].momentOfInertia = XMLUtils::GetAttribute<float>("momentOfInertia", wn);
    }

  TiXmlNode *swsn = n->IterateChildren("spinWheels", NULL);
  if (not swsn)
    throw ConfigurationError(n->GetDocument()->ValueStr(),
			     n->ValueStr() + " has no child child named 'spinWheels'");

  td.spinWheelData.resize(XMLUtils::GetNumChildren(dwsn, "wheel"));

  wn = NULL;
  for (int i = 0; (wn = swsn->IterateChildren(std::string("wheel"), wn)); ++i)
    {
      td.spinWheelData[i].relPos = HeloUtils::Ogre2BulletVector(XMLUtils::GetVectorParam<Ogre::Vector3>("relativePosition", wn));
      td.spinWheelData[i].axle = HeloUtils::Ogre2BulletVector(XMLUtils::GetVectorParam<Ogre::Vector3>("axle", wn));
      td.spinWheelData[i].radius = XMLUtils::GetAttribute<float>("radius", wn);
    }


  Tank *tank = new Tank(td, root);
  physics->addObject(tank);
  controllables.push_back(tank);
}

void Configuration::ParsePointList(TiXmlNode *parent, std::vector<std::pair<Ogre::Real, Ogre::Real> > &pair_list)
{
  TiXmlNode *pn = NULL;
  for (int i = 0; (pn = parent->IterateChildren(std::string("point"), pn)); ++i)
    {
      std::pair<Ogre::Real, Ogre::Real> point;
      point.first = XMLUtils::GetAttribute<Ogre::Real>("x", pn);
      point.second = XMLUtils::GetAttribute<Ogre::Real>("y", pn);
      pair_list.push_back(point);
    }
}


void Configuration::loadAirplane(TiXmlNode *n, const std::string &name, const Ogre::Vector3 &position)
{
  Airplane::AirplaneData ad;
  ad.position = position;
  ad.name = name;

  ad.meshname = XMLUtils::GetAttribute<std::string>("meshname", n);
  ad.weight = XMLUtils::GetAttribute<float>("weight", n);
  ad.size = XMLUtils::GetVectorParam<Ogre::Vector3>("size", n);

  try {
    TiXmlNode *wsn = XMLUtils::AssertGetNode(n, "wheels");
    ad.wheelData.resize(XMLUtils::GetNumChildren(wsn, "wheel"));

    TiXmlNode *wn = NULL;
    for (int i = 0; (wn = wsn->IterateChildren(std::string("wheel"), wn)); ++i)
      {
        ad.wheelData[i].relPos = HeloUtils::Ogre2BulletVector(XMLUtils::GetVectorParam<Ogre::Vector3>("relativePosition", wn));
        ad.wheelData[i].suspensionLength = XMLUtils::GetAttribute<float>("suspensionLength", wn);
        ad.wheelData[i].maxLengthUp = XMLUtils::GetAttribute<float>("maxLengthUp", wn);
        ad.wheelData[i].maxLengthDown = XMLUtils::GetAttribute<float>("maxLengthDown", wn);
        ad.wheelData[i].direction = HeloUtils::Ogre2BulletVector(XMLUtils::GetVectorParam<Ogre::Vector3>("direction", wn));
        ad.wheelData[i].axle = HeloUtils::Ogre2BulletVector(XMLUtils::GetVectorParam<Ogre::Vector3>("axle", wn));
        ad.wheelData[i].radius = XMLUtils::GetAttribute<float>("radius", wn);
        ad.wheelData[i].spring = XMLUtils::GetAttribute<float>("spring", wn);
        ad.wheelData[i].dampUp = XMLUtils::GetAttribute<float>("dampUp", wn);
        ad.wheelData[i].dampDown = XMLUtils::GetAttribute<float>("dampDown", wn);
        ad.wheelData[i].steerCoeff = XMLUtils::GetAttribute<float>("steerCoeff", wn);
        ad.wheelData[i].driveCoeff = XMLUtils::GetAttribute<float>("driveCoeff", wn);
        ad.wheelData[i].brakeCoeff = XMLUtils::GetAttribute<float>("brakeCoeff", wn);;
        ad.wheelData[i].momentOfInertia = XMLUtils::GetAttribute<float>("momentOfInertia", wn);
      }

    TiXmlNode *ensn = XMLUtils::AssertGetNode(n, "engines");
    ad.engineData.resize(XMLUtils::GetNumChildren(ensn, "engine"));
    TiXmlNode *en = NULL;
    for (int i = 0; (en = ensn->IterateChildren(std::string("engine"), en)); ++i)
      {
        ad.engineData[i].position = XMLUtils::GetVectorParam<Ogre::Vector3>("position", en);
        ad.engineData[i].direction = XMLUtils::GetVectorParam<Ogre::Vector3>("direction", en);
        ad.engineData[i].maxThrust = XMLUtils::GetAttribute<float>("maxthrust", en);
      }

    TiXmlNode *dyn = XMLUtils::AssertGetNode(n, "aerodynamics");
    ad.dragPolarK = XMLUtils::GetAttribute<float>("dragPolarK", dyn);
    ad.wingArea = XMLUtils::GetAttribute<float>("wingarea", dyn);
    ad.rudderSensitivity = XMLUtils::GetAttribute<float>("rudderSensitivity", dyn);
    ad.elevatorSensitivity = XMLUtils::GetAttribute<float>("elevatorSensitivity", dyn);
    ad.aileronSensitivity = XMLUtils::GetAttribute<float>("aileronSensitivity", dyn);

    TiXmlNode *stab = XMLUtils::AssertGetNode(n, "aerodynamics", "stability");
    ad.pitchStability1 = XMLUtils::GetAttribute<float>("pitch1", stab);
    ad.pitchStability2 = XMLUtils::GetAttribute<float>("pitch2", stab);
    ad.yawStability1 = XMLUtils::GetAttribute<float>("yaw1", stab);
    ad.yawStability2 = XMLUtils::GetAttribute<float>("yaw2", stab);
    ad.rollStability = XMLUtils::GetAttribute<float>("roll", stab);
      
    ParsePointList(XMLUtils::AssertGetNode(n, "aerodynamics", "liftcoefficient"),
                   ad.cl_alpha_values);
  } 
  catch (XMLUtils::NodeLookupError e)
    {
      ConfigurationError(n->GetDocument()->ValueStr(), e.what());
    }

  Airplane *airplane = new Airplane(ad, root);
  physics->addObject(airplane);
  controllables.push_back(airplane);
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

      std::string vehicle_class(XMLUtils::GetAttribute<std::string>("class", n));
      if (vehicle_class == "Car")
	loadCar(n, name, position);
      else if (vehicle_class == "Helicopter")
	loadHelicopter(n, name, position);
      else if (vehicle_class == "Tank")
	loadTank(n, name, position);
      else if (vehicle_class == "Airplane")
	loadAirplane(n, name, position);
      else
	ConfigurationError(xml->getName(), "Unsupported vehicle class'" + vehicle_class + "'");
    }
}

void Configuration::loadMission(std::string mission_name)
{
  assert(physics);
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
