#include "Configuration.h"

#include <Ogre.h>
#include <Plugins/BSPSceneManager/OgreBspSceneManagerPlugin.h>

#include "TinyXMLResourceManager.h"
#include "XML.h"
#include "Car.h"
#include "Helicopter.h"
#include "Tank.h"
#include "Airplane.h"

Configuration::Configuration(Ogre::Root *r) : startMission(""), root(r), physics(NULL), runPhysicsInThread(false),  python(false), lua(false)
{
  // TODO: lookup xterm in path
  xterm = "/usr/bin/xterm";
}

void Configuration::loadConfig()
{
  TinyXMLPtr xml = TinyXMLResourceManager::getSingleton().createResource("helo.xml", "helo").staticCast<TinyXMLResource>();

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
	  readSettings(sn);

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

void Configuration::readScriptEngine(TiXmlNode *se)
{
  TiXmlNode* run = NULL;

  while((run = se->IterateChildren("run", run))) {
    bool postconf = false;

    if (XMLUtils::HasAttribute("postconf", run))
      postconf = XMLUtils::GetAttribute<bool>("postconf", run);

    (postconf ? postScripts : preScripts).push_back(XMLUtils::GetAttribute<std::string>("script", run));
  }
}

void Configuration::readSettings(TiXmlNode *settings)
{
  runPhysicsInThread = XMLUtils::GetAttribute<bool>("physicsInThread", settings);

  if (XMLUtils::HasAttribute("xterm", settings))
    xterm = XMLUtils::GetAttribute<std::string>("xterm", settings);

  TiXmlNode *child = NULL;

  child = XMLUtils::AssertGetNode(settings, "Server");
  sconf.enabled = XMLUtils::GetAttribute<bool>("enabled", child);
  sconf.port = XMLUtils::GetAttribute<unsigned short>("port", child);
  sconf.num_clients = XMLUtils::GetAttribute<unsigned char>("num_clients", child);
  sconf.update_freq = XMLUtils::GetAttribute<unsigned char>("update_freq", child);

  while((child = settings->IterateChildren("Python", child)))
    {
      if ((python = XMLUtils::GetAttribute<bool>("enabled", child)))
	readScriptEngine(child);
    }

  while((child = settings->IterateChildren("Lua", child)))
    {
      if ((lua = XMLUtils::GetAttribute<bool>("enabled", child)))
	readScriptEngine(child);
    }

  if (python and lua)
    throw ConfigurationError(settings->GetDocument()->ValueStr(), "Scripting in both lua and python not supported");

}

void Configuration::readMissions(TiXmlNode *parent)
{
  if (XMLUtils::HasAttribute("start", parent))
    startMission = XMLUtils::GetAttribute<std::string>("start", parent);

  TiXmlNode *child = NULL;
  while((child = parent->IterateChildren(child)))
    {
      Mission mission;
      mission.name = XMLUtils::GetAttribute<std::string>("name", child);
      mission.path = XMLUtils::GetAttribute<std::string>("path", child);
      missions.push_back(mission);
    }

  // TODO: Should this really be necessary? Starting with empty
  // missions should be allowed.
  if (not missions.size())
    throw ConfigurationError(parent->GetDocument()->ValueStr(),
                             "No mission definitions found'");

  if (startMission != "" and not std::any_of(missions.begin(), missions.end(), [&](const Mission &m) {return m.name == startMission;}))
    throw ConfigurationError(parent->GetDocument()->ValueStr(), std::string("Mission ") + startMission + " not defined");
}

Car::WheelData loadSpinWheel(TiXmlNode *wn)
{
  Car::WheelData wd;
  wd.name = XMLUtils::GetAttribute<std::string>("name", wn);
  wd.relPos = HeloUtils::Ogre2BulletVector(XMLUtils::GetVectorParam<Ogre::Vector3>("relativePosition", wn));
  wd.axle = HeloUtils::Ogre2BulletVector(XMLUtils::GetVectorParam<Ogre::Vector3>("axle", wn));
  wd.meshname = XMLUtils::GetAttribute<Ogre::String>("meshname", wn);
  wd.radius = XMLUtils::GetAttribute<float>("radius", wn);
  return wd;
}

Car::DriveWheelData loadDriveWheel(TiXmlNode *wn)
{
  Tank::DriveWheelData wd;
  wd.name = XMLUtils::GetAttribute<std::string>("name", wn);
  wd.relPos = HeloUtils::Ogre2BulletVector(XMLUtils::GetVectorParam<Ogre::Vector3>("relativePosition", wn));
  wd.realRelPos = HeloUtils::Ogre2BulletVector(XMLUtils::GetVectorParam<Ogre::Vector3>("realRelativePosition", wn));
  wd.suspensionLength = XMLUtils::GetAttribute<float>("suspensionLength", wn);
  wd.maxLengthUp = XMLUtils::GetAttribute<float>("maxLengthUp", wn);
  wd.maxLengthDown = XMLUtils::GetAttribute<float>("maxLengthDown", wn);
  wd.direction = HeloUtils::Ogre2BulletVector(XMLUtils::GetVectorParam<Ogre::Vector3>("direction", wn));
  wd.axle = HeloUtils::Ogre2BulletVector(XMLUtils::GetVectorParam<Ogre::Vector3>("axle", wn));
  wd.radius = XMLUtils::GetAttribute<float>("radius", wn);
  wd.spring = XMLUtils::GetAttribute<float>("spring", wn);
  wd.dampUp = XMLUtils::GetAttribute<float>("dampUp", wn);
  wd.dampDown = XMLUtils::GetAttribute<float>("dampDown", wn);
  wd.steerCoeff = XMLUtils::GetAttribute<float>("steerCoeff", wn);
  wd.driveCoeff = XMLUtils::GetAttribute<float>("driveCoeff", wn);
  wd.brakeCoeff = XMLUtils::GetAttribute<float>("brakeCoeff", wn);;
  wd.momentOfInertia = XMLUtils::GetAttribute<float>("momentOfInertia", wn);
  wd.meshname = XMLUtils::GetAttribute<Ogre::String>("meshname", wn);
  return wd;
}

Car::SuspensionWheelData loadSuspensionWheel(TiXmlNode *wn)
{
  Car::SuspensionWheelData wd;
  wd.name = XMLUtils::GetAttribute<std::string>("name", wn);
  wd.relPos = HeloUtils::Ogre2BulletVector(XMLUtils::GetVectorParam<Ogre::Vector3>("relativePosition", wn));
  wd.suspensionLength = XMLUtils::GetAttribute<float>("suspensionLength", wn);
  wd.maxLengthUp = XMLUtils::GetAttribute<float>("maxLengthUp", wn);
  wd.maxLengthDown = XMLUtils::GetAttribute<float>("maxLengthDown", wn);
  wd.direction = HeloUtils::Ogre2BulletVector(XMLUtils::GetVectorParam<Ogre::Vector3>("direction", wn));
  wd.axle = HeloUtils::Ogre2BulletVector(XMLUtils::GetVectorParam<Ogre::Vector3>("axle", wn));
  wd.radius = XMLUtils::GetAttribute<float>("radius", wn);
  wd.spring = XMLUtils::GetAttribute<float>("spring", wn);
  wd.dampUp = XMLUtils::GetAttribute<float>("dampUp", wn);
  wd.dampDown = XMLUtils::GetAttribute<float>("dampDown", wn);
  wd.steerCoeff = XMLUtils::GetAttribute<float>("steerCoeff", wn);
  wd.driveCoeff = XMLUtils::GetAttribute<float>("driveCoeff", wn);
  wd.brakeCoeff = XMLUtils::GetAttribute<float>("brakeCoeff", wn);;
  wd.momentOfInertia = XMLUtils::GetAttribute<float>("momentOfInertia", wn);
  wd.meshname = XMLUtils::GetAttribute<Ogre::String>("meshname", wn);

  return wd;
}

Car::BodyData loadBody(TiXmlNode *n)
{
  Car::BodyData data;

  data.name = XMLUtils::GetAttribute<std::string>("name", n);
  data.meshname = XMLUtils::GetAttribute<std::string>("meshname", n);
  data.mass = XMLUtils::GetAttribute<float>("weight", n);
  data.size = XMLUtils::GetVectorParam<Ogre::Vector3>("size", n);
  data.relativePosition = XMLUtils::GetVectorParam<Ogre::Vector3>("relativePosition", n);
  data.colShapeOffset = XMLUtils::GetVectorParam<Ogre::Vector3>("collisionShapeOffset", n);
  data.isRaycaster = false;

  TiXmlNode *c = NULL;
  for (int i = 0; (c = n->IterateChildren("body", c)); ++i)
    data.children.push_back(loadBody(c));

  for (int i = 0; (c = n->IterateChildren("wheel", c)); ++i)
    {
      data.isRaycaster = true;
      std::string type = XMLUtils::GetAttribute<std::string>("type", c);
      if (type == "spin")
        data.wheels.push_back(loadSpinWheel(c));
      else if (type == "drive")
        data.driveWheels.push_back(loadDriveWheel(c));
      else if (type == "suspension")
        data.suspensionWheels.push_back(loadSuspensionWheel(c));
      else
        throw Configuration::ConfigurationError
          (c->GetDocument()->ValueStr(),
           std::string("Wheel cannot have type '") + type + "'");
    }
  return data;
}

Vehicle *Configuration::loadCar(TiXmlNode *n, const std::string &name, const Ogre::Vector3 &position, const Ogre::Vector3 &rotation)
{
  Car::CarData cd;
  cd.position = position;
  cd.rotation = rotation;
  cd.name = name;

  TiXmlNode *bsn = n->IterateChildren("bodies", NULL);
  if (not bsn)
    throw ConfigurationError(n->GetDocument()->ValueStr(),
			     n->ValueStr() + " has no child child named 'bodies'");
  TiXmlNode *bn = NULL;
  for (int i = 0; (bn = bsn->IterateChildren(std::string("body"), bn)); ++i)
    cd.bodies.push_back(loadBody(bn));

  Car *car = (new Car)->load(cd, root);
  physics->addObject(car);
  controllables.push_back(car);
  return car;
}

Vehicle *Configuration::loadHelicopter(TiXmlNode *n, const std::string &name, const Ogre::Vector3 &position, const Ogre::Vector3 &rotation)
{
  Helicopter::HelicopterData hd;
  hd.name = name;
  hd.pos = position;
  hd.rotation = rotation;

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
  return helicopter;
}

Vehicle *Configuration::loadTank(TiXmlNode *n, const std::string &name, const Ogre::Vector3 &position, const Ogre::Vector3 &rotation)
{
  Tank::TankData td;
  td.position = position;
  td.rotation = rotation;
  td.name = name;

  TiXmlNode *bsn = n->IterateChildren("bodies", NULL);
  if (not bsn)
    throw ConfigurationError(n->GetDocument()->ValueStr(),
			     n->ValueStr() + " has no child child named 'bodies'");
  TiXmlNode *bn = NULL;
  for (int i = 0; (bn = bsn->IterateChildren(std::string("body"), bn)); ++i)
    td.bodies.push_back(loadBody(bn));

  Tank *tank = (new Tank)->load(td, root);
  physics->addObject(tank);
  controllables.push_back(tank);
  return tank;
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


Vehicle *Configuration::loadAirplane(TiXmlNode *n, const std::string &name, const Ogre::Vector3 &position, const Ogre::Vector3 &rotation)
{
  // Airplane::AirplaneData ad;
  // ad.position = position;
  // ad.rotation = rotation;
  // ad.name = name;

  // ad.meshname = XMLUtils::GetAttribute<std::string>("meshname", n);
  // ad.weight = XMLUtils::GetAttribute<float>("weight", n);
  // ad.size = XMLUtils::GetVectorParam<Ogre::Vector3>("size", n);

  // try {
  //   TiXmlNode *wsn = XMLUtils::AssertGetNode(n, "wheels");
  //   ad.wheelData.resize(XMLUtils::GetNumChildren(wsn, "wheel"));

  //   TiXmlNode *wn = NULL;
  //   for (int i = 0; (wn = wsn->IterateChildren(std::string("wheel"), wn)); ++i)
  //     {
  //       ad.wheelData[i].relPos = HeloUtils::Ogre2BulletVector(XMLUtils::GetVectorParam<Ogre::Vector3>("relativePosition", wn));
  //       ad.wheelData[i].suspensionLength = XMLUtils::GetAttribute<float>("suspensionLength", wn);
  //       ad.wheelData[i].maxLengthUp = XMLUtils::GetAttribute<float>("maxLengthUp", wn);
  //       ad.wheelData[i].maxLengthDown = XMLUtils::GetAttribute<float>("maxLengthDown", wn);
  //       ad.wheelData[i].direction = HeloUtils::Ogre2BulletVector(XMLUtils::GetVectorParam<Ogre::Vector3>("direction", wn));
  //       ad.wheelData[i].axle = HeloUtils::Ogre2BulletVector(XMLUtils::GetVectorParam<Ogre::Vector3>("axle", wn));
  //       ad.wheelData[i].radius = XMLUtils::GetAttribute<float>("radius", wn);
  //       ad.wheelData[i].spring = XMLUtils::GetAttribute<float>("spring", wn);
  //       ad.wheelData[i].dampUp = XMLUtils::GetAttribute<float>("dampUp", wn);
  //       ad.wheelData[i].dampDown = XMLUtils::GetAttribute<float>("dampDown", wn);
  //       ad.wheelData[i].steerCoeff = XMLUtils::GetAttribute<float>("steerCoeff", wn);
  //       ad.wheelData[i].driveCoeff = XMLUtils::GetAttribute<float>("driveCoeff", wn);
  //       ad.wheelData[i].brakeCoeff = XMLUtils::GetAttribute<float>("brakeCoeff", wn);;
  //       ad.wheelData[i].momentOfInertia = XMLUtils::GetAttribute<float>("momentOfInertia", wn);
  //     }

  //   TiXmlNode *ensn = XMLUtils::AssertGetNode(n, "engines");
  //   ad.engineData.resize(XMLUtils::GetNumChildren(ensn, "engine"));
  //   TiXmlNode *en = NULL;
  //   for (int i = 0; (en = ensn->IterateChildren(std::string("engine"), en)); ++i)
  //     {
  //       ad.engineData[i].position = XMLUtils::GetVectorParam<Ogre::Vector3>("position", en);
  //       ad.engineData[i].direction = XMLUtils::GetVectorParam<Ogre::Vector3>("direction", en);
  //       ad.engineData[i].maxThrust = XMLUtils::GetAttribute<float>("maxthrust", en);
  //     }

  //   TiXmlNode *dyn = XMLUtils::AssertGetNode(n, "aerodynamics");
  //   ad.dragPolarK = XMLUtils::GetAttribute<float>("dragPolarK", dyn);
  //   ad.dragPolarD0 = XMLUtils::GetAttribute<float>("dragPolarD0", dyn);
  //   ad.wingArea = XMLUtils::GetAttribute<float>("wingarea", dyn);
  //   ad.wingAngle = XMLUtils::GetAttribute<float>("wingangle", dyn);
  //   ad.rudderSensitivity = XMLUtils::GetAttribute<float>("rudderSensitivity", dyn);
  //   ad.elevatorSensitivity = XMLUtils::GetAttribute<float>("elevatorSensitivity", dyn);
  //   ad.aileronSensitivity = XMLUtils::GetAttribute<float>("aileronSensitivity", dyn);

  //   TiXmlNode *stab = XMLUtils::AssertGetNode(n, "aerodynamics", "stability");
  //   ad.pitchStability1 = XMLUtils::GetAttribute<float>("pitch1", stab);
  //   ad.pitchStability2 = XMLUtils::GetAttribute<float>("pitch2", stab);
  //   ad.yawStability1 = XMLUtils::GetAttribute<float>("yaw1", stab);
  //   ad.yawStability2 = XMLUtils::GetAttribute<float>("yaw2", stab);
  //   ad.rollStability = XMLUtils::GetAttribute<float>("roll", stab);
      
  //   ParsePointList(XMLUtils::AssertGetNode(n, "aerodynamics", "liftcoefficient"),
  //                  ad.cl_alpha_values);

  //   TiXmlNode *hpsn = XMLUtils::AssertGetNode(n, "hardpoints");
  //   TiXmlNode *hpcfg = NULL;
  //   for (int i = 0; (hpcfg = hpsn->IterateChildren(std::string("conf"), hpcfg)); ++i)
  //     {
  //       std::string id = XMLUtils::GetAttribute<std::string>("id", hpcfg);
  //       std::string idstr = n->GetDocument()->ValueStr() + id;
  //       if (Hardpoint::HasConfig(idstr))
  //         continue;
        
  //       Hardpoint::Config cfg = loadHardpointConfig(hpcfg);
  //       Hardpoint::RegisterConfig(idstr, cfg);
  //     }
  // } 
  // catch (XMLUtils::NodeLookupError e)
  //   {
  //     ConfigurationError(n->GetDocument()->ValueStr(), e.what());
  //   }

    

  // Airplane *airplane = new Airplane(ad, root);
  // physics->addObject(airplane);
  // controllables.push_back(airplane);
  // return airplane;
  return nullptr;
}

Vehicle *Configuration::loadVehicle(const std::string &type, const std::string &name, const Ogre::Vector3 &position, const Ogre::Vector3 &rotation)
{
  Ogre::ResourceGroupManager &rgm = Ogre::ResourceGroupManager::getSingleton();
  if (not rgm.resourceGroupExists("Vehicles/" + type))
    {
      rgm.createResourceGroup("Vehicles/" + type, false);
      rgm.addResourceLocation(resource_base_path +"Vehicles/" + type, "FileSystem", type);
      rgm.initialiseResourceGroup(type);
      TinyXMLPtr xml = TinyXMLResourceManager::getSingleton().createResource(type + ".xml", type).staticCast<TinyXMLResource>();
      xml->load();
    }

  TinyXMLPtr xml = TinyXMLResourceManager::getSingleton().createOrRetrieve(type + ".xml", type).first.staticCast<TinyXMLResource>();

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
      Vehicle *v = nullptr;
      if (vehicle_class == "Car")
        v = loadCar(n, name, position, rotation);
      else if (vehicle_class == "Helicopter")
	v = loadHelicopter(n, name, position, rotation);
      else if (vehicle_class == "Tank")
	v = loadTank(n, name, position, rotation);
      else if (vehicle_class == "Airplane")
	v = loadAirplane(n, name, position, rotation);
      else
	throw ConfigurationError(xml->getName(), "Unsupported vehicle class'" + vehicle_class + "'");

      for (auto l : listeners)
        l->vehicleLoaded(type, name, position, rotation, v);

      return v;
    }
  return NULL;
}

Hardpoint::Config Configuration::loadHardpointConfig(TiXmlNode *n)
{
  Hardpoint::Config cfg;
  TiXmlNode *cfgnode = n;

  TiXmlNode *hp = NULL; 
  for (int i = 0; (hp = cfgnode->IterateChildren(std::string("hardpoint"), hp)); ++i)
    {
      Ogre::Vector3 pos = XMLUtils::GetVectorParam<Ogre::Vector3>("position", hp);;
      Ogre::Vector3 dir = XMLUtils::GetVectorParam<Ogre::Vector3>("direction", hp);;
      int hpid = XMLUtils::GetAttribute<int>("id", hp);;
      cfg.push_back(Hardpoint::Point(hpid, pos, dir));
    }
  return cfg;
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

      TinyXMLPtr xml = TinyXMLResourceManager::getSingleton().createResource( m->name + ".xml", m->name).staticCast<TinyXMLResource>();
 
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
                  Ogre::Vector3 rot;
                  if (XMLUtils::GetNumChildren(v, "rotation"))
                    rot = HeloUtils::Deg2Rad(XMLUtils::GetVectorParam<Ogre::Vector3>("rotation", v));
                  Vehicle *vehicle = loadVehicle(type, name, pos, rot);

                  // TODO: factor out

                  if (TiXmlNode *arms = XMLUtils::GetNode(v, "armaments"))
                    {
                      std::string cfgname = type + XMLUtils::GetAttribute<std::string>("hardpointConfig", arms);
                      vehicle->setHardpointConfig(Hardpoint::GetConfig(cfgname));
                      vehicle->addArmament(new DumbBomb, 0);
                        
                      TiXmlNode *arm = NULL;
                      for (int i = 0; (arm = arms->IterateChildren(std::string("armament"), arm)); ++i)
                        {
                          // TODO: do some clever arms loading
                        }
                    }

                }
            }
        }
    } 
  catch (XMLUtils::XMLError e)
    {
      throw ConfigurationError("Error loading mission '" + mission_name + "': " + e.what());
    }

}
