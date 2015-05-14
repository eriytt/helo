#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <string>
#include <vector>

#include <Ogre.h>

#include "HardPoints.h"

class TiXmlNode;
class Physics;
class Controllable;
class Vehicle;

class Configuration
{
public:
  class ConfigurationError : public std::runtime_error
  {
  protected:
    const std::string file;
    const std::string reason;
  public:
    ConfigurationError(const std::string &filename, const std::string &msg) :
      runtime_error(""), file(filename), reason(msg) {}

    ConfigurationError(const std::string &msg) :
      runtime_error(""), file(""), reason(msg) {}

    ~ConfigurationError() throw() {}

    virtual const char* what() const throw()
    {
      if (file != "")
        return std::string(file + ": " + reason).c_str();
      return reason.c_str();
    }
  };

protected:
  typedef struct
  {
    std::string name;
    std::string path;
  } Mission;

  /* typedef struct */
  /* { */
  /*   std::string name; */
  /*   std::string path; */
  /* } VehicleSpec; */

  std::vector<Controllable*> controllables;

protected:
  std::string resource_base_path;
  std::vector<Mission> missions;
  std::string startMission;
  //std::vector<VehicleSpec> vehicles;
  Ogre::Root *root;
  Physics *physics;
  bool runPhysicsInThread;
  std::string xterm;
  bool python;
  bool lua;
  std::vector<std::string> preScripts;
  std::vector<std::string> postScripts;

 public:
  Configuration(Ogre::Root *r);
  void setResourceBase(const std::string &base) {resource_base_path = base;}
  void loadConfig();
  void setPhysics(Physics *p) {physics = p;}
  void loadMission(std::string mission_name);
  const std::vector<Controllable*> &getControllables() {return controllables;}
  bool physicsInThread() {return runPhysicsInThread;}

  const std::string & xtermPath() {return xterm;}
  bool usePython() {return python;}
  bool useLua() {return lua;}
  const std::vector<std::string> &getPreScripts() {return preScripts;}
  const std::vector<std::string> &getPostScripts() {return postScripts;}
  const std::string & getStartMission() {return startMission;}

public:
  Vehicle *loadVehicle(const std::string &type, const std::string &name, const Ogre::Vector3 &position, const Ogre::Vector3 &rotation);
  
protected:
  void readMissions(TiXmlNode *parent);
  void readSettings(TiXmlNode *settings);
  void readScriptEngine(TiXmlNode *se);
  Vehicle *loadCar(TiXmlNode *n, const std::string &name, const Ogre::Vector3 &position, const Ogre::Vector3 &rotation);
  Vehicle *loadHelicopter(TiXmlNode *n, const std::string &name, const Ogre::Vector3 &position, const Ogre::Vector3 &rotation);
  Vehicle *loadTank(TiXmlNode *n, const std::string &name, const Ogre::Vector3 &position, const Ogre::Vector3 &rotation);
  Vehicle *loadAirplane(TiXmlNode *n, const std::string &name, const Ogre::Vector3 &position, const Ogre::Vector3 &rotation);
  Hardpoint::Config loadHardpointConfig(TiXmlNode *n);

  static void ParsePointList(TiXmlNode *parent, std::vector<std::pair<Ogre::Real, Ogre::Real> > &pair_list);
};

#endif // CONFIGURATION_H
