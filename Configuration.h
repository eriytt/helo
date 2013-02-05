#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <string>
#include <vector>

#include <Ogre.h>

class TiXmlNode;
class Physics;
class Controllable;

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

  typedef struct
  {
    std::string name;
    std::string path;
  } Vehicle;

  std::vector<Controllable*> controllables;
  
protected:
  std::string resource_base_path;
  std::vector<Mission> missions;
  std::vector<Vehicle> vehicles;
  Ogre::Root *root;
  Physics *physics;

 public:
  Configuration(Ogre::Root *r, Physics *p) : root(r), physics(p) {}
  void setResourceBase(const std::string &base) {resource_base_path = base;}
  void loadConfig();
  void loadMission(std::string mission_name);
  const std::vector<Controllable*> &getControllables() {return controllables;}

protected:
  void readMissions(TiXmlNode *parent);
  void readVehicles(TiXmlNode *parent);
  void loadVehicle(const std::string &type, const std::string &name, const Ogre::Vector3 &position);
  void loadCar(TiXmlNode *n, const std::string &name, const Ogre::Vector3 &position);
  void loadHelicopter(TiXmlNode *n, const std::string &name, const Ogre::Vector3 &position);
};

#endif // CONFIGURATION_H
