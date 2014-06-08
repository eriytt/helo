#ifndef HARDPOINTS_H
#define HARDPOINTS_H

#include <map>

#include <Ogre.h>

class Hardpoint 
{
 public:
  class Point
  {
  protected:
    int id;
    Ogre::Vector3 position;
    Ogre::Vector3 direction;
  public:
    Point(int i, const Ogre::Vector3 &pos, const Ogre::Vector3 &dir) :
      id(i), position(pos), direction(dir) {}
  };

  typedef std::vector<Point> Config;
  
 protected:
  static std::map<std::string, Config> configRegistry;

 public:
  static void RegisterConfig(const std::string &name, const Config &conf);
  static const Config &GetConfig(const std::string &name);
  static bool HasConfig(const std::string &name);
};

#endif // HARDPOINTS_H
