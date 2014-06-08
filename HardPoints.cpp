#include "HardPoints.h"

std::map<std::string, Hardpoint::Config> Hardpoint::configRegistry;

void Hardpoint::RegisterConfig(const std::string &name, const Config &conf)
{
  Hardpoint::configRegistry[name] = conf;
}

const Hardpoint::Config &Hardpoint::GetConfig(const std::string &name)
{
  // TODO: raise exception, how can this even compile?
  return Hardpoint::configRegistry[name];
}

bool Hardpoint::HasConfig(const std::string &name)
{
  return false;
}
