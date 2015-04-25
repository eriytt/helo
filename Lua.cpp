#include "Lua.h"


Lua::Lua(std::string program, bool console, const std::string &xterm)
{
}

Lua::~Lua()
{
  luaState = lua_open();
  luaL_openlibs(luaState);
}

bool Lua::needsToRun()
{
  return false;
}

void Lua::run(void)
{
}

void Lua::operator()(char *line)
{
}
