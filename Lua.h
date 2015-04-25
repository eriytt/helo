#ifndef LUA_H
#define LUA_H

#include <sstream>

#include <lua.h>
#include <lauxlib.h>
#include <lualib.h>

#include "Readline.h"
#include "ScriptEngine.h"

class LuaConsole: public XTermConsole
{
public:
  LuaConsole(const std::string &xterm = "xterm") : XTermConsole(xterm) {}
  virtual ~LuaConsole() {}

};


class Lua : public ScriptEngine, public Readline::LineHandler
{
protected:
  lua_State *luaState;
public:
  Lua(std::string program, bool console, const std::string &xterm = "xterm");
  virtual ~Lua();
  bool needsToRun();
  void run(void);

  // from ReadLine::LineHandler
  void operator()(char *line);
};

#endif // LUA_H
