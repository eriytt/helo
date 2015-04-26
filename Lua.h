#ifndef LUA_H
#define LUA_H

#include <sstream>

#include <lua.hpp>
#include <luaconf.h>

// TODO: Why the hell is this not defined in luaconf.h
#define LUA_PROMPT		"> "
#define LUA_PROMPT2		">> "

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
  LuaConsole *console;
  Readline *readline;
  std::string command;

public:
  static FILE *consoleFP;

public:
  Lua(std::string program, bool console, const std::string &xterm = "xterm");
  virtual ~Lua();
  bool needsToRun();
  void run(void);

  // from ReadLine::LineHandler
  void operator()(char *line);
};

#endif // LUA_H
