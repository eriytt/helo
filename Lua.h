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
public:
  typedef int (*LuaOpenFunc)(lua_State* L);


public:
  static void DumpStack(lua_State*);

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
  void openLib(LuaOpenFunc f);

  // from ScriptEngine
  bool needsToRun();
  void run(void);
  void runFile(const std::string &script);

  // from ReadLine::LineHandler
  void operator()(char *line);
};


#include "EventQueue.h"

template <typename T>
class LuaEventCallback : public EventQueue<T>::Event
{
protected:
  lua_State *L;
  int callback_registry_key;
public:
  LuaEventCallback(lua_State *L, int key) : L(L), callback_registry_key(key) {}
  virtual void operator()(T actual_time, T event_time, typename EventQueue<T>::EventID id) const {
    // set up the stack with the function and input arguments
    lua_rawgeti(L, LUA_REGISTRYINDEX, callback_registry_key);
    lua_pushinteger(L, actual_time);
    lua_pushinteger(L, event_time);
    lua_pushinteger(L, id);

    // do the call
    int err = lua_pcall(L, 3, 0, 0);
    if (err) {
      std::string errmsg("Lua event failed: ");
      switch (err)
	{
	case LUA_ERRRUN:
	  errmsg += "LUA_ERRRUN: "; break;
	case LUA_ERRMEM:
	  errmsg += "LUA_ERRMEM: "; break;
	case LUA_ERRERR:
	  errmsg += "LUA_ERRERR: "; break;
	default:
	  errmsg += "Unknown lua error code: "; break;
	}

      // Stack top (-1) contains the error message
      errmsg += lua_tostring(L, -1);
      lua_settop(L, 0);

      throw std::runtime_error(errmsg);
    }

    // Discard possible return values
    lua_settop(L, 0);
  }
};

#endif // LUA_H
