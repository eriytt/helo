#include "Lua.h"

#include <stdexcept>

#include <cassert>

#include <poll.h>

#include <unistd.h>

FILE *Lua::consoleFP = NULL;

static int luaB_print_console (lua_State *L) {
  int n = lua_gettop(L);  /* number of arguments */
  int i;
  lua_getglobal(L, "tostring");
  for (i=1; i<=n; i++) {
    const char *s;
    lua_pushvalue(L, -1);  /* function to be called */
    lua_pushvalue(L, i);   /* value to print */
    lua_call(L, 1, 1);
    s = lua_tostring(L, -1);  /* get result */
    if (s == NULL)
      return luaL_error(L, LUA_QL("tostring") " must return a string to "
                           LUA_QL("print"));
    if (i>1) fputs("\t", Lua::consoleFP);
    fputs(s, Lua::consoleFP);
    lua_pop(L, 1);  /* pop result */
  }
  fputs("\n", Lua::consoleFP);
  return 0;
}

Lua::Lua(std::string program, bool console, const std::string &xterm)
{
  if (console)
    {
      this->console = new LuaConsole(xterm);
      this->console->open();

      readline = Readline::CreateReadline(*this->console, LUA_PROMPT);

      readline->installCallback(this);
      consoleFP = fdopen(this->console->getFD(), "w");
      }
  else
    consoleFP = stdout;

  // TODO: error check
  luaState = lua_open();
  luaL_openlibs(luaState);

  if (console)
    // Use our own print that does not go to stdout
    lua_register(luaState, "print", luaB_print_console);
}

Lua::~Lua()
{
}

void Lua::openLib(LuaOpenFunc f)
{
  // TODO: do we need to clean the stack after this?
  if (not f(luaState))
    throw std::runtime_error("Unable to open library");
}

bool Lua::needsToRun()
{
  if (not console)
    return false;

  struct pollfd pfd = {
    .fd = console->getFD(),
    .events = POLLIN | POLLPRI,
    .revents = 0
  };

  return ::poll(&pfd, 1, 0) > 0;

}

void Lua::run(void)
{
  if (console)
    readline->readAsync();
}

void Lua::runFile(const std::string &script)
{
  assert(luaState);

  if (luaL_loadfile(luaState, script.c_str()) || lua_pcall(luaState, 0, 0, 0))
  // TODO: throw another exception?
    throw std::runtime_error(std::string("Cannot run lua file: ") + lua_tostring(luaState, -1));
}

void Lua::operator()(char *line)
{
    if (not line) // TODO: is this eof? close terminal?
    return;

  command.append(line);

  int res = luaL_loadbuffer(luaState, command.c_str(), command.length(), "interactive");

  if (res == LUA_ERRSYNTAX)
    {
      std::string msg(lua_tolstring(luaState, -1, NULL));
      std::string eof(LUA_QL("<eof>"));

      if (msg.rfind(eof) == (msg.length() - eof.length())) // endswith  '<eof>'
	{
	  // Not a complete line of code
	  lua_pop(luaState, 1);
	  command.append("\n");
	  readline->setPrompt(LUA_PROMPT2);
	}
      else
	{
	  // TODO: Are we sure that we have a console here?
	  write(console->getFD(), (msg + "\n").c_str(), msg.length() + 1);
	  readline->setPrompt(LUA_PROMPT);
	  command.clear();
	}
    }
  else if (res == LUA_ERRMEM)
    {
      // Throw exception?
      throw std::runtime_error("Lua out of memory");
    }
  else
    {
      res = lua_pcall(luaState, 0, 0, 0);
      if (res)
	{
	  std::string errmsg(lua_tostring(luaState, -1));
	  write(console->getFD(), (errmsg + '\n').c_str(), errmsg.length() + 1);
	  lua_pop(luaState, 1);  /* pop error message from the stack */
	}

      command.clear();
      readline->setPrompt(LUA_PROMPT);
    }

}

void Lua::DumpStack(lua_State* l)
{
    int i;
    int top = lua_gettop(l);

    printf("%d entries in stack 0x%p\n", top, l);

    for (i = 1; i <= top; i++)
    {  /* repeat for each level */
        int t = lua_type(l, i);
        switch (t) {
            case LUA_TSTRING:  /* strings */
                printf("string: '%s'\n", lua_tostring(l, i));
                break;
            case LUA_TBOOLEAN:  /* booleans */
                printf("boolean %s\n",lua_toboolean(l, i) ? "true" : "false");
                break;
            case LUA_TNUMBER:  /* numbers */
                printf("number: %g\n", lua_tonumber(l, i));
                break;
            default:  /* other values */
                printf("%s\n", lua_typename(l, t));
                break;
        }
    }
}
