#include "Lua.h"

#include <exception>

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
      throw std::exception(); // "Lua out of memory");
    }
  else
    {
      res = lua_pcall(luaState, 0, 0, 0);
      if (res)
	{
	  std::string errmsg("Lua error: ");
	  errmsg += lua_tostring(luaState, -1);
	  write(console->getFD(), errmsg.c_str(), errmsg.length());
	  lua_pop(luaState, 1);  /* pop error message from the stack */
	}

      command.clear();
      readline->setPrompt(LUA_PROMPT);
    }

}
