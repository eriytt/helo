#include "Readline.h"

#include <iostream>
#include <errno.h>

#include <unistd.h>

#include <readline/readline.h>

static Readline *singleton = nullptr;

Readline *Readline::CreateReadline(ExtConsole &con, const std::string &prompt)
{
  return new Readline(con, prompt);
}

Readline::Readline(ExtConsole &console, const std::string &prt) :
  console(console), console_fd(-1),  callback(nullptr)
{
  if (singleton)
    throw ReadlineException("Hello", "Only a single instance of Readline allowed");

  this->prompt = prt;
  singleton = this;
}

bool Readline::updateFD()
{
  int cfd = console.getFD();

  if (cfd < 0)
    {
      console_fd = -1;
      return false;
    }
  else if (cfd == console_fd)
    return true;

  console_fd = cfd;
  FILE *console_fp = fdopen(console_fd, "r+");
  if (not console_fp)
    throw ReadlineException("fdopen", strerror(errno)); // TODO: Read errno

  rl_instream = console_fp;
  rl_outstream = console_fp;

  return true;
}

void Readline::CallbackTrampoline(char *line)
{
  LineHandler *lh = singleton->callback;
  (*lh)(line);
}

void Readline::setPrompt(const std::string &prompt)
{
  this->prompt = prompt;
  rl_set_prompt(this->prompt.c_str());
}

void Readline::uninstallCallback()
{
  rl_callback_handler_remove();
  callback = nullptr;
}

void Readline::installCallback(LineHandler *cb)
{
  if (not updateFD())
    return;

  rl_callback_handler_install(prompt.c_str(), CallbackTrampoline);
  callback = cb;
}

std::string Readline::read()
{
  if (callback)
    throw ReadlineException("Readline::read()", "Console is in async mode, use poll");

 if (not updateFD())
    return "";

 char *l = readline(prompt.c_str());
 // TODO: check for null return?


 std::string line(l);
 free(l);
 // TODO: use move semantics if possible
 return line;
}

void Readline::write(const std::string &out)
{
  if (not updateFD())
    return;

  const char *s = out.c_str();
  unsigned int bytes_written = 0;
  unsigned int len = out.length();
  while (bytes_written < len)
    // TODO: handle errors
    bytes_written += ::write(console.getFD(), &s[bytes_written], len - bytes_written);  
}

void Readline::readAsync()
{
  if (not callback)
    throw ReadlineException("Readline::poll()", "Cannot poll in sync mode, use read");

  rl_callback_read_char();
}
