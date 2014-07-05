#include "Python.h"

#include <iostream>

#include <poll.h>


Python::Python(bool console, const std::string &xterm) : console(nullptr)
{
  if (console) {
    this->console = new PythonConsole(xterm);
    this->console->open();

    readline = Readline::CreateReadline(*this->console, ">>> ");

    readline->installCallback(this);
  }
}

bool Python::needsToRun()
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


void Python::run(void)
{
  //std::cout << "Running python" << std::endl;

  if (console)
    readline->readAsync();
}

void Python::operator()(char *line)
{
  std::cout << "Callback: " << line << std::endl;
}
