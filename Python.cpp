#include "Python.h"

#include <iostream>
#include <sstream>

#include <poll.h>

#include <Python.h>

Python::Python(std::string program, bool console, const std::string &xterm) : 
  console(nullptr)
{
  if (console) {
    this->console = new PythonConsole(xterm);
    this->console->open();

    readline = Readline::CreateReadline(*this->console, ">>> ");

    readline->installCallback(this);
  }

  prog = new char[program.size() + 1];
  program.copy(prog, program.size() + 1, 0);
  prog[program.size()] = 0;

  Py_SetProgramName(prog);  /* optional but recommended */
  Py_InitializeEx(0);


  if (console)
    {
      std::ostringstream os;
      os << "import sys,os\n"
	"f = os.fdopen(" << this->console->getFD() << ", 'w')\n"
	"sys.stdout  = f\n"
	"sys.stderr = f\n"
	"sys.stdin = f\n";
      std::cout << "Running python code:" << std::endl << os.str().c_str();
      PyRun_SimpleString(os.str().c_str());

      // TODO: init code module
      PyRun_SimpleString("from time import time,ctime\n"
                     "print '*************************************Today is',ctime(time())\n");
    }
}

Python::~Python()
{
  if (console)
    {
      readline->uninstallCallback();
      delete readline;

      console->close();
      delete console;
    }

  Py_Finalize();
  delete prog;
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
