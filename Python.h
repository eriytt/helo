#ifndef PYTHON_H
#define PYTHON_H

#include <sstream>

#include <Python.h>

#include "Readline.h"
#include "ScriptEngine.h"

class PythonConsole: public XTermConsole
{
public:
  PythonConsole(const std::string &xterm = "xterm") : XTermConsole(xterm) {}
  virtual ~PythonConsole() {}

};


class Python : public ScriptEngine, Readline::LineHandler
{
protected:
  PythonConsole *console;
  Readline *readline;
  char *prog;
  PyObject *interpreter;
  std::string command;

public:
  Python(std::string program, bool console, const std::string &xterm = "xterm");
  virtual ~Python();
  bool needsToRun();
  void run(void);
  void runFile(const std::string &script);

  // from ReadLine::LineHandler
  void operator()(char *line);
};

#endif // PYTHON_H
