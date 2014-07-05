#ifndef PYTHON_H
#define PYTHON_H

#include "Readline.h"

class PythonConsole: public XTermConsole
{
public:
  PythonConsole(const std::string &xterm = "xterm") : XTermConsole(xterm) {}

};


class Python : Readline::LineHandler
{
protected:
  PythonConsole *console;
  Readline *readline;

public:
  Python(bool console, const std::string &xterm = "xterm");
  bool needsToRun();
  void run(void);

  // from ReadLine::LineHandler
  void operator()(char *line);
};

#endif // PYTHON_H
