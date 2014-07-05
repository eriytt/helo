#ifndef EXTCONSOLE_H
#define EXTCONSOLE_H

#include <string>
//#include <unistd.h>
#include <exception>

class ExtConsoleException : public std::exception
{
protected:
  std::string msg;
  std::string err;

public:
  ExtConsoleException(const std::string &message, const std::string &error):
    std::exception(), msg(message), err(error) {}
  virtual ~ExtConsoleException() throw() {}
};

class ExtConsole
{
public:
  class CommandLineBuilder {
  public:
    virtual char **buildCommandLine(const std::string &executable,
				    const std::string &pts, int fd) = 0;
  };

protected:
  std::string executable;
  CommandLineBuilder *cmd_builder;
  int master_fd;
  int slave_fd;
  int console_pid;

public:
  ExtConsole(const std::string &path, CommandLineBuilder *cmd_builder);
  virtual void open();
  virtual void close();
  // virtual ssize_t read(void *buf, size_t count);
  // virtual ssize_t write(const void *buf, size_t count);
  virtual int getFD();
};

class XTermConsole : public ExtConsole
{
public:

  class XTermCommandLineBuilder : public ExtConsole::CommandLineBuilder {

  public:
    char **buildCommandLine(const std::string &executable, const std::string &pts, int fd);
  };

public:
  XTermConsole(const std::string &path = "/usr/bin/xterm"):
    ExtConsole(path, new XTermCommandLineBuilder()) {}
  void open();
};

#endif // EXTCONSOLE_H
