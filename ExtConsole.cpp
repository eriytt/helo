#include "ExtConsole.h"

#include <stdlib.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>

#include <cstring>
#include <cstdio>
#include <iostream>

ExtConsole::ExtConsole(const std::string &path, CommandLineBuilder *cmd):
  executable(path), cmd_builder(cmd), master_fd(-1), console_pid(0)
{
}

void ExtConsole::open()
{
  int fd;

  if (master_fd != -1)
    fd = master_fd;
  else
    {
      fd = posix_openpt(O_RDWR|O_NOCTTY);
      if (fd == -1)
	{
	  throw new ExtConsoleException("openpt", strerror(errno));
	}

      if (grantpt(fd) == -1)
	{
	  throw new ExtConsoleException("grantpt", strerror(errno));
	}

      if (unlockpt(fd) == -1)
	{
	  throw new ExtConsoleException("unlockpt", strerror(errno));
	}
    }

  std::string pts_name = ptsname(fd);

  if ((slave_fd = ::open(pts_name.c_str(), O_RDWR)) < 0)
    {
      throw new ExtConsoleException("(slave)open", strerror(errno));
    }


  pid_t pid = fork();
  if (pid == -1)
    {
      throw new ExtConsoleException("fork", strerror(errno));
    }

  if (pid > 0)
    {
      // This is the parent (slave side), it talks to the slave fd

      // ::close(master_fd);
      // TODO: can the slave side keep the master fd open so that a new 
      //       terminal can be opened from it?
      std::cout << "Forked console on pid " << pid << std::endl;
      master_fd = fd;
      console_pid = pid;
      return;
    }

  // This is the child (master side), it talkes to the master fd
  // It has no use for the slave fd
 // TODO: check for errors on close
  ::close(0);
  ::close(1);
  ::close(2);
  ::close(slave_fd); // TODO: check for error

  char **args = cmd_builder->buildCommandLine(executable, pts_name, fd);
  execvp(args[0], args);
  // exec failed, this will never terminate the slave side process
  throw new ExtConsoleException("exec", strerror(errno));
}

void ExtConsole::close()
{
  if (kill(console_pid, SIGTERM))
    throw new ExtConsoleException("kill", strerror(errno));
  // TODO: wait for SIGCHLD?
  console_pid = 0;
}


int ExtConsole::getFD()
{
  return slave_fd;
}

// xterm writes its X id as the first 8 bytes unconditionally,
// read them and drop them
void XTermConsole::open()
{
  ExtConsole::open();
  char xid_buf[8];
  int bytes_read = 0;

  while (bytes_read < 8)
    bytes_read += read(getFD(), xid_buf, sizeof(xid_buf) - bytes_read);
}

char **XTermConsole::XTermCommandLineBuilder::buildCommandLine(const std::string &executable, const std::string &pts, int fd)
{
  int arg = 0;
  char **args = new char*[100];
  args[arg] = new char[executable.size() + 1];
  executable.copy(args[arg], executable.size(), 0);
  args[arg][executable.size()] = 0;

  args[++arg] = new char[6];
  strncpy(args[arg], "-hold", 6);

  args[++arg] = new char[pts.size() + 10];
  snprintf(args[arg], pts.size() + 9, "-S%s/%d", pts.c_str(), fd);

  args[++arg] = 0;

  return args;
}
