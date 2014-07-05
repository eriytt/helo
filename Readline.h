#ifndef READLINE_H
#define READLINE_H

#include <exception>

#include "ExtConsole.h"

class ReadlineException: public std::exception
{
protected:
  std::string msg;
  std::string err;


public:
  ReadlineException(const std::string &msg, const std::string &err)
    : msg(msg), err(err) {}
  virtual const char* what() const throw()
  {
    // TODO: fix leak
    std::string *error_message = new std::string(msg + ": " + err);
    return error_message->c_str();
  }
};

class Readline
{
public:
  class LineHandler
  {
  public:
    virtual void operator()(char *line) = 0;
  };

 protected:
  ExtConsole &console;
  std::string prompt;
  int console_fd;
  LineHandler *callback;

private:
  Readline(ExtConsole &console, const std::string &prompt);

protected:
  bool updateFD();

public:
  static Readline *CreateReadline(ExtConsole &console, const std::string &prompt);
  static void CallbackTrampoline(char *);
  void setPrompt(const std::string &prompt) {this->prompt = prompt;}
  std::string read();
  void write(const std::string &out);
  void installCallback(LineHandler *callback);
  void uninstallCallback();
  void readAsync();
};

#endif // READLINE_H
