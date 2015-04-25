#ifndef SCRIPT_ENGINE
#define SCRIPT_ENGINE

class ScriptEngine
{
public:
  virtual ~ScriptEngine() {}
  virtual bool needsToRun() = 0;
  virtual void run(void) = 0;
};

#endif // SCRIPT_ENGINE
