#ifndef SERVER_H
#define SERVER_H

#include <mutex>
// #include <sys/types.h>
// #include <sys/socket.h>
#include <netinet/in.h>

#include <socketstream.hh>

#include "Thread.h"
#include "Configuration.h"
#include "Serialize.hh"
#include "Physics.h"

class Server: public Thread, public Configuration::LoadListener, public Physics::Listener
{
public:
  class Error: public std::runtime_error
  {
  public:
    Error(const std::string& what_arg): std::runtime_error(what_arg) {}
    Error(const char *what_arg): std::runtime_error(what_arg) {}
  };
  
private:
  unsigned short port;
  unsigned char num_clients;

  struct Client
  {
    struct sockaddr_in clientAddress;
    Serializer<swoope::socketstream> socket;

    Client(int fd, const struct sockaddr_in &con) :
      clientAddress(con)
    {
      socket.getStream().open(fd);
    }
  };

  struct ServerNode
  {
    static uint32_t idCounter;
    uint32_t id;
    const HeloMotionState *motionState;
    Ogre::Vector3 pos;
    Ogre::Quaternion rot;
    std::string meshName;
    bool dirty;
    ServerNode(const Ogre::Vector3 &position, const Ogre::Quaternion &rotation):
      id(++idCounter), motionState(nullptr), pos(position), rot(rotation), dirty(false) {}
  };

  class Updater: public ThreadWorker
  {
    friend class Server;
    Server *server;
  protected:
    Updater(Server *s): server(s) {}
    void work();
  };
  
private:
  int sockfd, newsockfd, n, pid;
  struct sockaddr_in serverAddress;

  std::mutex clientLock;
  std::vector<Client*> clients;
  std::mutex nodeLock;
  std::map<Ogre::SceneNode*, ServerNode*> nodes;
  unsigned char update_freq;

  void work();
  void vehicleLoaded(const std::string &type,
                     const std::string &name,
                     const Ogre::Vector3 &position,
                     const Ogre::Vector3 &rotation,
                     const Vehicle *v);
  void motionStateAdded(const HeloMotionState &ms);

protected:
  void updateClients();
  
public:
  Server(unsigned short port, unsigned char num_clients, unsigned char update_freq);
};

#endif // SERVER_H
