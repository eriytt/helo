#include "Server.h"
#include "Utils.h"
#include "Vehicle.h"


uint32_t Server::ServerNode::idCounter = 0;

Server::Server(unsigned short port, unsigned char num_clients, unsigned char update_freq)
  : update_freq(update_freq)
{ 
  // TODO: Error checking
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd == -1)
    throw Error(std::string("socket(): ") + std::strerror(errno));

  int reuse = 1;
  if (-1 == setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)))
    throw Error(std::string("setsockopt(): ") + std::strerror(errno));
  
  memset(&serverAddress, 0, sizeof(serverAddress));
  serverAddress.sin_family = AF_INET;
  serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);
  serverAddress.sin_port = htons(port);

  if (-1 == bind(sockfd, (struct sockaddr *)&serverAddress, sizeof(serverAddress)))
    throw Error(std::string("bind(): ") + std::strerror(errno));

  if (-1 == listen(sockfd, num_clients))
    throw Error(std::string("listen(): ") + std::strerror(errno));

  Thread *updateThread = new Thread(new Updater(this), 5.0);
  updateThread->start();
}

void Server::work()
{
  struct sockaddr_in addr;
  socklen_t socklen = sizeof(addr);
  int fd = accept(sockfd, reinterpret_cast<struct sockaddr*>(&addr), &socklen);
  if (fd < 0)
    {
      std::cerr << "Failed to accept connection, fd=" << fd << std::endl;
      throw Error(std::strerror(errno));
    }

  // TODO: close read side?
  std::cout << "Got new connection" << std::endl;

  
  auto client = new Client(fd, addr);

  nodeLock.lock();
  for (auto p: nodes)
    {
      auto n = p.second;
      client->socket << static_cast<unsigned short>(4712)
                     << n->id
                     << n->meshName
                     << n->pos
                     << n->rot;
    }
  nodeLock.unlock();
  client->socket << std::flush;

  
  std::lock_guard<std::mutex> lock(clientLock);
  clients.push_back(client);
}


void Server::vehicleLoaded(const std::string &type,
                           const std::string &name,
                           const Ogre::Vector3 &position,
                           const Ogre::Vector3 &rotation,
                           const Vehicle *v)
{
  auto trackable = dynamic_cast<const HeloUtils::Trackable*>(v);
  if (not trackable)
    {
      std::cout << type << ":" << name << " is not a Trackable" << std::endl;
      return;
    }

  Ogre::Matrix3 rotM3;
  rotM3.FromEulerAnglesZYX(Ogre::Radian(Ogre::Degree(rotation.x)),
                           Ogre::Radian(Ogre::Degree(rotation.y)),
                           Ogre::Radian(Ogre::Degree(rotation.z)));
  auto sn = trackable->getSceneNode();
  auto n = new ServerNode(position, Ogre::Quaternion(rotM3));

  for (auto o : sn->getAttachedObjectIterator())
    {
      auto ent = dynamic_cast<Ogre::Entity*>(o.second);
      if (not ent) continue;

      std::cout << "Loaded mesh name: " << ent->getMesh()->getName() << std::endl;
      n->meshName = ent->getMesh()->getName();
    }

  std::lock_guard<std::mutex> clock(clientLock);
  for (auto c: clients)
    {
      c->socket << static_cast<unsigned short>(4712)
                << n->id
                << n->meshName
                << n->pos
                << n->rot
                << std::flush;
    }

  std::lock_guard<std::mutex> nlock(nodeLock);
  nodes.insert(std::make_pair(sn, n));
}

void Server::motionStateAdded(const HeloMotionState &ms)
{
  std::lock_guard<std::mutex> lock(nodeLock);
  Ogre::SceneNode *sn = ms.getNode();

  auto it = nodes.find(sn);
  if (it == nodes.end()) return;

  ServerNode *node = it->second;
  node->motionState = &ms;
}

void Server::updateClients()
{
  std::lock_guard<std::mutex> clock(clientLock);
  if (not clients.size())
    return;
  
  std::lock_guard<std::mutex> nlock(nodeLock);
  for (auto p: nodes)
    {
      auto n = p.second;
      btTransform t;
      if (not n->motionState->getTransform(t))
        continue;

      Ogre::Vector3 pos(HeloUtils::Bullet2OgreVector(t.getOrigin()));
      Ogre::Quaternion rot(HeloUtils::Bullet2OgreQuaternion(t.getRotation()));
      if (pos != n->pos || rot != n->rot)
        {
          n->pos = pos;
          n->rot = rot;
          n->dirty = true;
        }
    }
  
  for (auto c: clients)
    {
      for (auto p: nodes)
        {
          auto n = p.second;
          if (n->dirty)
            c->socket << static_cast<unsigned short>(4711) << n->id << n->pos << n->rot;
        }
      c->socket << std::flush;
    }

  for (auto p: nodes)
    p.second->dirty = false;
}

void Server::Updater::work()
{
  server->updateClients();
  usleep(1.0f/server->update_freq * 1000000);
}
