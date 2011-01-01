#include <string>
#include <iostream>

#include "helo.h"
#include "Terrain.h"
#include "Physics.h"

// TODO: hopefully not needed here indefinately
#include "Helicopter.h"
#include "Car.h"
#include "Tank.h"

class Error {
public:
  static void ErrMessage(std::string err_string)
  {
    std::cerr << err_string << std::endl;
  }
};

heloApp::heloApp(): mExit(false), mRoot(0), cam(0), mKeyboard(0), mInputManager(0), terrain(0), physics(0),
		    current_vehicle(0)
{
}

heloApp::~heloApp()
{
  if (physics)
    delete physics;

  if (terrain)
    delete terrain;

  if (mRoot)
    delete mRoot;
}

int heloApp::main(int argc, char *argv[])
{
  initOGRE();

  terrain = new ::Terrain(mRoot);
  // TODO: maybe there should be some space above the highest top of the terrain?
  physics = new Physics(terrain->getBounds(), true);
  physics->addBody(terrain->createBody());


  Ogre::SceneManager *mgr = mRoot->getSceneManager("SceneManager");
  Ogre::Entity *ent = mgr->createEntity("Sphere", "Sphere.mesh");
  //ent->setCastShadows(true);
  sphere_node = mgr->getRootSceneNode()->createChildSceneNode("SphereNode");
  sphere_node->setPosition(Ogre::Vector3(1683 / 1.2, 50, 2116 / 1.2));
  sphere_node->attachObject(ent);
  sphere = physics->testSphere(Ogre::Vector3(1683 / 1.1, 50, 2116 / 1.1), 1.0, sphere_node);
  physics->addBody(sphere);

  Helicopter::HelicopterData hd;
  hd.name = "Helo0";
  hd.meshname = "Defender.mesh";
  hd.size = Ogre::Vector3(2.59, 2.72, 7.01);
  hd.pos = Ogre::Vector3(1757.96, 5.59526, 1625.3);
  hd.boomLength = 6.0;
  hd.weight = 599.0 + 70.0; // 599 helicopter + 70 pilot
  hd.collectiveSensitivity = 0.5;
  hd.cyclicRightSensitivity = 0.0125;
  hd.cyclicForwardSensitivity = 0.03;
  hd.steerSensitivity = 1.0;
  hd.rotorData.resize(1);
  hd.rotorData[0].weight = 200.0; // no idea what's correct
  hd.rotorData[0].diameter = 8.03;
  hd.rotorData[0].relpos = Ogre::Vector3(0.0, 2.45, 0.0);
  hd.rotorData[0].axis = Ogre::Vector3(0.0, 1.0, 0.0);
  hd.rotorData[0].rotation_axis = Ogre::Vector3::ZERO;
  hd.rotorData[0].torque = 313000.0 /*W*/ / (4000.0 /*RPM*/ / 60.0 /*seconds per minute*/ * 6.28 /*raidians per revolution*/);
  hd.rotorData[0].inertia = 100.0;
  hd.rotorData[0].maxLift = hd.weight * 9.81 * 2;
  hd.rotorData[0].tiltSensitivity = 0.0;

  Helicopter *defender = new Helicopter(hd, mRoot);
  physics->addObject(defender);

  hd.name = "Helo1";
  hd.meshname = "Chinook.mesh";
  hd.size = Ogre::Vector3(3.69, 3.1, 15.74);
  hd.pos = Ogre::Vector3(1767.96, 2.59526, 1625.3);
  hd.boomLength = 0.0;
  hd.weight = 10185.0 + 3 * 70;
  //hd.weight = 22680.0;
  hd.collectiveSensitivity = 0.8;
  hd.cyclicRightSensitivity = 0.5;
  hd.cyclicForwardSensitivity = 0.2;
  hd.steerSensitivity = 1.0;

  hd.rotorData.resize(2);
  // Front rotor
  hd.rotorData[TandemRotorHelicopter::FrontRotor].weight = 500.0; // no idea what's correct
  hd.rotorData[TandemRotorHelicopter::FrontRotor].diameter = 18.3;
  hd.rotorData[TandemRotorHelicopter::FrontRotor].relpos = Ogre::Vector3(0.0, 2.273, 6.377);
  hd.rotorData[TandemRotorHelicopter::FrontRotor].axis = Ogre::Vector3(0.0, 2.962, 0.339);
  hd.rotorData[TandemRotorHelicopter::FrontRotor].rotation_axis = Ogre::Vector3(0.0, -1.381, 12.075);
  hd.rotorData[TandemRotorHelicopter::FrontRotor].torque = 2796000.0 /*W*/ / (4000.0 /*RPM*/ / 60.0 /*seconds per minute*/ * 6.28 /*raidians per revolution*/);
  hd.rotorData[TandemRotorHelicopter::FrontRotor].inertia = 200.0;
  hd.rotorData[TandemRotorHelicopter::FrontRotor].maxLift = (22680 + 1000) * 9.81 / 2.0; // 22680 kg is the max takeoff weight
  hd.rotorData[TandemRotorHelicopter::FrontRotor].tiltSensitivity = 0.1;
  // Back rotor
  hd.rotorData[TandemRotorHelicopter::BackRotor].weight = 500.0; // no idea what's correct
  hd.rotorData[TandemRotorHelicopter::BackRotor].diameter = 18.3;
  hd.rotorData[TandemRotorHelicopter::BackRotor].relpos = Ogre::Vector3(0.0, 3.654, -5.698);
  hd.rotorData[TandemRotorHelicopter::BackRotor].axis = Ogre::Vector3(0.0, 2.962, 0.339);
  hd.rotorData[TandemRotorHelicopter::BackRotor].rotation_axis = Ogre::Vector3(0.0, -1.381, 12.075);
  hd.rotorData[TandemRotorHelicopter::BackRotor].torque = -2796000.0 /*W*/ / (4000.0 /*RPM*/ / 60.0 /*seconds per minute*/ * 6.28 /*raidians per revolution*/);
  hd.rotorData[TandemRotorHelicopter::BackRotor].inertia = 200.0;
  hd.rotorData[TandemRotorHelicopter::BackRotor].maxLift = (22680 + 1000) * 9.81 / 2.0; // 22680 kg is the max takeoff weight
  hd.rotorData[TandemRotorHelicopter::BackRotor].tiltSensitivity = 0.06;

  Helicopter *chinook = new TandemRotorHelicopter(hd, mRoot);
  physics->addObject(chinook);

  Car::CarData cd;
  cd.name = "hmmwv";
  cd.meshname = "hmmwv.mesh";
  cd.position = Ogre::Vector3(1780.96, 2.0, 1625.3);
  cd.size = Ogre::Vector3(2.1, 1.5, 4.6);
  cd.weight = 2340;

  cd.wheelData.resize(4);
  float wheelupdown = 0.131;
  float wheelrev = -1.524;
  float wheelfw = 1.785;
  float wheelrl = 0.925;
  btVector3 wheel_axle(1.0, 0.0, 0.0);
  btVector3 wheeldown(0.0, -1.0, 0.0);
  float spring = 40000;
  btScalar damping = 2000.0;
  float suspension_length = 0.7;
  // Front right wheel
  cd.wheelData[0].relPos = btVector3(-wheelrl, wheelupdown, wheelfw);
  cd.wheelData[0].suspensionLength = suspension_length;
  cd.wheelData[0].maxLengthUp = suspension_length - 0.25;
  cd.wheelData[0].maxLengthDown = suspension_length + 0.25;
  cd.wheelData[0].direction = wheeldown;
  cd.wheelData[0].axle = wheel_axle;
  cd.wheelData[0].radius = 0.4572;
  cd.wheelData[0].spring = spring;
  cd.wheelData[0].dampUp = damping;
  cd.wheelData[0].dampDown = damping;
  cd.wheelData[0].steerCoeff = 1.0;
  cd.wheelData[0].driveCoeff = 1.0;
  cd.wheelData[0].brakeCoeff = 1.0;
  cd.wheelData[0].momentOfInertia = 100.0;

  // Front left wheel
  cd.wheelData[1].relPos = btVector3(wheelrl, wheelupdown, wheelfw);
  cd.wheelData[1].suspensionLength = suspension_length;
  cd.wheelData[1].maxLengthUp = suspension_length - 0.25;
  cd.wheelData[1].maxLengthDown = suspension_length + 0.25;
  cd.wheelData[1].direction = wheeldown;
  cd.wheelData[1].axle = wheel_axle;
  cd.wheelData[1].radius = 0.4572;
  cd.wheelData[1].spring = spring;
  cd.wheelData[1].dampUp = damping;
  cd.wheelData[1].dampDown = damping;
  cd.wheelData[1].steerCoeff = 1.0;
  cd.wheelData[1].driveCoeff = 1.0;
  cd.wheelData[1].brakeCoeff = 1.0;
  cd.wheelData[1].momentOfInertia = 100.0;

  // Back right wheel
  cd.wheelData[2].relPos = btVector3(-wheelrl, wheelupdown, wheelrev);
  cd.wheelData[2].suspensionLength = suspension_length;
  cd.wheelData[2].maxLengthUp = suspension_length - 0.25;
  cd.wheelData[2].maxLengthDown = suspension_length + 0.25;
  cd.wheelData[2].direction = wheeldown;
  cd.wheelData[2].axle = wheel_axle;
  cd.wheelData[2].radius = 0.4572;
  cd.wheelData[2].spring = spring;
  cd.wheelData[2].dampUp = damping;
  cd.wheelData[2].dampDown = damping;
  cd.wheelData[2].steerCoeff = 0.0;
  cd.wheelData[2].driveCoeff = 1.0;
  cd.wheelData[2].brakeCoeff = 1.0;
  cd.wheelData[2].momentOfInertia = 100.0;

  // Back left wheel
  cd.wheelData[3].relPos = btVector3(wheelrl, wheelupdown, wheelrev);
  cd.wheelData[3].suspensionLength = suspension_length;
  cd.wheelData[3].maxLengthUp = suspension_length - 0.25;
  cd.wheelData[3].maxLengthDown = suspension_length + 0.25;
  cd.wheelData[3].direction = wheeldown;
  cd.wheelData[3].axle = wheel_axle;
  cd.wheelData[3].radius = 0.4572;
  cd.wheelData[3].spring = spring;
  cd.wheelData[3].dampUp = damping;
  cd.wheelData[3].dampDown = damping;
  cd.wheelData[3].steerCoeff = 0.0;
  cd.wheelData[3].driveCoeff = 1.0;
  cd.wheelData[3].brakeCoeff = 1.0;
  cd.wheelData[3].momentOfInertia = 100.0;

  Car *hmmwv = new Car(cd, mRoot);
  physics->addObject(hmmwv);

  cd.name = "m93a1";
  cd.meshname = "m93a1.mesh";
  cd.position = Ogre::Vector3(1760.96, 2.0, 1650.3);
  cd.size = Ogre::Vector3(2.98, 1.9, 7.33);
  cd.weight = 18300.0;

  cd.wheelData.resize(6);
  wheelupdown = 0.306;
  wheelrev = -1.642;
  float wheelmid = 0.547;
  wheelfw = 2.407;
  wheelrl = 1.364;
  spring = 150000;
  damping = 20000.0;
  suspension_length = 1.2;
  // Front right wheel
  cd.wheelData[0].relPos = btVector3(-wheelrl, wheelupdown, wheelfw);
  cd.wheelData[0].suspensionLength = suspension_length;
  cd.wheelData[0].maxLengthUp = suspension_length - 0.5;
  cd.wheelData[0].maxLengthDown = suspension_length + 0.35;
  cd.wheelData[0].direction = wheeldown;
  cd.wheelData[0].axle = wheel_axle;
  cd.wheelData[0].radius = 0.7046;
  cd.wheelData[0].spring = spring;
  cd.wheelData[0].dampUp = damping;
  cd.wheelData[0].dampDown = damping;
  cd.wheelData[0].steerCoeff = 1.0;
  cd.wheelData[0].driveCoeff = 5.0;
  cd.wheelData[0].brakeCoeff = 5.0;
  cd.wheelData[0].momentOfInertia = 300.0;

  // Middle left wheel
  cd.wheelData[1].relPos = btVector3(wheelrl, wheelupdown, wheelmid);
  cd.wheelData[1].suspensionLength = suspension_length;
  cd.wheelData[1].maxLengthUp = suspension_length - 0.5;
  cd.wheelData[1].maxLengthDown = suspension_length + 0.35;
  cd.wheelData[1].direction = wheeldown;
  cd.wheelData[1].axle = wheel_axle;
  cd.wheelData[1].radius = 0.7046;
  cd.wheelData[1].spring = spring;
  cd.wheelData[1].dampUp = damping;
  cd.wheelData[1].dampDown = damping;
  cd.wheelData[1].steerCoeff = 1.0;
  cd.wheelData[1].driveCoeff = 5.0;
  cd.wheelData[1].brakeCoeff = 5.0;
  cd.wheelData[1].momentOfInertia = 300.0;

  // Middle right wheel
  float back_to_front_dist = wheelfw - wheelrev;
  float back_to_mid_dist = wheelmid - wheelrev;
  cd.wheelData[2].relPos = btVector3(-wheelrl, wheelupdown, wheelmid);
  cd.wheelData[2].suspensionLength = suspension_length;
  cd.wheelData[2].maxLengthUp = suspension_length - 0.5;
  cd.wheelData[2].maxLengthDown = suspension_length + 0.35;
  cd.wheelData[2].direction = wheeldown;
  cd.wheelData[2].axle = wheel_axle;
  cd.wheelData[2].radius = 0.7046;
  cd.wheelData[2].spring = spring;
  cd.wheelData[2].dampUp = damping;
  cd.wheelData[2].dampDown = damping;
  cd.wheelData[2].steerCoeff =  back_to_mid_dist / back_to_front_dist;
  cd.wheelData[2].driveCoeff = 5.0;
  cd.wheelData[2].brakeCoeff = 5.0;
  cd.wheelData[2].momentOfInertia = 300.0;

  // Middle left wheel
  cd.wheelData[3].relPos = btVector3(wheelrl, wheelupdown, wheelfw);
  cd.wheelData[3].suspensionLength = suspension_length;
  cd.wheelData[3].maxLengthUp = suspension_length - 0.5;
  cd.wheelData[3].maxLengthDown = suspension_length + 0.35;
  cd.wheelData[3].direction = wheeldown;
  cd.wheelData[3].axle = wheel_axle;
  cd.wheelData[3].radius = 0.7046;
  cd.wheelData[3].spring = spring;
  cd.wheelData[3].dampUp = damping;
  cd.wheelData[3].dampDown = damping;
  cd.wheelData[3].steerCoeff = back_to_mid_dist / back_to_front_dist;
  cd.wheelData[3].driveCoeff = 5.0;
  cd.wheelData[3].brakeCoeff = 5.0;
  cd.wheelData[3].momentOfInertia = 300.0;

  // Back right wheel
  cd.wheelData[4].relPos = btVector3(-wheelrl, wheelupdown, wheelrev);
  cd.wheelData[4].suspensionLength = suspension_length;
  cd.wheelData[4].maxLengthUp = suspension_length - 0.5;
  cd.wheelData[4].maxLengthDown = suspension_length + 0.35;
  cd.wheelData[4].direction = wheeldown;
  cd.wheelData[4].axle = wheel_axle;
  cd.wheelData[4].radius = 0.7046;
  cd.wheelData[4].spring = spring;
  cd.wheelData[4].dampUp = damping;
  cd.wheelData[4].dampDown = damping;
  cd.wheelData[4].steerCoeff = 0.0;
  cd.wheelData[4].driveCoeff = 5.0;
  cd.wheelData[4].brakeCoeff = 5.0;
  cd.wheelData[4].momentOfInertia = 300.0;

  // Back left wheel
  cd.wheelData[5].relPos = btVector3(wheelrl, wheelupdown, wheelrev);
  cd.wheelData[5].suspensionLength = suspension_length;
  cd.wheelData[5].maxLengthUp = suspension_length - 0.5;
  cd.wheelData[5].maxLengthDown = suspension_length + 0.35;
  cd.wheelData[5].direction = wheeldown;
  cd.wheelData[5].axle = wheel_axle;
  cd.wheelData[5].radius = 0.7046;
  cd.wheelData[5].spring = spring;
  cd.wheelData[5].dampUp = damping;
  cd.wheelData[5].dampDown = damping;
  cd.wheelData[5].steerCoeff = 0.0;
  cd.wheelData[5].driveCoeff = 5.0;
  cd.wheelData[5].brakeCoeff = 5.0;
  cd.wheelData[5].momentOfInertia = 300.0;

  Car *m93a1 = new Car(cd, mRoot);
  physics->addObject(m93a1);


  cd.name = "abrams";
  cd.meshname = "hmmwv.mesh";
  cd.position = Ogre::Vector3(1780.96, 1.0, 1650.3);
  cd.size = Ogre::Vector3(3.66, 2.44, 7.93);
  cd.weight = 61300.0;

  cd.wheelData.resize(14);
  wheelupdown = 0.2;
  wheelrev = -3.0;
  float wheeldist = 1.0;
  wheelrl = 1.125;
  spring = 600000;
  damping = 50000.0;
  suspension_length = 0.9;
  // 1st right wheel
  cd.wheelData[0].relPos = btVector3(-wheelrl, wheelupdown, wheelrev + 6 * wheeldist);
  cd.wheelData[0].suspensionLength = suspension_length;
  cd.wheelData[0].maxLengthUp = suspension_length - 0.5;
  cd.wheelData[0].maxLengthDown = suspension_length + 0.35;
  cd.wheelData[0].direction = wheeldown;
  cd.wheelData[0].axle = wheel_axle;
  cd.wheelData[0].radius = 0.45;
  cd.wheelData[0].spring = spring;
  cd.wheelData[0].dampUp = damping;
  cd.wheelData[0].dampDown = damping;
  cd.wheelData[0].steerCoeff = 0.0;
  cd.wheelData[0].driveCoeff = 0.0;
  cd.wheelData[0].brakeCoeff = 0.0;
  cd.wheelData[0].momentOfInertia = 300.0;

  // 1st left wheel
  cd.wheelData[1].relPos = btVector3(wheelrl, wheelupdown, wheelrev + 6 *wheeldist);
  cd.wheelData[1].suspensionLength = suspension_length;
  cd.wheelData[1].maxLengthUp = suspension_length - 0.5;
  cd.wheelData[1].maxLengthDown = suspension_length + 0.35;
  cd.wheelData[1].direction = wheeldown;
  cd.wheelData[1].axle = wheel_axle;
  cd.wheelData[1].radius = 0.45;
  cd.wheelData[1].spring = spring;
  cd.wheelData[1].dampUp = damping;
  cd.wheelData[1].dampDown = damping;
  cd.wheelData[1].steerCoeff = 0.0;
  cd.wheelData[1].driveCoeff = 0.0;
  cd.wheelData[1].brakeCoeff = 0.0;
  cd.wheelData[1].momentOfInertia = 300.0;

  // 2nd right wheel
  cd.wheelData[2].relPos = btVector3(-wheelrl, wheelupdown, wheelrev + 5 * wheeldist);
  cd.wheelData[2].suspensionLength = suspension_length;
  cd.wheelData[2].maxLengthUp = suspension_length - 0.5;
  cd.wheelData[2].maxLengthDown = suspension_length + 0.35;
  cd.wheelData[2].direction = wheeldown;
  cd.wheelData[2].axle = wheel_axle;
  cd.wheelData[2].radius = 0.45;
  cd.wheelData[2].spring = spring;
  cd.wheelData[2].dampUp = damping;
  cd.wheelData[2].dampDown = damping;
  cd.wheelData[2].steerCoeff = 0.0;
  cd.wheelData[2].driveCoeff = 0.0;
  cd.wheelData[2].brakeCoeff = 0.0;
  cd.wheelData[2].momentOfInertia = 300.0;

  // 2nd left wheel
  cd.wheelData[3].relPos = btVector3(wheelrl, wheelupdown, wheelrev + 5 *wheeldist);
  cd.wheelData[3].suspensionLength = suspension_length;
  cd.wheelData[3].maxLengthUp = suspension_length - 0.5;
  cd.wheelData[3].maxLengthDown = suspension_length + 0.35;
  cd.wheelData[3].direction = wheeldown;
  cd.wheelData[3].axle = wheel_axle;
  cd.wheelData[3].radius = 0.45;
  cd.wheelData[3].spring = spring;
  cd.wheelData[3].dampUp = damping;
  cd.wheelData[3].dampDown = damping;
  cd.wheelData[3].steerCoeff = 0.0;
  cd.wheelData[3].driveCoeff = 0.0;
  cd.wheelData[3].brakeCoeff = 0.0;
  cd.wheelData[3].momentOfInertia = 300.0;

  // 3rd right wheel
  cd.wheelData[4].relPos = btVector3(-wheelrl, wheelupdown, wheelrev + 4 * wheeldist);
  cd.wheelData[4].suspensionLength = suspension_length;
  cd.wheelData[4].maxLengthUp = suspension_length - 0.5;
  cd.wheelData[4].maxLengthDown = suspension_length + 0.35;
  cd.wheelData[4].direction = wheeldown;
  cd.wheelData[4].axle = wheel_axle;
  cd.wheelData[4].radius = 0.45;
  cd.wheelData[4].spring = spring;
  cd.wheelData[4].dampUp = damping;
  cd.wheelData[4].dampDown = damping;
  cd.wheelData[4].steerCoeff = 0.0;
  cd.wheelData[4].driveCoeff = 0.0;
  cd.wheelData[4].brakeCoeff = 0.0;
  cd.wheelData[4].momentOfInertia = 300.0;

  // 3rd left wheel
  cd.wheelData[5].relPos = btVector3(wheelrl, wheelupdown, wheelrev + 4 *wheeldist);
  cd.wheelData[5].suspensionLength = suspension_length;
  cd.wheelData[5].maxLengthUp = suspension_length - 0.5;
  cd.wheelData[5].maxLengthDown = suspension_length + 0.35;
  cd.wheelData[5].direction = wheeldown;
  cd.wheelData[5].axle = wheel_axle;
  cd.wheelData[5].radius = 0.45;
  cd.wheelData[5].spring = spring;
  cd.wheelData[5].dampUp = damping;
  cd.wheelData[5].dampDown = damping;
  cd.wheelData[5].steerCoeff = 0.0;
  cd.wheelData[5].driveCoeff = 0.0;
  cd.wheelData[5].brakeCoeff = 0.0;
  cd.wheelData[5].momentOfInertia = 300.0;

  // 4th right wheel
  cd.wheelData[6].relPos = btVector3(-wheelrl, wheelupdown, wheelrev + 3 * wheeldist);
  cd.wheelData[6].suspensionLength = suspension_length;
  cd.wheelData[6].maxLengthUp = suspension_length - 0.5;
  cd.wheelData[6].maxLengthDown = suspension_length + 0.35;
  cd.wheelData[6].direction = wheeldown;
  cd.wheelData[6].axle = wheel_axle;
  cd.wheelData[6].radius = 0.45;
  cd.wheelData[6].spring = spring;
  cd.wheelData[6].dampUp = damping;
  cd.wheelData[6].dampDown = damping;
  cd.wheelData[6].steerCoeff = 0.0;
  cd.wheelData[6].driveCoeff = 0.0;
  cd.wheelData[6].brakeCoeff = 0.0;
  cd.wheelData[6].momentOfInertia = 300.0;

  // 4th left wheel
  cd.wheelData[7].relPos = btVector3(wheelrl, wheelupdown, wheelrev + 3 *wheeldist);
  cd.wheelData[7].suspensionLength = suspension_length;
  cd.wheelData[7].maxLengthUp = suspension_length - 0.5;
  cd.wheelData[7].maxLengthDown = suspension_length + 0.35;
  cd.wheelData[7].direction = wheeldown;
  cd.wheelData[7].axle = wheel_axle;
  cd.wheelData[7].radius = 0.45;
  cd.wheelData[7].spring = spring;
  cd.wheelData[7].dampUp = damping;
  cd.wheelData[7].dampDown = damping;
  cd.wheelData[7].steerCoeff = 0.0;
  cd.wheelData[7].driveCoeff = 0.0;
  cd.wheelData[7].brakeCoeff = 0.0;
  cd.wheelData[7].momentOfInertia = 300.0;

  // 5th right wheel
  cd.wheelData[8].relPos = btVector3(-wheelrl, wheelupdown, wheelrev + 2 * wheeldist);
  cd.wheelData[8].suspensionLength = suspension_length;
  cd.wheelData[8].maxLengthUp = suspension_length - 0.5;
  cd.wheelData[8].maxLengthDown = suspension_length + 0.35;
  cd.wheelData[8].direction = wheeldown;
  cd.wheelData[8].axle = wheel_axle;
  cd.wheelData[8].radius = 0.45;
  cd.wheelData[8].spring = spring;
  cd.wheelData[8].dampUp = damping;
  cd.wheelData[8].dampDown = damping;
  cd.wheelData[8].steerCoeff = 0.0;
  cd.wheelData[8].driveCoeff = 0.0;
  cd.wheelData[8].brakeCoeff = 0.0;
  cd.wheelData[8].momentOfInertia = 300.0;

  // 5th left wheel
  cd.wheelData[9].relPos = btVector3(wheelrl, wheelupdown, wheelrev + 2 * wheeldist);
  cd.wheelData[9].suspensionLength = suspension_length;
  cd.wheelData[9].maxLengthUp = suspension_length - 0.5;
  cd.wheelData[9].maxLengthDown = suspension_length + 0.35;
  cd.wheelData[9].direction = wheeldown;
  cd.wheelData[9].axle = wheel_axle;
  cd.wheelData[9].radius = 0.45;
  cd.wheelData[9].spring = spring;
  cd.wheelData[9].dampUp = damping;
  cd.wheelData[9].dampDown = damping;
  cd.wheelData[9].steerCoeff = 0.0;
  cd.wheelData[9].driveCoeff = 0.0;
  cd.wheelData[9].brakeCoeff = 0.0;
  cd.wheelData[9].momentOfInertia = 300.0;

  // 6th right wheel
  cd.wheelData[10].relPos = btVector3(-wheelrl, wheelupdown, wheelrev + 1 * wheeldist);
  cd.wheelData[10].suspensionLength = suspension_length;
  cd.wheelData[10].maxLengthUp = suspension_length - 0.5;
  cd.wheelData[10].maxLengthDown = suspension_length + 0.35;
  cd.wheelData[10].direction = wheeldown;
  cd.wheelData[10].axle = wheel_axle;
  cd.wheelData[10].radius = 0.45;
  cd.wheelData[10].spring = spring;
  cd.wheelData[10].dampUp = damping;
  cd.wheelData[10].dampDown = damping;
  cd.wheelData[10].steerCoeff = 0.0;
  cd.wheelData[10].driveCoeff = 0.0;
  cd.wheelData[10].brakeCoeff = 0.0;
  cd.wheelData[10].momentOfInertia = 300.0;

  // 6th left wheel
  cd.wheelData[11].relPos = btVector3(wheelrl, wheelupdown, wheelrev + 1 * wheeldist);
  cd.wheelData[11].suspensionLength = suspension_length;
  cd.wheelData[11].maxLengthUp = suspension_length - 0.5;
  cd.wheelData[11].maxLengthDown = suspension_length + 0.35;
  cd.wheelData[11].direction = wheeldown;
  cd.wheelData[11].axle = wheel_axle;
  cd.wheelData[11].radius = 0.45;
  cd.wheelData[11].spring = spring;
  cd.wheelData[11].dampUp = damping;
  cd.wheelData[11].dampDown = damping;
  cd.wheelData[11].steerCoeff = 0.0;
  cd.wheelData[11].driveCoeff = 0.0;
  cd.wheelData[11].brakeCoeff = 0.0;
  cd.wheelData[11].momentOfInertia = 300.0;

  // 7th right wheel
  cd.wheelData[12].relPos = btVector3(-wheelrl, wheelupdown, wheelrev + 0 * wheeldist);
  cd.wheelData[12].suspensionLength = suspension_length;
  cd.wheelData[12].maxLengthUp = suspension_length - 0.5;
  cd.wheelData[12].maxLengthDown = suspension_length + 0.35;
  cd.wheelData[12].direction = wheeldown;
  cd.wheelData[12].axle = wheel_axle;
  cd.wheelData[12].radius = 0.45;
  cd.wheelData[12].spring = spring;
  cd.wheelData[12].dampUp = damping;
  cd.wheelData[12].dampDown = damping;
  cd.wheelData[12].steerCoeff = 0.0;
  cd.wheelData[12].driveCoeff = 0.0;
  cd.wheelData[12].brakeCoeff = 0.0;
  cd.wheelData[12].momentOfInertia = 300.0;

  // 7th left wheel
  cd.wheelData[13].relPos = btVector3(wheelrl, wheelupdown, wheelrev + 0 * wheeldist);
  cd.wheelData[13].suspensionLength = suspension_length;
  cd.wheelData[13].maxLengthUp = suspension_length - 0.5;
  cd.wheelData[13].maxLengthDown = suspension_length + 0.35;
  cd.wheelData[13].direction = wheeldown;
  cd.wheelData[13].axle = wheel_axle;
  cd.wheelData[13].radius = 0.45;
  cd.wheelData[13].spring = spring;
  cd.wheelData[13].dampUp = damping;
  cd.wheelData[13].dampDown = damping;
  cd.wheelData[13].steerCoeff = 0.0;
  cd.wheelData[13].driveCoeff = 0.0;
  cd.wheelData[13].brakeCoeff = 0.0;
  cd.wheelData[13].momentOfInertia = 300.0;


  Tank *abrams = new Tank(cd, mRoot);
  physics->addObject(abrams);



  current_vehicle = defender;
  current_car = abrams;
  cam->setAutoTracking(true, current_vehicle->getSceneNode());
  cam->setAutoTracking(true, current_car->getSceneNode());

  physics->finishConfiguration();

  while (not mExit) {
    //std::cout << "rendering" << std::endl;
    physics->step();
    physics->sync();
    //btTransform trans = sphere->getWorldTransform();
    //Ogre::Real mat[16];
    // Ogre::matrix omat(mat[0],mat[1],mat[2],mat[3],
    // 		      mat[4],mat[5],mat[6],mat[7],
    // 		      mat[8],mat[9],mat[10],mat[11],
    // 		      mat[12],mat[13],mat[14],mat[15]);
    // std::cout << mat[0] << " " << mat[1]<< " "  << mat[2]<< " "  << mat[3]<< " "  << std::endl
    // 	      << mat[4]<< " "  << mat[5]<< " "  << mat[6]<< " "  << mat[7]<< " "  << std::endl
    // 	      << mat[8]<< " "  << mat[9]<< " "  << mat[10]<< " "  << mat[11]<< " "   << std::endl
    // 	      << mat[12]<< " "  << mat[13]<< " "  << mat[14]<< " "  << mat[15]<< " "  << std::endl;
    //std::cout << mat[12] << " "  << mat[13] << " "  << mat[14] <<  std::endl;
    //sphere_node->setPosition(Ogre::Vector3(mat[12], mat[13], mat[14]));
    //trans.getOpenGLMatrix(mat);

    Ogre::SceneNode *n;
    n = current_vehicle->getSceneNode();
    n = current_car->getSceneNode();
    Ogre::Vector3 campos = n->convertLocalToWorldPosition(Ogre::Vector3(3.0, 0.0, -20.0));
    campos.y = n->_getDerivedPosition().y + 3.0;
    //cam->setPosition(campos);
    doOgreUpdate();
    //usleep(50000);
  }

  return 0;
}

bool heloApp::keyPressed(const OIS::KeyEvent &e)
{
  switch (e.key)
    {
    case OIS::KC_SPACE:
      current_vehicle->startEngines(true);
      break;

    case OIS::KC_W:
      current_vehicle->setCollective(1.0);
      break;
    case OIS::KC_S:
      current_vehicle->setCollective(-1.0);
      break;

    case OIS::KC_A:
      current_vehicle->setSteer(1.0);
      break;
    case OIS::KC_D:
      current_vehicle->setSteer(-1.0);
      break;

    case OIS::KC_I:
      current_vehicle->setCyclicForward(1.0);
      break;
    case OIS::KC_K:
      current_vehicle->setCyclicForward(-1.0);
      break;

    case OIS::KC_L:
      current_vehicle->setCyclicRight(1.0);
      break;
    case OIS::KC_J:
      current_vehicle->setCyclicRight(-1.0);
      break;

    default: /* Other keys not handled*/
      break;
    }

  return true;
}

bool heloApp::keyReleased(const OIS::KeyEvent &e)
{
  switch (e.key)
    {
    case OIS::KC_W:
    case OIS::KC_S:
      current_vehicle->setCollective(0.0);
      break;

    case OIS::KC_A:
    case OIS::KC_D:
      current_vehicle->setSteer(0.0);
      break;

    case OIS::KC_I:
    case OIS::KC_K:
      current_vehicle->setCyclicForward(0.0);
      break;

    case OIS::KC_L:
    case OIS::KC_J:
      current_vehicle->setCyclicRight(0.0);
      break;

    default: /* Other keys not handled*/
      break;
    }

  return true;
}

bool heloApp::buttonPressed(const OIS::JoyStickEvent &e, int idx)
{
  //printf("Joystick button %d pressed\n", idx);
  switch (idx)
    {
    case 0:
      current_vehicle->startEngines(true);
      break;
    default:
      break;
    }
  return true;
}

bool heloApp::buttonReleased(const OIS::JoyStickEvent &e, int idx)
{
  printf("Joystick button %d released\n", idx);
  return true;
}

bool heloApp::axisMoved(const OIS::JoyStickEvent &e, int idx)
{
  //printf("Joystick axis %d moved\n", idx);
  // // float axis_val = e.state.mAxes[idx].abs / static_cast<float>(OIS::JoyStick::MAX_AXIS);

  // // switch (idx)
  // //   {
  // //   case 1:
  // //     current_vehicle->setCollective(-axis_val);
  // //     break;
  // //   case 0:
  // //     current_vehicle->setSteer(-axis_val);
  // //     break;
  // //   case 4:
  // //     current_vehicle->setCyclicForward(-axis_val);
  // //     break;
  // //   case 3:
  // //     current_vehicle->setCyclicRight(axis_val);
  // //     break;

  // //   default:
  // //     break;
  // //   }

  float collective = -e.state.mAxes[1].abs / static_cast<float>(OIS::JoyStick::MAX_AXIS);
  float steer = -e.state.mAxes[0].abs / static_cast<float>(OIS::JoyStick::MAX_AXIS);
  float cyclic_forward = -e.state.mAxes[4].abs / static_cast<float>(OIS::JoyStick::MAX_AXIS);
  float cyclic_right = e.state.mAxes[3].abs / static_cast<float>(OIS::JoyStick::MAX_AXIS);
  current_vehicle->setCollective(collective);
  current_vehicle->setSteer(steer);
  current_vehicle->setCyclicForward(cyclic_forward);
  current_vehicle->setCyclicRight(cyclic_right);

  current_car->setSteer(e.state.mAxes[3].abs / static_cast<float>(OIS::JoyStick::MAX_AXIS) * 0.5);
  current_car->setThrottle(-e.state.mAxes[1].abs / static_cast<float>(OIS::JoyStick::MAX_AXIS));

  return true;
}


void heloApp::handleInput()
{
  // if (mKeyboard->isKeyDown(OIS::KC_A))
  //   cam->yaw(Ogre::Radian(0.01));
  // if (mKeyboard->isKeyDown(OIS::KC_D))
  //   cam->yaw(Ogre::Radian(-0.01));
  // if (mKeyboard->isKeyDown(OIS::KC_W))
  //   cam->pitch(Ogre::Radian(0.01));
  // if (mKeyboard->isKeyDown(OIS::KC_S))
  //   cam->pitch(Ogre::Radian(-0.01));

  // if (mKeyboard->isKeyDown(OIS::KC_UP))
  //   cam->moveRelative(Ogre::Vector3(0.0, 0.0, -1.0));
  // if (mKeyboard->isKeyDown(OIS::KC_DOWN))
  //   cam->moveRelative(Ogre::Vector3(0.0, 0.0, 1.0));

  if (mKeyboard->isKeyDown(OIS::KC_SPACE))
    //setSpherePosition(cam->getPosition() + (cam->getDirection() * 100.0));
    current_vehicle->startEngines(true);

  // current_vehicle->setCollective(mKeyboard->isKeyDown(OIS::KC_I) ? 1.0 :
  // 				 mKeyboard->isKeyDown(OIS::KC_K) ? -1.0 : 0.0);
  // current_vehicle->setCyclic(mKeyboard->isKeyDown(OIS::KC_W) ? 1.0 :
  // 			     mKeyboard->isKeyDown(OIS::KC_S) ? -1.0 : 0.0,
  // 			     mKeyboard->isKeyDown(OIS::KC_D) ? 1.0 :
  // 			     mKeyboard->isKeyDown(OIS::KC_A) ? -1.0 : 0.0);
  // current_vehicle->setSteer(mKeyboard->isKeyDown(OIS::KC_J) ? 1.0 :
  // 			    mKeyboard->isKeyDown(OIS::KC_L) ? -1.0 : 0.0);

  current_vehicle->setCollective(mKeyboard->isKeyDown(OIS::KC_W) ? 1.0 :
				 mKeyboard->isKeyDown(OIS::KC_S) ? -1.0 : 0.0);
  current_vehicle->setCyclic(mKeyboard->isKeyDown(OIS::KC_I) ? 1.0 :
			     mKeyboard->isKeyDown(OIS::KC_K) ? -1.0 : 0.0,
			     mKeyboard->isKeyDown(OIS::KC_L) ? 1.0 :
			     mKeyboard->isKeyDown(OIS::KC_J) ? -1.0 : 0.0);
  current_vehicle->setSteer(mKeyboard->isKeyDown(OIS::KC_A) ? 1.0 :
			    mKeyboard->isKeyDown(OIS::KC_D) ? -1.0 : 0.0);



  //if (mKeyboard->isKeyDown(OIS::KC_P))
  //  std::cout << "Height: " << sphere_node->getPosition().y << std::endl;

}

void heloApp::setSpherePosition(const Ogre::Vector3 &newPos)
{
  btTransform tr;
  tr.setIdentity();
  tr.setOrigin(btVector3(newPos.x, newPos.y, newPos.z));
  std::cout << "New position: " << newPos.x << " "  << newPos.y << " "  << newPos.z <<  std::endl;
  //btMotionState *mt = sphere->getMotionState();
  sphere->setCenterOfMassTransform(tr);
  sphere->setLinearVelocity(btVector3(0, 0, 0));
  sphere->setAngularVelocity(btVector3(0, 0, 0));
  sphere->activate();
}


int main(int argc, char *argv[])
{
  heloApp app;
  return app.main(argc, argv);
}
