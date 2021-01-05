#include <qi/session.hpp>
#include <qi/anyobject.hpp>
#include <qi/log.hpp>


//vrep includes
extern "C"
{
#include "extApi.h"
#include "extApiPlatform.h"
}

#include <iostream>
#include <chrono>
#include <thread>

class VrepSimulation
{
public:
  // you can omit the session if you don't need it
  VrepSimulation(qi::SessionPtr session);

  bool init(uint robot_id);
  void jointControl();
  bool setFPS(uint fps);

  int ServicegetClientID();
  bool assignClientID(uint robot_id);

private:
  qi::SessionPtr _session;
  simxInt clientID;
  qi::AnyObject motionManager;
  qi::AnyObject postureManager;

  bool getFirstHandles(uint robot_id);
  std::vector<std::vector<std::vector<simxInt *>>> firstHandles;
  std::chrono::milliseconds waitInterval;

};