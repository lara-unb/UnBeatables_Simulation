#include "./vrepsimulation.h"

extern "C"
{
#include "extApi.c"
#include "extApiPlatform.c"
}

VrepSimulation::VrepSimulation(qi::SessionPtr session)
    : _session(session)
{
  // set default interval
  this->waitInterval = std::chrono::milliseconds(1000 / 100);

  // Alocate handles array

  //number of joints
  this->firstHandles.resize(26);
  for (size_t i = 0; i < 26; i++)
  {
    // number of robots
    this->firstHandles[i].resize(1);
    for (size_t j = 0; j < 1; j++)
    {
      //maximum number of objects in a joint (8 in each finger)
      this->firstHandles[i][j].resize(8);
      for (size_t k = 0; k < 8; k++)
      {
        this->firstHandles[i][j][k] = new simxInt;
      }
    }
  }
}

bool VrepSimulation::init(uint robot_id)
{
  bool initClientID = this->assignClientID(robot_id);
  std::cout << "Client ID " << this->clientID << std::endl;
  if (initClientID)
  {
    this->motionManager = this->_session->service("ALMotion");
    this->postureManager = this->_session->service("ALRobotPosture");

    qiLogInfo("VrepSimulation", "wakeup");
    this->motionManager.call<bool>("wakeUp");
    qiLogInfo("VrepSimulation", "StandZero");
    this->postureManager.call<bool>("goToPosture", "StandZero", 0.5);

    qiLogInfo("VrepSimulation", "Getting first handles");
    this->getFirstHandles(robot_id);

    return true;
  }
  else
  {
    qiLogError("VrepSimulation", "Could not connect to vrep");
  }
}

bool VrepSimulation::assignClientID(uint robot_id)
{
  this->clientID = simxStart((simxChar *)"127.0.0.1", 20000 + robot_id, true, true, 5000, 5);
  if (this->clientID == -1)
  {
    qiLogError("VrepSimulation", "Couldn't connect to vrep simulation. Is it started?");
    return false;
  }
  else
  {
    qiLogInfo("VrepSimulation", "Connected with clientID %s", this->clientID);
    return true;
  }
}

bool VrepSimulation::getFirstHandles(uint robot_id)
{
  simxGetObjectHandle(clientID, ("HeadYaw#" + std::to_string(robot_id)).c_str(), this->firstHandles[0][0][0], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("HeadPitch#" + std::to_string(robot_id)).c_str(), this->firstHandles[1][0][0], simx_opmode_oneshot_wait);
  // Left Leg
  std::cout << "-> Left Leg for NAO : " << robot_id << std::endl;
  simxGetObjectHandle(clientID, ("LHipYawPitch3#" + std::to_string(robot_id)).c_str(), this->firstHandles[2][0][0], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("LHipRoll3#" + std::to_string(robot_id)).c_str(), this->firstHandles[3][0][0], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("LHipPitch3#" + std::to_string(robot_id)).c_str(), this->firstHandles[4][0][0], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("LKneePitch3#" + std::to_string(robot_id)).c_str(), this->firstHandles[5][0][0], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("LAnklePitch3#" + std::to_string(robot_id)).c_str(), this->firstHandles[6][0][0], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("LAnkleRoll3#" + std::to_string(robot_id)).c_str(), this->firstHandles[7][0][0], simx_opmode_oneshot_wait);
  // Right Leg
  std::cout << "-> Right Leg for NAO : " << robot_id << std::endl;
  simxGetObjectHandle(clientID, ("RHipYawPitch3#" + std::to_string(robot_id)).c_str(), this->firstHandles[8][0][0], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("RHipRoll3#" + std::to_string(robot_id)).c_str(), this->firstHandles[9][0][0], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("RHipPitch3#" + std::to_string(robot_id)).c_str(), this->firstHandles[10][0][0], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("RKneePitch3#" + std::to_string(robot_id)).c_str(), this->firstHandles[11][0][0], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("RAnklePitch3#" + std::to_string(robot_id)).c_str(), this->firstHandles[12][0][0], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("RAnkleRoll3#" + std::to_string(robot_id)).c_str(), this->firstHandles[13][0][0], simx_opmode_oneshot_wait);
  // Left Arm
  std::cout << "-> Left Arm for NAO : " << robot_id << std::endl;
  simxGetObjectHandle(clientID, ("LShoulderPitch3#" + std::to_string(robot_id)).c_str(), this->firstHandles[14][0][0], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("LShoulderRoll3#" + std::to_string(robot_id)).c_str(), this->firstHandles[15][0][0], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("LElbowYaw3#" + std::to_string(robot_id)).c_str(), this->firstHandles[16][0][0], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("LElbowRoll3#" + std::to_string(robot_id)).c_str(), this->firstHandles[17][0][0], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("LWristYaw3#" + std::to_string(robot_id)).c_str(), this->firstHandles[18][0][0], simx_opmode_oneshot_wait);
  // Right Arm
  std::cout << "-> Right Arm for NAO : " << robot_id << std::endl;
  simxGetObjectHandle(clientID, ("RShoulderPitch3#" + std::to_string(robot_id)).c_str(), this->firstHandles[19][0][0], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("RShoulderRoll3#" + std::to_string(robot_id)).c_str(), this->firstHandles[20][0][0], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("RElbowYaw3#" + std::to_string(robot_id)).c_str(), this->firstHandles[21][0][0], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("RElbowRoll3#" + std::to_string(robot_id)).c_str(), this->firstHandles[22][0][0], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("RWristYaw3#" + std::to_string(robot_id)).c_str(), this->firstHandles[23][0][0], simx_opmode_oneshot_wait);
  // Left fingers
  std::cout << "-> Left Fingers for NAO : " << robot_id << std::endl;
  simxGetObjectHandle(clientID, ("NAO_LThumbBase#" + std::to_string(robot_id)).c_str(), this->firstHandles[24][0][0], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("Revolute_joint8#" + std::to_string(robot_id)).c_str(), this->firstHandles[24][0][1], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("NAO_LLFingerBase#" + std::to_string(robot_id)).c_str(), this->firstHandles[24][0][2], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("Revolute_joint12#" + std::to_string(robot_id)).c_str(), this->firstHandles[24][0][3], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("Revolute_joint14#" + std::to_string(robot_id)).c_str(), this->firstHandles[24][0][4], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("NAO_LRFinger_Base#" + std::to_string(robot_id)).c_str(), this->firstHandles[24][0][5], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("Revolute_joint11#" + std::to_string(robot_id)).c_str(), this->firstHandles[24][0][6], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("Revolute_joint13#" + std::to_string(robot_id)).c_str(), this->firstHandles[24][0][7], simx_opmode_oneshot_wait);

  // Right Fingers
  std::cout << "-> Right Fingers for NAO : " << robot_id << std::endl;
  simxGetObjectHandle(clientID, ("NAO_RThumbBase#" + std::to_string(robot_id)).c_str(), this->firstHandles[25][0][0], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("Revolute_joint0#" + std::to_string(robot_id)).c_str(), this->firstHandles[25][0][1], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("NAO_RLFingerBase#" + std::to_string(robot_id)).c_str(), this->firstHandles[25][0][2], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("Revolute_joint5#" + std::to_string(robot_id)).c_str(), this->firstHandles[25][0][3], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("Revolute_joint6#" + std::to_string(robot_id)).c_str(), this->firstHandles[25][0][4], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("NAO_RRFinger_Base#" + std::to_string(robot_id)).c_str(), this->firstHandles[25][0][5], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("Revolute_joint2#" + std::to_string(robot_id)).c_str(), this->firstHandles[25][0][6], simx_opmode_oneshot_wait);
  simxGetObjectHandle(clientID, ("Revolute_joint3#" + std::to_string(robot_id)).c_str(), this->firstHandles[25][0][7], simx_opmode_oneshot_wait);
}

void VrepSimulation::jointControl()
{
  while (this->clientID != -1)
  {
    //  slow down a little
    std::this_thread::sleep_for(this->waitInterval);

    // Getting joint's angles from Choregraphe (please check your robot's IP)
    std::vector<float> commandAngles = this->motionManager.call<std::vector<float>>("getAngles", "Body", false);
    // Allow the robot to move in VRep using choregraphe's angles

    simxSetJointTargetPosition(clientID, *this->firstHandles[0][0][0], commandAngles[0], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[1][0][0], commandAngles[1], simx_opmode_streaming);
    // Left Leg
    simxSetJointTargetPosition(clientID, *this->firstHandles[2][0][0], commandAngles[8], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[3][0][0], commandAngles[9], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[4][0][0], commandAngles[10], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[5][0][0], commandAngles[11], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[6][0][0], commandAngles[12], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[7][0][0], commandAngles[13], simx_opmode_streaming);
    // Right Leg
    simxSetJointTargetPosition(clientID, *this->firstHandles[8][0][0], commandAngles[14], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[9][0][0], commandAngles[15], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[10][0][0], commandAngles[16], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[11][0][0], commandAngles[17], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[12][0][0], commandAngles[18], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[13][0][0], commandAngles[19], simx_opmode_streaming);
    // Left Arm
    simxSetJointTargetPosition(clientID, *this->firstHandles[14][0][0], commandAngles[2], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[15][0][0], commandAngles[3], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[16][0][0], commandAngles[4], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[17][0][0], commandAngles[5], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[18][0][0], commandAngles[6], simx_opmode_streaming);
    // Right Arm
    simxSetJointTargetPosition(clientID, *this->firstHandles[19][0][0], commandAngles[20], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[20][0][0], commandAngles[21], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[21][0][0], commandAngles[22], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[22][0][0], commandAngles[23], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[23][0][0], commandAngles[24], simx_opmode_streaming);
    // Left Fingers
    simxSetJointTargetPosition(clientID, *this->firstHandles[24][0][0], 1.0 - commandAngles[7], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[24][0][1], 1.0 - commandAngles[7], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[24][0][2], 1.0 - commandAngles[7], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[24][0][3], 1.0 - commandAngles[7], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[24][0][4], 1.0 - commandAngles[7], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[24][0][5], 1.0 - commandAngles[7], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[24][0][6], 1.0 - commandAngles[7], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[24][0][7], 1.0 - commandAngles[7], simx_opmode_streaming);
    // Right Fingers
    simxSetJointTargetPosition(clientID, *this->firstHandles[25][0][0], 1.0 - commandAngles[25], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[25][0][1], 1.0 - commandAngles[25], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[25][0][2], 1.0 - commandAngles[25], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[25][0][3], 1.0 - commandAngles[25], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[25][0][4], 1.0 - commandAngles[25], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[25][0][5], 1.0 - commandAngles[25], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[25][0][6], 1.0 - commandAngles[25], simx_opmode_streaming);
    simxSetJointTargetPosition(clientID, *this->firstHandles[25][0][7], 1.0 - commandAngles[25], simx_opmode_streaming);
  }
  std::cout << "End of simulation" << std::endl;
}

bool VrepSimulation::setFPS(uint fps)
{
  this->waitInterval = std::chrono::milliseconds(1000 / fps);
}

int VrepSimulation::ServicegetClientID()
{
  return this->clientID;
}

// this macro will register your methods
QI_REGISTER_MT_OBJECT(VrepSimulation, ServicegetClientID, assignClientID, init, jointControl, setFPS);
