#include <stdio.h>
#include <math.h>
#include <stdlib.h>

// ode_robots
#include <ode_robots/simulation.h>
#include <ode_robots/odeagent.h>
#include <ode_robots/playground.h>
#include <ode_robots/passivebox.h>
#include <ode_robots/terrainground.h>

// selforg
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>

#include <legWheelBotPopulationEvaluator.h>

// The robot
#include <legWheelBot.h>

//#include "utils/sim_robots/legwheelbot/legWheelBot.h"
#include "legWheelBotDifferentialDriveController.h"

#include "legWheelSim.h"
#include "terrainGenerator.h"

LegWheelSim::LegWheelSim(TerrainType terrainType, LegWheelBotConf conf) { 
  LegWheelSim::terrainType = terrainType;
  LegWheelSim::conf = conf;
}
LegWheelSim::~LegWheelSim() { }

using namespace lpzrobots;
using namespace std;

void LegWheelSim::start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) {
  
  // Initial position and orientation of the camera (use 'p' in graphical window to find out)
      setCameraHomePos(Pos(-14,14, 10),  Pos(-135, -24, 0));
      // Some simulation parameters can be set here
      global.odeConfig.setParam("controlinterval", 1);
      global.odeConfig.setParam("gravity", -9.8); 
	
      float terrainHeight = TerrainGenerator::setupTerrain(odeHandle,osgHandle,global, terrainType, 0.0f, 5.0f);
      
      // Instantiating the robot 
      OdeRobot* robot = new LegWheelBot(odeHandle, osgHandle, conf, "LegWheelBot");
      // Placing the robot in the scene
      float placeHeight = 0.05 + terrainHeight + (conf.leftWheel.spokeLength + conf.leftWheel.wheelRadius, conf.rightWheel.spokeLength + conf.rightWheel.spokeLength);
      ((OdeRobot*)robot)->place(Pos(.0, .0, terrainHeight)); 
      
      // Instantiatign the controller   
      controller = new LegWheelBotDifferentialDriveController("LegWheelBotDifferentialDriveController", "$ID$");
      // Create the wiring with color noise
      AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(.0));
      // Create Agent  
      OdeAgent* agent = new OdeAgent(global);
      // Agent initialisation
      agent->init(controller, robot, wiring);
      // Adding the agent to the agents list
      global.agents.push_back(agent);
      global.configs.push_back(agent); 
}
    
// /* Functions not used here but typically useful */
// void LegWheelSim::addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
// }
// 
// bool LegWheelSim::command ( const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down )
// {
//   return false;
// }
// 
// void LegWheelSim::bindingDescription(osg::ApplicationUsage & au) const {
//   au.addKeyboardMouseBinding(osgGA::GUIEventAdapter::KEY_Control_L, osgGA::GUIEventAdapter);
// }
