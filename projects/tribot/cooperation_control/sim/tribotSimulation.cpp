#include "tribotSimulation.h"

#include <selforg/one2onewiring.h>
#include <selforg/noisegenerator.h>

#include "factory.h"
#include "tribot.h"
#include "basiccontroller.h"

#include <ode_robots/odeagent.h>

#include <fstream>
#include <iostream>

using namespace tribot;

TribotSimulation::TribotSimulation(TribotSimTaskHandle tribotSimTaskHandle)
  : tribotSimTaskHandle(tribotSimTaskHandle)
{
  addPaletteFile("colors/UrbanExtraColors.gpl");
  addColorAliasFile("colors/UrbanColorSchema.txt");
  setGroundTexture("Images/whiteground.jpg");
  setGroundTexture("Images/greenground.rgb");
  setTaskNameSuffix(tribotSimTaskHandle.name);
  setupFileStream(tribotSimTaskHandle.name + ".csv");
}

void TribotSimulation::setupFileStream(std::string filename) {
  this->stream = new std::fstream();
  stream->open(filename, std::fstream::out);
  (*stream) << "\"step\",";
  (*stream) << "\"name1\",\"x1\",\"y1\",\"z1\",";
  (*stream) << "\"name2\",\"x2\",\"y2\",\"z2\",";
  (*stream) << "\"goal_x\",\"goal_y\",\"goal_z\"\n";
}

  /**
   * Sets the initial Camera position as a
   * starting point at "from" and going towards "to"
   */
void TribotSimulation::initializeCamera(lpzrobots::Pos from, lpzrobots::Pos to) {
    setCameraHomePos(from, to);
}

  /**
   * Contains all the parameters for initializing
   * the playground
   */
lpzrobots::Playground * TribotSimulation::initializePlayground(const lpzrobots::GlobalData& globalData,
                                                        const lpzrobots::OdeHandle& odeHandle,
                                                        const lpzrobots::OsgHandle& osgHandle)
{
  lpzrobots::Playground* playground
    = new lpzrobots::Playground(odeHandle, osgHandle,
                     osg::Vec3(100, 0.2, 0.3));
  //playground->setTexture(0, 0, lpzrobots::TextureDescr("Images/wall_bw.jpg",-1.5,-3));
  playground->setPosition(osg::Vec3(0.0, 0.0, 0.0));
  // need to declare type of forward declaration
  std::vector<lpzrobots::AbstractObstacle*> obstacles = globalData.obstacles;
  obstacles.push_back(playground);

  return playground;
}

/**
   * Creates and adds the robots to the simulation
   */
lpzrobots::OdeAgent * TribotSimulation::createRobot(const lpzrobots::OdeHandle& odeHandle,
                                                    const lpzrobots::OsgHandle& osgHandle,
                                                    lpzrobots::GlobalData& global,
                                                    const Position& goal,
                                                    const TribotAgentConfig& agentConfig)
{
  auto robot = new lpzrobots::Tribot(odeHandle,
                                     osgHandle,
                                     lpzrobots::Tribot::getDefaultConfig(),
                                     agentConfig.name);
  robot->place(agentConfig.position);

  auto controller = new BasicController(robot,
                                        goal,
                                        agentConfig.name,
                                        tribotSimTaskHandle.soundGenerator);
  One2OneWiring* wiring =
    new One2OneWiring(new ColorUniformNoise(.1));

  lpzrobots::OdeAgent * agent = new lpzrobots::OdeAgent(global);

  agent->init(controller, robot, wiring);

  global.agents.push_back(agent);
  global.configs.push_back(agent);

  return agent;
}

void TribotSimulation::initializeRobots(const lpzrobots::OdeHandle& odeHandle,
                                        const lpzrobots::OsgHandle& osgHandle,
                                        lpzrobots::GlobalData& global,
                                        const Position& goal)
{
  lpzrobots::OdeAgent * agent1 = createRobot(odeHandle,
                                             osgHandle,
                                             global,
                                             goal,
                                             tribotSimTaskHandle.agent1);
  lpzrobots::OdeAgent * agent2 = createRobot(odeHandle,
                                             osgHandle,
                                             global,
                                             goal,
                                             tribotSimTaskHandle.agent2);

  // Sets robot2 to controller1
  BasicController * controller1 = dynamic_cast<BasicController*>(agent1->getController());
  controller1->setMatePositionReference(dynamic_cast<lpzrobots::Tribot*>(agent2->getRobot()));
  // Sets robot1 to controller2
  BasicController * controller2 = dynamic_cast<BasicController*>(agent2->getController());
  controller2->setMatePositionReference(dynamic_cast<lpzrobots::Tribot*>(agent1->getRobot()));
}

/**
 * All objects, agents etc should be initialized in this method
 */
void TribotSimulation::start(const lpzrobots::OdeHandle & odeHandle,
                             const lpzrobots::OsgHandle & osgHandle,
                             lpzrobots::GlobalData &globalData,
                             lpzrobots::SimulationTaskHandle &simTaskHandle,
                             int taskId)
{
  initializePlayground(globalData, odeHandle, osgHandle);
  initializeSimulationParameters(globalData);
  lpzrobots::AbstractObstacle * goal = initializeGoal(odeHandle, osgHandle, globalData);
  initializeRobots(odeHandle,
                   osgHandle,
                   globalData,
                   goal->getPosition());
  initializeCamera(lpzrobots::Pos(10, 10, 100), lpzrobots::Pos(-10, -10, 0));
}

lpzrobots::AbstractObstacle *
TribotSimulation::initializeGoal(const lpzrobots::OdeHandle& odeHandle,
                                 const lpzrobots::OsgHandle& osgHandle,
                                 lpzrobots::GlobalData& globalData)
{
  Factory * factory = new Factory(odeHandle, osgHandle, globalData);
  lpzrobots::PassiveBox * passiveBox = factory->createBox(1,1,1,
                                                          tribotSimTaskHandle.goal.position);
  return factory->initFixedJoint(passiveBox).front();
}


void TribotSimulation::initializeSimulationParameters(const lpzrobots::GlobalData & globalData)
{
  lpzrobots::OdeConfig config = globalData.odeConfig;
  config.setParam("simstepsize", 0.01);
  config.setParam("noise", 0.3);
}

/**
 * Allows for inspection at each step
 */
void TribotSimulation::addCallback(lpzrobots::GlobalData &globalData,
                                   bool draw,
                                   bool pause,
                                   bool control,
                                   lpzrobots::SimulationTaskHandle & handle,
                                   int taskId)
{
  std::vector<lpzrobots::OdeAgent*> agents = globalData.agents;
  bool goalFlag = false;
  Position goal;
  (*this->stream) << globalData.sim_step << ",";
  for (std::vector<lpzrobots::OdeAgent*>::iterator agent = agents.begin();
       agent != agents.end();
       agent++) {
    BasicController * controller = dynamic_cast<BasicController*>((*agent)->getController());
    lpzrobots::Tribot * bot = controller->getRobot();
    (*this->stream) << getLine(bot);
    double dist = controller->rangeToGoal();
    //std::cout << "dist: " << dist << "\n";
    goal = controller->getGoal();
    if (dist <= tribotSimTaskHandle.goal.winDistance) {
      goalFlag = true;
    }
  }
  (*this->stream) << goal.x << "," << goal.y << "," << goal.z << "\n";
  if(goalFlag) {
    std::cout << "Reached Goal\n";
    // Will end simulation as goal condition
    simulation_time_reached = true;
  }
  if(globalData.sim_step >= 8000) {
    std::cout << "Sim ended noncomplete at step 5000\n";
    simulation_time_reached = true;
  }
}

std::string TribotSimulation::getLine(lpzrobots::Tribot * bot) {
  std::stringstream str;
  str << "\"" << bot->getName() << "\",";
  Position position = bot->getPosition();
  str << position.x  << "," << position.y << "," << position.z << ",";
  return str.str();
}

TribotSimulation::~TribotSimulation() {
  std::cout << "Closing Stream\n";
  this->stream->close();
}

bool TribotSimulation::restart(const lpzrobots::OdeHandle &,
                          const lpzrobots::OsgHandle &,
                          lpzrobots::GlobalData &globalData,
                          lpzrobots::SimulationTaskHandle &,
                          int taskId)
{
  return false;
}
