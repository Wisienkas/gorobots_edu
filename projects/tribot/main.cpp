// Include simulation Environment header
#include <ode_robots/simulation.h>
// Include robot container (agent)
#include <ode_robots/odeagent.h>
// include playground
#include <ode_robots/playground.h>
// wiring simple
#include <selforg/one2onewiring.h>
// robot
// put include here for robot (need to find right robot)

// Include a controller of some kind here
#include "basiccontroller.h"

// joint and fixation
#include <ode_robots/joint.h>
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>
#include <selforg/abstractcontroller.h>

// include colors
#include <ode_robots/color.h>

// Regular include
#include <iostream> // why I need this?

//#include "tribot.h"
#include "factory.h"
#include "lizard_ear.h"

using namespace std;
using namespace lpzrobots;

class TribotSim : public Simulation {
  public:

  TribotSim() {
    addPaletteFile("colors/UrbanExtraColors.gpl");
    addColorAliasFile("colors/UrbanColorSchema.txt");
    setGroundTexture("Images/whiteground.jpg");
    setGroundTexture("Images/greenground.rgb");
  }

  /**
   * Sets the initial Camera position as a
   * starting point at "from" and going towards "to"
   */
  void initializeCamera(Pos from, Pos to) {
    setCameraHomePos(from, to);
  }

  /**
   * Contains all the parameters for initializing
   * the playground
   */
  Playground * initializePlayground(const GlobalData& globalData,
                            const OdeHandle& odeHandle,
                            const OsgHandle& osgHandle)
  {
    Playground* playground
      = new Playground(odeHandle, osgHandle,
                                  osg::Vec3(100, 0.2, 0.3));
    playground->setTexture(0,0,TextureDescr("Images/wall_bw.jpg",-1.5,-3));
    playground->setPosition(osg::Vec3(0,0,.0));
    // need to declare type of forward declaration
    std::vector<AbstractObstacle*> obstacles = globalData.obstacles;
    obstacles.push_back(playground);

    return playground;
  }

  /**
   * Contains simulation parameters
   */
  void initializeSimulationParameters(const GlobalData& globalData)
  {
    OdeConfig config = globalData.odeConfig;
    config.setParam("controlinterval", 10);
    config.setParam("simstepsize", 0.01);
    config.setParam("noise", 0.3);
  }

  /**
   * Will put fixed objects into the world
   */
  std::vector<lpzrobots::AbstractObstacle*> initializeFixedObjects(const OdeHandle& odeHandle,
                              const OsgHandle& osgHandle,
                              GlobalData& global)
  {
    Factory * factory = new Factory(odeHandle, osgHandle, global);
    PassiveBox * passiveBox = factory->createBox(1,1,1, osg::Vec3(-4,20,0));
    return factory->initFixedJoint(passiveBox);
  }

  /**
   * Creates and adds the robots to the simulation
   */
  OdeAgent * createRobot(const OdeHandle& odeHandle,
                         const OsgHandle& osgHandle,
                         GlobalData& global,
                         const Pos& position,
                         const Position& goal,
                         const std::string& name)
  {
    auto robot = new Tribot(odeHandle, osgHandle, Tribot::getDefaultConfig(), "Tribot");
    robot->place(position);

    auto controller = new BasicController(robot, goal, name);
    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(.1));

    OdeAgent* agent = new OdeAgent(global);

    agent->init(controller, robot, wiring);

    global.agents.push_back(agent);
    global.configs.push_back(agent);

    return agent;
  }

  void initializeRobots(const OdeHandle& odeHandle,
                        const OsgHandle& osgHandle,
                        GlobalData& global,
                        const Position& goal)
  {
    // Put all the agents in a list.
    std::vector<OdeAgent*> agents;
    agents.push_back(createRobot(odeHandle, osgHandle, global, Pos(0, 0, 0), goal, "agent1"));
    agents.push_back(createRobot(odeHandle, osgHandle, global, Pos(5, 0, 0), goal, "agent2"));


    // Combine all robots from list
    for (vector<OdeAgent*>::iterator i = agents.begin(); i != agents.end(); ++i) {
      BasicController* controller = dynamic_cast<BasicController*>((*i)->getController());
      if(controller == nullptr) {
        continue;
      }
      for (vector<OdeAgent*>::iterator j = agents.begin(); j != agents.end(); ++j) {
        Tribot* teammate = dynamic_cast<Tribot*>((*j)->getRobot());
        if(i == j || teammate == nullptr) {
          continue;
        }
        controller->setMatePositionReference(teammate);
      }
    }

    for (vector<OdeAgent*>::iterator i = agents.begin(); i != agents.end(); ++i) {
      BasicController* controller = dynamic_cast<BasicController*>((*i)->getController());
      if(controller == nullptr) {
        continue;
      }
      //controller->printTeam();
    }

  }

  /**
   * Contains all the setup for starting the simulation
   */
  virtual void start(const OdeHandle& odeHandle,
                     const OsgHandle& osgHandle,
                     GlobalData& global)
  {
    initializeCamera(Pos(10, 10, 100), Pos(-10, -10, 0));

    initializeSimulationParameters(global);
    initializePlayground(global, odeHandle, osgHandle);

    std::vector<lpzrobots::AbstractObstacle*> obstacles =
      initializeFixedObjects(odeHandle, osgHandle, global);

    initializeRobots(odeHandle, osgHandle, global, obstacles.front()->getPosition());

  }
};

void test();

/**
 * Main method to start the program
 */
int main(int argc, char **argv)
{
  //test();
  //return 0;
  TribotSim sim;
  return sim.run(argc, argv) ? 0 : 1;
}

#include "toolbox.h"
#include "sinewave.h"
void test() {
  int size = 500;
  std::vector<std::vector<double>> values = toolbox::sinewaveSampling(M_PI, size, 50000, 1700);
  std::vector<double>::iterator a = values.front().begin();
  std::vector<double>::iterator b = values.back().begin();
  for (int i = 0; i < size; i++) {
    //std::cout << "L: " << *a << " R: " << *b << "\n";
    a++;
    b++;
  }
  //LizardEar *ear = new LizardEar(size);
  //LizardEarSum sum = ear->filter(values.front(), values.back());
  //std::cout << "L: " << sum.left << "\nR: " << sum.right << "\n";
  lizard_ear *ear2 = new lizard_ear();
  ear2->filter(values.front(), values.back());
  std::cout << "L: " << std::log10(ear2->sumL) * 20 << "\nR: " << std::log10(ear2->sumR) * 20 << "\n";
  std::cout << "diff: " << std::log10(ear2->sumL) * 20 - std::log10(ear2->sumR) * 20 << "\n";
}
