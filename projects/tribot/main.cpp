// Include simulation Environment header
#include <ode_robots/simulation.h>
// include robot container (agent)
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

#include "tribot.h"
#include "factory.h"


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
  void initializeFixedObjects(const OdeHandle& odeHandle,
                              const OsgHandle& osgHandle,
                              GlobalData& global)
  {

  }

  /**
   * Creates and adds the robots to the simulation
   */
  OdeAgent * createRobot(const OdeHandle& odeHandle,
                         const OsgHandle& osgHandle,
                         GlobalData& global,
                         const Pos& position)
  {
    auto robot = new Tribot(odeHandle, osgHandle);
    robot->place(position);

    auto controller = new BasicController();
    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(.1));

    OdeAgent* agent = new OdeAgent(global);

    agent->init(controller, robot, wiring);

    global.agents.push_back(agent);
    global.configs.push_back(agent);

    return agent;
  }

  void initializeRobots(const OdeHandle& odeHandle,
                        const OsgHandle& osgHandle,
                        GlobalData& global)
  {
    createRobot(odeHandle, osgHandle, global, Pos(4, 0, 0));
    createRobot(odeHandle, osgHandle, global, Pos(-4, 0, 0));
  }

  /**
   * Contains all the setup for starting the simulation
   */
  virtual void start(const OdeHandle& odeHandle,
                     const OsgHandle& osgHandle,
                     GlobalData& global)
  {
    initializeCamera(Pos(10, 10, 30), Pos(-10, -10, 0));

    initializeSimulationParameters(global);
    initializePlayground(global, odeHandle, osgHandle);

    initializeFixedObjects(odeHandle, osgHandle, global);

    initializeRobots(odeHandle, osgHandle, global);

  }
};

/**
 * Main method to start the program
 */
int main(int argc, char **argv)
{
  TribotSim sim;
  return sim.run(argc, argv) ? 0 : 1;
}
