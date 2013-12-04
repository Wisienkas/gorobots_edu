/***************************************************************************
 *   Copyright (C) 2012 by                                                 *
 *    Timo Nachstedt <nachstedt@physik3.gwdg.de>                           *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 ***************************************************************************/


// include simulation environment stuff
#include <ode_robots/simulation.h>
// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>
// playground
#include <ode_robots/playground.h>
// simple wiring
#include <selforg/one2onewiring.h>
// the controller
#include "controllers/nejihebi/nejihebiexamplecontroller.h"
#include "controllers/nejihebi/nejihebilpzinterface.h"
// the robot
#include <ode_robots/nejihebi.h>
// we need to increase the stack size
#include <sys/resource.h>


class ThisSim : public lpzrobots::Simulation {
  public:

    ThisSim(){
      controller = 0;
      lpzController = 0;
      snake = 0;
    }

    /**
     * starting function (executed once at the beginning of the simulation loop)
     */
    virtual void start(const lpzrobots::OdeHandle& odeHandle,
        const lpzrobots::OsgHandle& osgHandle,
        lpzrobots::GlobalData& global) {

      // set initial camera position
      setCameraHomePos(
          lpzrobots::Pos(-0.0114359, 6.66848, 0.922832),
          lpzrobots::Pos(178.866, -7.43884, 0));

      // set simulation parameters
      global.odeConfig.setParam("controlinterval", 2);
      global.odeConfig.setParam("simstepsize", 0.05);

      // add playground
      lpzrobots::Playground* playground
        = new lpzrobots::Playground(odeHandle, osgHandle,
                                    osg::Vec3(60, 1.0, 1.5));
      playground->setPosition(osg::Vec3(0,0,0));
      global.obstacles.push_back(playground);

      // Add snake-like robot using screw drive mechanism
      snake = new lpzrobots::Nejihebi(
          odeHandle,
          osgHandle.changeColor(lpzrobots::Color(1, 1, 1)));

      // put snake a little bit in the air
      snake->place(osg::Matrix::translate(.0, .0, 2.0));

      // create controller and lpz interface
      controller = new NejihebiExampleController();
      lpzController = new NejihebiLpzInterface(controller);

      // neccessary to map sensor and motor values into the range of -1 to 1
      lpzController->setRobotMaxSpeed(
          snake->getConf().screwbase.maxSpeed,
          snake->getConf().jointUnit.maxVel);

      // create wiring
      One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.0));

      // create agent and init it with controller, robot and wiring
      lpzrobots::OdeAgent* agent = new lpzrobots::OdeAgent(global);
      agent->init(lpzController, snake, wiring);

      // inform global variable over everything that happened:
      global.configs.push_back(snake);
      global.agents.push_back(agent);
      global.configs.push_back(lpzController);

    }

    /**
     * add own key handling stuff here, just insert some case values
     */
    virtual bool command(const lpzrobots::OdeHandle&,
        const lpzrobots::OsgHandle&,
        lpzrobots::GlobalData& globalData,
        int key,
        bool down)
        {
      if (down) { // only when key is pressed, not when released
        switch (char(key)) {
          default:
            return false;
            break;
        }
      }
      return false;
    }
  protected:
    NejihebiExampleController*   controller;
    NejihebiLpzInterface* lpzController;
    lpzrobots::Nejihebi*  snake;
};

int main(int argc, char **argv) {

  // We have to increase the stack size as there are really a lot of collisions
  // to treat for this robot
  const rlim_t kStackSize = 32 * 1024 * 1024;   // min stack size = 16 MB
  struct rlimit rl;
  int result = getrlimit(RLIMIT_STACK, &rl);
  if (result == 0) {
      if (rl.rlim_cur < kStackSize) {
          rl.rlim_cur = kStackSize;
          result = setrlimit(RLIMIT_STACK, &rl);
          if (result != 0)
            std::cerr << "setrlimit returned result = " << result << "\n";
      }
  } else {
    std::cerr << "tetrlimit returned result = " << result << "\n";
  }

  ThisSim sim;
  sim.setGroundTexture("Images/greenground.rgb");
  return sim.run(argc, argv) ? 0 : 1;
}

