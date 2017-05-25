/***************************************************************************
 *   Copyright                                                             *
 *    poramate@physik3.gwdg.de                                             *
 *    fhesse@physik3.gwdg.de                                               *
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
 *   $Log: main.cpp,v $                                                    *
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
// the robot
#include <millipede.h>

// include the controller
#include <gait_learning.h>

#define ERROR std::cout << "passed over here.. \n";

// joint needed for fixation of the robot in the beginning
#include <ode_robots/joint.h>

// add head file for creating a sphere by Ren ------------
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>
#include <selforg/abstractcontroller.h>
#include <ode_robots/color.h>
#include <ode_robots/terrainground.h>



#include <iostream>
#include <unistd.h>
using namespace std;

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;


std::vector<lpzrobots::AbstractObstacle*> obst;
//std::vector<lpzrobots::FixedJoint*> fixator;
// add head file for creating a sphere by Ren ------------

class gait_learningsim : public lpzrobots::Simulation {
  public:

  gait_learningsim(){
    addPaletteFile("colors/UrbanExtraColors.gpl");
    addColorAliasFile("colors/UrbanColorSchema.txt");
    // you can replace color mappings in your own file, see colors/UrbanColorSchema.txt
    // addColorAliasFile("myColorSchema.txt");
    setGroundTexture("Images/whiteground.jpg"); // gets its color from the schema
    //setTitle("centered text");
    //setCaption("right aligned text");
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
    global.odeConfig.setParam("controlinterval", 1);
//    global.odeConfig.setParam("simstepsize", 0.0025);
    global.odeConfig.setParam("noise", 0.010);
    global.odeConfig.setParam("realtimefactor",2);

    // add playground
//    lpzrobots::Playground* playground
//    = new lpzrobots::Playground(odeHandle, osgHandle,
//        osg::Vec3(10, 0.2, 0.3));
//    playground->setTexture(0,0,lpzrobots::TextureDescr("Images/wall_bw.jpg",-1.5,-3));
//    playground->setPosition(osg::Vec3(0,0,.0));
//    global.obstacles.push_back(playground);

    int diedIn = 0;
    initial_S = 0.1;
    initial_trWeight = 3;
    Bs = 0.0;
    Bf = 0.0;

    S_step = 0.0;
    trWeight_step = 0.0;
    Bs_step = 0.00005;
    Bf_step = 0.0005;

    S = initial_S + S_step*diedIn;
    trWeight = initial_trWeight + trWeight_step*diedIn;
    Bs += Bs_step*diedIn;
//    Bf += Bf_step*diedIn; // To make bf change in every sim and not after a whole group

    simN = diedIn;
    count = 0;
    instantiateAgent(global);

    number_of_runs = 1;

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
    return false;
  }

  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
      if (globalData.sim_step >= simulation_time)
          simulation_time_reached=true;
  }

  virtual bool restart(const OdeHandle&, const OsgHandle&, GlobalData& global)
  {
      simN++;
      if ( simN >= number_of_runs)
          return false;

      if (agent!=0) {
          OdeAgentList::iterator itr = find(global.agents.begin(),global.agents.end(),agent);
          if (itr!=global.agents.end())
          {
              global.agents.erase(itr);
          }
          delete agent;
          agent = 0;
      }

      instantiateAgent(global);

      return true;
  }

  void setSimulationDuration(double seconds) {
      simulation_time = (long)(seconds/0.01);
  }

  void instantiateAgent(GlobalData& global) {
      // Add millipedeII robot
      lpzrobots::MillipedeConf myMillipedeConf = lpzrobots::Millipede::getDefaultConf(1.0 /*_scale*/, 4 /*_legspersegment*/, 3 /*_nofsegments*/, 1 /*_useShoulder*/,1 /*_useFoot*/,1 /*_useBack*/);
//      myMillipedeConf.rubberFeet = true;
      lpzrobots::OdeHandle rodeHandle = odeHandle;
      rodeHandle.substance = lpzrobots::Substance(0.8, 0.01, 40.0, 0.5);//lpzrobots::Substance::getDefaultSubstance();
//      auto terrainground = new lpzrobots::TerrainGround(rodeHandle, osgHandle.changeColor(lpzrobots::Color(83.0/255.0, 48.0/255.0, 0.0)), "rough6.ppm", "", 34.9, 34.9, 0.2);
//      terrainground->setPose(osg::Matrix::translate(4, 0, 0.19));
//      global.obstacles.push_back(terrainground);


      //Change weight over time to check evolution of sensory feedback strength
      myMillipedeConf.frontMass = trWeight;


      millipede = new lpzrobots::Millipede(
          rodeHandle,
          osgHandle.changeColor(lpzrobots::Color(2, 1, 1)),
          myMillipedeConf, "Millipede");


      // put millipede a little bit in the air
      millipede->place(osg::Matrix::translate(.0, .0, 0.4)*osg::Matrix::rotate(.0, .0, .0, .0));

      // add a speed sensor to the robot (attached to the "main primitive" (-1)
      //  (specifiy index if needed)
      millipede->addSensor(std::make_shared<SpeedSensor>(1), Attachment(-1));

      controller = new gait_learning(myMillipedeConf);
      controller->initializeCPGs();
      controller->S = S;
      controller->simN = simN;
      // create wiring
      One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise());

      // create agent and init it with controller, robot and wiring
      agent = new lpzrobots::OdeAgent(global);
      agent->init(controller, millipede, wiring);

      // inform global variable over everything that happened:
      global.configs.push_back(millipede);
      global.agents.push_back(agent);
      global.configs.push_back(controller);


      setSimulationDuration(151);
      //usleep(500000);
  }


protected:
  lpzrobots::OdeAgent* agent;
  lpzrobots::Joint* robotfixator;
  gait_learning* controller;
  lpzrobots::Millipede* millipede;
  int number_of_runs, runs_x, count;
  double S, trWeight, Bf, Bs;
  double S_step, trWeight_step, Bf_step, Bs_step;
  double initial_followed, initial_S, initial_sfeed, initial_trWeight;
  int simN;
};

int main(int argc, char **argv)
{
  gait_learningsim sim;
  sim.setGroundTexture("Images/greenground.rgb");
  return sim.run(argc, argv) ? 0 : 1;
}




//Locomotion control for complex behaviour of multiple legged systems
