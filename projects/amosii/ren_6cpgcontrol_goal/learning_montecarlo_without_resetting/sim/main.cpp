/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
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

#include <ode_robots/amosII.h>
//#include <amosII.h>


// the controller
#include "amosIIcontrol.h"
// joint needed for fixation of the robot in the beginning
#include <ode_robots/joint.h>

// add head file for creating a sphere by Ren ------------
#include <ode_robots/globaldata.h>
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>
#include <selforg/abstractcontroller.h>
#include <ode_robots/color.h>
#include <iostream>
using namespace std;
std::vector<lpzrobots::AbstractObstacle*> obst;
//std::vector<lpzrobots::FixedJoint*> fixator;
// add head file for creating a sphere by Ren ------------

class ThisSim : public lpzrobots::Simulation {
  public:

    ThisSim() {
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
    virtual void start(const lpzrobots::OdeHandle& odeHandle, const lpzrobots::OsgHandle& osgHandle,
        lpzrobots::GlobalData& global) {
      // set initial camera position
      setCameraHomePos(lpzrobots::Pos(-0.0114359, 6.66848, 0.922832), lpzrobots::Pos(178.866, -7.43884, 0));

      // set simulation parameters
      global.odeConfig.setParam("controlinterval", 10);
      global.odeConfig.setParam("simstepsize", 0.01);

      // add playground
//      lpzrobots::Playground* playground = new lpzrobots::Playground(odeHandle, osgHandle, osg::Vec3(10, 0.2, 0.3));
//      playground->setTexture(0, 0, lpzrobots::TextureDescr("Images/wall_bw.jpg", -1.5, -3));
//      playground->setPosition(osg::Vec3(0, 0, .0));
//      global.obstacles.push_back(playground);

//      //----------create a sphere as the target by Ren-----------------------------
//      //the first sphere
//      lpzrobots::PassiveSphere* s1 = new lpzrobots::PassiveSphere(odeHandle, osgHandle, 0.1);
//      s1->setPosition(osg::Vec3(3.0, 0.0, 0.1));
//      s1->setTexture("Images/dusty.rgb");
//      s1->setColor(lpzrobots::Color(1, 0, 0));
//      obst.push_back(s1);
//      global.obstacles.push_back(s1);
//      lpzrobots::FixedJoint* fixator1 = new lpzrobots::FixedJoint(s1->getMainPrimitive(), global.environment);
//      fixator1->init(odeHandle, osgHandle);
//
//      //the second sphere
//      lpzrobots::PassiveSphere* s2 = new lpzrobots::PassiveSphere(odeHandle, osgHandle, 0.1);
//      s2->setPosition(osg::Vec3(0.0, 3.0, 0.1));
//      s2->setTexture("Images/dusty.rgb");
//      s2->setColor(lpzrobots::Color(0, 1, 0));
//      obst.push_back(s2);
//      global.obstacles.push_back(s2);
//      lpzrobots::FixedJoint* fixator2 = new lpzrobots::FixedJoint(s2->getMainPrimitive(), global.environment);
//      fixator2->init(odeHandle, osgHandle);
//
//      //the third sphere
//      lpzrobots::PassiveSphere* s3 = new lpzrobots::PassiveSphere(odeHandle, osgHandle, 0.1);
//      s3->setPosition(osg::Vec3(0.0, -3.0, 0.1));
//      s3->setTexture("Images/dusty.rgb");
//      s3->setColor(lpzrobots::Color(0, 0, 1));
//      obst.push_back(s3);
//      global.obstacles.push_back(s3);
//      lpzrobots::FixedJoint* fixator3 = new lpzrobots::FixedJoint(s3->getMainPrimitive(), global.environment);
//      fixator3->init(odeHandle, osgHandle);
//
//      //----------create a sphere as the target by Ren-----------------------------

      // Add amosII robot
      lpzrobots::AmosIIConf myAmosIIConf = lpzrobots::AmosII::getDefaultConf(1.0 /*_scale*/, 1 /*_useShoulder*/,
          1 /*_useFoot*/, 1 /*_useBack*/);
      myAmosIIConf.rubberFeet = true;
      lpzrobots::OdeHandle rodeHandle = odeHandle;
      rodeHandle.substance = lpzrobots::Substance(3.0, 0.0, 50.0, 0.8);
//      //------------------- Link the sphere to the Goal Sensor by Ren---------------
//      for (int i = 0; i < obst.size(); i++) {
//        myAmosIIConf.GoalSensor_references.push_back(obst.at(i)->getMainPrimitive());
//      }
//      //------------------- Link the sphere to the Goal Sensor by Ren---------------
      amos = new lpzrobots::AmosII(rodeHandle, osgHandle.changeColor(lpzrobots::Color(1, 1, 1)), myAmosIIConf,
          "AmosII");

      // define the usage of the individual legs
      amos->setLegPosUsage(amos->L0, amos->LEG);
      amos->setLegPosUsage(amos->L1, amos->LEG);
      amos->setLegPosUsage(amos->L2, amos->LEG);
      amos->setLegPosUsage(amos->R0, amos->LEG);
      amos->setLegPosUsage(amos->R1, amos->LEG);
      amos->setLegPosUsage(amos->R2, amos->LEG);

      // put amos a little bit in the air
      amos->place(osg::Matrix::translate(.0, .0, 0.2));

      controller = new AmosIIControl(); //TripodGait18DOF();
      // create wiring
      One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise());

      // create agent and init it with controller, robot and wiring
      lpzrobots::OdeAgent* agent = new lpzrobots::OdeAgent(global);
      agent->init(controller, amos, wiring);

      // create a fixed joint to hold the robot in the air at the beginning
//      robotfixator = new lpzrobots::FixedJoint(amos->getMainPrimitive(), global.environment);
//      robotfixator->init(odeHandle, osgHandle, false);

      // inform global variable over everything that happened:
      global.configs.push_back(amos);
      global.agents.push_back(agent);
      global.configs.push_back(controller);

      std::cout << "\n\n" << "################################\n" << "#   Press x to free amosII!    #\n"
          << "################################\n" << "\n\n" << std::endl;

    }

    /**
     * add own key handling stuff here, just insert some case values
     */
    virtual bool command(const lpzrobots::OdeHandle&, const lpzrobots::OsgHandle&, lpzrobots::GlobalData& globalData,
        int key, bool down) {
//      if (down) { // only when key is pressed, not when released
//        switch (char(key)) {
//          case 'x':
//            if (robotfixator) {
//              std::cout << "dropping robot" << std::endl;
//              delete robotfixator;
//              robotfixator = NULL;
//            }
//            break;
//          default:
//            return false;
//            break;
//        }
//      }
      return false;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    //	 optional additional callback function which is called every simulation step.
    //	 Called between physical simulation step and drawing.
    //   @param draw indicates that objects are drawn in this timestep
    //	 @param pause always false (only called of simulation is running)
    //   @param control indicates that robots have been controlled this timestep
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual void addCallback(lpzrobots::GlobalData& globalData, bool draw, bool pause, bool control) {
      // for demonstration: set simsteps for one cycle to 60.000/currentCycle (10min/currentCycle)
      // if simulation_time_reached is set to true, the simulation cycle is finished

//      //----------------------------Reset Function-----------------------------------------
//      if (globalData.sim_step >= 6000) {
//        if (robotfixator) {
//          std::cout << "dropping robot" << std::endl;
//          delete robotfixator;
//          robotfixator = NULL;
//        }
//      }
//      if (globalData.sim_step >= 100) //a small delay to make sure the robot will not restart at beginning!
//          {
//        if ((((AmosIIControl*) controller)->get_DistancetoGoal() <= 0.1)
//            && (((AmosIIControl*) controller)->control_adaptiveclimbing.getlearning())) {
//          //simulation_time_reached=true;
//          amos->place(osg::Matrix::translate(.0, .0, 0.2));
//        }





      //-------------Do not need this for continuous learning (no reset)------------KOH-------------------------------------//

//      if (!((AmosIIControl*) controller)->control_adaptiveclimbing.getlearning()) {
//          std::cout<<"simulation stopped according to specified condition"<<std::endl;
//          simulation_time_reached = true;
//        }
      //-------------Do not need this for continuous learning (no reset)------------KOH-------------------------------------//




//      }
//      //-----------------------------------------------------------------------------------
    }

//    //////////////////////////////////////////////////////////////////////////////////////
//    //							reset function                                          //
//    //////////////////////////////////////////////////////////////////////////////////////
//    virtual bool restart(const lpzrobots::OdeHandle& odeHandle, const lpzrobots::OsgHandle& osgHandle, lpzrobots::GlobalData& global)
//    {
//    	if (this->currentCycle == 2)
//    	{
//    		//clean robots
//    	  	while (global.agents.size() > 0)
//    	  	{
//    	  		lpzrobots::OdeAgent* agent = *global.agents.begin();
//    	  		AbstractController* controller = agent->getController();
//    	  		lpzrobots::OdeRobot* robot = agent->getRobot();
//    	  		AbstractWiring* wiring = agent->getWiring();
//
//    	  		global.configs.erase(std::find(global.configs.begin(),global.configs.end(), controller));
//
//    	  		delete controller;
//    	  		delete robot;
//    	  		delete wiring;
//    	  		delete (agent);
//
//    	  		global.agents.erase(global.agents.begin());
//    	  	}
//
//    	  	// clean the playgrounds
//    	  	while (global.obstacles.size() > 0)
//    	  	{
//    	  		std::vector<lpzrobots::AbstractObstacle*>::iterator iter = global.obstacles.begin();
//    	  		delete (*iter);
//    	  		global.obstacles.erase(iter);
//    	  	}
//    	  	std::cout << "end."<<endl;
//    	  	return false; // don't restart, just quit
//    	}
//
//    	// Now we must delete all robots and agents from the simulation and create new robots and agents.
//    	while (global.agents.size() > 0)
//    	{
//    		lpzrobots::OdeAgent* agent = *global.agents.begin();
//    	  	AbstractController* cont = agent->getController();
//    	  	lpzrobots::OdeRobot* robot = agent->getRobot();
//    	  	AbstractWiring* wiring = agent->getWiring();
//
//    	  	global.configs.erase(std::find(global.configs.begin(AmosIIControl),global.configs.end(), cont));
//
//    	  	//this calls destroy:
//    	  	delete cont;
//    	  	delete robot;
//    	  	std::cout << "after delete robot"<<endl;
//    	  	delete wiring;
//    	  	std::cout << "after delete wiring"<<endl;
//    	  	delete agent;
//
//    	  	std::cout << "after delete agent"<<endl;
//    	  	global.agents.erase(global.agents.begin());
//    	  	std::cout << "after global.agents.erase"<<endl;
//    	}
//    	cout<<"deleted agent!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
//
//    	// Add amosII robot
//    	lpzrobots::AmosIIConf myAmosIIConf = lpzrobots::AmosII::getDefaultConf(1.0 /*_scale*/,1 /*_useShoulder*/,1 /*_useFoot*/,1 /*_useBack*/);
//        myAmosIIConf.rubberFeet = true;
//        lpzrobots::OdeHandle rodeHandle = odeHandle;
//        rodeHandle.substance = lpzrobots::Substance(3.0, 0.0, 50.0, 0.8);
//        //------------------- Link the sphere to the Goal Sensor by Ren---------------
//        //for(int i = 0; i<obst.size(); i++)
//        //{
//   	    //	  myAmosIIConf.GoalSensor_references.push_back(obst.at(i)->getMainPrimitive());
//        //}
//        //------------------- Link the sphere to the Goal Sensor by Ren---------------
//        amos = new lpzrobots::AmosII(
//   	    		  rodeHandle,
//   	    		  osgHandle.changeColor(lpzrobots::Color(1, 1, 1)),
//   	    		  myAmosIIConf, "AmosII");
//
//        // define the usage of the individual legs
//    	amos->setLegPosUsage(amos->L0, amos->LEG);
//    	amos->setLegPosUsage(amos->L1, amos->LEG);
//    	amos->setLegPosUsage(amos->L2, amos->LEG);
//    	amos->setLegPosUsage(amos->R0, amos->LEG);
//    	amos->setLegPosUsage(amos->R1, amos->LEG);
//    	amos->setLegPosUsage(amos->R2, amos->LEG);
//
//    	// put amos a little bit in the air
//    	amos->place(osg::Matrix::translate(.0, .0, 0.5));
//
//    	controller = new AmosIIControl();//TripodGait18DOF();
//    	// create wiring
//    	One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise());
//
//    	// create agent and init it with controller, robot and wiring
//    	lpzrobots::OdeAgent* agent = new lpzrobots::OdeAgent(global);
//    	agent->init(controller, amos, wiring);
//
//    	// create a fixed joint to hold the robot in the air at the beginning
//    	//robotfixator = new lpzrobots::FixedJoint(
//    	//    amos->getMainPrimitive(),
//    	//    global.environment);
//    	//robotfixator->init(odeHandle, osgHandle, false);
//
//    	// inform global variable over everything that happened:
//    	global.configs.push_back(amos);
//    	global.agents.push_back(agent);
//    	global.configs.push_back(controller);
//    	return true;
//    }
  protected:
//    lpzrobots::Joint* robotfixator;
    AbstractController* controller;
    lpzrobots::AmosII* amos;
};

int main(int argc, char **argv) {
  ThisSim sim;
  sim.setGroundTexture("Images/greenground.rgb");
  return sim.run(argc, argv) ? 0 : 1;
}

