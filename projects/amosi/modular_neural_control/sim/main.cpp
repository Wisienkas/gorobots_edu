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
// the controller
#include "controllers/amosii/modular_neural_control/amosIIcontrol.h"
#include "StoyanController.h"
//#include "projects/practices_solutions/amosii/examplecontroller_d_buchwald.h"
// joint needed for fixation of the robot in the beginning
#include <ode_robots/joint.h>

// add head file for creating a sphere by Ren ------------
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>
#include <selforg/abstractcontroller.h>
#include <ode_robots/color.h>
#include <iostream>

//ROS Stuff
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <sstream>

bool newInput;
int rosInput;
int threshold = 0;

using namespace std;
std::vector<lpzrobots::AbstractObstacle*> obst;
//std::vector<lpzrobots::FixedJoint*> fixator;
// add head file for creating a sphere by Ren ------------

/// ------------------------------ ROS SUBSCRIBER -------------------------------------//

ros::Publisher chatter_pub;
// Callback Function:

/// ------------------------------ ROS SUBSCRIBER -------------------------------------//
class ThisSim: public lpzrobots::Simulation {

public:

	ThisSim() {
		addPaletteFile("colors/UrbanExtraColors.gpl");
		addColorAliasFile("colors/UrbanColorSchema.txt");
		// you can replace color mappings in your own file, see colors/UrbanColorSchema.txt
		// addColorAliasFile("myColorSchema.txt");
		setGroundTexture("Images/whiteground.jpg"); // gets its color from the schema
		setTitle("Adaptive Climbing Behavior of Walking Machines");

		//setCaption("right aligned text");
	}

	/**
	 * starting function (executed once at the beginning of the simulation loop)
	 */
	virtual void start(const lpzrobots::OdeHandle& odeHandle,
			const lpzrobots::OsgHandle& osgHandle,
			lpzrobots::GlobalData& global) {




		// set initial camera position
		setCameraHomePos(lpzrobots::Pos(-0.0114359, 6.66848, 0.922832),
				lpzrobots::Pos(178.866, -7.43884, 0));

		// set simulation parameters
		global.odeConfig.setParam("controlinterval", 10);
		global.odeConfig.setParam("simstepsize", 0.01);
		global.odeConfig.setParam("noise", 0.02); // 0.02

		// add playgrounds for three different experiments
		lpzrobots::OdeHandle playgroundHandle = odeHandle;
		playgroundHandle.substance = lpzrobots::Substance(100.0, 0.0, 50.0,
				0.0); //substance for playgrounds (NON-SLIPPERY!!!)
		double steplength = 0.43; //AMOS II body length

		//EXPERIMENTAL SETUP 1: SINGLE OBSTACLE (Adaption to different obstacle altitudes and walking gaits)
		bool climbing_experiment_setup = false;
		if (climbing_experiment_setup) {
			lpzrobots::Playground* playground = new lpzrobots::Playground(
					playgroundHandle, osgHandle, osg::Vec3(2, 0.6, 0.07), 1,
					false);
			playground->setTexture(0, 0,
					lpzrobots::TextureDescr("Images/wall_bw.jpg", -0.5, -3));
			playground->setPosition(osg::Vec3(0, 0, .0));
			global.obstacles.push_back(playground);
		}

		//EXPERIMENTAL SETUP 2: STAIRS DECREASING IN LENGTH (Finding the minimal step length x which AMOS is able to negotiate)
		bool stair_experiment_setup = false;
		if (stair_experiment_setup) {
			lpzrobots::Playground* stair1 = new lpzrobots::Playground(
					playgroundHandle, osgHandle,
					osg::Vec3(2.0, 4.0 * steplength, 0.08), 1, false);
			lpzrobots::Playground* stair2 = new lpzrobots::Playground(
					playgroundHandle, osgHandle,
					osg::Vec3(2.0 + 4.0 * steplength, 2.0 * steplength, 0.16),
					1, false);
			lpzrobots::Playground* stair3 = new lpzrobots::Playground(
					playgroundHandle, osgHandle,
					osg::Vec3(2.0 + (4.0 + 2.0) * steplength, 1.0 * steplength,
							0.24), 1, false);
			lpzrobots::Playground* stair4 = new lpzrobots::Playground(
					playgroundHandle, osgHandle,
					osg::Vec3(2.0 + (4.0 + 2.0 + 1.0) * steplength,
							0.5 * steplength, 0.32), 1, false);
			lpzrobots::Playground* stair5 = new lpzrobots::Playground(
					playgroundHandle, osgHandle,
					osg::Vec3(2.0 + (4.0 + 2.0 + 1.0 + 0.5) * steplength, 1.0,
							0.40), 1, false);
			stair1->setPosition(osg::Vec3(0, 0, .0));
			stair2->setPosition(osg::Vec3(0, 0, .0));
			stair3->setPosition(osg::Vec3(0, 0, .0));
			stair4->setPosition(osg::Vec3(0, 0, .0));
			stair5->setPosition(osg::Vec3(0, 0, .0));
			global.obstacles.push_back(stair1);
			global.obstacles.push_back(stair2);
			global.obstacles.push_back(stair3);
			global.obstacles.push_back(stair4);
			global.obstacles.push_back(stair5);
		}

		//----------create a sphere as the target by Ren-----------------------------
		//the first sphere
		//    lpzrobots::PassiveSphere* s1 = new lpzrobots::PassiveSphere(odeHandle, osgHandle, 0.1);
		//    s1->setPosition(osg::Vec3(3.0, 0.0, 0.1));
		//    s1->setTexture("Images/dusty.rgb");
		//    s1->setColor(lpzrobots::Color(1,0,0));
		//    obst.push_back(s1);
		//    global.obstacles.push_back(s1);
		//    lpzrobots::FixedJoint* fixator1 = new  lpzrobots::FixedJoint(s1->getMainPrimitive(), global.environment);
		//    fixator1->init(odeHandle, osgHandle);

		//the second sphere
		//    lpzrobots::PassiveSphere* s2 = new lpzrobots::PassiveSphere(odeHandle, osgHandle, 0.1);
		//    s2->setPosition(osg::Vec3(0.0, 3.0, 0.1));
		//    s2->setTexture("Images/dusty.rgb");
		//    s2->setColor(lpzrobots::Color(0,1,0));
		//    obst.push_back(s2);
		//    global.obstacles.push_back(s2);
		//    lpzrobots::FixedJoint* fixator2 = new  lpzrobots::FixedJoint(s2->getMainPrimitive(), global.environment);
		//    fixator2->init(odeHandle, osgHandle);

		//the third sphere
		//    lpzrobots::PassiveSphere* s3 = new lpzrobots::PassiveSphere(odeHandle, osgHandle, 0.1);
		//    s3->setPosition(osg::Vec3(0.0, -3.0, 0.1));
		//    s3->setTexture("Images/dusty.rgb");
		//    s3->setColor(lpzrobots::Color(0,0,1));
		//    obst.push_back(s3);
		//    global.obstacles.push_back(s3);
		//    lpzrobots::FixedJoint* fixator3 = new  lpzrobots::FixedJoint(s3->getMainPrimitive(), global.environment);
		//    fixator3->init(odeHandle, osgHandle);

		//----------create a sphere as the target by Ren-----------------------------

		/*******  End Modify Environment *******/

		// Add amosII robot
		lpzrobots::AmosIIConf myAmosIIConf = lpzrobots::AmosII::getDefaultConf(
				1.0 /*_scale*/, 1 /*_useShoulder*/, 1 /*_useFoot*/,
				1 /*_useBack*/);
		myAmosIIConf.rubberFeet = true;

		//lpzrobots::AmosIIConf myAmosIIConf = lpzrobots::AmosII::getAmosIIv1Conf(1.0 /*_scale*/,1 /*_useShoulder*/,1 /*_useFoot*/,1 /*_useBack*/);
		//myAmosIIConf.rubberFeet = true;

		//myAmosIIConf.legContactSensorIsBinary = true;
		lpzrobots::OdeHandle rodeHandle = odeHandle;
		rodeHandle.substance = lpzrobots::Substance(3.0, 0.0, 50.0, 0.8);

		//------------------- Link the sphere to the Goal Sensor by Ren---------------
		for (unsigned int i = 0; i < obst.size(); i++) {
			myAmosIIConf.GoalSensor_references.push_back(
					obst.at(i)->getMainPrimitive());
		}
		//------------------- Link the sphere to the Goal Sensor by Ren---------------

		amos = new lpzrobots::AmosII(rodeHandle,
				osgHandle.changeColor(lpzrobots::Color(1, 1, 1)), myAmosIIConf,
				"AmosII");

		// define the usage of the individual legs
		amos->setLegPosUsage(amos->L0, amos->LEG);
		amos->setLegPosUsage(amos->L1, amos->LEG);
		amos->setLegPosUsage(amos->L2, amos->LEG);
		amos->setLegPosUsage(amos->R0, amos->LEG);
		amos->setLegPosUsage(amos->R1, amos->LEG);
		amos->setLegPosUsage(amos->R2, amos->LEG);

		// put amos a little bit in the air
		amos->place(
				osg::Matrix::translate(.0, .0, 0.0)
						* osg::Matrix::rotate(M_PI / 180 * (-5), 0, 0, 1));

		//controller = new AmosIIControl();//TripodGait18DOF();

		controller = new StoyanController();

		// create wiring
		One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise());

		// create agent and init it with controller, robot and wiring
		lpzrobots::OdeAgent* agent = new lpzrobots::OdeAgent(global);
		agent->init(controller, amos, wiring);

		// Possibility to add tracking for robot
		bool track = true;
		if (track)
			agent->setTrackOptions(
					TrackRobot(true, false, false, true, "", 60)); // Display trace
		//if(track) agent->setTrackOptions(TrackRobot(false,false,false, false, ""));

		// create a fixed joint to hold the robot in the air at the beginning
		//    robotfixator = new lpzrobots::FixedJoint(
		//        amos->getMainPrimitive(),
		//        global.environment);
		//    robotfixator->init(odeHandle, osgHandle, false);

		// inform global variable over everything that happened:
		global.configs.push_back(amos);
		global.agents.push_back(agent);
		global.configs.push_back(controller);

		std::cout << "\n\n" << "################################\n"
				<< "#   Press x to free amosII!    #\n"
				<< "################################\n" << "\n\n" << std::endl;
	}

	/**
	 * add own key handling stuff here, just insert some case values
	 */

	virtual bool command(const lpzrobots::OdeHandle&,
			const lpzrobots::OsgHandle&, lpzrobots::GlobalData& globalData,
			int key, bool down) {

		if (down) { // only when key is pressed, not when released
			switch (char(key)) {
			case 'l':
				((StoyanController*) controller)->UL = -1;
				((StoyanController*) controller)->UR = +1;
				break;
			case 'x':
				if (robotfixator) {
					std::cout << "dropping robot" << std::endl;
					delete robotfixator;
					robotfixator = NULL;
				}
				break;
			case 'r':
				//amos->place(osg::Matrix::translate(.0, .0, 0.0) * osg::Matrix::rotate(0.0, -M_PI / 180 * (-5), 1, 0));
				//((AmosIIControl*) controller)->preprocessing_learning.switchon_IRlearning = false;
				simulation_time_reached = true;
				std::cout << "RESET" << endl;
				break;
			case 'b':
				if (((AmosIIControl*) controller)->control_adaptiveclimbing.at(
						0)->switchon_backbonejoint) {
					((AmosIIControl*) controller)->control_adaptiveclimbing.at(
							0)->switchon_backbonejoint = false;
					std::cout << "BJC is OFF" << endl;
				} else {
					((AmosIIControl*) controller)->control_adaptiveclimbing.at(
							0)->switchon_backbonejoint = true;
					((AmosIIControl*) controller)->y.at(BJ_m) = 0.0;
					std::cout << "BJC is ON" << endl;
				}
				break;
			case 'a':
				if (((AmosIIControl*) controller)->control_adaptiveclimbing.at(
						0)->switchon_obstacle) {
					((AmosIIControl*) controller)->control_adaptiveclimbing.at(
							0)->switchon_obstacle = false;
					std::cout << "OA is OFF" << endl;
				} else {
					((AmosIIControl*) controller)->control_adaptiveclimbing.at(
							0)->switchon_obstacle = true;
					((AmosIIControl*) controller)->control_adaptiveclimbing.at(
							0)->switchon_reflexes = false;
					((AmosIIControl*) controller)->control_adaptiveclimbing.at(
							0)->switchon_irreflexes = false;
					((AmosIIControl*) controller)->control_adaptiveclimbing.at(
							0)->switchon_purefootsignal = false;
					std::cout << "OA is ON" << endl;
				}
				break;

			case 'e':
				if (((AmosIIControl*) controller)->control_adaptiveclimbing.at(
						0)->switchon_allreflexactions) {
					((AmosIIControl*) controller)->control_adaptiveclimbing.at(
							0)->switchon_allreflexactions = false;
					((AmosIIControl*) controller)->control_adaptiveclimbing.at(
							0)->switchon_reflexes = false;
					((AmosIIControl*) controller)->control_adaptiveclimbing.at(
							0)->switchon_irreflexes = false;
					((AmosIIControl*) controller)->control_adaptiveclimbing.at(
							0)->switchon_purefootsignal = false;
					std::cout << "Reflex is OFF" << endl;
				} else {
					((AmosIIControl*) controller)->control_adaptiveclimbing.at(
							0)->switchon_allreflexactions = true;
					((AmosIIControl*) controller)->control_adaptiveclimbing.at(
							0)->switchon_reflexes = true;
					((AmosIIControl*) controller)->control_adaptiveclimbing.at(
							0)->switchon_irreflexes = true;
					((AmosIIControl*) controller)->control_adaptiveclimbing.at(
							0)->switchon_purefootsignal = true;
					std::cout << "Reflex is ON" << endl;
				}
				break;

			default:
				return false;

				break;
			}
		}

		return false;
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////
	//   optional additional callback function which is called every simulation step.
	//   Called between physical simulation step and drawing.
	//   @param draw indicates that objects are drawn in this timestep
	//   @param pause always false (only called of simulation is running)
	//   @param control indicates that robots have been controlled this timestep
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	virtual void addCallback(lpzrobots::GlobalData& globalData, bool draw,
			bool pause, bool control) {
		ros::spinOnce();

		if (newInput)
				{
					if (rosInput>threshold)
					{
						{
							((StoyanController*)controller)->UL = -1;
							((StoyanController*)controller)->UR = +1;
						}
					}
					else
					{
						((StoyanController*)controller)->UL = +1;
						((StoyanController*)controller)->UR = +1;
					}
					newInput = false;
				}
		// for demonstration: set simsteps for one cycle to 60.000/currentCycle (10min/currentCycle)
		// if simulation_time_reached is set to true, the simulation cycle is finished

		//----------------------------Reset Function-----------------------------------------
		//      if (globalData.sim_step >= 6000) {
		//        if (robotfixator) {
		//          std::cout << "dropping robot" << std::endl;
		//          delete robotfixator;
		//          robotfixator = NULL;
		//        }
		//      }
//		if (globalData.sim_step >= 0) //a small delay to make sure the robot will not restart at beginning!
//		{
//			if (((AmosIIControl*) controller)->preprocessing_learning.switchon_IRlearning
//					&& ((AmosIIControl*) controller)->x.at(25) > 0.9 && ((AmosIIControl*) controller)->x.at(26) > 0.9) {
//				((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->bj_output.at(0) = 0.0;
//				((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->bj_output.at(5) = 0.0;
//				((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->m.at(BJ_m) = 0.0;
//				((AmosIIControl*) controller)->control_adaptiveclimbing.at(0)->switchon_backbonejoint = false;
//				((AmosIIControl*) controller)->y.at(BJ_m) = 0.0;
//				amos->place(osg::Matrix::translate(.0, .0, 0.0) * osg::Matrix::rotate(0.0, -M_PI / 180 * (-5), 1, 0));
//			}
//
//		}
		//-----------------------------------------------------------------------------------
	}

protected:
	lpzrobots::Joint* robotfixator;
	AbstractController* controller;
	lpzrobots::AmosII* amos;

};

void RosCallback(const std_msgs::Int32::ConstPtr& msg) {
	ROS_INFO_STREAM(msg->data);
	chatter_pub.publish(msg);
	rosInput=msg->data;
	newInput=true;

}

int main(int argc, char **argv) {
	//Init ROS
	ros::init(argc, argv, "amosI_listener");
	//Iint the Node
	ros::NodeHandle n;
	//Init Subscriber
	ros::Subscriber sub = n.subscribe("bluetooth", 1000, RosCallback);
	chatter_pub = n.advertise<std_msgs::Int32>("amospub", 1000);
	//End of ROS stuff

	ThisSim sim;
	sim.setGroundTexture("Images/greenground.rgb");

	return sim.run(argc, argv) ? 0 : 1;

}

