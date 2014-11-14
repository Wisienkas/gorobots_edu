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
#include "controllers/amosIIcontrol.h"
// joint needed for fixation of the robot in the beginning
#include <ode_robots/joint.h>

// add head file for creating a sphere by Ren ------------
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>
#include <selforg/abstractcontroller.h>
#include <ode_robots/color.h>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <chrono>

using namespace std;
using namespace lpzrobots;
std::vector<lpzrobots::AbstractObstacle*> obst;
//std::vector<lpzrobots::FixedJoint*> fixator;
// add head file for creating a sphere by Ren ------------
bool track = false;

//Parameters for climbing experiments
int climb_height = 2;
unsigned trial_count = 0;
unsigned reflex_count = 0;
unsigned success_count = 0;
unsigned yrange_count = 0;
unsigned stuck_count = 0;
unsigned convergence_count = 1;
unsigned noconv = 0;
unsigned copyfile;

double start_trial_time = 0.0;
double trial_time = 0.0;
double sum_time = 0.0;
double average_time = 0.0;
double start_convergence_time = 0.0;
double convergence_time = 0.0;
double weight_factor = 0.00;
double sum_weights = 0.0;

vector<unsigned> no_delta_time;
vector<unsigned> no_delta_maxtime;

time_t timer;
int start_time = time(&timer);
int end_time;
int elapsed_seconds;

ofstream outFile1;
ofstream outFile2;
ofstream outFile3;

bool climbing_experiment_setup;

class ThisSim : public lpzrobots::Simulation {
public:

	ThisSim() {
		addPaletteFile("colors/UrbanExtraColors.gpl");
		addColorAliasFile("colors/UrbanColorSchema.txt");
		// you can replace color mappings in your own file, see colors/UrbanColorSchema.txt
		// addColorAliasFile("myColorSchema.txt");
		setGroundTexture("Images/whiteground.jpg"); // gets its color from the schema
		//setTitle("Adaptive Obstacle Negotiation");
		//setCaption("D Goldschmidt 2013");
	}

	/**
	 * starting function (executed once at the beginning of the simulation loop)
	 */
	virtual void start(const lpzrobots::OdeHandle& odeHandle, const lpzrobots::OsgHandle& osgHandle,
			lpzrobots::GlobalData& global) {
		// set initial camera position
		setCameraHomePos(lpzrobots::Pos(-0.0114359+climb_height*10.0+0.05, 6.66848, 0.922832), lpzrobots::Pos(178.866, -7.43884, 0));

		//Learning experiments init
		no_delta_time.resize(2);
		no_delta_maxtime.resize(2);

		// set simulation parameters
		global.odeConfig.setParam("controlinterval", 10);
		global.odeConfig.setParam("simstepsize", 0.01);
		global.odeConfig.setParam("noise", 0.02); // 0.02

		// add playgrounds for three different experiments
		lpzrobots::OdeHandle playgroundHandle = odeHandle;
		playgroundHandle.substance = lpzrobots::Substance(1000.0, 0.0, 1000.0, 0.0); //substance for playgrounds (NON-SLIPPERY!!!)
		double steplength = 0.43; //AMOS II body length

		//EXPERIMENTAL SETUP 1: SINGLE OBSTACLE (Adaption to different obstacle altitudes and walking gaits)
		climbing_experiment_setup = 1;
		for(int i=0;i<30;i++){
			double obstacle_height = 0.01+0.01*i;
			double obstacle_distance = 1.5;
			if (climbing_experiment_setup) {
				lpzrobots::Playground* playground = new lpzrobots::Playground(playgroundHandle, osgHandle, osg::Vec3(obstacle_distance, 0.6,
						obstacle_height), 1, false);
				playground->setTexture(0, 0, lpzrobots::TextureDescr("Images/wall_bw.jpg", -0.5, -3));
				playground->setPosition(osg::Vec3(10*i, 0, .0));
				global.obstacles.push_back(playground);
			}
		}


		//EXPERIMENTAL SETUP 2: STAIRS DECREASING IN LENGTH (Finding the minimal step length x which AMOS is able to negotiate)
		bool stair_experiment_setup = 1;
		double base_height = 6; //in cm
		if (stair_experiment_setup) {
			lpzrobots::Playground* stair1 = new lpzrobots::Playground(playgroundHandle, osgHandle, osg::Vec3(2.0, 2.0
					* steplength, 0.01*base_height), 1, false);
			lpzrobots::Playground* stair2 = new lpzrobots::Playground(playgroundHandle, osgHandle, osg::Vec3(2.0 + 2.0
					* steplength, 1.0 * steplength, 0.02*base_height), 1, false);
			lpzrobots::Playground* stair3 = new lpzrobots::Playground(playgroundHandle, osgHandle, osg::Vec3(2.0 + (2.0
					+ 1.0) * steplength, 0.5 * steplength, 0.03*base_height), 1, false);
			lpzrobots::Playground* stair4 = new lpzrobots::Playground(playgroundHandle, osgHandle, osg::Vec3(2.0 + (0.5
					+ 2.0 + 1.0) * steplength, 0.25 * steplength, 0.04*base_height), 1, false);
			lpzrobots::Playground* stair5 = new lpzrobots::Playground(playgroundHandle, osgHandle, osg::Vec3(2.0 + (0.25
					+ 2.0 + 1.0 + 0.5) * steplength, 0.25 * steplength, 0.05*base_height), 1, false);
			lpzrobots::Playground* stair6 = new lpzrobots::Playground(playgroundHandle, osgHandle, osg::Vec3(2.0 + (0.25
					+ 2.0 + 1.0 + 0.5 + 0.25) * steplength, 0.25 * steplength, 0.06*base_height), 1, false);
			lpzrobots::Playground* stair7 = new lpzrobots::Playground(playgroundHandle, osgHandle, osg::Vec3(2.0 + (0.25
					+ 2.0 + 1.0 + 0.5 + 0.5) * steplength, 0.25 * steplength, 0.07*base_height), 1, false);
			lpzrobots::Playground* stair8 = new lpzrobots::Playground(playgroundHandle, osgHandle, osg::Vec3(2.0 + (0.25
					+ 2.0 + 1.0 + 0.5 + 0.75) * steplength, 0.25 * steplength, 0.08*base_height), 1, false);
			lpzrobots::Playground* stair9 = new lpzrobots::Playground(playgroundHandle, osgHandle, osg::Vec3(2.0 + (0.25
					+ 2.0 + 1.0 + 0.5 + 0.75 + 0.5) * steplength, 0.5 * steplength, 0.09*base_height), 1, false);
			lpzrobots::Playground* stair10 = new lpzrobots::Playground(playgroundHandle, osgHandle, osg::Vec3(2.0 + (0.25
					+ 2.0 + 1.0 + 0.5 + 0.75 + 0.5 + 0.75) * steplength, 2.0 * steplength, 0.1*base_height), 1, false);
			stair1->setPosition(osg::Vec3(-10., 0, .0));
			stair2->setPosition(osg::Vec3(-10., 0, .0));
			stair3->setPosition(osg::Vec3(-10., 0, .0));
			stair4->setPosition(osg::Vec3(-10., 0, .0));
			stair5->setPosition(osg::Vec3(-10., 0, .0));
			stair6->setPosition(osg::Vec3(-10., 0, .0));
			stair7->setPosition(osg::Vec3(-10., 0, .0));
			stair8->setPosition(osg::Vec3(-10., 0, .0));
			stair9->setPosition(osg::Vec3(-10., 0, .0));
			stair10->setPosition(osg::Vec3(-10., 0, .0));

			global.obstacles.push_back(stair1);
			global.obstacles.push_back(stair2);
			global.obstacles.push_back(stair3);
			global.obstacles.push_back(stair4);
			global.obstacles.push_back(stair5);
			global.obstacles.push_back(stair6);
			global.obstacles.push_back(stair7);
			global.obstacles.push_back(stair8);
			global.obstacles.push_back(stair9);
			global.obstacles.push_back(stair10);
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
		lpzrobots::AmosIIConf myAmosIIConf = lpzrobots::AmosII::getAmosIIv2Conf(1.0 /*_scale*/, 1 /*_useShoulder*/,
				1 /*_useFoot*/, 1 /*_useBack*/);
		myAmosIIConf.rubberFeet = true;

		//lpzrobots::AmosIIConf myAmosIIConf = lpzrobots::AmosII::getAmosIIv1Conf(1.0 /*_scale*/,1 /*_useShoulder*/,1 /*_useFoot*/,1 /*_useBack*/);
		//myAmosIIConf.rubberFeet = true;

		//myAmosIIConf.legContactSensorIsBinary = true;
		lpzrobots::OdeHandle rodeHandle = odeHandle;
		rodeHandle.substance = lpzrobots::Substance(100.0, 0.0, 50.0, 0.8);

		//------------------- Link the sphere to the Goal Sensor by Ren---------------
		for (unsigned int i = 0; i < obst.size(); i++) {
			myAmosIIConf.GoalSensor_references.push_back(obst.at(i)->getMainPrimitive());
		}
		//------------------- Link the sphere to the Goal Sensor by Ren---------------

		amos
		= new lpzrobots::AmosII(rodeHandle, osgHandle.changeColor(lpzrobots::Color(1, 1, 1)), myAmosIIConf, "AmosII");

		// define the usage of the individual legs
		amos->setLegPosUsage(amos->L0, amos->LEG);
		amos->setLegPosUsage(amos->L1, amos->LEG);
		amos->setLegPosUsage(amos->L2, amos->LEG);
		amos->setLegPosUsage(amos->R0, amos->LEG);
		amos->setLegPosUsage(amos->R1, amos->LEG);
		amos->setLegPosUsage(amos->R2, amos->LEG);

		// put amos a little bit in the air
		if(climbing_experiment_setup)
			amos->place(osg::Matrix::translate(climb_height*10.0+0.05, .0, 0.0) * osg::Matrix::rotate(0.0, -M_PI / 180 * (-5), 1, 0));
		else
			amos->place(osg::Matrix::translate(0.0, .0, 0.0) * osg::Matrix::rotate(0.0, -M_PI / 180 * (-5), 1, 0));

		controller = new AmosIIControl();//TripodGait18DOF();
		// create wiring
		One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise());

		// create agent and init it with controller, robot and wiring
		lpzrobots::OdeAgent* agent = new lpzrobots::OdeAgent(global);
		agent->init(controller, amos, wiring);

		// Possibility to add tracking for robot
		if (track)
			agent->setTrackOptions(TrackRobot(true, false, false, true, "", 60)); // Display trace
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

		std::cout << "\n\n" << "################################\n" << "#   Press x to free amosII!    #\n"
				<< "################################\n" << "\n\n" << std::endl;
	}


	/**************************Reset Function***************************************************************/
	virtual bool restart(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
	{
		// set camera position
		setCameraHomePos(lpzrobots::Pos(-0.0114359+climb_height*10.0+0.05, 6.66848, 0.922832), lpzrobots::Pos(178.866, -7.43884, 0));

		// inform global variable over everything that happened:
		global.configs.erase(global.configs.begin());

		delete amos;
		//delete (agent);
		global.agents.pop_back();

		//Add AMOSII robot
		// Add amosII robot
		lpzrobots::AmosIIConf myAmosIIConf = lpzrobots::AmosII::getDefaultConf(1.0 /*_scale*/, 1 /*_useShoulder*/,
				1 /*_useFoot*/, 1 /*_useBack*/);
		myAmosIIConf.rubberFeet = true;

		//lpzrobots::AmosIIConf myAmosIIConf = lpzrobots::AmosII::getAmosIIv1Conf(1.0 /*_scale*/,1 /*_useShoulder*/,1 /*_useFoot*/,1 /*_useBack*/);
		//myAmosIIConf.rubberFeet = true;

		//myAmosIIConf.legContactSensorIsBinary = true;
		lpzrobots::OdeHandle rodeHandle = odeHandle;
		rodeHandle.substance = lpzrobots::Substance(3.0, 0.0, 50.0, 0.8);

		//------------------- Link the sphere to the Goal Sensor by Ren---------------
		for (unsigned int i = 0; i < obst.size(); i++) {
			myAmosIIConf.GoalSensor_references.push_back(obst.at(i)->getMainPrimitive());
		}
		//------------------- Link the sphere to the Goal Sensor by Ren---------------

		amos
		= new lpzrobots::AmosII(rodeHandle, osgHandle.changeColor(lpzrobots::Color(1, 1, 1)), myAmosIIConf, "AmosII");

		// define the usage of the individual legs
		amos->setLegPosUsage(amos->L0, amos->LEG);
		amos->setLegPosUsage(amos->L1, amos->LEG);
		amos->setLegPosUsage(amos->L2, amos->LEG);
		amos->setLegPosUsage(amos->R0, amos->LEG);
		amos->setLegPosUsage(amos->R1, amos->LEG);
		amos->setLegPosUsage(amos->R2, amos->LEG);

		// put amos a little bit in the air
		amos->place(osg::Matrix::translate(climb_height*10.0+0.05, .0, 0.0) * osg::Matrix::rotate(0.0, -M_PI / 180 * (-5), 1, 0));

		//Create wiring
		One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise());

		// create agent and init it with controller, robot and wiring
		lpzrobots::OdeAgent* agent = new OdeAgent(global);
		agent->init(controller, amos, wiring);

		// Possibility to add tracking for robot
		if (track) agent->setTrackOptions(TrackRobot(true, false, false, true, "", 60)); // Display trace

		// inform global variable over everything that happened:
		global.configs.push_back(amos);
		global.agents.push_back(agent);
		global.configs.push_back(controller);

		return true;
	}

	/**
	 * add own key handling stuff here, just insert some case values
	 */

	virtual bool command(const lpzrobots::OdeHandle&, const lpzrobots::OsgHandle&, lpzrobots::GlobalData& globalData,
			int key, bool down) {
		if (down) { // only when key is pressed, not when released
			switch (char(key)) {
			case 's':
				climb_height = -1;
				climbing_experiment_setup = false;
				((AmosIIControl*) controller)->control_adaptiveclimbing.climb_height = -1;
				simulation_time_reached=true;
				std::cout << "STAIRS" << endl;
				break;

			case 'k':
				((AmosIIControl*) controller)->preprocessing_learning.weight_reset =true;
				((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input += 0.01;
				((AmosIIControl*) controller)->preprocessing_learning.outFilenpp1 <<  ((AmosIIControl*) controller)->preprocessing_learning.c << ' ' <<  ((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25) << ' ' <<  ((AmosIIControl*) controller)->preprocessing_learning.rho1.at(26) << endl;
				simulation_time_reached=true;
				std::cout << "SUCCESSFUL" << endl;
				break;

			case 'n':
				climb_height++;
				((AmosIIControl*) controller)->control_adaptiveclimbing.climb_height++;
				simulation_time_reached=true;
				std::cout << "Starting trial for h = " << ((AmosIIControl*) controller)->control_adaptiveclimbing.climb_height+1 << " cm" << endl;
				break;

			case 'm':
				climb_height--;
				((AmosIIControl*) controller)->control_adaptiveclimbing.climb_height--;
				simulation_time_reached=true;
				if(climb_height<-1){
					climb_height = -1;
					((AmosIIControl*) controller)->control_adaptiveclimbing.climb_height = -1;
				}
				if(climb_height>0){
					std::cout << "Starting trial for h = " << ((AmosIIControl*) controller)->control_adaptiveclimbing.climb_height+1 << " cm" << endl;
					climbing_experiment_setup = true;
				}
				else{
					std::cout << "Starting trial for stairs." << endl;
					climbing_experiment_setup = false;
				}
				break;

			case 't':
				((AmosIIControl*) controller)->control_adaptiveclimbing.trial++;
				break;

			case 'x':
				if (robotfixator) {
					std::cout << "dropping robot" << std::endl;
					delete robotfixator;
					robotfixator = NULL;
				}
				break;

			case 'r':
				simulation_time_reached=true;
				std::cout << "RESET" << endl;
				break;

			case 'b':
				if (((AmosIIControl*) controller)->control_adaptiveclimbing.switchon_backbonejoint) {
					((AmosIIControl*) controller)->control_adaptiveclimbing.switchon_backbonejoint = false;
					std::cout << "BJC is OFF" << endl;
				} else {
					((AmosIIControl*) controller)->control_adaptiveclimbing.switchon_backbonejoint = true;
					((AmosIIControl*) controller)->y.at(BJ_m) = 0.0;
					std::cout << "BJC is ON" << endl;
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

	virtual void resetSignals(){
		for(unsigned i = 25; i < 27; i++){
			((AmosIIControl*) controller)->control_adaptiveclimbing.old_position = climb_height*10.0;
			//        ((AmosIIControl*) controller)->control_adaptiveclimbing.average_robot_speed = 0.01;
			((AmosIIControl*) controller)->preprocessing_learning.ir_reflex_reset.at(i) = 0.0;
			((AmosIIControl*) controller)->preprocessing_learning.ir_reflex_output.at(i) = 0.0;
			((AmosIIControl*) controller)->preprocessing_learning.ir_predic_output.at(i) = 0.0;
			((AmosIIControl*) controller)->preprocessing_learning.irlearn_output_prolong.at(i) = 0.0;
			((AmosIIControl*) controller)->preprocessing_learning.preprosensor.at(i) = 0.0;
			((AmosIIControl*) controller)->preprocessing_learning.us_delayline.at(i)->Reset();
		}
		for(int i =0;i<5;i++){
			((AmosIIControl*) controller)->control_adaptiveclimbing.bj_output.at(i) = 0.0;
		}
	}

	virtual void resetWeights(){
		((AmosIIControl*) controller)->preprocessing_learning.countR=0;
		((AmosIIControl*) controller)->preprocessing_learning.countL=0;
		((AmosIIControl*) controller)->preprocessing_learning.countRmax=0;
		((AmosIIControl*) controller)->preprocessing_learning.countLmax=0;
		success_count = 0;
		trial_count = 0;
		((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25) = 0.0;
		((AmosIIControl*) controller)->preprocessing_learning.rho1.at(26) = 0.0;
	}

	virtual void assignWeights(){
		switch(climb_height){
		case -1:
			((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.1;
			break;

		case 0:
			((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.0;
			break;

		case 1:
			((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.0;
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.09)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.02;
			break;

		case 2:
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.03)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.0013751208;
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.06)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.02380602;
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.09)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.04733075;
			break;

		case 3:
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.03)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.0127828044;
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.06)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.06056026;
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.09)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.11591125;
			break;

		case 4:
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.03)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.0353061778;
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.06)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.1196138;
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.09)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.20249225;
			break;

		case 5:
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.03)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.0552709222;
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.06)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.1584148;
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.09)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.31070975;
			break;

		case 6:
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.03)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.0759043111;
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.06)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.1908349;
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.09)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.35421075;
			break;

		case 7:
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.03)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.0982798556;
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.06)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.2090401;
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.09)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.394640375;
			break;

		case 8:
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.03)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.118333;
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.06)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.2625295;
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.09)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.434494375;
			break;

		case 9:
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.03)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.1360623333;
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.06)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.3162431667;
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.09)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.463625375;
			break;

		case 10:
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.03)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.15528525;
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.06)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.3162431667;
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.09)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.5190135;
			break;

		case 11:
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.03)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.179403875;
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.06)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.3162431667;
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.09)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.954150625;
			break;

		case 12:
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.03)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.1905715;
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.06)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.3162431667;
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.09)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=1.7226175;
			break;

		case 13:
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.03)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.271058;
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.06)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.3162431667;
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.09)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=2.2244225;
			break;

		case 14:
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.03)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.271058;
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.06)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=0.3162431667;
			if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input == 0.09)
				((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25)=2.2244225;
			break;
		}
	}

	virtual void addCallback(lpzrobots::GlobalData& globalData, bool draw, bool pause, bool control) {
		// for demonstration: set simsteps for one cycle to 60.000/currentCycle (10min/currentCycle)
    		  // if simulation_time_reached is set to true, the simulation cycle is finished

    		  if(globalData.sim_step >= 0 && globalData.sim_step % 10 == 0 /*&& climbing_experiment_setup*/){

    			  trial_time = ((AmosIIControl*) controller)->control_adaptiveclimbing.global_count * 0.1 - start_trial_time;
    			  ((AmosIIControl*) controller)->control_adaptiveclimbing.relative_position = ((AmosIIControl*) controller)->x.at(BX_pos) - climb_height*10.0;

    			  //Reflex -> Reset
    			  if ( ((AmosIIControl*) controller)->preprocessing_learning.ir_reflex_reset.at(25) > 0.98 && ((AmosIIControl*) controller)->preprocessing_learning.ir_reflex_reset.at(26) > 0.98){
    				  reflex_count++;
    				  trial_count++;
    				  printf("%u ", success_count);
    				  printf("%u ", trial_count);
    				  printf("%f\n", trial_time);

    				  start_trial_time = ((AmosIIControl*) controller)->control_adaptiveclimbing.global_count*0.1;

    				  resetSignals();
    				  simulation_time_reached=true;
    			  }

    			  //Success
    			  if( ((AmosIIControl*) controller)->x.at(BX_pos) > (1.0 + climb_height*10) && climbing_experiment_setup){
    				  success_count++;
    				  trial_count++;
    				  printf("%u ", success_count);
    				  printf("%u ", trial_count);
    				  printf("%f\n", trial_time);

    				  sum_time += trial_time;
    				  average_time = sum_time / success_count;
    				  start_trial_time = ((AmosIIControl*) controller)->control_adaptiveclimbing.global_count*0.1;

    				  resetSignals();
    				  simulation_time_reached=true;
    			  }
    			  //  && globalData.sim_step > 100

    			  //Fail: Out of Y-range
    			  if( ((AmosIIControl*) controller)->x.at(BY_pos) > 10.0 || ((AmosIIControl*) controller)->x.at(BY_pos) < -10.0){
    				  yrange_count++;
    				  trial_count++;
    				  printf("%u ", success_count);
    				  printf("%u ", trial_count);
    				  printf("%f\n", trial_time);

    				  start_trial_time = ((AmosIIControl*) controller)->control_adaptiveclimbing.global_count*0.1;
    				  /*if(((AmosIIControl*) controller)->preprocessing_learning.convergence)*/

    				  resetSignals();
    				  simulation_time_reached=true;
    			  }

    			  //Fail: Out of time
    			  if( trial_time > 600.0){
    				  stuck_count++;
    				  trial_count++;
    				  printf("%u ", success_count);
    				  printf("%u ", trial_count);
    				  printf("%f\n", trial_time);

    				  start_trial_time = ((AmosIIControl*) controller)->control_adaptiveclimbing.global_count*0.1;
    				  /*if(((AmosIIControl*) controller)->preprocessing_learning.convergence)*/

    				  resetSignals();
    				  simulation_time_reached=true;
    			  }


    			  //----------------------------Only Learning Mode-----------------------------------------

    			  if( ((AmosIIControl*) controller)->preprocessing_learning.switchon_IRlearning ){

    				  if(((AmosIIControl*) controller)->preprocessing_learning.countR > 6000){
    					  ((AmosIIControl*) controller)->preprocessing_learning.ir_learnrate.at(25) = 0.0;
    				  }
    				  if(((AmosIIControl*) controller)->preprocessing_learning.countL > 6000){
    					  ((AmosIIControl*) controller)->preprocessing_learning.ir_learnrate.at(26) = 0.0;
    				  }

    				  convergence_time = ((AmosIIControl*) controller)->control_adaptiveclimbing.global_count*0.1 - start_convergence_time;


    				  if( (((AmosIIControl*) controller)->preprocessing_learning.ir_learnrate.at(25) == 0.0
    						  && ((AmosIIControl*) controller)->preprocessing_learning.ir_learnrate.at(26) == 0.0)
    						  || convergence_time > 96000){

    					  end_time = time(&timer);
    					  elapsed_seconds = end_time - start_time;
    					  struct tm * timeinfo = localtime (&timer);
    					  printf ("Finished computation at %s", asctime(timeinfo));
    					  printf("Elapsed time: %u \n", elapsed_seconds);
    					  start_time = time(&timer);

    					  if(convergence_time > 96000){
    						  noconv++;
    					  }

    					  if(noconv > 1){
    						  printf("Successful trial for c = %f \n", ((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input);
    						  if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input==0.18){
    							  ((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input = 0.01;
    						  }
    						  if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input==0.19){
    							  ((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input = -0.01;
    						  }
    						  if(((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input==0.17){
    							  ((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input = 0.00;
    						  }
    						  ((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input+=0.03;
    						  climb_height = 0;
    					  }


    					  sum_weights += ((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25);
    					  sum_weights += ((AmosIIControl*) controller)->preprocessing_learning.rho1.at(26);

    					  //Write File
    					  outFile1
    					  << convergence_count << ' '
    					  << climb_height + 1 << ' '
    					  << ((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input << ' '
    					  << ((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25) << ' '
    					  << ((AmosIIControl*) controller)->preprocessing_learning.rho1.at(26) << ' '
    					  << convergence_time << ' '
    					  //                << 0.1*((AmosIIControl*) controller)->preprocessing_learning.countRmax  << ' '
    					  //                << 0.1*((AmosIIControl*) controller)->preprocessing_learning.countLmax << ' '
    					  //                << 0.1*((AmosIIControl*) controller)->preprocessing_learning.countRnet  << ' '
    					  //                << 0.1*((AmosIIControl*) controller)->preprocessing_learning.countLnet << ' '
    					  << success_count << " "
    					  << trial_count << " "
    					  <<endl;
    					  //Copy File
    					  //            copyfile = system ("cp Neuralpreprocessing.txt ../../../../../../dropdego/Dropbox/NSCFS13/JOURNAL/figures/exp_gh.txt");

    					  if(convergence_count==5 && noconv == 0){
    						  //All obstacles done -> next gait
    						  outFile3
    						  << climb_height + 1 << ' '
    						  << ((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input << ' '
    						  << sum_weights/(convergence_count*2.0) << endl;
    						  sum_weights = 0.0;

    						  climb_height++;
    						  convergence_count=0;
    					  }
    					  //Height message + Next Obstacle
    					  printf("(c = %f) ", ((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input);
    					  printf("Successful trial for h = %u cm \n", climb_height+1);
    					  printf("Learned weight = %f (R)", ((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25) );
    					  printf("\t %f (L) \n", ((AmosIIControl*) controller)->preprocessing_learning.rho1.at(26));

    					  printf("Starting trial for h = %u cm \n", climb_height+1);
    					  start_convergence_time = ((AmosIIControl*) controller)->control_adaptiveclimbing.global_count*0.1;
    					  printf("Starting trial at %f s \n", start_convergence_time);
    					  if(convergence_count < 5)
    						  convergence_count++;
    					  /*            ((AmosIIControl*) controller)->preprocessing_learning.convergence = false;*/

    					  resetSignals();
    					  resetWeights();
    					  ((AmosIIControl*) controller)->preprocessing_learning.ir_learnrate.at(25) = 0.01;
    					  ((AmosIIControl*) controller)->preprocessing_learning.ir_learnrate.at(26) = 0.01;
    					  simulation_time_reached=true;
    				  }

    				  if(((AmosIIControl*) controller)->preprocessing_learning.drho1.at(25)!=0.0 /*&& !((AmosIIControl*) controller)->preprocessing_learning.convergence*/){
    					  if(((AmosIIControl*) controller)->preprocessing_learning.countRmax<((AmosIIControl*) controller)->preprocessing_learning.countR){
    						  ((AmosIIControl*) controller)->preprocessing_learning.countRmax=((AmosIIControl*) controller)->preprocessing_learning.countR;
    					  }
    					  ((AmosIIControl*) controller)->preprocessing_learning.countR=0;
    				  }
    				  if(((AmosIIControl*) controller)->preprocessing_learning.drho1.at(26)!=0.0 /*&& !((AmosIIControl*) controller)->preprocessing_learning.convergence*/){
    					  if(((AmosIIControl*) controller)->preprocessing_learning.countLmax<((AmosIIControl*) controller)->preprocessing_learning.countL){
    						  ((AmosIIControl*) controller)->preprocessing_learning.countLmax=((AmosIIControl*) controller)->preprocessing_learning.countL;
    					  }
    					  ((AmosIIControl*) controller)->preprocessing_learning.countL=0;
    				  }

    				  if(convergence_count == 10 && noconv != 0){
    					  noconv--;
    				  }

    			  }

    			  //----------------------------Only Performance Mode-----------------------------------------

    			  if(!((AmosIIControl*) controller)->preprocessing_learning.switchon_IRlearning){

    				  assignWeights();
    				  ((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25) += weight_factor;
    				  ((AmosIIControl*) controller)->preprocessing_learning.rho1.at(26)=((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25);

    				  if(trial_count == 30){

    					  printf("%f ", ((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input);
    					  printf("%u ", climb_height+1);
    					  printf("%f ", ((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25));
    					  printf("%u ", success_count);
    					  printf("%u ", trial_count);
    					  printf("%f\n", average_time);

    					  //Write File
    					  outFile2
    					  << ((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input << " "
    					  << climb_height + 1 << " "
    					  << ((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25) << " "
    					  << success_count << " "
    					  << trial_count << " "
    					  << reflex_count << " "
    					  << yrange_count << " "
    					  << stuck_count << " "
    					  << average_time << endl;
    					  //Copy File
    					  //            copyfile = system ("cp Neuralpreprocessing.txt ../../../../../../dropdego/Dropbox/NSCFS13/JOURNAL/figures/exp_gh.txt");


    					  weight_factor+=0.1;

    					  //All obstacles done -> next gait
    					  if(weight_factor > 1.0){
    						  printf("Successful trial for c = %f\n", ((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input);
    						  climb_height++;
    						  if(climb_height == 14){
    							  climb_height = 2;
    							  ((AmosIIControl*) controller)->control_adaptiveclimbing.nlc->Control_input+=0.03;
    						  }
    						  weight_factor = 0.0;
    					  }

    					  success_count = 0;
    					  trial_count = 0;
    					  reflex_count = 0;
    					  stuck_count = 0;
    					  yrange_count = 0;
    					  sum_time = 0.0;
    					  resetSignals();
    					  simulation_time_reached=true;
    				  } //----------------END Performance done
    			  } //----------------END Performance only

    		  } //----------------END GlobalCount%10
	} //----------------END addCallback


protected:
	lpzrobots::Joint* robotfixator;
	AbstractController* controller;
	lpzrobots::AmosII* amos;

};

int main(int argc, char **argv) {
	outFile1.open("Learned_Weights_FINAL.txt");
	outFile2.open("Performance_FINAL.txt");
	outFile3.open("Mean_Weights_FINAL.txt");
	ThisSim sim;
	sim.setGroundTexture("Images/greenground.rgb");
	return sim.run(argc, argv) ? 0 : 1;
}

