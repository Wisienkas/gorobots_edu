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
#include <ode_robots/complexplayground.h>
#include <ode_robots/terrainground.h>
// simple wiring
#include <selforg/one2onewiring.h>
// the robot
#include <ode_robots/amosII.h>
// the controller
#include "controllers/amosii/integrated_neural_control/amosIIcontrol.h"
// joint needed for fixation of the robot in the beginning
#include <ode_robots/joint.h>

// add head file for creating a sphere by Ren ------------
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>
#include <selforg/abstractcontroller.h>
#include <ode_robots/color.h>
#include <iostream>
#include <fstream>
using namespace std;
// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
vector<lpzrobots::AbstractObstacle*> obst;
vector<lpzrobots::AbstractObstacle*> goals;
ofstream amos_pos;

/// Simulation parameters
int total_tstep            = 0;
int max_time               = 6000.;  //maximum simulation time (in secs)
double max_distance        = 0.85;   //maximum distance until reset (in m)
int success_counter        = 0;

/// Experimental parameters
double obstacle_height     = 0.10;   // in [m]
bool single_obstacle       = true;   // single obstacle for obstacle negotiation
bool stairs                = false;  // multiple steps with height = obstacle_height
bool use_koh               = false;  // Koh's small escape terrain, therefore both variables use_box & use_box_difficult have to be set to 0.
bool use_box               = false;  // sets up the normal difficult terrain. With "angle" below the boxes can be turned. if angle is set to 0 the boxes in both difficulties are aligned to the grid.
bool use_box_difficult     = false;  // to use this you have to set also use_box=1
bool add_goals             = false;  // ren's goal sensor
bool navigation_setup      = false;  // navigation experiments

/// Camera  parameters
//double cam_pos[3]        = {-2.5, -2.5, 15.};      // This is for navigation experiments (bird's eye)
//double cam_angle[3]      = {0, -90, -90};
double cam_pos[3]          = {0.9, 2., 0.25};        // Camera position (x, y, z)
double cam_angle[3]        = {178.866, -7.43884, 0}; // Camera look-to position (angle) (x, y, z)
//int cam_mode = 0;                                  // Camera mode (0: Static, 1: Follow, 2: TV, 3: Race)

/// Robot options
bool use_broadusangle      = false;  // for avoiding obstacles
bool track                 = true;   // write tracking file
double amos_init_angle     = 0;      // in degrees

/// Controller options
int use_amosii_v           = 2;      // select AMOSII version
bool use_mCPG              = true;   // use multiple CPGs
bool use_navi              = false;  //
bool use_muscles           = false;
bool use_bjc               = true;
bool use_reflexes          = true;
bool use_obstacleavoidance = false;
///////////////////////End of bool variables of different functions


class ThisSim : public lpzrobots::Simulation {
public:

	ThisSim() {
		//addPaletteFile("colors/UrbanExtraColors.gpl");
		//addColorAliasFile("colors/UrbanColorSchema.txt");
		setTitle("       Modular Robot Control Environment (MoRoCo)");
		setCaption("2015");
		amos_pos.open("amos_pos.dat");
	}

    virtual void add_home(GlobalData& global){
        cout << "Added home" << endl;
        PassiveSphere* Home;
        Substance* substHome = new Substance();
        substHome->setCollisionCallback(0,0);
        substHome->toNoContact();
        Home = new PassiveSphere(odeHandle, osgHandle, 0.2);	//TODO
        Home->setSubstance(*substHome);
        Home->setPosition(osg::Vec3(0., 0., -0.35));
    }

    virtual void add_goal(GlobalData& global, double posx, double posy){
        cout << "Added goal at (" << posx << ", " << posy << ")" <<endl;
        PassiveSphere* goal;
        Substance* substGoal = new Substance();
        substGoal->setCollisionCallback(0,0);
        substGoal->toNoContact();
        goal = new PassiveSphere(odeHandle, osgHandle, 0.2);	//TODO
        goal->setSubstance(*substGoal);
        goal->setPosition(osg::Vec3(posx, posy, -0.35));
        goal->setColor(Color(0,255,0));
        goals.push_back(goal);
    }

	virtual void add_obstacle(GlobalData& global, double x, double y, double height, double width=1.0){
		lpzrobots::OdeHandle playgroundHandle = odeHandle;
		playgroundHandle.substance = lpzrobots::Substance(100.0, 0.0, 50.0, 0.0); //substance for playgrounds (NON-SLIPPERY!!!)
		lpzrobots::Playground* playground = new lpzrobots::Playground(playgroundHandle, osgHandle, osg::Vec3(2, width,
				height), 1, false);
		playground->setTexture(0, 0, lpzrobots::TextureDescr("Images/wall_bw.jpg", -0.5, -3));
		playground->setPosition(osg::Vec3(x, y, .0));
		global.obstacles.push_back(playground);
	}

	double distance(){
		double dx = pow(amos->getPosition().x,2);
		double dy = pow(amos->getPosition().y,2);
		return sqrt(dx+dy);
	}

	double distance(double x, double y){
		double dx = pow(amos->getPosition().x - x,2);
		double dy = pow(amos->getPosition().y - y,2);
		return sqrt(dx+dy);
	}

	double distance(lpzrobots::AbstractObstacle* obj){
		double dx = pow(amos->getPosition().x - obj->getPosition().x,2);
		double dy = pow(amos->getPosition().y - obj->getPosition().y,2);
		return sqrt(dx+dy);
	}

	/**
	 * starting function (executed once at the beginning of the simulation loop)
	 */
	virtual void start(const lpzrobots::OdeHandle& odeHandle, const lpzrobots::OsgHandle& osgHandle,
			lpzrobots::GlobalData& global) {

		/// set initial camera position
		setCameraHomePos(lpzrobots::Pos(cam_pos), lpzrobots::Pos(cam_angle));
		setCameraMode(Static);

		/// set simulation parameters
		global.odeConfig.setParam("controlinterval", 10);
		global.odeConfig.setParam("simstepsize", 0.01);
		global.odeConfig.setParam("noise", 0.02); // 0.02

		/// add playgrounds for three different experiments
		lpzrobots::OdeHandle playgroundHandle = odeHandle;
		playgroundHandle.substance = lpzrobots::Substance(100.0, 0.0, 50.0, 0.0); //substance for playgrounds (NON-SLIPPERY!!!)
		//double steplength = 0.43; //AMOS II body length

		/// Single obstacle setup
		if(single_obstacle){
			add_obstacle(global, 0., 0., obstacle_height);
		}
		if(navigation_setup){
			add_home(global);
			add_goal(global, 0., -5.);
			add_goal(global, -5., -5.);
		}


		//myAmosIIConf.legContactSensorIsBinary = true;
		lpzrobots::OdeHandle rodeHandle = odeHandle;
		rodeHandle.substance = lpzrobots::Substance(3.0, 0.0, 50.0, 0.8);

		//------------------- Link the sphere to the Goal Sensor by Ren---------------
		for (unsigned int i = 0; i < obst.size(); i++) {
			myAmosIIConf.GoalSensor_references.push_back(obst.at(i)->getMainPrimitive());
		}


		// it is possible to chose between AMOSIIv1 and AMOSIIv2
		bool use_amosii_version1 = false;
		bool use_amosii_version2 = true;

		if (use_amosii_v == 1){
			std::cout<<std::endl<<std::endl<<"AMOSII  VERSION 1 SELECTED!"<<std::endl<<std::endl;
			// using amosII version 1
			// Add amosIIv1 robot
			myAmosIIConf = lpzrobots::AmosII::getAmosIIv1Conf(1.0 /*_scale*/,1 /*_useShoulder*/,1 /*_useFoot*/,1 /*_useBack*/,true /*for using foot feedback mechanism*/);
			myAmosIIConf.rubberFeet = true;

			amos = new lpzrobots::AmosII(
					rodeHandle,
					osgHandle.changeColor(lpzrobots::Color(1, 1, 1)),
					myAmosIIConf, "AmosIIv1");
			controller = new AmosIIControl(use_amosii_v, use_mCPG, use_muscles, use_navi);
		}
		else if (use_amosii_v == 2){
			std::cout<<std::endl<<std::endl<<"AMOSII  VERSION 2 SELECTED!"<<std::endl<<std::endl;
			myAmosIIConf = lpzrobots::AmosII::getAmosIIv2Conf(1.0 /*_scale*/,1 /*_useShoulder*/,1 /*_useFoot*/,1 /*_useBack*/,true /*for using foot feedback mechanism*/);                    myAmosIIConf.rubberFeet = true;
			myAmosIIConf.legContactSensorIsBinary=false;

			amos = new lpzrobots::AmosII(
					rodeHandle,
					osgHandle.changeColor(lpzrobots::Color(1, 1, 1)),
					myAmosIIConf, "AmosIIv2");
			controller = new AmosIIControl(use_amosii_v, use_mCPG, use_muscles, use_navi);
			((AmosIIControl*) controller)->locomotion_control->desired_angle = amos_init_angle;
		}
		else {
			std::cout<<"select only one version of AMOSII !"<<std::endl;
			assert(use_amosii_version1 != use_amosii_version2);
		}

		if (use_broadusangle){
			myAmosIIConf.usAngleX=0.7; //set Angle of the US sensors, they should cover the robots width
		}

		if(!use_bjc)
			((AmosIIControl*) controller)->flipBJCbool();
		if(!use_reflexes)
			((AmosIIControl*) controller)->flipReflexesbool();

		//amos= new lpzrobots::AmosII(rodeHandle, osgHandle.changeColor(lpzrobots::Color(1, 1, 1)), myAmosIIConf, "AmosII");

		// define the usage of the individual legs
		amos->setLegPosUsage(amos->L0, amos->LEG);
		amos->setLegPosUsage(amos->L1, amos->LEG);
		amos->setLegPosUsage(amos->L2, amos->LEG);
		amos->setLegPosUsage(amos->R0, amos->LEG);
		amos->setLegPosUsage(amos->R1, amos->LEG);
		amos->setLegPosUsage(amos->R2, amos->LEG);

		amos->place(osg::Matrix::rotate(M_PI*amos_init_angle/180,0,0,1) * osg::Matrix::translate(0.0,0.0,0.1));
		if (use_box){amos->place(osg::Matrix::rotate(-0.38,0,0,1) * osg::Matrix::translate(3,-1,0.3));}
		if (use_koh){amos->place(osg::Matrix::rotate(-0.75,0,0,1) * osg::Matrix::translate(0.5,1.5,0.3));}

		// create wiring
		One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise());

		// create agent and init it with controller, robot and wiring
		lpzrobots::OdeAgent* agent = new lpzrobots::OdeAgent(global);
		agent->init(controller, amos, wiring);

		// Possibility to add tracking for robot
		if (track) agent->setTrackOptions(TrackRobot(false, false, false, true, "", 60)); // Display trace

		// inform global variable over everything that happened:
		global.configs.push_back(amos);
		global.agents.push_back(agent);
		global.configs.push_back(controller);
	}


	/**************************Reset Function***************************************************************/
	virtual bool restart(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
	{
		// ADD:: Unit test
		//if (step_limit >0 and globalData.sim_step > step_limit)
		printf("N(success) = %u\n",success_counter);
		if(0.01*total_tstep > max_time)	//Experiment for 1000mins=16hrs40 (@8x ~ 2hrs)
			return false;
		//return false;

		// inform global variable over everything that happened:
		global.configs.erase(global.configs.begin());

		delete amos;
		//delete (agent);
		global.agents.pop_back();

		//Add AMOSII robot
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


		if (use_broadusangle){
			//**************Eduard
			//set Angle of the US sensors, they should cover the robots width
			myAmosIIConf.usAngleX=0.7;
			////////////////Eduard End
		}

		amos= new lpzrobots::AmosII(rodeHandle, osgHandle.changeColor(lpzrobots::Color(1, 1, 1)), myAmosIIConf, "AmosII");

		// define the usage of the individual legs
		amos->setLegPosUsage(amos->L0, amos->LEG);
		amos->setLegPosUsage(amos->L1, amos->LEG);
		amos->setLegPosUsage(amos->L2, amos->LEG);
		amos->setLegPosUsage(amos->R0, amos->LEG);
		amos->setLegPosUsage(amos->R1, amos->LEG);
		amos->setLegPosUsage(amos->R2, amos->LEG);

		//Place AMOSII accordingly
		((AmosIIControl*) controller)->locomotion_control->desired_angle = amos_init_angle;
		amos->place(osg::Matrix::rotate(M_PI*amos_init_angle/180,0,0,1) * osg::Matrix::translate(0.0,0.0,0.0));
		if (use_box){amos->place(osg::Matrix::rotate(-0.38,0,0,1) * osg::Matrix::translate(3,-1,0.3));}
		if (use_koh){amos->place(osg::Matrix::rotate(-0.75,0,0,1) * osg::Matrix::translate(0.5,1.5,0.3));}

		//Create wiring
		One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise());

		// create agent and init it with controller, robot and wiring
		lpzrobots::OdeAgent* agent = new OdeAgent(global);
		agent->init(controller, amos, wiring);

		// Possibility to add tracking for robot
		if (track) agent->setTrackOptions(TrackRobot(false, false, false, true, "", 60)); // Display trace

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
			case 'b': //switch on/off backbone joint control
				((AmosIIControl*) controller)->flipBJCbool();
				break;
			case 'C': //enable sensory feedback mechanism
				((AmosIIControl*) controller)->enableContactForceMech();
				break;
			case  'c':  //disable sensory feedback mechanism
				((AmosIIControl*) controller)->disableContactForceMech();
				break;
			case 'o':  //switch on/off obstacle avoidance
				((AmosIIControl*) controller)->flipOAbool();
				break;
			case 'q':  //increase modulatory input (frequency)
				((AmosIIControl*) controller)->increaseFrequency();
				break;
			case 'r':
				simulation_time_reached=true;
				std::cout << "RESET" << endl;
				break;
			case 'P':  //print current position
				printf("(x,y) = (%2.3f,%2.3f), (vx, vy) = (%2.3f,%2.3f)\n",amos->getPosition().x,amos->getPosition().y, amos->getSpeed().x, amos->getSpeed().y);
				break;
			case 's': //decrease modulatory input (frequency)
				((AmosIIControl*) controller)->decreaseFrequency();
				break;
			case 'l': //switch on learning for climbing TODO switchon function
				((AmosIIControl*) controller)->preprocessing_learning.switchon_IRlearning = true;
				break;
			case 'M': //enable oscillator coupling (fully connected network)
				((AmosIIControl*) controller)->enableOscillatorCoupling();
				break;
			case  'm': //disable oscillator coupling (fully connected network)
				((AmosIIControl*) controller)->disableOscillatorCoupling();
				break;
			case  'i': //Turn by 45 and reset
				amos_init_angle += 45;
				simulation_time_reached=true;
				std::cout << "RESET" << endl;
				break;
			case  'u': //turn desired angle by 45
				((AmosIIControl*) controller)->locomotion_control->desired_angle += 45;
				std::cout << "NEW DESIRED ANGLE = " << ((AmosIIControl*) controller)->locomotion_control->desired_angle << "°" << endl;
				break;

			case  '#':// select tetrapod gait
				((AmosIIControl*) controller)->enableTetrapodGait();
				break;
			case  'w': // select wave
				((AmosIIControl*) controller)->enableWaveGait();
				break;
			case  '!': // select tripod gait
				((AmosIIControl*) controller)->enableTripodGait();
				break;
			case  '$': //// select irregular gait
				((AmosIIControl*) controller)->enableIrregularGait();
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
	virtual void addCallback(lpzrobots::GlobalData& globalData, bool draw, bool pause, bool control) {
		// for demonstration: set simsteps for one cycle to 60.000/currentCycle (10min/currentCycle)
		// if simulation_time_reached is set to true, the simulation cycle is finished


		if (globalData.sim_step >= 0 && globalData.sim_step%10==0) //a small delay to make sure the robot will not restart at beginning!
		{
			if(globalData.sim_step%100==0)
				amos_pos << currentCycle << "\t" << 0.01*total_tstep << "\t" << amos->getPosition().x << "\t" << amos->getPosition().y  << endl;
		}
		total_tstep++;
		//-----------------------------------------------------------------------------------
	}

protected:
	lpzrobots::Joint* robotfixator;
	AbstractController* controller;
	lpzrobots::AmosIIConf myAmosIIConf;
	lpzrobots::AmosII* amos;

};

int main(int argc, char **argv) {
	ThisSim sim;
	//sim.setGroundTexture("Images/desert.jpg");
	return sim.run(argc, argv) ? 0 : 1;
}

