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

/**
 * @author: subhi shaker Barikhan
 * Date:27.05.2014
 *
 *This file gives a tutorial how to use the new controller (AmosIIControl(int aAMOStype,bool mMCPGs,bool mMuscleModelisEnabled) ).
 *This controller has numerous privileges:
 *1) selection between AMOSv1 (aAMOStype=1) and AMOSv2 (aAMOStype=2).
 *2) selection between single CPG-based controller (mMCPGs=false) and Multiple CPGs-based control (mMCPGs=true).
 *3) possibility to utilize muscle model (mMuscleModelisEnabled=true).
 *
 *
 *
 * Provided Multiple CPGs control is selected, you should first of all enable the oscillator coupling (oscillatorsCouplingIsEnabled)by
 * entering 'M' then select the gait.
 * specifying the walking pattern can be as follows:
 * clicking  '!': tripod.
 * clicking  '@': tetrapod.
 * clicking  '#': wave.
 * clicking  '$': irregular gait.
 *
 * Note: Multiple CPGs-based control can be combined with sensory feedback to achieve  adaptive locomotion. This can be realized by
 * following previous instruction and enabling sensory feedback function by entering 'C'.
 * */


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
std::vector<lpzrobots::AbstractObstacle*> obst;
ofstream amos_pos;

/// Simulation parameters
int total_tstep=0;
int max_time=6000.;			//maximum simulation time (in secs)
double max_distance=0.85;		//maximum distance until reset (in m)
int success_counter=0;

/// Camera  parameters
double cam_pos[3] = {0.9, 2., 0.25};           // Camera position (x, y, z)
double cam_angle[3] = {178.866, -7.43884, 0}; // Camera look-to position (angle) (x, y, z)
//int cam_mode = 0;                             // Camera mode (0: Static, 1: Follow, 2: TV, 3: Race)

/// Environment options
bool single_obstacle = true;
double obstacle_height = 0.12;  // in [m]
bool stairs = false;
bool use_koh = false;           // kohs small escape terrain, therefore both variables use_box & use_box_difficult have to be set to 0.
bool use_box = false;           // sets up the normal difficult terrain. With "angle" below the boxes can be turned. if angle is set to 0 the boxes in both difficulties are aligned to the grid.
bool use_box_difficult = true;  // to use this you have to set also use_box=1
bool add_goals = false;         // ren's goal sensor

/// Robot options
bool use_broadusangle=false;    // for avoiding obstacles
bool track = true;              // write tracking file

/// Controller options
int use_amosii_v = 2;
bool use_mCPG = true;
bool use_navi = false;
bool use_muscles = false;
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
			controller = new AmosIIControl(use_amosii_v, use_mCPG, use_muscles/*, false*/);
		}
		else if (use_amosii_v == 2){
			std::cout<<std::endl<<std::endl<<"AMOSII  VERSION 2 SELECTED!"<<std::endl<<std::endl;
			myAmosIIConf = lpzrobots::AmosII::getAmosIIv2Conf(1.0 /*_scale*/,1 /*_useShoulder*/,1 /*_useFoot*/,1 /*_useBack*/,true /*for using foot feedback mechanism*/);                    myAmosIIConf.rubberFeet = true;
			myAmosIIConf.legContactSensorIsBinary=false;

			amos = new lpzrobots::AmosII(
					rodeHandle,
					osgHandle.changeColor(lpzrobots::Color(1, 1, 1)),
					myAmosIIConf, "AmosIIv2");
			controller = new AmosIIControl(use_amosii_v, use_mCPG, use_muscles/*, false*/);
		}
		else {
			std::cout<<"select only one version of AMOSII !"<<std::endl;
			assert(use_amosii_version1 != use_amosii_version2);
		}

		if (use_broadusangle){
			myAmosIIConf.usAngleX=0.7; //set Angle of the US sensors, they should cover the robots width
		}

		//amos= new lpzrobots::AmosII(rodeHandle, osgHandle.changeColor(lpzrobots::Color(1, 1, 1)), myAmosIIConf, "AmosII");

		// define the usage of the individual legs
		amos->setLegPosUsage(amos->L0, amos->LEG);
		amos->setLegPosUsage(amos->L1, amos->LEG);
		amos->setLegPosUsage(amos->L2, amos->LEG);
		amos->setLegPosUsage(amos->R0, amos->LEG);
		amos->setLegPosUsage(amos->R1, amos->LEG);
		amos->setLegPosUsage(amos->R2, amos->LEG);

		amos->place(osg::Matrix::rotate(0.0,0,0,1) * osg::Matrix::translate(0.2,0.0,0.1));
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
		if(((AmosIIControl*) controller)->x.at(25) > 0.9 && ((AmosIIControl*) controller)->x.at(26) > 0.9)
			printf("BUMP\tRho1 [R] = %2.6f\tRho1 [L] = %2.6f\n", ((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25), ((AmosIIControl*) controller)->preprocessing_learning.rho1.at(26));
		if(distance() > max_distance){
			printf("SUCCESS\tRho1 [R] = %2.6f\tRho1 [L] = %2.6f\n", ((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25), ((AmosIIControl*) controller)->preprocessing_learning.rho1.at(26));
			success_counter++;
		}
		if(globalData.sim_step > 60000)
			printf("TIME\tRho1 [R] = %2.6f\tRho1 [L] = %2.6f\n", ((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25), ((AmosIIControl*) controller)->preprocessing_learning.rho1.at(26));

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
		amos->place(osg::Matrix::rotate(0.0,0,0,1) * osg::Matrix::translate(0.0,0.0,0.0));
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
			case 'C': //enable sensory feedback mechanism
				((AmosIIControl*) controller)->enableContactForceMech();
				break;
			case  'c':  //disable sensory feedback mechanism
				((AmosIIControl*) controller)->disableContactForceMech();
				break;
			case 'q':  //increase modulatory input (frequency)
				((AmosIIControl*) controller)->increaseFrequency();
				break;
			case 'r':
				simulation_time_reached=true;
				std::cout << "RESET" << endl;
				break;
			case 'p':  //print current position
				printf("(x,y) = (%2.3f,%2.3f)\n",amos->getPosition().x,amos->getPosition().y);
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
				amos_pos << currentCycle << "\t" << 0.01*total_tstep << "\t" << amos->getPosition().x << "\t" << amos->getPosition().y << "\t" << ((AmosIIControl*) controller)->preprocessing_learning.rho1.at(25) << "\t" << ((AmosIIControl*) controller)->preprocessing_learning.rho1.at(26) << endl;
			if (((AmosIIControl*) controller)->preprocessing_learning.switchon_IRlearning
					&& ((AmosIIControl*) controller)->x.at(25) > 0.9 && ((AmosIIControl*) controller)->x.at(26) > 0.9) {
				((AmosIIControl*) controller)->preprocessing_learning.ir_predic_output.at(25) = 0.0;
				((AmosIIControl*) controller)->preprocessing_learning.ir_predic_output.at(26) = 0.0;
				((AmosIIControl*) controller)->preprocessing_learning.ir_reflex_output.at(25) = 0.0;
				((AmosIIControl*) controller)->preprocessing_learning.ir_reflex_output.at(26) = 0.0;
				((AmosIIControl*) controller)->preprocessing_learning.irlearn_output_prolong.at(25) = 0.0;
				((AmosIIControl*) controller)->preprocessing_learning.irlearn_output_prolong.at(26) = 0.0;
				((AmosIIControl*) controller)->preprocessing_learning.us_delayline.at(25)->Reset();
				((AmosIIControl*) controller)->preprocessing_learning.us_delayline.at(26)->Reset();

				//TODO Reset function BJC
				((AmosIIControl*) controller)->locomotion_control->bj_output.at(0) = 0.0;
				((AmosIIControl*) controller)->locomotion_control->bj_output.at(5) = 0.0;
				((AmosIIControl*) controller)->locomotion_control->m.at(BJ_m) = 0.0;
				//((AmosIIControl*) controller)->locomotion_control->switchon_backbonejoint = false;

				simulation_time_reached = true;
			}

			if (((AmosIIControl*) controller)->preprocessing_learning.switchon_IRlearning
					&& (distance() > max_distance || globalData.sim_step > 60000)) {

				((AmosIIControl*) controller)->preprocessing_learning.ir_predic_output.at(25) = 0.0;
				((AmosIIControl*) controller)->preprocessing_learning.ir_predic_output.at(26) = 0.0;
				((AmosIIControl*) controller)->preprocessing_learning.ir_reflex_output.at(25) = 0.0;
				((AmosIIControl*) controller)->preprocessing_learning.ir_reflex_output.at(26) = 0.0;
				((AmosIIControl*) controller)->preprocessing_learning.irlearn_output_prolong.at(25) = 0.0;
				((AmosIIControl*) controller)->preprocessing_learning.irlearn_output_prolong.at(26) = 0.0;
				((AmosIIControl*) controller)->preprocessing_learning.us_delayline.at(25)->Reset();
				((AmosIIControl*) controller)->preprocessing_learning.us_delayline.at(26)->Reset();

				//TODO Reset function BJC
				((AmosIIControl*) controller)->locomotion_control->bj_output.at(0) = 0.0;
				((AmosIIControl*) controller)->locomotion_control->bj_output.at(5) = 0.0;
				((AmosIIControl*) controller)->locomotion_control->m.at(BJ_m) = 0.0;
				//((AmosIIControl*) controller)->locomotion_control->switchon_backbonejoint = false;

				simulation_time_reached = true;
			}
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
