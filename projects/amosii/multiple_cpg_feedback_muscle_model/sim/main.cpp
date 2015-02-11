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
#include "controllers/amosii/modular_neural_control/amosIIcontrol.h"
// joint needed for fixation of the robot in the beginning
#include <ode_robots/joint.h>

// add head file for creating a sphere by Ren ------------
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>
#include <selforg/abstractcontroller.h>
#include <ode_robots/color.h>
#include <iostream>
using namespace std;
// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
std::vector<lpzrobots::AbstractObstacle*> obst;

// add head file for creating a sphere by Ren ------------



//************Bool variables of different functions
//dennis climbing
bool climbing_experiment_setup = false;
bool stair_experiment_setup = false;
//eduard avoiding obstacles, add different box terrain, to activate change false to true
bool use_koh = 0; //kohs small escape terrain, therefore both variables use_box & use_box_difficult have to be set to 0.
bool use_box = 1; //sets up the normal difficult terrain. With "angle" below the boxes can be turned. if angle is set to 0 the boxes in both difficulties are aligned to the grid.
bool use_box_difficult = 1; //to use this you have to set also use_box=1
//ren's goal sensor
bool add_goals=0;

//for avoiding obstacles
bool use_broadusangle=true;
//write tracking file
bool track = true;
///////////////////////End of bool variables of different functions


class ThisSim : public lpzrobots::Simulation {
public:

	ThisSim() {
		//addPaletteFile("colors/UrbanExtraColors.gpl");
		//addColorAliasFile("colors/UrbanColorSchema.txt");
		setTitle("       Modular Robot Control Environment (MoRoCo)");
		setCaption("2015");
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
		global.odeConfig.setParam("noise", 0.02); // 0.02

		// add playgrounds for three different experiments
		lpzrobots::OdeHandle playgroundHandle = odeHandle;
		playgroundHandle.substance = lpzrobots::Substance(100.0, 0.0, 50.0, 0.0); //substance for playgrounds (NON-SLIPPERY!!!)
		double steplength = 0.43; //AMOS II body length

		//myAmosIIConf.legContactSensorIsBinary = true;
		lpzrobots::OdeHandle rodeHandle = odeHandle;
		rodeHandle.substance = lpzrobots::Substance(3.0, 0.0, 50.0, 0.8);

		//------------------- Link the sphere to the Goal Sensor by Ren---------------
		for (unsigned int i = 0; i < obst.size(); i++) {
			myAmosIIConf.GoalSensor_references.push_back(obst.at(i)->getMainPrimitive());


		}


		// it is possible to chose between AMOSIIv1 and AMOSIIv2
		bool use_amosii_version1 = false;
		bool use_amosii_version2 =true ;

		if (use_amosii_version1 && !use_amosii_version2){
			std::cout<<std::endl<<std::endl<<"AMOSII  VERSION 1 SELECTED!"<<std::endl<<std::endl;
			// using amosII version 1
			// Add amosIIv1 robot
			myAmosIIConf = lpzrobots::AmosII::getAmosIIv1Conf(1.0 /*_scale*/,1 /*_useShoulder*/,1 /*_useFoot*/,1 /*_useBack*/,true /*for using foot feedback mechanism*/);
			myAmosIIConf.rubberFeet = true;

			amos = new lpzrobots::AmosII(
					rodeHandle,
					osgHandle.changeColor(lpzrobots::Color(1, 1, 1)),
					myAmosIIConf, "AmosIIv1");
			controller = new AmosIIControl(/*AMOSv1*/1,/*MCPGs=true*/true,/*Muscle Model =true*/false);
		}
		else {
			std::cout<<"select only one version of AMOSII !"<<std::endl;
			assert(use_amosii_version1 != use_amosii_version2);
		}

		if (use_amosii_version2 && !use_amosii_version1){
			std::cout<<std::endl<<std::endl<<"AMOSII  VERSION 2 SELECTED!"<<std::endl<<std::endl;
			myAmosIIConf = lpzrobots::AmosII::getAmosIIv2Conf(1.0 /*_scale*/,1 /*_useShoulder*/,1 /*_useFoot*/,1 /*_useBack*/,true /*for using foot feedback mechanism*/);                    myAmosIIConf.rubberFeet = true;
			myAmosIIConf.legContactSensorIsBinary=false;

			amos = new lpzrobots::AmosII(
					rodeHandle,
					osgHandle.changeColor(lpzrobots::Color(1, 1, 1)),
					myAmosIIConf, "AmosIIv2");
			controller = new AmosIIControl(/*AMOSv2*/2,/*MCPGs=true*/true,/*Muscle Model =true*/false);


		}




		if (use_broadusangle){
			//**************Eduard
			//set Angle of the US sensors, they should cover the robots width
			myAmosIIConf.usAngleX=0.7;
			////////////////Eduard End
		}

		//amos= new lpzrobots::AmosII(rodeHandle, osgHandle.changeColor(lpzrobots::Color(1, 1, 1)), myAmosIIConf, "AmosII");

		// define the usage of the individual legs
		amos->setLegPosUsage(amos->L0, amos->LEG);
		amos->setLegPosUsage(amos->L1, amos->LEG);
		amos->setLegPosUsage(amos->L2, amos->LEG);
		amos->setLegPosUsage(amos->R0, amos->LEG);
		amos->setLegPosUsage(amos->R1, amos->LEG);
		amos->setLegPosUsage(amos->R2, amos->LEG);


		if (use_box){amos->place(osg::Matrix::rotate(-0.38,0,0,1) * osg::Matrix::translate(3,-1,0.3));}
		if (use_koh){amos->place(osg::Matrix::rotate(-0.75,0,0,1) * osg::Matrix::translate(0.5,1.5,0.3));}


		//controller = new AmosIIControl(1,false,false,false);//TripodGait18DOF();
		// create wiring
		One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise());

		// create agent and init it with controller, robot and wiring
		lpzrobots::OdeAgent* agent = new lpzrobots::OdeAgent(global);
		agent->init(controller, amos, wiring);

		// Possibility to add tracking for robot

		if (track) agent->setTrackOptions(TrackRobot(true, false, false, true, "", 60)); // Display trace



		// inform global variable over everything that happened:
		global.configs.push_back(amos);
		global.agents.push_back(agent);
		global.configs.push_back(controller);
	}

	/**
	 * add own key handling stuff here, just insert some case values
	 */

	virtual bool command(const lpzrobots::OdeHandle&, const lpzrobots::OsgHandle&, lpzrobots::GlobalData& globalData,
			int key, bool down) {
		if (down) { // only when key is pressed, not when released
			switch (char(key)) {
			case 'b': //switch backbone joint off
				((AmosIIControl*) controller)->control_adaptiveclimbing->switchon_backbonejoint=false;
				break;
			case 'C': //enable sensory feedback mechanism
				((AmosIIControl*) controller)->enableContactForceMech();
				break;
			case  'c':  //disable sensory feedback mechanism
				((AmosIIControl*) controller)->disableContactForceMech();
				break;
			case 'q':  //increase modulatory input (frequency)
				((AmosIIControl*) controller)->increaseFrequency();
				break;
			case 's': //decrease modulatory input (frequency)
				((AmosIIControl*) controller)->decreaseFrequency();
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
			case  '@': // select wave
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


		if (globalData.sim_step >= 0) //a small delay to make sure the robot will not restart at beginning!
		{
			if (((AmosIIControl*) controller)->preprocessing_learning.switchon_IRlearning
					&& ((AmosIIControl*) controller)->x.at(25) > 0.9 && ((AmosIIControl*) controller)->x.at(26) > 0.9) {
				((AmosIIControl*) controller)->control_adaptiveclimbing->bj_output.at(0) = 0.0;
				((AmosIIControl*) controller)->control_adaptiveclimbing->bj_output.at(5) = 0.0;
				((AmosIIControl*) controller)->control_adaptiveclimbing->m.at(BJ_m) = 0.0;
				((AmosIIControl*) controller)->control_adaptiveclimbing->switchon_backbonejoint = false;
				((AmosIIControl*) controller)->y.at(BJ_m) = 0.0;
				amos->place(osg::Matrix::translate(.0, .0, 0.0) * osg::Matrix::rotate(0.0, -M_PI / 180 * (-5), 1, 0));
			}
		}
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
	sim.setGroundTexture("Images/greenground.rgb");
	return sim.run(argc, argv) ? 0 : 1;
}

