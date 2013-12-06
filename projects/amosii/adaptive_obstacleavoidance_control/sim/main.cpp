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
		setTitle("Adaptive Climbing Behavior of Walking Machines");
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
		global.odeConfig.setParam("noise", 0.02); // 0.02

		// add playgrounds for three different experiments
		lpzrobots::OdeHandle playgroundHandle = odeHandle;
		playgroundHandle.substance = lpzrobots::Substance(100.0, 0.0, 50.0, 0.0); //substance for playgrounds (NON-SLIPPERY!!!)
		double steplength = 0.43; //AMOS II body length

		//EXPERIMENTAL SETUP 1: SINGLE OBSTACLE (Adaption to different obstacle altitudes and walking gaits)
		bool climbing_experiment_setup = false;
		if (climbing_experiment_setup) {
			lpzrobots::Playground* playground = new lpzrobots::Playground(playgroundHandle, osgHandle, osg::Vec3(2, 0.6,
					0.10), 1, false);
			playground->setTexture(0, 0, lpzrobots::TextureDescr("Images/wall_bw.jpg", -0.5, -3));
			playground->setPosition(osg::Vec3(0, 0, .0));
			global.obstacles.push_back(playground);
		}

		//EXPERIMENTAL SETUP 2: STAIRS DECREASING IN LENGTH (Finding the minimal step length x which AMOS is able to negotiate)
		bool stair_experiment_setup = false;
		if (stair_experiment_setup) {
			lpzrobots::Playground* stair1 = new lpzrobots::Playground(playgroundHandle, osgHandle, osg::Vec3(2.0, 4.0
					* steplength, 0.08), 1, false);
			lpzrobots::Playground* stair2 = new lpzrobots::Playground(playgroundHandle, osgHandle, osg::Vec3(2.0 + 4.0
					* steplength, 2.0 * steplength, 0.16), 1, false);
			lpzrobots::Playground* stair3 = new lpzrobots::Playground(playgroundHandle, osgHandle, osg::Vec3(2.0 + (4.0
					+ 2.0) * steplength, 1.0 * steplength, 0.24), 1, false);
			lpzrobots::Playground* stair4 = new lpzrobots::Playground(playgroundHandle, osgHandle, osg::Vec3(2.0 + (4.0
					+ 2.0 + 1.0) * steplength, 0.5 * steplength, 0.32), 1, false);
			lpzrobots::Playground* stair5 = new lpzrobots::Playground(playgroundHandle, osgHandle, osg::Vec3(2.0 + (4.0
					+ 2.0 + 1.0 + 0.5) * steplength, 1.0, 0.40), 1, false);
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


		//-----------------------------Eduard-----------------------------------
		// add box terrain
		// to activate change false to true
		bool use_koh = 0;
		bool use_box = 1;
		bool use_box_difficult = 1;


		if (use_box){
			double angle = 1.0;
			double height = 0.10;
			Substance BoxSubstance(2.0,0.001,100.0,0.4); //(roughness,slip,hardness,elasticity)
			OdeHandle oodeHandle = odeHandle;
			oodeHandle.substance = BoxSubstance;//.toRubber(boxsubparam);//toMetal(boxsubparam);

			//walls
			PassiveBox* s9 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(10 /*length*/,1 /*width*/,height /*height*/), 0.0);
			PassiveBox* s10 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(1 /*length*/,10 /*width*/,height /*height*/), 0.0);
			PassiveBox* s11 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(1 /*length*/,10 /*width*/,height /*height*/), 0.0);
			PassiveBox* s12 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(10 /*length*/,1 /*width*/,height /*height*/), 0.0);
			//walls end


			PassiveBox* s1 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(1 /*length*/,1 /*width*/,height /*height*/), 0.0);
			PassiveBox* s2 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(1 /*length*/,1 /*width*/,height /*height*/), 0.0);
			PassiveBox* s3 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(1 /*length*/,1 /*width*/,height /*height*/), 0.0);
			PassiveBox* s4= new PassiveBox(oodeHandle, osgHandle, osg::Vec3(1 /*length*/,1 /*width*/,height /*height*/), 0.0);
			PassiveBox* s5 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(1 /*length*/,1 /*width*/,height /*height*/), 0.0);
			PassiveBox* s6 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(1 /*length*/,1 /*width*/,height /*height*/), 0.0);
			PassiveBox* s7 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(1 /*length*/,1 /*width*/,height /*height*/), 0.0);
			PassiveBox* s8 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(1 /*length*/,1 /*width*/,height /*height*/), 0.0);

			PassiveBox* s13 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(1 /*length*/,1 /*width*/,height /*height*/), 0.0);
			PassiveBox* s14 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(1 /*length*/,1 /*width*/,height /*height*/), 0.0);
			PassiveBox* s15 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(1 /*length*/,1 /*width*/,height /*height*/), 0.0);

			s2->setPose(osg::Matrix::rotate(2.0 * angle,0,0,1) *osg::Matrix::translate(1,1,height/2));
			s3->setPose(osg::Matrix::rotate(4.0 * angle,0,0,1) *osg::Matrix::translate(2,2,height/2));
			s4->setPose(osg::Matrix::rotate(2.0* angle,0,0,1) *osg::Matrix::translate(3,-2,height/2));

			s5->setPose(osg::Matrix::rotate(1.0 * angle,0,0,1) *osg::Matrix::translate(5,-1,height/2));
			s6->setPose(osg::Matrix::rotate(1.0 * angle,0,0,1) *osg::Matrix::translate(6,-2,height/2));
			s7->setPose(osg::Matrix::rotate(5.0 * angle,0,0,1) *osg::Matrix::translate(4,2,height/2));
			s8->setPose(osg::Matrix::rotate(3.0 * angle,0,0,1) *osg::Matrix::translate(5,-3,height/2));

			s13->setPose(osg::Matrix::translate(3,4,height/2));
			s14->setPose(osg::Matrix::translate(7,0.5,height/2));
			s15->setPose(osg::Matrix::translate(-1,-4,height/2));

			global.obstacles.push_back(s2);
			global.obstacles.push_back(s3);
			global.obstacles.push_back(s4);
			global.obstacles.push_back(s5);
			global.obstacles.push_back(s6);
			global.obstacles.push_back(s7);
			global.obstacles.push_back(s8);

			global.obstacles.push_back(s13);
			global.obstacles.push_back(s14);
			global.obstacles.push_back(s15);

			if(use_box_difficult==1)
			{
				PassiveBox* s16 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(1 /*length*/,1 /*width*/,height /*height*/), 0.0);
				PassiveBox* s17 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(1 /*length*/,1 /*width*/,height /*height*/), 0.0);
				PassiveBox* s18 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(1 /*length*/,1 /*width*/,height /*height*/), 0.0);
				PassiveBox* s19= new PassiveBox(oodeHandle, osgHandle, osg::Vec3(1 /*length*/,1 /*width*/,height /*height*/), 0.0);
				PassiveBox* s20 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(1 /*length*/,1 /*width*/,height /*height*/), 0.0);
				PassiveBox* s21 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(1 /*length*/,1 /*width*/,height /*height*/), 0.0);
				PassiveBox* s22 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(1 /*length*/,1 /*width*/,height /*height*/), 0.0);
				PassiveBox* s23 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(1 /*length*/,1 /*width*/,height /*height*/), 0.0);
				PassiveBox* s24 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(1 /*length*/,1 /*width*/,height /*height*/), 0.0);
				PassiveBox* s25 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(1 /*length*/,1 /*width*/,height /*height*/), 0.0);
				PassiveBox* s26 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(1 /*length*/,1 /*width*/,height /*height*/), 0.0);

				s16->setPose(osg::Matrix::rotate(3.0 * angle,0,0,1) *osg::Matrix::translate(0,3,height/2));
				s17->setPose(osg::Matrix::rotate(2.0 * angle,0,0,1) *osg::Matrix::translate(0,-2,height/2));
				s18->setPose(osg::Matrix::rotate(4.0 * angle,0,0,1) *osg::Matrix::translate(-1.5,-2,height/2));
				s19->setPose(osg::Matrix::rotate(2.0* angle,0,0,1) *osg::Matrix::translate(1.5,-4,height/2));
				s20->setPose(osg::Matrix::rotate(3.0 * angle,0,0,1) *osg::Matrix::translate(7,-4,height/2));
				s21->setPose(osg::Matrix::rotate(2.0 * angle,0,0,1) *osg::Matrix::translate(3.2,0,height/2));
				s22->setPose(osg::Matrix::rotate(4.0 * angle,0,0,1) *osg::Matrix::translate(1.5,-1,height/2));

				s23->setPose(osg::Matrix::rotate(3.0 * angle,0,0,1) *osg::Matrix::translate(5.2,3.8,height/2));
				s24->setPose(osg::Matrix::rotate(2.0 * angle,0,0,1) *osg::Matrix::translate(7,4,height/2));
				s25->setPose(osg::Matrix::rotate(2.0 * angle,0,0,1) *osg::Matrix::translate(5.8,2,height/2));
				s26->setPose(osg::Matrix::rotate(1.0 * angle,0,0,1) *osg::Matrix::translate(3.5,-4,height/2));

				global.obstacles.push_back(s16);
				global.obstacles.push_back(s17);
				global.obstacles.push_back(s18);
				global.obstacles.push_back(s19);

				global.obstacles.push_back(s20);
				global.obstacles.push_back(s21);
				global.obstacles.push_back(s22);
				global.obstacles.push_back(s23);
				global.obstacles.push_back(s24);
				global.obstacles.push_back(s25);
				global.obstacles.push_back(s26);
			}

			s1->setPose(osg::Matrix::rotate(3.0 * angle,0,0,1) *osg::Matrix::translate(0,0,height/2));

			//for the walls
			s9->setPose(osg::Matrix::translate(3,5,height/2));
			s10->setPose(osg::Matrix::translate(-2,0,height/2));
			s11->setPose(osg::Matrix::translate(8,0,height/2));
			s12->setPose(osg::Matrix::translate(3,-5,height/2));

			global.obstacles.push_back(s1);
			global.obstacles.push_back(s9);
			global.obstacles.push_back(s10);
			global.obstacles.push_back(s11);
			global.obstacles.push_back(s12);

		}


		if (use_koh){
			double angle = 0.0;
			double height = 0.10;
			Substance BoxSubstance(2.0,0.001,100.0,0.4); //(roughness,slip,hardness,elasticity)
			OdeHandle oodeHandle = odeHandle;
			oodeHandle.substance = BoxSubstance;//.toRubber(boxsubparam);//toMetal(boxsubparam);
			PassiveBox* s1 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(5 /*length*/,1 /*width*/,height /*height*/), 0.0);
			PassiveBox* s2 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(1/*length*/,5 /*width*/,height /*height*/), 0.0);
			PassiveBox* s3 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(2 /*length*/,1 /*width*/,height /*height*/), 0.0);
			PassiveBox* s4= new PassiveBox(oodeHandle, osgHandle, osg::Vec3(1 /*length*/,2 /*width*/,height /*height*/), 0.0);

			s1->setPose(osg::Matrix::translate(0,0,height/2));
			s2->setPose(osg::Matrix::translate(2,2,height/2));
			s3->setPose(osg::Matrix::translate(1,4,height/2));
			s4->setPose(osg::Matrix::translate(-2,1,height/2));

			global.obstacles.push_back(s1);
			global.obstacles.push_back(s2);
			global.obstacles.push_back(s3);
			global.obstacles.push_back(s4);
		}
		//*****************************************Eduard********************





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


		//set Angle of the US sensors, they should cover the robots width
		myAmosIIConf.usAngleX=0.7;


		amos= new lpzrobots::AmosII(rodeHandle, osgHandle.changeColor(lpzrobots::Color(1, 1, 1)), myAmosIIConf, "AmosII");

		// define the usage of the individual legs
		amos->setLegPosUsage(amos->L0, amos->LEG);
		amos->setLegPosUsage(amos->L1, amos->LEG);
		amos->setLegPosUsage(amos->L2, amos->LEG);
		amos->setLegPosUsage(amos->R0, amos->LEG);
		amos->setLegPosUsage(amos->R1, amos->LEG);
		amos->setLegPosUsage(amos->R2, amos->LEG);

		// put amos a little bit in the air
		//amos->place(osg::Matrix::translate(.0, .0, 0.0) * osg::Matrix::rotate(M_PI / 180 * (-5), 0, 0, 1));

		if (use_box){amos->place(osg::Matrix::rotate(-0.38,0,0,1) * osg::Matrix::translate(3,-1,0.3));}
		if (use_koh){amos->place(osg::Matrix::rotate(-0.75,0,0,1) * osg::Matrix::translate(0.5,1.5,0.3));}


		controller = new AmosIIControl();//TripodGait18DOF();
		// create wiring
		One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise());

		// create agent and init it with controller, robot and wiring
		lpzrobots::OdeAgent* agent = new lpzrobots::OdeAgent(global);
		agent->init(controller, amos, wiring);

		// Possibility to add tracking for robot
		bool track = true;
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

	/**
	 * add own key handling stuff here, just insert some case values
	 */

	virtual bool command(const lpzrobots::OdeHandle&, const lpzrobots::OsgHandle&, lpzrobots::GlobalData& globalData,
			int key, bool down) {
		if (down) { // only when key is pressed, not when released
			switch (char(key)) {
			case 'x':
				if (robotfixator) {
					std::cout << "dropping robot" << std::endl;
					delete robotfixator;
					robotfixator = NULL;
				}
				break;
			case 'r':
				amos->place(osg::Matrix::translate(.0, .0, 0.0) * osg::Matrix::rotate(0.0, -M_PI / 180 * (-5), 1, 0));
				((AmosIIControl*) controller)->preprocessing_learning.switchon_IRlearning = false;
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
			case 'a':
				if (((AmosIIControl*) controller)->control_adaptiveclimbing.switchon_obstacle) {
					((AmosIIControl*) controller)->control_adaptiveclimbing.switchon_obstacle = false;
					std::cout << "OA is OFF" << endl;
				} else {
					((AmosIIControl*) controller)->control_adaptiveclimbing.switchon_obstacle = true;
					((AmosIIControl*) controller)->control_adaptiveclimbing.switchon_reflexes=false;
					((AmosIIControl*) controller)->control_adaptiveclimbing.switchon_irreflexes=false;
					((AmosIIControl*) controller)->control_adaptiveclimbing.switchon_purefootsignal=false;
					std::cout << "OA is ON" << endl;
				}
				break;

			case 'e':
				if (((AmosIIControl*) controller)->control_adaptiveclimbing.switchon_allreflexactions) {
					((AmosIIControl*) controller)->control_adaptiveclimbing.switchon_allreflexactions = false;
					((AmosIIControl*) controller)->control_adaptiveclimbing.switchon_reflexes=false;
					((AmosIIControl*) controller)->control_adaptiveclimbing.switchon_irreflexes=false;
					((AmosIIControl*) controller)->control_adaptiveclimbing.switchon_purefootsignal=false;
					std::cout << "Reflex is OFF" << endl;
				} else {
					((AmosIIControl*) controller)->control_adaptiveclimbing.switchon_allreflexactions = true;
					((AmosIIControl*) controller)->control_adaptiveclimbing.switchon_reflexes=true;
					((AmosIIControl*) controller)->control_adaptiveclimbing.switchon_irreflexes=true;
					((AmosIIControl*) controller)->control_adaptiveclimbing.switchon_purefootsignal=true;
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
	virtual void addCallback(lpzrobots::GlobalData& globalData, bool draw, bool pause, bool control) {
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
		if (globalData.sim_step >= 0) //a small delay to make sure the robot will not restart at beginning!
		{
			if (((AmosIIControl*) controller)->preprocessing_learning.switchon_IRlearning
					&& ((AmosIIControl*) controller)->x.at(25) > 0.9 && ((AmosIIControl*) controller)->x.at(26) > 0.9) {
				((AmosIIControl*) controller)->control_adaptiveclimbing.bj_output.at(0) = 0.0;
				((AmosIIControl*) controller)->control_adaptiveclimbing.bj_output.at(5) = 0.0;
				((AmosIIControl*) controller)->control_adaptiveclimbing.m.at(BJ_m) = 0.0;
				((AmosIIControl*) controller)->control_adaptiveclimbing.switchon_backbonejoint = false;
				((AmosIIControl*) controller)->y.at(BJ_m) = 0.0;
				amos->place(osg::Matrix::translate(.0, .0, 0.0) * osg::Matrix::rotate(0.0, -M_PI / 180 * (-5), 1, 0));
			}
		}
		//-----------------------------------------------------------------------------------
	}

protected:
	lpzrobots::Joint* robotfixator;
	AbstractController* controller;
	lpzrobots::AmosII* amos;

};

int main(int argc, char **argv) {
	ThisSim sim;
	sim.setGroundTexture("Images/greenground.rgb");
	return sim.run(argc, argv) ? 0 : 1;
}

