/***************************************************************************
 *   Copyright (C) 2005-2013 LpzRobots development team                    *
 *   Authors:                                                              *
 *    Simón Smith <artificialsimon at ed dot ac dot uk>                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
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

// Simulation
#include <ode_robots/simulation.h>
// Noise generator
#include <selforg/noisegenerator.h>
// Agent: bind robot, controller and wiring together
#include <ode_robots/odeagent.h>
// The robot
#include "differential.h"
// Robot's controller
#include "basiccontroller.h"
// Robot's wiring
#include <selforg/one2onewiring.h>
// Environment and obstacles
#include <ode_robots/playground.h>
#include <ode_robots/passivebox.h>


using namespace lpzrobots;
using namespace std;

int numberSetup = 3;
int sampleRate = 10;

class ThisSim : public Simulation
{
public:
	ThisSim() { }
	~ThisSim() { }

	/// start() is called at the start and should create all the object (obstacles, agents...).
	virtual void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
	{
		// Initial position and orientation of the camera (use 'p' in graphical window to find out)
//		setCameraHomePos(Pos(-14,14, 10),  Pos(-135, -24, 0));
		setCameraHomePos(Pos(10,0, 50),  Pos(0, 0, 0));
		// Some simulation parameters can be set here
		global.odeConfig.setParam("controlinterval", 1);
		global.odeConfig.setParam("gravity", -9.8);
		global.odeConfig.setParam("noise", 0.);

		/** New robot instance */
		// Get the default configuration of the robot
		DifferentialConf conf = Differential::getDefaultConf();
		// Values can be modified locally
		conf.wheelMass = .5;
		// Instantiating the robot
		OdeRobot* robot = new Differential(odeHandle, osgHandle, conf, "Differential robot");
		// Placing the robot in the scene
		((OdeRobot*)robot)->place(Pos(.0, 0, .2));
		// Instantiating the controller
		//auto controller = new BasicController("Basic Controller");
		auto controller = new BasicController("Basic Controller","SLAM/sensorData/sensorData.dat",
				sampleRate);
		// Create the wiring with color noise
		AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(.1));
		// Create Agent
		OdeAgent* agent = new OdeAgent(global);
		// Agent initialisation
		agent->init(controller, robot, wiring);
		// Adding the agent to the agents list
		global.agents.push_back(agent);
		global.configs.push_back(agent);

		//		global.configs.push_back(robot);
		//		global.configs.push_back(controller);
		/** Environment and obstacles */
		// New playground
		//		Playground* playground = new Playground(odeHandle, osgHandle,osg::Vec3(15., .2, 1.2), 1);
		//		// Set colours
		//		playground->setGroundColor(Color(.784, .784, .0));
		//		playground->setColor(Color(1., .784, .082, .3));
		//		// Set position
		//		playground->setPosition(osg::Vec3(.0, .0, .1));
		//		// Adding playground to obstacles list
		//		global.obstacles.push_back(playground);

		//Create maze
		Substance BoxSubstance(2.0,0.001,100.0,0.4); //(roughness,slip,hardness,elasticity)
		OdeHandle oodeHandle = odeHandle;
		oodeHandle.substance = BoxSubstance;
		double height = 2;
		double width;
		//Parameters box (lenghtX, lenghtY, lenghtZ)
		if(numberSetup == 1){
			width = 3;
			//First Left Wall
			PassiveBox* wall1 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(width,20,height),0);
			wall1->setPose(osg::Matrix::translate(-7, 10, height/2));
			global.obstacles.push_back(wall1);
			//Middle block
			PassiveBox* wall2 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(12,20,height),0);
			wall2->setPose(osg::Matrix::translate(9.5, 10, height/2));
			global.obstacles.push_back(wall2);
			//top wall
			PassiveBox* wall3 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(20, width, height) , 0.0);
			wall3->setPose(osg::Matrix::translate(10,30,height/2));
			global.obstacles.push_back(wall3);
			//Right Wall
			PassiveBox* wall4 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(width,20,height),0);
			wall4->setPose(osg::Matrix::translate(25, 10, height/2));
			global.obstacles.push_back(wall4);
			//top wall
			PassiveBox* wall5 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(20, width, height) , 0.0);
			wall5->setPose(osg::Matrix::translate(15,0,height/2));
			global.obstacles.push_back(wall5);
			//Tilted Walls
			PassiveBox* wall6 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(15, width, height) , 0.0);
			wall6->setPose(osg::Matrix::rotate(M_PI/3,0,0,1) * osg::Matrix::translate(-4,25,height/2));
			global.obstacles.push_back(wall6);
			PassiveBox* wall7 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(10, width, height) , 0.0);
			wall7->setPose(osg::Matrix::rotate(-M_PI/3,0,0,1) * osg::Matrix::translate(22,25,height/2));
			global.obstacles.push_back(wall7);
		}

		if(numberSetup == 2){
			width = 3;
			//First Left Wall
			PassiveBox* wall1 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(width,200,height),0);
			wall1->setPose(osg::Matrix::translate(-4.6, 50, height/2));
			global.obstacles.push_back(wall1);
			//Right wall
//			PassiveBox* wall2 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(width,200,height),0);
//			wall2->setPose(osg::Matrix::translate(4.5, 50, height/2));
//			global.obstacles.push_back(wall2);
			PassiveBox* wall2 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(width,200,height),0);
			wall2->setPose(osg::Matrix::translate(9.5, 50, height/2));
			global.obstacles.push_back(wall2);
		}

		if(numberSetup == 3){
			width = 3;
			//First Left Wall
			PassiveBox* wall1 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(width,20,height),0);
			wall1->setPose(osg::Matrix::translate(-7, 0, height/2));
			global.obstacles.push_back(wall1);
			//Middle block
			PassiveBox* wall2 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(12,20,height),0);
			wall2->setPose(osg::Matrix::translate(9.5, 0, height/2));
			global.obstacles.push_back(wall2);
			//top wall
			PassiveBox* wall3 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(20, width, height) , 0.0);
			wall3->setPose(osg::Matrix::translate(10,20,height/2));
			global.obstacles.push_back(wall3);
			//Right Wall
			PassiveBox* wall4 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(width,20,height),0);
			wall4->setPose(osg::Matrix::translate(25, 0, height/2));
			global.obstacles.push_back(wall4);
			//Tilted Walls
			PassiveBox* wall6 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(15, width, height) , 0.0);
			wall6->setPose(osg::Matrix::rotate(M_PI/3,0,0,1) * osg::Matrix::translate(-4,15,height/2));
			global.obstacles.push_back(wall6);
			PassiveBox* wall7 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(12, width, height) , 0.0);
			wall7->setPose(osg::Matrix::rotate(-M_PI/3,0,0,1) * osg::Matrix::translate(23,15,height/2));
			global.obstacles.push_back(wall7);

			PassiveBox* wall8 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(20, width, height) , 0.0);
			wall8->setPose(osg::Matrix::translate(10,-20,height/2));
			global.obstacles.push_back(wall8);
			//Tilted Walls
			PassiveBox* wall9 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(15, width, height) , 0.0);
			wall9->setPose(osg::Matrix::rotate(M_PI/3,0,0,1) * osg::Matrix::translate(22,-15,height/2));
			global.obstacles.push_back(wall9);
			PassiveBox* wall10 = new PassiveBox(oodeHandle, osgHandle, osg::Vec3(12, width, height) , 0.0);
			wall10->setPose(osg::Matrix::rotate(-M_PI/3,0,0,1) * osg::Matrix::translate(-3,-14,height/2));
			global.obstacles.push_back(wall10);
		}
	}

};

int main (int argc, char **argv)
{
	// New simulation
	ThisSim sim;
	// set Title of simulation
	sim.setTitle("Simple Test SLAM");
	// Simulation begins
	return sim.run(argc, argv) ? 0 : 1;
}




