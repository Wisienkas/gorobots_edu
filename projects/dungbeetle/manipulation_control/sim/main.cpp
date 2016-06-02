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
#include "dungbeetle.h"
//#include <ode_robots/dungbeetle.h>
// the controller
//#include "emptycontroller.h"
#include "modular_neural_control_sphere.h"
#include "modular_neural_control_cylinder.h"
//#include "controllers/dungbeetle/modular_neural_control/amosIIcontrol.h"

// joint needed for fixation of the robot in the beginning
#include <ode_robots/joint.h>

// Objects
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivecapsule.h>
#include <ode_robots/passivebox.h>
#include <selforg/abstractcontroller.h>
#include <ode_robots/color.h>
#include <iostream>
#include <cstdio>
#include <ctime>
#include <math.h>
#include <stdlib.h>

using namespace std;
using namespace lpzrobots;

bool track = false;
bool cylinder_visible = true;
bool sphere_object = true;
bool cylinder_object = false;
bool bump = false;
//If both stationary_push and boxing are false, then soft push is automatically true
bool stationary_push = false;
bool boxing = false;
bool rough_terrain = false;

double t = 0;

class ThisSim : public lpzrobots::Simulation {
public:

	ThisSim() {

		addPaletteFile("colors/UrbanExtraColors.gpl");
		addColorAliasFile("colors/UrbanColorSchema.txt");
		// you can replace color mappings in your own file, see colors/UrbanColorSchema.txt
		// addColorAliasFile("myColorSchema.txt");
		setGroundTexture("Images/whiteground.jpg"); // gets its color from the schema
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
		global.odeConfig.setParam("noise", 0.02);

		
		// add playground
		lpzrobots::OdeHandle playgroundHandle = odeHandle;
		playgroundHandle.substance = lpzrobots::Substance(100.0, 0.0, 50.0, 0.0); //substance for playgrounds (NON-SLIPPERY!!!) 
		//     (roughness,slip,hardness,elasticity)

		lpzrobots::Playground* playground = new lpzrobots::Playground(playgroundHandle, osgHandle, osg::Vec3(10, 0.05,
				0.18), 1, false);
		playground->setTexture(0, 0, lpzrobots::TextureDescr("Images/wall_bw.jpg", -0.5, -3));
		playground->setPosition(osg::Vec3(0, 0, .0));
		global.obstacles.push_back(playground);
		
		if (bump)
		{
			// step obstacle
			double boxmass = 1;
			double box_height = 0.05; //meters
			double box_long = 0.001; //meters
			double box_width = 2; //meters

			lpzrobots::OdeHandle boxHandle = odeHandle;
			boxHandle.substance = lpzrobots::Substance(3.0, 0.0, 50.0, 0.0); //(roughness,slip,hardness,elasticity)
				
			box = new PassiveBox(boxHandle, osgHandle, osg::Vec3(box_long, box_width, box_height), boxmass);
			box->setTexture("Images/wall.jpg");
			box->setPose(osg::Matrix::translate(-0.6, 0.0, box_height/2));

			boxfixator = new lpzrobots::FixedJoint(box->getMainPrimitive(), global.environment);
			boxfixator->init(boxHandle, osgHandle, false);

			global.obstacles.push_back(box);
		}
		
		if (rough_terrain)
		{
			terrain.resize(14);
			terrainfixator.resize(14);
			
			double terrain_mass = 1;
			double terrain_height = 0.002;
			double terrain_long = 0.02;
			double terrain_width = 2;
			
			lpzrobots::OdeHandle terrainHandle = odeHandle;
			terrainHandle.substance = lpzrobots::Substance(3.0, 0.0, 50.0, 0.0); //(roughness,slip,hardness,elasticity)
			
			for (int i = 0; i < 14; i++)
			{
				terrain.at(i) = new PassiveBox(terrainHandle, osgHandle, osg::Vec3(terrain_long, terrain_width, terrain_height), terrain_mass);
				terrain.at(i)->setTexture("Images/wall.jpg");
				terrain.at(i)->setPose(osg::Matrix::translate(-0.2-(i*0.06), 0.0, terrain_height/2));
			
				terrainfixator.at(i) = new lpzrobots::FixedJoint(terrain.at(i)->getMainPrimitive(), global.environment);
				terrainfixator.at(i)->init(terrainHandle, osgHandle, false);
			
				global.obstacles.push_back(terrain.at(i));
			}
			
		}
		
		// Create cylinder to push
		double radius_sphere = 0.15; //meters
		double radius_cylinder = 0.09; //meters
		double height_cylinder = 0.6; //meters // 0.0 = sphere and 0.0 < cylinder
		double mass_object = 2;
		
		lpzrobots::OdeHandle objectHandle = odeHandle;
		objectHandle.substance = lpzrobots::Substance(2, 0.0, 30.0, 0.2)/*(3.0, 0.0, 50.0, 0.0)*/; //(roughness,slip,hardness,elasticity)
		
		if (sphere_object) sphere = new PassiveSphere(objectHandle, osgHandle, radius_sphere, mass_object);
		else if (cylinder_object) cylinder = new PassiveCapsule(objectHandle, osgHandle, radius_cylinder, height_cylinder, mass_object);
		
		if (cylinder_visible)
		{
			if (sphere_object)
			{
				sphere->setPose(osg::Matrix::translate(0.1, 0, radius_sphere));
				spherefixator = new lpzrobots::FixedJoint(sphere->getMainPrimitive(), global.environment);
				spherefixator->init(objectHandle, osgHandle, false);
				global.obstacles.push_back(sphere);
			}
			else if (cylinder_object)
			{
				cylinder->setPose(osg::Matrix::rotate(M_PI/2,1,0,0) * osg::Matrix::translate(0.1,0,radius_cylinder));
				cylinderfixator = new lpzrobots::FixedJoint(cylinder->getMainPrimitive(), global.environment);
			    cylinderfixator->init(objectHandle, osgHandle, false);
				global.obstacles.push_back(cylinder);
			}
		}


		/*******  End Modify Environment *******/

		// Add dungbeetle robot
		lpzrobots::dungbeetleConf mydungbeetleConf = lpzrobots::dungbeetle::getDefaultConf(1.0 /*_scale*/, 0 /*_useShoulder*/,
				1 /*_useFoot*/, 1 /*_useBack*/);
		mydungbeetleConf.rubberFeet = true;

		lpzrobots::OdeHandle rodeHandle = odeHandle;
		rodeHandle.substance = lpzrobots::Substance(3.0, 0.0, 50.0, 0.8);

		amos = new lpzrobots::dungbeetle(rodeHandle, osgHandle.changeColor(lpzrobots::Color(1, 1, 1)), mydungbeetleConf, "dungbeetle");

		// define the usage of the individual legs
		amos->setLegPosUsage(amos->L0, amos->LEG);
		amos->setLegPosUsage(amos->L1, amos->LEG);
		amos->setLegPosUsage(amos->L2, amos->LEG);
		amos->setLegPosUsage(amos->R0, amos->LEG);
		amos->setLegPosUsage(amos->R1, amos->LEG);
		amos->setLegPosUsage(amos->R2, amos->LEG);

		// put amos a little bit in the air
		amos->place(osg::Matrix::translate(0.545, 0.0, 0.0));

		controller_sphere = new Modular_neural_control_sphere();
		controller_cylinder = new Modular_neural_control_cylinder();
	
		controller_sphere->bump = bump;
		controller_sphere->stationary_push = stationary_push;
		controller_sphere->boxing = boxing;
		controller_cylinder->bump = bump;
		controller_cylinder->stationary_push = stationary_push;
		controller_cylinder->boxing = boxing;

		// create wiring
		One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise());

		// create agent and init it with controller, robot and wiring
		lpzrobots::OdeAgent* agent = new lpzrobots::OdeAgent(global);
		if (sphere_object) agent->init(controller_sphere, amos, wiring);
		else if (cylinder_object) agent->init(controller_cylinder, amos, wiring);

		// Possibility to add tracking for robot
		if (track)
			agent->setTrackOptions(TrackRobot(true, false, false, true, "", 60)); // Display trace
		//if(track) agent->setTrackOptions(TrackRobot(false,false,false, false, ""));
		
		// create a fixed joint to hold the robot in the air at the beginning
		//robotfixator = new lpzrobots::FixedJoint(amos->getMainPrimitive(), global.environment);
		//robotfixator->init(odeHandle, osgHandle, false);
	
		// inform global variable over everything that happened:
		global.configs.push_back(amos);
		global.agents.push_back(agent);
		if (sphere_object) global.configs.push_back(controller_sphere);
		else if (cylinder_object) global.configs.push_back(controller_cylinder);

		std::cout << "\n\n" << "################################\n" << "#   Press x to free amosII!    #\n"
				<< "################################\n" << "\n\n" << std::endl;
	}


/**************************Reset Function***************************************************************/
    	virtual bool restart(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
   	{
       	 	// inform global variable over everything that happened:
        	global.configs.erase(global.configs.begin());

        	delete amos;
        	
        	//delete (agent);
        	global.agents.pop_back();
        	
			if (sphere_object) 
			{
				delete controller_sphere;
				delete spherefixator;
			}
        	else if (cylinder_object) 
			{
				delete controller_cylinder;
				delete cylinderfixator;
			}

		    if (cylinder_visible)
			{
				double radius_sphere = 0.15;
				double radius_cylinder = 0.09;
				
				lpzrobots::OdeHandle objectHandle = odeHandle;
				objectHandle.substance = lpzrobots::Substance(2, 0.0, 30.0, 0.2)/*(3.0, 0.0, 50.0, 0.0)*/; //(roughness,slip,hardness,elasticity)				
				
				if (sphere_object)
				{
					sphere->setPose(osg::Matrix::translate(0.1, 0, radius_sphere));
					sphere->setColor(lpzrobots::Color(1,1,1));
					spherefixator = new lpzrobots::FixedJoint(sphere->getMainPrimitive(), global.environment);
					spherefixator->init(objectHandle, osgHandle, false);
				}
				else if (cylinder_object)
				{
					cylinder->setPose(osg::Matrix::rotate(M_PI/2,1,0,0) * osg::Matrix::translate(0.1,0,radius_cylinder));
					cylinder->setColor(lpzrobots::Color(1,1,1));
					cylinderfixator = new lpzrobots::FixedJoint(cylinder->getMainPrimitive(), global.environment);
					cylinderfixator->init(objectHandle, osgHandle, false);
				}
			}

			// Add amosII robot
			lpzrobots::dungbeetleConf mydungbeetleConf = lpzrobots::dungbeetle::getDefaultConf(1.0 /*_scale*/, 0 /*_useShoulder*/,
					1 /*_useFoot*/, 1 /*_useBack*/);
			mydungbeetleConf.rubberFeet = true;

			lpzrobots::OdeHandle rodeHandle = odeHandle;
			rodeHandle.substance = lpzrobots::Substance(3.0, 0.0, 50.0, 0.8);

			amos
			= new lpzrobots::dungbeetle(rodeHandle, osgHandle.changeColor(lpzrobots::Color(1, 1, 1)), mydungbeetleConf, "dungbeetle");

			// define the usage of the individual legs
			amos->setLegPosUsage(amos->L0, amos->LEG);
			amos->setLegPosUsage(amos->L1, amos->LEG);
			amos->setLegPosUsage(amos->L2, amos->LEG);
			amos->setLegPosUsage(amos->R0, amos->LEG);
			amos->setLegPosUsage(amos->R1, amos->LEG);
			amos->setLegPosUsage(amos->R2, amos->LEG);
			

			// put amos a little bit in the air
			amos->place(osg::Matrix::translate(0.545, 0.0, 0.0));

			controller_sphere = new Modular_neural_control_sphere();
			controller_cylinder = new Modular_neural_control_cylinder();

			started = false;
			finished = false;
			controller_sphere->finished = false;
			controller_sphere->bump = bump;
			controller_sphere->stationary_push = stationary_push;
			controller_sphere->boxing = boxing;
			controller_cylinder->finished = false;
			controller_cylinder->bump = bump;
			controller_cylinder->stationary_push = stationary_push;
			controller_cylinder->boxing = boxing;

       	 	//Create wiring
        	One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise());	
        	
        	// create agent and init it with controller, robot and wiring
        	lpzrobots::OdeAgent* agent = new OdeAgent(global);
        	if (sphere_object) agent->init(controller_sphere, amos, wiring);
			else if (cylinder_object) agent->init(controller_cylinder, amos, wiring);

        	// Possibility to add tracking for robot
        	if (track) agent->setTrackOptions(TrackRobot(true, false, false, true, "", 60)); // Display trace

			// create a fixed joint to hold the robot in the air at the beginning
		    //robotfixator = new lpzrobots::FixedJoint(amos->getMainPrimitive(), global.environment);
		    //robotfixator->init(odeHandle, osgHandle, false);
			

        	// inform global variable over everything that happened:
       	 	global.configs.push_back(amos);
        	global.agents.push_back(agent);
        	if (sphere_object) global.configs.push_back(controller_sphere);
			else if (cylinder_object) global.configs.push_back(controller_cylinder);

        	return true;
    	}


	/**
	 * add own key handling stuff here, just insert some case values
	 */

	virtual bool command(const lpzrobots::OdeHandle&, const lpzrobots::OsgHandle&, lpzrobots::GlobalData& globalData,
			int key, bool down) {
		if (down) { // only when key is pressed, not when released
			switch (char(key)) {
			case 'r':
				simulation_time_reached=true;

				//close all GUI loggers
				for (OdeAgentList::iterator p = globalData.agents.begin(); p != globalData.agents.end(); ++p)
				{
					(*p)->removePlotOption(GuiLogger);
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
		
		// Put angular friction on a object
		if (sphere_object)
		{
			PassiveSphere* s = dynamic_cast<PassiveSphere*>(sphere);
	      	float friction = 50;
	      	if(s)
	      	{
				Pos svel = s->getMainPrimitive()->getVel();
				s->getMainPrimitive()->applyForce(-svel*friction);
			}
		}
		
		
		if (sphere_object)
		{
			if (controller_sphere->start && !started)
			{
				sphere->setColor(lpzrobots::Color(1,0.2,0.2));
				delete spherefixator;
				spherefixator = NULL;
				started = true;
				t = globalData.time;
			}
		}
		else if (cylinder_object)
		{
			if (controller_cylinder->start && !started)
			{
				cylinder->setColor(lpzrobots::Color(1,0.2,0.2));
				delete cylinderfixator;
				cylinderfixator = NULL;
				started = true;
			}
		}
		
		controller_sphere->time = globalData.time - t;
		
		// Check if object has been pushed to goal
		if (sphere_object)
		{
			Position sphere_pos = sphere->getPosition();
			
			if (sphere_pos.x < /*-0.72*/ -0.855 && !finished)
			{
				sphere->setColor(lpzrobots::Color(0.2,1,0.2));
				//finished = true;
				//controller_sphere->finished = true;
			}
		}
		else if (cylinder_object)
		{
			Position cylinder_pos = cylinder->getPosition();
			
			if (cylinder_pos.x < /*-0.72*/ -0.855 && !finished)
			{
				cylinder->setColor(lpzrobots::Color(0.2,1,0.2));
				//finished = true;
				//controller_cylinder->finished = true;
			}
		}

	}

protected:
	
	lpzrobots::PassiveSphere* sphere;
	lpzrobots::PassiveCapsule* cylinder;
	lpzrobots::PassiveBox* box;
	
	std::vector<lpzrobots::PassiveBox*> terrain;
	
	lpzrobots::Joint* robotfixator;
	lpzrobots::Joint* boxfixator;
	
	std::vector<lpzrobots::Joint*> terrainfixator;
	
	lpzrobots::Joint* cylinderfixator;
	lpzrobots::Joint* spherefixator;
	
	Modular_neural_control_sphere* controller_sphere;
	Modular_neural_control_cylinder* controller_cylinder;
	
	lpzrobots::dungbeetle* amos;
	
	bool started = false;
	bool finished = false;
};

int main(int argc, char **argv) {
	ThisSim sim;
	sim.setGroundTexture("Images/greenground.rgb");
	return sim.run(argc, argv) ? 0 : 1;
}

