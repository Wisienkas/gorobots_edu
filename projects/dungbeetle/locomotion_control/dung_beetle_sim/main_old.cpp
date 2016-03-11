
// include simulation environment stuff
#include <ode_robots/simulation.h>
// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>
// playground
#include <ode_robots/playground.h>
// simple wiring
#include <selforg/one2onewiring.h>
// the robot
//#include <ode_robots/dungBeetle.h>


#include <ode_robots/dungbeetle.h>


// the controller
#include "controllers/dungbeetle/modular_neural_control/dung_beetle/dungBeetlecontrol.h"

#include <ode_robots/joint.h>

#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>
#include <selforg/abstractcontroller.h>
#include <ode_robots/color.h>
#include <iostream>

using namespace std;
using namespace lpzrobots;
std::vector<lpzrobots::AbstractObstacle*> obst;

bool track = true;


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
		global.odeConfig.setParam("noise", 0.02); // 0.02

		// add playgrounds for three different experiments
		lpzrobots::OdeHandle playgroundHandle = odeHandle;
		playgroundHandle.substance = lpzrobots::Substance(100.0, 0.0, 50.0, 0.0); //substance for playgrounds (NON-SLIPPERY!!!)
		double steplength = 0.43;

		//PLAYGROUND
			lpzrobots::Playground* playground = new lpzrobots::Playground(playgroundHandle, osgHandle, osg::Vec3(2, 0.6,
					0.07), 1, false);
			playground->setTexture(0, 0, lpzrobots::TextureDescr("Images/wall_bw.jpg", -0.5, -3));
			playground->setPosition(osg::Vec3(0, 0, .0));
			global.obstacles.push_back(playground);

		//PLAYGROUND

		// Add dungBeetle robot
		DungBeetleConf myDungBeetleConf = dungBeetle::getDefaultConf(1.0 /*_scale*/, 0 /*_useShoulder*/,
				1 /*_useFoot*/, 1 /*_useBack*/);
		myDungBeetleConf.rubberFeet = true;

		lpzrobots::OdeHandle rodeHandle = odeHandle;
		rodeHandle.substance = lpzrobots::Substance(3.0, 0.0, 50.0, 0.8);

		//------------------- Link the sphere to the Goal Sensor by Ren---------------
		for (unsigned int i = 0; i < obst.size(); i++) {
			myDungBeetleConf.GoalSensor_references.push_back(obst.at(i)->getMainPrimitive());
		}
		//------------------- Link the sphere to the Goal Sensor by Ren---------------

		dungBeetleRobot
		= new dungBeetle(rodeHandle, osgHandle.changeColor(lpzrobots::Color(1, 1, 1)), myDungBeetleConf, "dungBeetleRobot");

		// define the usage of the individual legs
		dungBeetleRobot->setLegPosUsage(dungBeetleRobot->L0, dungBeetleRobot->LEG);
		dungBeetleRobot->setLegPosUsage(dungBeetleRobot->L1, dungBeetleRobot->LEG);
		dungBeetleRobot->setLegPosUsage(dungBeetleRobot->L2, dungBeetleRobot->LEG);
		dungBeetleRobot->setLegPosUsage(dungBeetleRobot->R0, dungBeetleRobot->LEG);
		dungBeetleRobot->setLegPosUsage(dungBeetleRobot->R1, dungBeetleRobot->LEG);
		dungBeetleRobot->setLegPosUsage(dungBeetleRobot->R2, dungBeetleRobot->LEG);



		//

		// put dungBeetle a little bit in the air
		dungBeetleRobot->place(osg::Matrix::translate(.0, .0, 0.25) * osg::Matrix::rotate(M_PI / 180 * (-5), 0, 0, 1));
		//dungBeetleRobot->place(osg::Matrix::translate(.0, .0, 0.0) * osg::Matrix::rotate(M_PI, 1, 0, 0));




		controller = new dungBeetlecontrol(/*dungBeetle*/1,/*MCPGs=true*/false,/*Muscle Model =true*/false);

		// create wiring
		One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise());

		// create agent and init it with controller, robot and wiring
		lpzrobots::OdeAgent* agent = new lpzrobots::OdeAgent(global);
		agent->init(controller, dungBeetleRobot, wiring);

		//put dung beetl ein the air

		 robotfixator = new lpzrobots::FixedJoint(
		        dungBeetleRobot->getMainPrimitive(),
		        global.environment);
		    robotfixator->init(odeHandle, osgHandle, false);



		    std::cout << "\n\n"
		        << "################################\n"
		        << "#   Press x to free dungBeetle!    #\n"
		        << "################################\n"
		       << "\n\n" << std::endl;

		    //

       // Possibility to add tracking for robot
		if (track)
			agent->setTrackOptions(TrackRobot(true, false, false, true, "", 60)); // Display trace

		// inform global variable over everything that happened:
		global.configs.push_back(dungBeetleRobot);
		global.agents.push_back(agent);
		global.configs.push_back(controller);



	}


	 virtual bool command(const lpzrobots::OdeHandle&,
	      const lpzrobots::OsgHandle&,
	      lpzrobots::GlobalData& globalData,
	      int key,
	      bool down)
	  {
	    if (down) { // only when key is pressed, not when released
	      switch (char(key)) {
	        case 'x':
	          if (robotfixator) {
	            std::cout << "dropping robot" << std::endl;
	            delete robotfixator;
	            robotfixator = NULL;
	          }
	          break;
	        default:
	          return false;
	          break;
	      }
	    }
	    return false;
	  }


/**************************Reset Function***************************************************************/
    	virtual bool restart(const OdeHandle& odeHandle, const OsgHandle& osgHandle, lpzrobots::GlobalData& global)
   	{
       	 	// inform global variable over everything that happened:
        	global.configs.erase(global.configs.begin());

        	delete dungBeetleRobot;
        	//delete (agent);
        	global.agents.pop_back();

       // Add dungBeetle robot
		DungBeetleConf myDungBeetleConf = dungBeetle::getDefaultConf(1.0 /*_scale*/, 0 /*_useShoulder*/,
				1 /*_useFoot*/, 1 /*_useBack*/);
		myDungBeetleConf.rubberFeet = true;

		lpzrobots::OdeHandle rodeHandle = odeHandle;
		rodeHandle.substance = lpzrobots::Substance(3.0, 0.0, 50.0, 0.8);

		//------------------- Link the sphere to the Goal Sensor by Ren---------------
		for (unsigned int i = 0; i < obst.size(); i++) {
			myDungBeetleConf.GoalSensor_references.push_back(obst.at(i)->getMainPrimitive());
		}
		//------------------- Link the sphere to the Goal Sensor by Ren---------------

		dungBeetleRobot
		= new dungBeetle(rodeHandle, osgHandle.changeColor(lpzrobots::Color(1, 1, 1)), myDungBeetleConf, "dungBeetle");

		// define the usage of the individual legs
		dungBeetleRobot->setLegPosUsage(dungBeetleRobot->L0, dungBeetleRobot->LEG);
		dungBeetleRobot->setLegPosUsage(dungBeetleRobot->L1, dungBeetleRobot->LEG);
		dungBeetleRobot->setLegPosUsage(dungBeetleRobot->L2, dungBeetleRobot->LEG);
		dungBeetleRobot->setLegPosUsage(dungBeetleRobot->R0, dungBeetleRobot->LEG);
		dungBeetleRobot->setLegPosUsage(dungBeetleRobot->R1, dungBeetleRobot->LEG);
		dungBeetleRobot->setLegPosUsage(dungBeetleRobot->R2, dungBeetleRobot->LEG);

		// put dungBeetle a little bit in the air
		dungBeetleRobot->place(osg::Matrix::translate(.0, .0, 0.0) * osg::Matrix::rotate(M_PI / 180 * (-5), 0, 0, 1));

       	 	//Create wiring
        	One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise());

        	// create agent and init it with controller, robot and wiring
        	lpzrobots::OdeAgent* agent = new OdeAgent(global);
        	agent->init(controller, dungBeetleRobot, wiring);

        	// Possibility to add tracking for robot
        	if (track) agent->setTrackOptions(TrackRobot(false, false, false, true, "", 60)); // Display trace

        	// inform global variable over everything that happened:
       	 	global.configs.push_back(dungBeetleRobot);
        	global.agents.push_back(agent);
        	global.configs.push_back(controller);

        	return true;
    	}


	////////////////////////////////////////////////////////////////////////////////////////////////////////
	//   optional additional callback function which is called every simulation step.
	//   Called between physical simulation step and drawing.
	//   @param draw indicates that objects are drawn in this timestep
	//   @param pause always false (only called of simulation is running)
	//   @param control indicates that robots have been controlled this timestep
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	virtual void addCallback(lpzrobots::GlobalData& globalData, bool draw, bool pause, bool control) {

		//-----------------------------------------------------------------------------------
	}

protected:
	lpzrobots::Joint* robotfixator;
	AbstractController* controller;
	dungBeetle* dungBeetleRobot;

};

int main(int argc, char **argv) {
	ThisSim sim;
	sim.setGroundTexture("Images/greenground.rgb");
	return sim.run(argc, argv) ? 0 : 1;
}

