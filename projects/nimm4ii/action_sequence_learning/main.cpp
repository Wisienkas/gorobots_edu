#include <stdio.h>

//RANDOM
#include <stdlib.h>  /* RANDOM_MAX */

// include ode library
#include <ode-dbl/ode.h>

// include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/noisegenerator.h>

// include simulation environment stuff
#include <ode_robots/simulation.h>

// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>

// used wiring
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>

// used robot...
#include <ode_robots/nimm2.h>
#include <ode_robots/nimm4.h>
#include <fourwheeledrpos_gripper.h>


#include <selforg/trackrobots.h>
// used arena
#include <ode_robots/playground.h>
// used passive spheres
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>

// controller
#include "aslcontroller.h"

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

// controller
ASLController* qcontroller;

// robot
FourWheeledRPosGripper* vehicle;

// relative_sensors
std::vector<AbstractObstacle*> relative_sensor_obst;

// DSW
Primitives grippables;
Primitives boxPrimitives;

// number of runs before stopping
int runs = 10;


class ThisSim : public Simulation {
public:

	/********************************************************************************************
	*** generate 3 goal boxes return Primitives(vector of Primitive*) of boxes to add to grippables
	********************************************************************************************/
	void generate_boxes(GlobalData& global)
	{
		double length = 1.38;
		double width = 1.38;
		double height = 0.9;
		Substance material(5.0,10.0,99.0,1.0);
		double mass = 0.1;
		PassiveBox* b1;
	  	b1 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(length, width, height),mass);
	  	b1->setColor(Color(0,1,0));
	  	b1->setSubstance(material);
	  	b1->setPose(osg::Matrix::rotate(0, 0,0, 1) * osg::Matrix::translate(-5,0,1.5));
		global.obstacles.push_back(b1);
		
			PassiveBox* b2;
	  	b2 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(length, width, height),mass);
	  	b2->setColor(Color(1,0,0));
	  	b2->setSubstance(material);
	  	b2->setPose(osg::Matrix::rotate(0, 0,0, 1) * osg::Matrix::translate(5,0,1.5));
		global.obstacles.push_back(b2);

			PassiveBox* b3;
	  	b3 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(length, width, height),mass);
	  	b3->setColor(Color(1,0,0));
	  	b3->setSubstance(material);  	
	  	b3->setPose(osg::Matrix::rotate(0, 0,0, 1) * osg::Matrix::translate(0,-5,1.5));
		global.obstacles.push_back(b3);	
		
		boxPrimitives.push_back(b1->getMainPrimitive());
		boxPrimitives.push_back(b2->getMainPrimitive());
		boxPrimitives.push_back(b3->getMainPrimitive());
	
		// adding boxes to obstacle vector so it can be connected to sensors
		relative_sensor_obst.push_back(b1);
		relative_sensor_obst.push_back(b2);
		relative_sensor_obst.push_back(b3);
		
	}	
	
	/********************************************************************************************
	*** set up Playground
	********************************************************************************************/
	void setup_Playground(GlobalData& global)
	{

		// inner Platform
		double length_pg = 0.0;
		double width_pg = 9.5;
		double height_pg = 1.0;
		
		Playground* playground = new Playground(odeHandle, osgHandle.changeColor(Color(0.6,0.0,0.6)),
				osg::Vec3(length_pg /*length*/, width_pg /*width*/, height_pg/*height*/), /*factorxy = 1*/1, /*createGround=true*/true /*false*/);

		playground->setPosition(osg::Vec3(0,0,0.0));
		global.obstacles.push_back(playground);

		// DSW: outer_pg = outter platform
  		double length_outer_pg = 22.0;
		double width_outer_pg = 5.0;
		double height_outer_pg = 1.0;

		Playground* outer_playground = new Playground(odeHandle, osgHandle.changeColor(Color(0.6,0.0,0.6)),
					osg::Vec3(length_outer_pg /*length*/, width_outer_pg /*width*/, height_outer_pg/*height*/), /*factorxy = 1*/1, /*createGround=true*/true /*false*/);
		outer_playground->setPosition(osg::Vec3(0,0,0.0));
		global.obstacles.push_back(outer_playground);

  		// DSW: outer_pg2 = walls
  		double length_outer_pg2 = 32.0;
		double width_outer_pg2 = 1.0;
		double height_outer_pg2 = 2.0;

		Playground* outer_playground2 = new Playground(odeHandle, osgHandle.changeColor(Color(0.6,0.0,0.6)),
					osg::Vec3(length_outer_pg2 /*length*/, width_outer_pg2 /*width*/, height_outer_pg2/*height*/), /*factorxy = 1*/1, /*createGround=true*/true /*false*/);
		outer_playground2->setPosition(osg::Vec3(0,0,0.0));
  		global.obstacles.push_back(outer_playground2);
	}

	/********************************************************************************************
	*** starting function (executed once at the beginning of the simulation loop)
	********************************************************************************************/
	void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
	{

		/**************************************************************************************************
		***			Camera Position
		**************************************************************************************************/
		//setCameraHomePos(Pos(0, 40, 10),  Pos(0, 0, 0)); // viewing full scene from side
		setCameraHomePos(Pos(0, 5, 5),  Pos(0, 0, 0)); // normal
		//setCameraHomePos(Pos(0, 3, 3),  Pos(0, 0, 0)); // closer for screenshots
		//setCameraHomePos(Pos(0, 20, 20),  Pos(0, 0, 0));

		/**************************************************************************************************
		***			Simulation Parameters
		**************************************************************************************************/
		//1) - set noise to 0.1
		global.odeConfig.noise= 0.0;//0.02;//0.05;
		//2) - set controlinterval -> default = 1
		global.odeConfig.setParam("controlinterval", 1);/*update frequency of the simulation ~> amos = 20*/
		//3) - set simulation setp size
		global.odeConfig.setParam("simstepsize", 0.01); /*stepsize of the physical simulation (in seconds)*/
		//Update frequency of simulation = 1*0.01 = 0.01 s ; 100 Hz
		//4) - set gravity if not set then it uses -9.81 =earth gravity
		//global.odeConfig.setParam("gravity", -9.81);

		/**************************************************************************************************
		***			Set up Environment
		**************************************************************************************************/
		
		setup_Playground(global);
		Substance material(5.0,10.0,99.0,1.0);
		this->setGroundSubstance(material);

		/**************************************************************************************************
		***			Set up 3 pushable boxes and add the first one as graspable
		************************************************************************************************/

		generate_boxes(global);
		grippables.push_back(boxPrimitives[0]);
//		grippables.push_back(boxPrimitives[1]);
//		grippables.push_back(boxPrimitives[2]);		

		/**************************************************************************************************
		***			Set up robot and controller
		**************************************************************************************************/

		//1) Activate IR sensors
  		FourWheeledConfGripper fconf = FourWheeledRPosGripper::getDefaultConf();

		///2) relative sensors
		for (unsigned int i=0; i < relative_sensor_obst.size(); i++){
			fconf.rpos_sensor_references.push_back(relative_sensor_obst.at(i)->getMainPrimitive());
		}
		vehicle = new FourWheeledRPosGripper(odeHandle, osgHandle, fconf);

		/****Initial position of Nimm4******/
    	Pos pos(0.0 , 0.0 , 1.0);
    	//setting position and orientation
    	vehicle->place(osg::Matrix::rotate(-1.0, 0, 0, 1) *osg::Matrix::translate(pos));
		
		qcontroller = new ASLController("1","1");
		qcontroller->setGrippablesAndVehicle(vehicle,grippables);
		global.configs.push_back(qcontroller);

		// create pointer to one2onewiring
		AbstractWiring*  wiring = new One2OneWiring(new ColorUniformNoise(0.1));

		// create pointer to agent
		OdeAgent* agent = new OdeAgent(global);

		agent->init(qcontroller, vehicle, wiring);///////////// Initial controller!!!
		global.agents.push_back(agent);


	}

	/********************************************************************************************
	*** restart function
	*******************************************************************************************/
	virtual bool restart(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
	{

		std::cout << "\n begin restart " << currentCycle << "\n";

		std::cout<<"Current Cycle"<<this->currentCycle<<std::endl;

		// remove agents
		while (global.agents.size() > 0)
		{
			OdeAgent* agent = *global.agents.begin();

			AbstractController* controller = agent->getController();

			OdeRobot* robot = agent->getRobot();
			AbstractWiring* wiring = agent->getWiring();

			global.configs.erase(std::find(global.configs.begin(),
					global.configs.end(), controller));

			delete robot;
			delete wiring;

			global.agents.erase(global.agents.begin());

		}
		// clean the playgrounds
		while (global.obstacles.size() > 0)
		{
			std::vector<AbstractObstacle*>::iterator iter =
					global.obstacles.begin();
				delete (*iter);
			global.obstacles.erase(iter);
		}
		boxPrimitives.clear();
		relative_sensor_obst.clear();
		grippables.clear();

        ///////////////Recreate Robot Start//////////////////////////////////////////////////////////////////////////////////////
		/**************************************************************************************************
		***			Set up Environment
		**************************************************************************************************/
		setup_Playground(global);
	
		/**************************************************************************************************
		***			Set up 3 pushable boxes and add the first one as graspable
		************************************************************************************************/

		generate_boxes(global);
		grippables.push_back(boxPrimitives[0]);
//		grippables.push_back(boxPrimitives[1]);
//		grippables.push_back(boxPrimitives[2]);		

		/**************************************************************************************************
		***			Set up robot and controller
		**************************************************************************************************/

		//1) Activate IR sensors
  		FourWheeledConfGripper fconf = FourWheeledRPosGripper::getDefaultConf();

		///2) relative sensors
		for (unsigned int i=0; i < relative_sensor_obst.size(); i++){
			fconf.rpos_sensor_references.push_back(relative_sensor_obst.at(i)->getMainPrimitive());
		}
		vehicle = new FourWheeledRPosGripper(odeHandle, osgHandle, fconf);

		/****Initial position of Nimm4******/
    	Pos pos(0.0 , 0.0 , 1.0);
    	//setting position and orientation
    	vehicle->place(osg::Matrix::rotate(-1.0, 0, 0, 1) *osg::Matrix::translate(pos));
		
		// only set new grippables otherwise keep old controller
		qcontroller->setGrippablesAndVehicle(vehicle,grippables);
		global.configs.push_back(qcontroller);

		// create pointer to one2onewiring
		AbstractWiring*  wiring = new One2OneWiring(new ColorUniformNoise(0.1));

		// create pointer to agent
		OdeAgent* agent = new OdeAgent(global);

		agent->init(qcontroller, vehicle, wiring);///////////// Initial controller!!!
		global.agents.push_back(agent);


		std::cout << "\n end restart " << currentCycle << "\n";
		// restart!

		qcontroller->setReset(false);

		if (currentCycle < runs) return true;
		else return false;

	}
	
	/** optional additional callback function which is called every simulation step.
	      Called between physical simulation step and drawing.
	      @param draw indicates that objects are drawn in this timestep
	      @param pause always false (only called of simulation is running)
	      @param control indicates that robots have been controlled this timestep
	 */


	virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control)
	{
		//std::cout<<globalData.sim_step<<std::endl;
		//if (globalData.sim_step > 300) simulation_time_reached=true;
		simulation_time_reached = qcontroller->getReset();
		if (qcontroller->getDrop()) globalData.removeExpiredObjects(99999999); // expires gripped objects -> removes link and no longer gripped
	}



};


int main (int argc, char **argv)
{
	ThisSim sim;

	return sim.run(argc, argv) ? 0 : 1;



}
