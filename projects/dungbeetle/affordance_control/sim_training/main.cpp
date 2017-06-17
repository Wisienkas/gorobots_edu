
// include simulation environment stuff
#include <ode_robots/simulation.h>
// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>
// playground
#include <ode_robots/playground.h>
// simple wiring
#include <selforg/one2onewiring.h>
// the robot
#include "utils/sim_robots/dungbeetle/dungbeetle_laser.cpp"

#include <unistd.h>
// the controller
#include "controllers/dungbeetle/affordance_control/modularneurocontroller.cpp"
#include <ode_robots/joint.h>

#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>
#include <ode_robots/passivecapsule.h>

#include <selforg/abstractcontroller.h>
#include <ode_robots/color.h>
#include <iostream>
// #include <stdio.h>

//using namespace std;
using namespace cv;
using namespace lpzrobots;
std::vector<lpzrobots::AbstractObstacle*> obst;

bool track = true;
bool mCPGS = false;

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

	    //add obstacle
    double length = 1;
    double width = 0.2;
    double height = 0.2;
    bool obstacle_active = false;
    trainingFlag = true;
   lpzrobots::OdeHandle objHandle = odeHandle;
   objHandle.substance.toMetal(0.1);	

//SET OBJECT TO BE PUSHED
//0-CAPSULE
//1-BOX
//2-SPHERE
//---------------------
objectIndex = 1;
   switch(objectIndex){
				case 0:
				c1 = new PassiveCapsule(objHandle, osgHandle, 0.3,1.0,10);
    			c1->setColor(Color(1,0,0));
    			c1->setPose(osg::Matrix::rotate(/*-0.5*(M_PI/4)*/ M_PI/2, 1,0, 0) * osg::Matrix::translate(0.5, 0.0,0.3) /* pose*/);
    			global.obstacles.push_back(c1);
				break;
				case 1:
				b1 =new PassiveBox(objHandle, osgHandle, osg::Vec3(length, width, height/*size*/),0.1);
				b1->setColor(Color(1,0,0));
      			b1->setPose(osg::Matrix::rotate(/*-0.5*(M_PI/4)*/ (M_PI/2), 0,0, 1) * osg::Matrix::translate(0.35,0.0,0.1) /* pose*/);
      			global.obstacles.push_back(b1);
				break;

				case 2:
   				s1 = new lpzrobots::PassiveSphere(odeHandle, osgHandle, 0.3,1);
    			s1->setColor(Color(1,0,0));
    			s1->setPose(osg::Matrix::rotate(/*-0.5*(M_PI/4)*/ (M_PI/2), 0,0, 1) * osg::Matrix::translate(0.5, 0.0,0.3) /* pose*/);
    			global.obstacles.push_back(s1);
				break;
			}
   
		double steplength = 0.43;

		dungbeetleConf myDungBeetleConf = dungbeetle::getDefaultConf(1.0 /*_scale*/, 0 /*_useShoulder*/,
				1 /*_useFoot*/, 1 /*_useBack*/);
		myDungBeetleConf.rubberFeet = true;
		myDungBeetleConf.useLidar = true;


		lpzrobots::OdeHandle rodeHandle = odeHandle;
		rodeHandle.substance = lpzrobots::Substance(3.0, 0.0, 50.0, 0.8);

		dungBeetleRobot
		= new dungbeetle(rodeHandle, osgHandle.changeColor(lpzrobots::Color(1, 1, 1)), myDungBeetleConf, "dungBeetleRobot");

		// define the usage of the individual legs
		dungBeetleRobot->setLegPosUsage(dungBeetleRobot->L0, dungBeetleRobot->LEG);
		dungBeetleRobot->setLegPosUsage(dungBeetleRobot->L1, dungBeetleRobot->LEG);
		dungBeetleRobot->setLegPosUsage(dungBeetleRobot->L2, dungBeetleRobot->LEG);
		dungBeetleRobot->setLegPosUsage(dungBeetleRobot->R0, dungBeetleRobot->LEG);
		dungBeetleRobot->setLegPosUsage(dungBeetleRobot->R1, dungBeetleRobot->LEG);
		dungBeetleRobot->setLegPosUsage(dungBeetleRobot->R2, dungBeetleRobot->LEG);



		//

		// put dungBeetle a little bit in the air
		dungBeetleRobot->place(osg::Matrix::translate(.0, .0, 0) * osg::Matrix::rotate(0, 0, 0, 1));
		//dungBeetleRobot->place(osg::Matrix::translate(.0, .0, 0.0) * osg::Matrix::rotate(M_PI, 1, 0, 0));




		//controller = new dungBeetlecontrol(/*dungBeetle*/1,/*MCPGs=true*/false,/*Muscle Model =true*/false);
		controller = new modularNeuroController(1,mCPGS,false);
		
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
		namedWindow("laser",CV_WINDOW_AUTOSIZE);

		



	}


	 virtual bool command(const lpzrobots::OdeHandle&,
	      const lpzrobots::OsgHandle&,
	      lpzrobots::GlobalData& globalData,
	      int key,
	      bool down)
	  {
	    if (down) { // only when key is pressed, not when released
	    	getCommand(key);
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
        	//global.configs.clear();

        	// delete dungBeetleRobot;
        	// //delete (agent);
        	// global.agents.pop_back();

        	delete c1;
        	global.obstacles.pop_back();

        	lpzrobots::OdeHandle objHandle = odeHandle;
   			objHandle.substance.toMetal(0.1);	

        	c1 = new PassiveCapsule(objHandle, osgHandle, 0.3,1.0,10);
    		c1->setColor(Color(1,0,0));
   			c1->setPose(osg::Matrix::rotate(/*-0.5*(M_PI/4)*/ M_PI/2, 1,0, 0) * osg::Matrix::translate(0.5, 0.0,0.3) /* pose*/);
    		global.obstacles.push_back(c1);
    		usleep(1000);

   			




  //      // Add dungBeetle robot
		// dungbeetleConf myDungBeetleConf = dungbeetle::getDefaultConf(1.0 /*_scale*/, 0 /*_useShoulder*/,
		// 		1 /*_useFoot*/, 1 /*_useBack*/);
		// myDungBeetleConf.rubberFeet = true;

		// lpzrobots::OdeHandle rodeHandle = odeHandle;
		// rodeHandle.substance = lpzrobots::Substance(3.0, 0.0, 50.0, 0.8);



		// dungBeetleRobot = new dungbeetle(rodeHandle, osgHandle.changeColor(lpzrobots::Color(1, 1, 1)), myDungBeetleConf, "dungBeetle");

		// // define the usage of the individual legs
		// dungBeetleRobot->setLegPosUsage(dungBeetleRobot->L0, dungBeetleRobot->LEG);
		// dungBeetleRobot->setLegPosUsage(dungBeetleRobot->L1, dungBeetleRobot->LEG);
		// dungBeetleRobot->setLegPosUsage(dungBeetleRobot->L2, dungBeetleRobot->LEG);
		// dungBeetleRobot->setLegPosUsage(dungBeetleRobot->R0, dungBeetleRobot->LEG);
		// dungBeetleRobot->setLegPosUsage(dungBeetleRobot->R1, dungBeetleRobot->LEG);
		// dungBeetleRobot->setLegPosUsage(dungBeetleRobot->R2, dungBeetleRobot->LEG);

		// // put dungBeetle a little bit in the air
		// dungBeetleRobot->place(osg::Matrix::translate(.0, .0, 0) * osg::Matrix::rotate(0, 0, 0, 1));

		// robotfixator = new lpzrobots::FixedJoint(
		// dungBeetleRobot->getMainPrimitive(),
		// global.environment);
		// robotfixator->init(odeHandle, osgHandle, false);

  //      	 	//Create wiring
  //       	One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise());

  //       	// create agent and init it with controller, robot and wiring
  //       	lpzrobots::OdeAgent* agent = new OdeAgent(global);
  //       	agent->init(controller, dungBeetleRobot, wiring);

  //       	// Possibility to add tracking for robot
  //       	if (track) agent->setTrackOptions(TrackRobot(false, false, false, true, "", 60)); // Display trace
      // b1 = new PassiveBox(objHandle, osgHandle, osg::Vec3(1, 0.2, 0.2/*size*/),0.1);
      // b1->setColor(Color(1,0,0));
      // b1->setPose(osg::Matrix::rotate(/*-0.5*(M_PI/4)*/ (M_PI/2), 0,0, 1) * osg::Matrix::translate(distan,0.0,0.1) /* pose*/);
      // global.obstacles.push_back(b1);
        	// inform global variable over everything that happened:
       	 //global.configs.push_back(dungBeetleRobot);
       	 //global.agents.push_back(agent);
         //global.configs.push_back(controller);

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


			
		if(sim_flag){
			cout << "\n/------------------------------------------"<< counter <<"---------------------------------------------/\n"<<endl;
			sim_flag=false;
			pushed = false;
			cout << "\n/------------------------------------------------------------------------------------------------------/\n"<<endl;
			restart(odeHandle,osgHandle,globalData);
			// delete s1;
			// globalData.obstacles.pop_back();
			// s1 = new lpzrobots::PassiveSphere(odeHandle, osgHandle, 0.3,1);
   //  		s1->setColor(Color(1,0,0));
   //  		s1->setPose(osg::Matrix::rotate(/*-0.5*(M_PI/4)*/ (M_PI/2), 0,0, 1) * osg::Matrix::translate(distan, 0.0,0.3) /* pose*/);
   //  		globalData.obstacles.push_back(s1);

		}
	}

protected:
	lpzrobots::Joint* robotfixator;
	AbstractController* controller;
	dungbeetle* dungBeetleRobot;
	lpzrobots::PassiveSphere* s1;
	PassiveBox* b1;
	PassiveCapsule* c1;


};

int main(int argc, char **argv) {
	ThisSim sim;
	sim.setGroundTexture("Images/greenground.rgb");
	return sim.run(argc, argv) ? 0 : 1;
}

