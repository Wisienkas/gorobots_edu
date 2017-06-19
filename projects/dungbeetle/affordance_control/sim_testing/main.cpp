
// include simulation environment stuff
#include <ode_robots/simulation.h>
// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>
// playground
#include <ode_robots/playground.h>
// simple wiring
#include <selforg/one2onewiring.h>
// the robot
<<<<<<< HEAD
//#include <ode_robots/dungBeetle.h>


#include "utils/sim_robots/dungbeetle/dungbeetle_laser.h"
=======
#include "utils/sim_robots/dungbeetle/dungbeetle_laser.cpp"
>>>>>>> d7e6b105acaa0fa52730375fd20873511d4c69c5

// the controller
//#include "controllers/dungbeetle/Michelangelo/dung_beetle/dungBeetlecontrol.h"
#include "controllers/dungbeetle/manipulation_control/modular_neural_control_sphere.cpp"
#include "controllers/dungbeetle/manipulation_control/modular_neural_control_cylinder.cpp"
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
bool finished = false;
bool created = false;

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



		 srand (time(NULL));
		// // add playgrounds for three different experiments
		// lpzrobots::OdeHandle playgroundHandle = odeHandle;
		// playgroundHandle.substance = lpzrobots::Substance(100.0, 0.0, 50.0, 0.0); //substance for playgrounds (NON-SLIPPERY!!!)
		double steplength = 0.43;
<<<<<<< HEAD
		ob_rand =0;//rand() % 3;
=======
		ob_rand = rand() % 3;
>>>>>>> d7e6b105acaa0fa52730375fd20873511d4c69c5
	
		// // //PLAYGROUND
		// 	lpzrobots::Playground* playground = new lpzrobots::Playground(playgroundHandle, osgHandle, osg::Vec3(6, 1,
		// 			0.07), 1, false);
		// 	playground->setTexture(0, 0, lpzrobots::TextureDescr("Images/wall_bw.jpg", -0.5, -3));
		// 	playground->setPosition(osg::Vec3(0, 0, .0));
		// 	global.obstacles.push_back(playground);

		//PLAYGROUND

		// Add dungBeetle robot
		dungbeetleConf myDungBeetleConf = dungbeetle::getDefaultConf(1.0 /*_scale*/, 0 /*_useShoulder*/,
				1 /*_useFoot*/, 1 /*_useBack*/);
		myDungBeetleConf.rubberFeet = true;
<<<<<<< HEAD
=======
		myDungBeetleConf.useLidar = true;
>>>>>>> d7e6b105acaa0fa52730375fd20873511d4c69c5

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
		dungBeetleRobot->place(osg::Matrix::translate(.0, .0, 0) * osg::Matrix::rotate(M_PI, 0, 0, 1));
		//dungBeetleRobot->place(osg::Matrix::translate(.0, .0, 0.0) * osg::Matrix::rotate(M_PI, 1, 0, 0));




		//controller = new dungBeetlecontrol(/*dungBeetle*/1,/*MCPGs=utrue*/false,/*Muscle Model =true*/false);
		//controller = new Modular_neural_control_cylinder();//new modularNeuroController(1,mCPGS,false);

		One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise());

		// create agent and init it with controller, robot and wiring
		lpzrobots::OdeAgent* agent = new lpzrobots::OdeAgent(global);
<<<<<<< HEAD

=======
		trainingFlag = false;
>>>>>>> d7e6b105acaa0fa52730375fd20873511d4c69c5
		if(!created){
				lpzrobots::OdeHandle objectHandle = odeHandle;
    			objectHandle.substance = lpzrobots::Substance(3, 0.0, 50.0, 0.0);			
    			switch(ob_rand){
				case 0:
				cylinder_object = true;
				sphere_object = false;
				box_object = false;
				cylinder = new PassiveCapsule(objectHandle, osgHandle, 0.09,0.6,2);
				cylinder->setColor(Color(1,1,1));
    			cylinder->setPose(osg::Matrix::rotate(/*-0.5*(M_PI/4)*/ M_PI, 0,0, 1) *osg::Matrix::rotate(/*-0.5*(M_PI/4)*/ M_PI/2, 1,0, 0) * osg::Matrix::translate(-1.3, 0.0,0.09) /* pose*/);
     			fixator = new  lpzrobots::FixedJoint(cylinder->getMainPrimitive(), globalData.environment);
      			fixator->init(odeHandle, osgHandle,false);
      			globalData.obstacles.push_back(cylinder);
				

    //     		agent->init(cylinder_controller, dungBeetleRobot, wiring);
    //     		globalData.configs.push_back(cylinder_controller);
        		created = true;
				break;
				case 1:
				cylinder_object = false;
				sphere_object = false;
				box_object = true;

				box = new PassiveBox(objectHandle, osgHandle, osg::Vec3(1, 0.3, 0.3/*size*/),2);
				box->setColor(Color(1,1,1));
    			box->setPose(osg::Matrix::rotate(/*-0.5*(M_PI/4)*/ (M_PI/2), 0,0, 1) * osg::Matrix::translate(-1.3, 0.0,0.075) /* pose*/);
     			fixator = new  lpzrobots::FixedJoint(box->getMainPrimitive(), globalData.environment);
      			fixator->init(odeHandle, osgHandle,false);
      			globalData.obstacles.push_back(box);
				



         		created = true;
				break;

				case 2:
				cylinder_object = false;
				sphere_object = true;
				box_object = false;
				sphere = new lpzrobots::PassiveSphere(objectHandle, osgHandle, 0.18,2);
				sphere->setColor(Color(1,1,1));
    			sphere->setPose(osg::Matrix::rotate(/*-0.5*(M_PI/4)*/ (M_PI/2), 0,0, 1) * osg::Matrix::translate(-1.3, 0.0,0.18) /* pose*/);
     			fixator = new  lpzrobots::FixedJoint(sphere->getMainPrimitive(), globalData.environment);
      			fixator->init(odeHandle, osgHandle,false);
      			globalData.obstacles.push_back(sphere);
				

				
    //     		agent->init(sphere_controller, dungBeetleRobot, wiring);
    //     		globalData.configs.push_back(sphere_controller);
        		created = true;
				break;
			}
		}
		affordance = new modularNeuroController(1,mCPGS,false);

		agent->init(affordance, dungBeetleRobot, wiring);

<<<<<<< HEAD
		//put dung beetl ein the air

		 // robotfixator = new lpzrobots::FixedJoint(
		 //        dungBeetleRobot->getMainPrimitive(),
		 //        global.environment);
		 //    robotfixator->init(odeHandle, osgHandle, false);



		    std::cout << "\n\n"
		        << "################################\n"
		        << "#   Press x to free dungBeetle!    #\n"
		        << "################################\n"
		       << "\n\n" << std::endl;

		    //
=======
>>>>>>> d7e6b105acaa0fa52730375fd20873511d4c69c5

       // Possibility to add tracking for robot
		if (track)
			agent->setTrackOptions(TrackRobot(true, false, false, true, "", 60)); // Display trace

		// inform global variable over everything that happened:
		global.configs.push_back(dungBeetleRobot);
		global.agents.push_back(agent);
		//global.configs.push_back(controller);
		namedWindow("laser",CV_WINDOW_AUTOSIZE);

		



	}


<<<<<<< HEAD
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
=======
>>>>>>> d7e6b105acaa0fa52730375fd20873511d4c69c5


/**************************Reset Function***************************************************************/
    	virtual bool restart(const OdeHandle& odeHandle, const OsgHandle& osgHandle, lpzrobots::GlobalData& global)
   	{
       	 	//delete agent;


		//

		// put dungBeetle a little bit in the air
		
        	// //delete (agent);
        	
     
       	 	
 			One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise());
 			lpzrobots::OdeAgent* agent = new lpzrobots::OdeAgent(global);
       	 	// inform global variable over everything that happened:
        	
   			if(transporting){
   			delete dungBeetleRobot;
   		   	dungBeetleRobot = NULL;
   			 
   			  global.configs.clear();
   			  global.agents.pop_back();

   			  // Add dungBeetle robot
		dungbeetleConf myDungBeetleConf = dungbeetle::getDefaultConf(1.0 /*_scale*/, 0 /*_useShoulder*/,
				1 /*_useFoot*/, 1 /*_useBack*/);
		myDungBeetleConf.rubberFeet = true;

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

   		dungBeetleRobot->place(osg::Matrix::translate(-0.4, .0, 0) * osg::Matrix::rotate(0, 0, 0, 1));
   			switch((int)mode){
   				case 0:
   				cout << "BOX"<<endl;
   				cout << "HARD PUSHING"<<endl;
   				
				box_controller =  new Modular_neural_control_cylinder();//new modularNeuroController(1,mCPGS,false);
				box_controller->finished = false;
				box_controller->bump = true;
        		box_controller->stationary_push = false;
        		box_controller->boxing = false;
        		agent->init(box_controller, dungBeetleRobot, wiring);
        		global.agents.push_back(agent);
    			global.configs.push_back(box_controller);
    			transporting = false;
    			step2 = true;
    			break;
    			case 1:
    			cout << "CAPSULE"<<endl;
   				cout << "STATIONARY PUSHING"<<endl;
   				cylinder_controller =  new Modular_neural_control_cylinder();//new modularNeuroController(1,mCPGS,false);
				cylinder_controller->finished = false;
				cylinder_controller->bump = false;
<<<<<<< HEAD
        		cylinder_controller->stationary_push = true;
=======
        		cylinder_controller->stationary_push = false;
>>>>>>> d7e6b105acaa0fa52730375fd20873511d4c69c5
        		cylinder_controller->boxing = false;
   				agent->init(cylinder_controller, dungBeetleRobot, wiring);
   				global.agents.push_back(agent);
    			global.configs.push_back(cylinder_controller);
    			transporting = false;
    			step2 = true;
    			break;
    			case -1:
    			cout << "BALL"<<endl;
   				cout << "SOFT PUSHING"<<endl;
   				sphere_controller =  new Modular_neural_control_sphere();
				sphere_controller->finished = false;
				sphere_controller->bump = false;
        		sphere_controller->stationary_push = false;
        		sphere_controller->boxing = false;
   				agent->init(sphere_controller, dungBeetleRobot, wiring);
   				global.agents.push_back(agent);
    			global.configs.push_back(sphere_controller);
    			transporting = false;
    			step2 = true;
    			break;


   			}
   		}else if(finished)
   		{	 srand (time(NULL));
   			 counterpush =0;	
   		   	delete dungBeetleRobot;
   		   	dungBeetleRobot = NULL;
   			 
   			  global.configs.clear();
   			  global.agents.pop_back();

   			  // Add dungBeetle robot
		dungbeetleConf myDungBeetleConf = dungbeetle::getDefaultConf(1.0 /*_scale*/, 0 /*_useShoulder*/,
				1 /*_useFoot*/, 1 /*_useBack*/);
		myDungBeetleConf.rubberFeet = true;

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

   			
   			if(cylinder_object)
   			{
   				delete cylinder_controller;
   				delete cylinder;
   			}
   			else if(box_object)
   			{
   				delete box_controller;
   				delete box;
   			}
   			else if(sphere_object)
   			{
   				delete sphere_controller;
   				delete sphere;
   			}
   			globalData.obstacles.pop_back();
   			created = false;
   			detected = false;
   			turning = false;
   			finished = false;
   			started = false;
   			step2 = false;
   		dungBeetleRobot->place(osg::Matrix::translate(.0, .0, 0) * osg::Matrix::rotate(M_PI, 0, 0, 1));
   			if(!created){
				lpzrobots::OdeHandle objectHandle = odeHandle;
    			objectHandle.substance = lpzrobots::Substance(2, 0.0, 30.0, 0.2);
    			ob_rand = rand() % 3;
			switch(ob_rand){
				case 0:
				cylinder_object = true;
				sphere_object = false;
				box_object = false;
				cylinder = new PassiveCapsule(objectHandle, osgHandle, 0.09,0.6,2);
				cylinder->setColor(Color(1,1,1));
    			cylinder->setPose(osg::Matrix::rotate(/*-0.5*(M_PI/4)*/ M_PI/2, 1,0, 0) * osg::Matrix::translate(-1.3, 0.0,0.09) /* pose*/);
     			// fixator = new  lpzrobots::FixedJoint(cylinder->getMainPrimitive(), globalData.environment);
      		// 	fixator->init(odeHandle, osgHandle,false);
      			globalData.obstacles.push_back(cylinder);
				break;
				case 1:
				cylinder_object = false;
				sphere_object = false;
				box_object = true;

				box = new PassiveBox(objectHandle, osgHandle, osg::Vec3(1, 0.3, 0.3/*size*/),2);
				box->setColor(Color(1,1,1));
    			box->setPose(osg::Matrix::rotate(/*-0.5*(M_PI/4)*/ (M_PI/2), 0,0, 1) * osg::Matrix::translate(-1.3, 0.0,0.075) /* pose*/);
     			fixator = new  lpzrobots::FixedJoint(box->getMainPrimitive(), globalData.environment);
      			fixator->init(odeHandle, osgHandle,false);
      			globalData.obstacles.push_back(box);
         		created = true;
				break;

				case 2:
				cylinder_object = false;
				sphere_object = true;
				box_object = false;
				sphere = new lpzrobots::PassiveSphere(objectHandle, osgHandle, 0.18,2);
				sphere->setColor(Color(1,1,1));
    			sphere->setPose(osg::Matrix::rotate(/*-0.5*(M_PI/4)*/ (M_PI/2), 0,0, 1) * osg::Matrix::translate(-1.3, 0.0,0.18) /* pose*/);
     			fixator = new  lpzrobots::FixedJoint(sphere->getMainPrimitive(), globalData.environment);
      			fixator->init(odeHandle, osgHandle,false);
      			globalData.obstacles.push_back(sphere);
        		created = true;
				break;
			}
			agent->init(affordance, dungBeetleRobot, wiring);
		}
		
			global.configs.push_back(dungBeetleRobot);
			global.agents.push_back(agent);
	}
   			
   			
         

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

		if((transporting || finished))
			restart(odeHandle,osgHandle,globalData);

		 if (cylinder_object)
        {
            PassiveCapsule* c = dynamic_cast<PassiveCapsule*>(cylinder);
              float friction = 50;
              if(c)
              {
                Pos cvel = c->getMainPrimitive()->getVel();
                c->getMainPrimitive()->applyForce(-cvel*friction);
            }
        }

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


		 if (step2 && mode == 1)
        {	
            if (cylinder_controller->push && !started)
            {	
            	// counterpush++;
            	// if(counterpush == 20){
                //cylinder->setColor(lpzrobots::Color(1,0.2,0.2));
                delete fixator;
                fixator = NULL;
                started = true;
            	//}
            }
        }

        		 if (step2 && mode == -1)
        {
            if (sphere_controller->push && !started)
            {	
    
            	counterpush++;
            	if(counterpush == 20){
                //sphere->setColor(lpzrobots::Color(1,0.2,0.2));
                delete fixator;
                fixator = NULL;
                started = true;
            	}
            }
        }

        		 if (step2 && mode == 0)
        {
            if (box_controller->push && !started)
            {	
            	counterpush++;
            	if(counterpush == 20){
                //box->setColor(lpzrobots::Color(1,0.2,0.2));
                delete fixator;
                fixator = NULL;
                started = true;
            	}
            }
        }

        if (sphere_object)
        {
            Position sphere_pos = sphere->getPosition();

            if (sphere_pos.x < /*-0.72*/ -2.1 && !finished)
            {
                sphere->setColor(lpzrobots::Color(0.2,1,0.2));
                finished = true;
                //controller_sphere->finished = true;
            }
        }
        else if (cylinder_object)
        {
            Position cylinder_pos = cylinder->getPosition();

            if (cylinder_pos.x < /*-0.72*/ -2.1 && !finished)
            {
                cylinder->setColor(lpzrobots::Color(0.2,1,0.2));
                finished = true;

                //controller_cylinder->finished = true;
            }
        }
        else if(box_object)
        {
            Position box_pos = box->getPosition();

            if (box_pos.x < /*-0.72*/ -2.1 && !finished)
            {
                box->setColor(lpzrobots::Color(0.2,1,0.2));
                finished = true;
                
            }
        }	


			//	cout << "\n/------------------------------------------"<< counter <<"---------------------------------------------/\n"<<endl;
		// if(sim_flag && distan < 1.6){
			
		// 	sim_flag=false;
		// 	distan += 0.1;
		// 	//pushed = false;
		// 	cout << "\n/-------------------------------------------"<<distan<<"--------------------------------------------/\n"<<endl;
		// 	restart(odeHandle,osgHandle,globalData);

		// }
	}

protected:
	lpzrobots::Joint* robotfixator;
	std::vector<lpzrobots::PassiveBox*> terrain;
	std::vector<lpzrobots::Joint*> terrainfixator;
	AbstractController* affordance;
	Modular_neural_control_cylinder* cylinder_controller,*box_controller;
	Modular_neural_control_sphere* sphere_controller;
	dungbeetle* dungBeetleRobot;
	lpzrobots::PassiveSphere* sphere;
	lpzrobots::FixedJoint* fixator;
	PassiveBox* box;
	PassiveCapsule* cylinder;
	bool step2 = false;
	bool started = false;
	int counterpush=0;
	int ob_rand;
};

int main(int argc, char **argv) {
	ThisSim sim;
	sim.setGroundTexture("Images/greenground.rgb");
	return sim.run(argc, argv) ? 0 : 1;
}

