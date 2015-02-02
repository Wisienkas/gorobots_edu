
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
#include <fourwheeledrpos.h>


#include <selforg/trackrobots.h>
// used arena
#include <ode_robots/playground.h>
// used passive spheres
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>
#include <ode_robots/complexplayground.h>
#include <ode_robots/terrainground.h>



// used controller
//#include <selforg/invertnchannelcontroller.h>
//#include <selforg/invertmotorspace.h>
//#include <selforg/sinecontroller.h>
//#include "randomcontroller.h"
//#include "openinvertnchannelcontroller.h"
//#include "qlearninghomeokmcontroller.h"
//#include "qlearningmcontroller.h"

//#include "acicocontroller.h" // Two separate angle inputs
//#include "acicocontrollerv2.h" // One angle input BEST
//#include "acicocontrollerv3.h" // One angle input and one angle_dot input BAD
//#include "acicocontrollerv4.h" // One angle input and one distance sensor
//#include "acicocontrollerv5.h" // One angle input and one distance sensor 4 synapses
//#include "acicocontrollerv6.h" // One angle input and one distance sensor 4 synapses Stable version Correct TD error!
//#include "acicocontrollerv7.h" // One angle input and one distance and one IR sensor 6 synapses
//#include "acicocontrollerv8.h" // One angle input and one distance and one IR sensor 6 synapses
//#include "acicocontrollerv9.h" // **WORK_ AC with obstacles_FixedIRweights: One angle input and one distance and one IR sensor 6 synapses separated linear controllers
//#include "acicocontrollerv10.h" // Pure AC with Obstacles Change all weights: One angle input and one distance and one IR sensor 6 synapses separated linear controllers
//#include "acicocontrollerv11.h" // ICO + AC with Obstacles Change all weights: One angle input and one distance and one IR sensor 6 synapses separated linear controllers
//#include "acicocontrollerv12.h" // ICO goal directed steering angle
//#include "acicocontrollerv13.h" // ICO + AC 2 goals MAX 4 goals no obstacle
#include <controllers/nimm4ii/acicorccriticcontroller.h> // ICO + AC 3 goals no obstacle


// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

//OpenInvertNChannelController * ocontroller;

//select
//QLearningHomeokMController* qcontroller;
//QLearningMController* qcontroller;

//****Setup of ACcontrol*****//
//ACICOController* qcontroller;
//ACICOControllerV2* qcontroller; //The best one for goal oriented navigation, one input = theta
//ACICOControllerV3* qcontroller;
//ACICOControllerV4* qcontroller;
//ACICOControllerV5* qcontroller;
//ACICOControllerV6* qcontroller;
//ACICOControllerV7* qcontroller;
//ACICOControllerV8* qcontroller; // Stable version All sensors without obstacles
//ACICOControllerV9* qcontroller; // Stable version All sensors with obstacles and Fixed IR weights
//ACICOControllerV10* qcontroller;
//ACICOControllerV11* qcontroller;
//ACICOControllerV12* qcontroller;
//ACICOControllerV13* qcontroller;
AcIcoRcCriticController* qcontroller;

bool tmaze_on = false;

bool obstacle_on = true; //true;//true;
bool drawtrace_on = true;//true;//---------------------------------------------------------------------------TEST

bool random_positon_spheres = false;
bool random_positon = false;//true;
bool random_positon_frist = false;//-------------------------------------------------CH
bool random_orientation = true;

int shiftObstacle = 0;

#define MAX_x   11.0//10.0 //11.0//9.0	//Max range-------------------------------------------------CH VERY IMPORTANT FACTOR
#define MIN_x   4.0//4.0  //5.0 (TEST) //Min range-------------------------------------------------CH VERY IMPORTANT FACTOR

#define MAX_y   2.5//5.0 // 9.0 	//Max range
#define MIN_y   -2.5//-5.0//-9.0 	//Min range

int pos_obstacle_x = 7.0; //5.0; //7.0; //7
int pos_obstacle_y = 0.0;//5.0; //0; //7.0;//0; //0 //-7.0 // 0

#define MAX_or	M_PI/3 //45 // 60 deg
#define MIN_or	-M_PI/3  //45 // 60 deg

double random_or;
double random_position_S;
double random_position_S_x;
double random_position_S_y;
double random_position;
double random_positionx;
double random_positiony;


int number_spheres = 4; // SET NUMBER of TARGET MAX = 4
int number_boxes = 1 ;//2; // SET NUMBER of BOXES obstacles

int global_counter = 1;

bool repeat_experiment = true;//true; //if select true then set the follwing parameters
int repeat_number = 500; //500;// 1000; for 200 repetitions of the program
double time_factor = 0.25 ; //0.25;//0.25/2;

//0.25/4 = 7.5s
//0.25/2 = 15 s
// 0.25 = 30 s
// 0.5 = 1 min for each run,
// 1 = 2 mins for each run,
//1.5 = 3 mins for each run

double position_S = 0.0;
int position_x = 20;

bool delete_controller = false; //false;// false if wants to repeat experiments but don't want to reset controller//DON'T delete controller if DON't want to reset controller!------------------------------------------------------(FRANK)


//***************************//

bool exploration_active = false;
Joint* fixator2;


std::vector<AbstractObstacle*> obst;
std::vector<FixedJoint*> fixator;



class ThisSim : public Simulation {
public:

	// starting function (executed once at the beginning of the simulation loop)
	void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
	{

		// set initial camera position
		setCameraHomePos(Pos(-1.14383, 10.1945, 42.7865),  Pos(179.991, -77.6244, 0));


		// initialization simulation parameters

		//1) - set noise to 0.1
		global.odeConfig.noise= 0.05;//0.05;//0.05;//0.05;

		//2) - set controlinterval -> default = 1
		global.odeConfig.setParam("controlinterval", 1);/*update frequency of the simulation ~> amos = 20*/
		//3) - set simulation setp size
		global.odeConfig.setParam("simstepsize", 0.01); /*stepsize of the physical simulation (in seconds)*/
		//Update frequency of simulation = 1*0.01 = 0.01 s ; 100 Hz

		//4) - set gravity if not set then it uses -9.81 =earth gravity
		//global.odeConfig.setParam("gravity", -9.81);


		//5) - set Playground as boundary:
		// - create pointer to playground (odeHandle contains things like world and space the
		//   playground should be created in; odeHandle is generated in simulation.cpp)
		// - setting geometry for each wall of playground:
		//   setGeometry(double length, double width, double	height)
		// - setting initial position of the playground: setPosition(double x, double y, double z)
		// - push playground in the global list of obstacles(globla list comes from simulation.cpp)

		// odeHandle and osgHandle are global references
		// vec3 == length, width, height


		double length_pg = 35;//28;//0.0; //45, 32, 22
		double width_pg = 0.5;//0.0;  //0.2
		double height_pg = 0.5;//0.0; //0.5

		Playground* playground = new Playground(odeHandle, osgHandle.changeColor(Color(0.6,0.0,0.6)),
				osg::Vec3(length_pg /*length*/, width_pg /*width*/, height_pg/*height*/), /*factorxy = 1*/1, /*createGround=true*/true /*false*/);
		playground->setPosition(osg::Vec3(4,0,0.0)); // playground positionieren und generieren
		// register playground in obstacles list


		global.obstacles.push_back(playground);






		//6) - add passive spheres as TARGET!
		// - create pointer to sphere (with odehandle, osghandle and
		//   optional parameters radius and mass,where the latter is not used here) )
		// - set Pose(Position) of sphere
		// - set a texture for the sphere
		// - add sphere to list of obstacles

		PassiveSphere* s1;


		for (int i=0; i < number_spheres; i++){
			s1 = new PassiveSphere(odeHandle, osgHandle, 0.5);

			//Target 1
			if(random_positon_spheres)
			{
				/*******Generating Random Position****/
				random_position_S  = ((MAX_x-MIN_x)*((float)rand()/RAND_MAX))+MIN_x; // Input 0 (m) between -2.4 and 2.4
				std::cout<<"\n\n\n\n\n"<<"Inital Random Target X position"<<" = "<<-1*random_position_S<<"\t"<<"Inital Random Target Y position"<<" = "<<random_position_S<<"\n\n\n\n";
				/************************************/
				if (i==0) s1->setPosition(osg::Vec3(-1*random_position_S,random_position_S,0/*2*/));
			}
			else
			{
				//position_S = 0.0;// -2.0---------------------------------------------------------------------------TEST
				if (i==0) s1->setPosition(osg::Vec3(-10.0 /*position_S*/, position_S, 0.5/*0*/));



			}

			//Target 2

			//if (i==1) s1->setPosition(osg::Vec3(15-11.0 /*position_S*/, position_S+10, 0.5/*0*/));

			if (i==1 && tmaze_on == false) s1->setPosition(osg::Vec3(0.0 /*position_S*/, position_S+5, 0.5/*0*/)); //Green Sphere

			if (i==1 && tmaze_on == true) s1->setPosition(osg::Vec3(-3.0 /*position_S*/, position_S+10, 0.5/*0*/)); //Green Sphere


			//if (i==1) s1->setPosition(osg::Vec3(0.0 /*position_S*/, position_S-5, 0.5/*0*/)); //Green Sphere

			//Target 3
			//if (i==2) s1->setPosition(osg::Vec3(15-11.0 /*position_S*/, position_S-10, 0.5/*0*/));

			if (i==2 && tmaze_on == false) s1->setPosition(osg::Vec3(0.0 /*position_S*/, position_S-5, 0.5/*0*/)); //Blue sphere

			if (i==2 && tmaze_on == true) s1->setPosition(osg::Vec3(-3.0 /*position_S*/, position_S-10, 0.5/*0*/)); //Blue sphere

			//if (i==2) s1->setPosition(osg::Vec3(22.0 /*position_S*/, position_S+15, 0.5/*0*/)); //Blue sphere


			//Target 4
			if (i==3) s1->setPosition(osg::Vec3(-10.0 /*position_S*/, position_S, 0.5/*0*/));
			//if (i==3) s1->setPosition(osg::Vec3(-10, 10,2/*2*/));

			s1->setTexture("Images/dusty.rgb"/*dusty.rgb*/);
			//Target 1
			if (i==0){ s1->setColor(Color(1,0,0)); }
			//Target 2
			if (i==1){ s1->setColor(Color(0,1,0)); }
			//Target 3
			if (i==2){ s1->setColor(Color(0,0,1)); }
			//Target 4
			if (i==3){ s1->setColor(Color(1,1,0)); }
			obst.push_back(s1);
			global.obstacles.push_back(s1);
			// fix sphere (in actual position) to simulation
			//fixator.push_back(new  FixedJoint(s1->getMainPrimitive(), global.environment));  //create pointer
			//fixator.at(i)->init(odeHandle, osgHandle);
			fixator2 = new  FixedJoint(s1->getMainPrimitive(), global.environment);
			fixator2->init(odeHandle, osgHandle);
		}




		//		int number_spheres = 4;
		//		PassiveSphere* s1;
		//		for (int i=0; i < number_spheres; i++){
		//			s1 = new PassiveSphere(odeHandle, osgHandle, 0.5);
		//			if (i==0) s1->setPosition(osg::Vec3(-10,-10,2));
		//			if (i==1) s1->setPosition(osg::Vec3( 10, 10,2));
		//			if (i==2) s1->setPosition(osg::Vec3( 10, -10,2));
		//			if (i==3) s1->setPosition(osg::Vec3(-10, 10,2));
		//
		//			s1->setTexture("Images/dusty.rgb");
		//			if (i==0){ s1->setColor(Color(1,0,0)); }
		//			if (i==1){ s1->setColor(Color(0,1,0)); }
		//			if (i==2){ s1->setColor(Color(0,0,1)); }
		//			if (i==3){ s1->setColor(Color(1,1,0)); }
		//			obst.push_back(s1);
		//			global.obstacles.push_back(s1);
		//			// fix sphere (in actual position) to simulation
		//			fixator.push_back(  new  FixedJoint(s1->getMainPrimitive(), global.environment));  //create pointer
		//			fixator.at(i)->init(odeHandle, osgHandle);
		//		}

		//7) - add passive box as OBSTACLES
		// - create pointer to Box (with odehandle, osghandle and
		//   optional parameters radius and mass,where the latter is not used here) )
		// - set Pose(Position) of sphere
		// - set a texture for the sphere
		// - add sphere to list of obstacles




		PassiveBox* b1;
		double length = 6.5; //6.5 ; //20.0;//20.0; //4.5; //6.0 ; //4.5 /*for learning*/; //3.5 /*for*/;//testing//---------------------------------------------------------------------------TEST
		double width = 1.0;
		double height = 1.0;


		if(obstacle_on)
		{
			random_position_S = 0.0;

						//for (int i=0; i < number_boxes /*SET NUMBER OF OBSTACLES*/; i++){
							b1 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(length, width, height /*size*/));
							//b1->setTexture("dusty.rgb");
							b1->setColor(Color(1,0,0));
							//b1->setPosition(osg::Vec3(/*-4.5+*/i*4.5,3+i,0)); // Fixed robot position
							//osg::Matrix pose;
							//			pose.setTrans(osg::Vec3(/*-4.5+*/i*4.5,3+i,0));
							//b1->setPose(osg::Matrix::rotate(0.5*(M_PI/2), 0,0, 1) * osg::Matrix::translate(/*-4.5+*/i*4.5,3+i,0.5) /* pose*/);


							b1->setPose(osg::Matrix::rotate(/*-0.5*(M_PI/4)*/ (M_PI/2), position_S /*0*/,0,1) * osg::Matrix::translate(/*i*4.5+*/pos_obstacle_x, pos_obstacle_y+random_position_S/*i*-pos_obstacle_y*2.0*//*0+i*/,height/2) /* pose*/);

							random_position_S  = ((MAX_x-MIN_x)*((float)rand()/RAND_MAX))+MIN_x;
							global.obstacles.push_back(b1);
							fixator.push_back(  new  FixedJoint(b1->getMainPrimitive(), global.environment));  //create pointer
							//fixator.at(i)->init(odeHandle, osgHandle);
							fixator.at(0)->init(odeHandle, osgHandle);

		if(number_boxes == 2){

			b1 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(3.0, width, height /*size*/));
											//b1->setTexture("dusty.rgb");
			b1->setColor(Color(1,0,0));
			b1->setPose(osg::Matrix::rotate(/*-0.5*(M_PI/4)*/ (M_PI/2), 0 /*0*/,0,1) * osg::Matrix::translate(2,5,0.5) /* pose*/);
     		global.obstacles.push_back(b1);
			fixator.push_back(  new  FixedJoint(b1->getMainPrimitive(), global.environment));  //create pointer
															//fixator.at(i)->init(odeHandle, osgHandle);
			fixator.at(1)->init(odeHandle, osgHandle);

		}

		if(number_boxes >= 4) {
			int k = 0;
			while(k<=number_boxes)
			{
				b1 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(3.0, width, height /*size*/));
				b1->setColor(Color(1,0,0));
				random_position_S_x  = ((MAX_x-MIN_x)*((float)rand()/RAND_MAX))+MIN_x;
				random_position_S_y  = ((MAX_y-MIN_y)*((float)rand()/RAND_MAX))+MIN_y;
				b1->setPose(osg::Matrix::rotate(/*-0.5*(M_PI/4)*/ (M_PI/2), random_position_S_x /*0*/,0,1) * osg::Matrix::translate(/*i*4.5+*/random_position_S_x, random_position_S_y/*i*-pos_obstacle_y*2.0*//*0+i*/,height/2) /* pose*/);

				global.obstacles.push_back(b1);
											fixator.push_back(  new  FixedJoint(b1->getMainPrimitive(), global.environment));  //create pointer
											//fixator.at(i)->init(odeHandle, osgHandle);
											fixator.at(0)->init(odeHandle, osgHandle);

											k++;
			}


		}


		}





		// use Nimm2 vehicle as robot:
		// - get default configuration for nimm2
		// - activate bumpers, cigar mode and infrared front sensors of the nimm2 robot
		// - create pointer to nimm2 (with odeHandle, osg Handle and configuration)
		// - place robot
		//     Nimm2Conf c = Nimm2::getDefaultConf();
		//     c.force   = 4;
		//     c.bumper  = true;
		//     c.cigarMode  = true;
		//     // c.irFront = true;
		//     OdeRobot* vehicle = new Nimm2(odeHandle, osgHandle, c, "Nimm2");
		//     vehicle->place(Pos(0,0,0));


		////////////////////////////////////Call this set up and Control//////////////////////////////////////////////
		bool ac_ico_robot = true;
		if (ac_ico_robot)
		{

			//0)

			//qcontroller->setReset(1);

			//1) Activate IR sensors
		  FourWheeledConf fconf =FourWheeledRPos::getDefaultConf();

			///2) relative sensors
			for (int i=0; i < number_spheres; i++){
				fconf.rpos_sensor_references.push_back(obst.at(i)->getMainPrimitive());
			}

			OdeRobot* vehicle3 = new FourWheeledRPos(odeHandle, osgHandle, fconf);

			//deprecated: ((Nimm4*)vehicle3)->setBodyColor(Color(0.0,1.0,0.0));
			//deprecated: ((Nimm4*)vehicle3)->setBodyTexture("../../../../osg/data/Images/wood.rgb");
			//deprecated: ((Nimm4*)vehicle3)->setWheelTexture("../../../../osg/data/Images/chess.rgb");

			//((Nimm4*)vehicle3)->setRelPosSensorReference(global.obstacles.back()->getMainPrimitive());


			/****Initial position of Nimm4******/
			//Generate random position between x = +-10, y = +-10
			//Position x = +-10
			//Position y = +-10
			//vehicle3->place(Pos(0/*x*/,0/*y*/,0/*z*/));

			if(random_positon_frist)
			{
				/*******Generating Random Position****/
				//srand (time(NULL));
				random_position  = ((MAX_x-MIN_x)*((float)rand()/RAND_MAX))+MIN_x; // Input 0 (m) between -2.4 and 2.4
				/************************************/
				do
				{
					int r = rand();
					std::cout << "random value 1: " << r << std::endl;
					random_position  = ((MAX_x-MIN_x)*((float)r/RAND_MAX))+MIN_x;
					//std::cout<<"\n\n\n\n\n"<<"Inital Random_Robot X position"<<" = "<<random_position<<"\t"<<"Inital Random_Robot Y position"<<" = "<<-1*random_position<<"\n\n\n\n";

				}while(abs(abs(position_S) - abs(random_position)) <= 4);
				//while(abs(abs(random_position_S) - abs(random_position)) <= 4);

				std::cout<<"\n\n"<<"Inital Random X position"<<" = "<<random_position<<"\t"<<"Inital Random Y position"<<" = "<<-1*random_position<<"\n\n";
				//vehicle3->place(Pos(position_x-10.0 /* 20 random_position*/  /*+x = to left (sphere in front of robot), -x = to right sphere behind robot*/, 0.0 /*-1*random_position*/ /*+y = to up, -y = to down*/,0.0/*z*/));
				vehicle3->place(Pos(position_x-15.0 /* 20 random_position*/  /*+x = to left (sphere in front of robot), -x = to right sphere behind robot*/, 0.0 /*-1*random_position*/ /*+y = to up, -y = to down*/,0.0/*z*/));
			}
			else
			{
				//Pos pos(position_x-5.0/*5.5x, +x = to left, -x = to right*/,0.0/*y*/,0.0/*z*/);

				//setting position and orientation
				//vehicle3->place(osg::Matrix::rotate(0.0, 0, 0, 1) *osg::Matrix::translate(pos));

				//setting only position
				//vehicle3->place(Pos(position_x-5.0/*5.5x, +x = to left, -x = to right*/,0.0/*y*/,0.0/*z*/));



				/*******Generating Random Position****/
				//srand (time(NULL));
				int r = rand();
				std::cout << "random value 2: " << r << std::endl;
				random_or  = ((MAX_or-MIN_or)*((float)r/RAND_MAX))+MIN_or; // between -pi/3 (60 deg) and pi/3 (60 deg)
				/************************************/

				Pos pos(position_x-5.0/*, +x = to left, -x = to right*/,0.0/*y*/,0.0/*z*/);

				//Pos pos(position_x/*, +x = to left, -x = to right*/,0.0/*y*/,0.0/*z*/);

			//	Pos pos(position_x-5.0/*, +x = to left, -x = to right*/,-2.0/*y*/,0.0/*z*/);
				std::cout << "random_or1: " << random_or << std::endl;

				//setting position and orientation
				vehicle3->place(osg::Matrix::rotate(random_or, 0, 0, 1) *osg::Matrix::translate(pos));
			//	vehicle3->place(osg::Matrix::rotate(1, 0, 0, -1) *osg::Matrix::translate(pos));
			//	vehicle3->place(osg::Matrix::rotate(0, 0, 0, -1) *osg::Matrix::translate(pos));
				//setting only position
				//vehicle3->place(Pos(position_x-5.0/*5.5x, +x = to left, -x = to right*/,0.0/*y*/,0.0/*z*/));


			}


			//AC Lernen
			//qcontroller = new ACICOController(); ///////////////////// call this function NEW controller!!
			//qcontroller = new ACICOControllerV2(); ///////////////////// call this function NEW controller!!
			//qcontroller = new ACICOControllerV3(); ///////////////////// call this function NEW controller!!
			//qcontroller = new ACICOControllerV4(); ///////////////////// call this function NEW controller!!
			//qcontroller = new ACICOControllerV5(); ///////////////////// call this function NEW controller!!
			//qcontroller = new ACICOControllerV6(); ///////////////////// call this function NEW controller!!
			//qcontroller = new ACICOControllerV7(); ///////////////////// call this function NEW controller!!
			//qcontroller = new ACICOControllerV8(); ///////////////////// call this function NEW controller!!
			//qcontroller = new ACICOControllerV9(); ///////////////////// call this function NEW controller!!
			//qcontroller = new ACICOControllerV10(); ///////////////////// call this function NEW controller!!

			//WORK AC-ICO
			//qcontroller = new ACICOControllerV11(); ///////////////////// call this function NEW controller!!
			//qcontroller = new ACICOControllerV12(); ///////////////////// call this function NEW controller!!
			//qcontroller = new ACICOControllerV13(); ///////////////////// call this function NEW controller!!
			qcontroller = new AcIcoRcCriticController(); ///////////////////// call this function NEW controller!!


			//((ACICOControllerV12*)qcontroller)->setCurrentCycle(currentCycle);
			//((ACICOControllerV13*)qcontroller)->setCurrentCycle(currentCycle);
			((AcIcoRcCriticController*)qcontroller)->setCurrentCycle(currentCycle);
			qcontroller->position(position_x); //send the position

			//Homeo + Q
			//qcontroller = new QLearningHomeokMController();
			//Q Lernen
			//qcontroller = new QLearningMController();

			qcontroller->setParam("eps", 0.1);
			global.configs.push_back(qcontroller);

			// create pointer to one2onewiring
			AbstractWiring*  wiring3 = new One2OneWiring(new ColorUniformNoise(0.1));

			// create pointer to agent

			plotoptions.push_back(PlotOption(NoPlot /*select "File" to save signals, "NoPlot" to not save*/));
			OdeAgent* agent3 = new OdeAgent(plotoptions);



			agent3->init(qcontroller, vehicle3, wiring3);///////////// Initial controller!!!
			global.agents.push_back(agent3);

			if(drawtrace_on)
						{
							TrackRobot* track3 = new TrackRobot(/*bool trackPos*/true,
									/*bool trackSpeed*/false,
									/*bool trackOrientation*/false,
									/*bool displayTrace*/true
							/*const char *scene="", int interval=1*/,"",60);
							agent3->setTrackOptions(*track3);
						}

		}

		////////////////////////////////////Another set up and Control//////////////////////////////////////////////
		bool random_controlled_robot =false;
//		if (random_controlled_robot){
//			// robot 1
//			OdeRobot* vehicle = new Nimm4(odeHandle, osgHandle, "Nimm4");
//			//Deprecated:
//			//((Nimm4*)vehicle)->setBodyColor(Color(0.0,0.0,1.0));
//			//((Nimm4*)vehicle)->setBodyTexture("../../../../osg/data/Images/wood.rgb");
//			//((Nimm4*)vehicle)->setWheelTexture("../../../../osg/data/Images/chess.rgb");
//			vehicle->place(Pos(0,1,0));
//
//			// create pointer to controller
//			AbstractController *controller = new RandomController();
//			global.configs.push_back(controller);
//
//			// create pointer to one2onewiring
//			One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
//
//			// create pointer to agent
//			OdeAgent* agent = new OdeAgent(plotoptions);
//			agent->init(controller, vehicle, wiring);
//			// show robot´s trace in simulation
//			TrackRobot* track = new TrackRobot(/*bool trackPos*/false,
//					/*bool trackSpeed*/false,
//					/*bool trackOrientation*/false,
//					/*bool displayTrace*/false//,
//			/*const char *scene="", int interval=1*/);
//			agent->setTrackOptions(*track);
//			global.agents.push_back(agent);
//		}

		////////////////////////////////////Another set up and Control//////////////////////////////////////////////
//		bool manually_steered_robot = false;
//		if (manually_steered_robot){
//			// 2nd robot
//			OdeRobot* vehicle2 = new Nimm4(odeHandle, osgHandle, "Nimm4",
//					/*double size=*/1.0, /*double force =3*/1.5, /*double speed=*/15,
//					/*bool sphereWheels =*/true);
//
//      //Deprecated
//			//((Nimm4*)vehicle2)->setBodyColor(Color(1.0,0.0,0.0));
//			//((Nimm4*)vehicle2)->setBodyTexture("../../../../osg/data/Images/wood.rgb");
//			//((Nimm4*)vehicle2)->setWheelTexture("../../../../osg/data/Images/chess.rgb");
//			vehicle2->place(Pos(1,1,0));
//			OpenInvertNChannelController ocontroller = new OpenInvertNChannelController(10);
//			ocontroller->setParam("eps", 0.1);
//			ocontroller->setBiasUpdateRule(no); //bias is always zero
//			global.configs.push_back(ocontroller);
//
//			// create pointer to one2onewiring
//			AbstractWiring*  wiring2 = new One2OneWiring(new ColorUniformNoise(0.1));
//
//			// create pointer to agent
//			OdeAgent* agent2 = new OdeAgent(plotoptions);
//			TrackRobot* track2 = new TrackRobot(/*bool trackPos*/false,
//					/*bool trackSpeed*/false,
//					/*bool trackOrientation*/false,
//					/*bool displayTrace*/true //,
//			/*const char *scene="", int interval=1*/);
//			agent2->setTrackOptions(*track2);
//			agent2->init(ocontroller, vehicle2, wiring2);
//			global.agents.push_back(agent2);
//		}

		showParams(global.configs);
	}

	//  add own key handling stuff here, just insert some case values
	virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
	{
		if (down) { // only when key is pressed, not when released
			std::vector<double> f;
			switch ( (char) key )
			{

			 case 'f':
				//				qcontroller-> learn=false;
				//				std::cout <<"learning deactivated"<< std::endl;
				break;
			case 't':
				//				qcontroller->homeokinetic_controller -> setBiasUpdateRule(org);
				break;
				//			case 'x':
				//				if(fixator) delete fixator;
				//				fixator=0;
				//				break;
				//
			case 'x':
				std::cout<<"dropping spheres"<< std::endl;
				for (int i=0; i<fixator.size(); i++){
					delete fixator.at(i);
				}
				fixator.clear();
				break;
//			case 'l': // turn left
//				std::cout <<"turn left"<< std::endl;
//				f= ocontroller->getMotorCommandFactor();
//				f.at(0)-=0.2;
//				f.at(2)-=0.2;
//				ocontroller->setMotorCommandFactor(f);
//				std::cout<<"f= "<<f.at(0)<<"  "<<f.at(1)<<"  "<<f.at(2)<<"  "<<f.at(3)<<"  "<<std::endl;
//				break;
//			case 'r': // turn right
//				f = ocontroller->getMotorCommandFactor();
//				f.at(1)-=0.2;
//				f.at(3)-=0.2;
//				ocontroller->setMotorCommandFactor(f);
//				std::cout<<"f= "<<f.at(0)<<"  "<<f.at(1)<<"  "<<f.at(2)<<"  "<<f.at(3)<<"  "<<std::endl;
//				break;
//			case 'n': // turn right
//				f = ocontroller->getMotorCommandFactor();
//				for (int i=0; i<4; i++){
//					f.at(i)=1;
//				}
//				std::cout<<"f= "<<f.at(0)<<"  "<<f.at(1)<<"  "<<f.at(2)<<"  "<<f.at(3)<<"  "<<std::endl;
//				ocontroller->setMotorCommandFactor(f);
//				break;
//			case 's': // activate steering control
//				//qcontroller->setProgrammedSteeringControl(true);
//				qcontroller->setLearnedSteeringControl(true);
//				std::cout<<"steering activated"<<std::endl;
//				break;
//			case 'd': // deactivate steering control
//				//qcontroller->setProgrammedSteeringControl(false);
//				qcontroller->setLearnedSteeringControl(false);
//				std::cout<<"steering deactivated"<<std::endl;
//				break;
			case 'q': // print Q-table
				qcontroller->printQTable();
				break;
			case 'e': // toggle exploration (activate/deactivate)
				if (exploration_active){
					exploration_active = false;
					std::cout<<"deactivating exploration"<<std::endl;
				} else {
					exploration_active = true;
					std::cout<<"activating exploration"<<std::endl;
				}
				qcontroller->setExplorationActive(exploration_active);
				break;
			case 'u': // move forward
				qcontroller->setMC(0.6, 0.6);
				break;
			case 'j': // move backward
				qcontroller->setMC(-0.6, -0.6);
				break;
			case 'b': // turn left
				qcontroller->setMC(-0.3, 0.3);
				break;
			case 'k': // turn right
				qcontroller->setMC(0.3, -0.3);
				break;
			case ' ': // stop
				qcontroller->setMC(0,0);
				break;
			default:
				return false;
				break;
			}
		}
		return false;
	}


	/**************************Reset Function***************************************************************/
	virtual bool restart(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
	{
		std::cout << "\n begin restart " << currentCycle << "\n";

		std::cout<<"Current Cycle"<<this->currentCycle<<std::endl;

		 global_counter++;

		//Temporary variable storing the pointer to the controller if the latter should not be resetted
		AbstractController* temp_controller;

		//OdeAgent* temp_agent;

		// We would like to have 10 runs!
		// after it we must clean all and return false because we don't want a new restart
		if (this->currentCycle == repeat_number) // This will be delete all after finish simulation!!!!!!
		{



			//clean robots
			while (global.agents.size() > 0)
			{
				OdeAgent* agent = *global.agents.begin();
				AbstractController* controller = agent->getController();
				OdeRobot* robot = agent->getRobot();
				AbstractWiring* wiring = agent->getWiring();

				global.configs.erase(std::find(global.configs.begin(),
						global.configs.end(), controller));
				delete controller;///////////////////////////////////////////////This will finally delete which is OK

				delete robot;
				delete wiring;

				delete (agent);
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
			std::cout << "end.";
			return false; // don't restart, just quit
		}

		// Now we must delete all robots and agents from the simulation and create new robots and agents.
		// BUT NOT CONTROLLER for learning
		while (global.agents.size() > 0)
		{
			//	  std::cout << "\n MAIN WHILE LOOP" << currentCycle << "\n";
			OdeAgent* agent = *global.agents.begin();


			AbstractController* controller = agent->getController();

			//((ACICOControllerV12*)controller)->setCurrentCycle(currentCycle);
			//((ACICOControllerV13*)controller)->setCurrentCycle(currentCycle);
			((AcIcoRcCriticController*)controller)->setCurrentCycle(currentCycle);

			OdeRobot* robot = agent->getRobot();
			AbstractWiring* wiring = agent->getWiring();

			global.configs.erase(std::find(global.configs.begin(),
					global.configs.end(), controller));


			delete robot;
			delete wiring;

			//DON'T delete controller if DON't want to reset controller!------------------------------------------------------(FRANK)
			if(delete_controller)
			{
			delete controller;
			//this calls destroy:

			} else { // instead save the pointer in a temporary variable
				temp_controller = controller;
		//		temp_agent = agent;
			}

			//Need to add put the current controller to temp controller?????----STILL NOT IMPLEMENT-----------------------------(FRANK)


			global.agents.erase(global.agents.begin());
			//	  std::cout << "\n END OF MAIN WHILE LOOP" << currentCycle << "\n";

		}

        ///////////////Recreate Robot Start//////////////////////////////////////////////////////////////////////////////////////
		//1) Activate IR sensors
    FourWheeledConf fconf =FourWheeledRPos::getDefaultConf();

		///2) relative sensors
		for (int i=0; i < number_spheres; i++)
		{
			fconf.rpos_sensor_references.push_back(obst.at(i)->getMainPrimitive());
		}

		OdeRobot* vehicle3 = new FourWheeledRPos(odeHandle, osgHandle, fconf, "FourWheeled");

		// Deprecated
		//((Nimm4*)vehicle3)->setBodyColor(Color(0.0,1.0,0.0));
		//((Nimm4*)vehicle3)->setBodyTexture("../../../../osg/data/Images/wood.rgb");
		//((Nimm4*)vehicle3)->setWheelTexture("../../../../osg/data/Images/chess.rgb");

		//((Nimm4*)vehicle3)->setRelPosSensorReference(global.obstacles.back()->getMainPrimitive());


		/****Initial position of Nimm4******/
		//Generate random position between x = +-10, y = +-10
		//Position x = +-10
		//Position y = +-10
		//vehicle3->place(Pos(0/*x*/,0/*y*/,0/*z*/));

		if(random_positon)
		{
			/*******Generating Random Position****/
			//srand (time(NULL));
			random_position  = ((MAX_x-MIN_x)*((float)rand()/RAND_MAX))+MIN_x; // Input 0 (m) between -2.4 and 2.4
			/************************************/
			do
			{
				random_positionx  = ((MAX_x-MIN_x)*((float)rand()/RAND_MAX))+MIN_x;
				random_positiony  = ((MAX_y-MIN_y)*((float)rand()/RAND_MAX))+MIN_y;
				//std::cout<<"\n\n\n\n\n"<<"Inital Random_Robot X position"<<" = "<<random_position<<"\t"<<"Inital Random_Robot Y position"<<" = "<<-1*random_position<<"\n\n\n\n";

			}while(abs(abs(random_position_S) - abs(random_positionx)) <= 4 || abs(abs(pos_obstacle_x) - abs(random_positionx)) <= 1.5 || abs(random_positiony) <= 0.5);

			std::cout<<"\n\n"<<"Reset Inital Random X position"<<" = "<<random_positionx<<"\t"<<"Reset Inital Random Y position"<<" = "<<random_positiony<<"\n\n";

			vehicle3->place(Pos(random_positionx/*10    +x = to left, -x = to right*/, random_positiony /*  +y = to close, -y = to far*/,0/*z*/));
		}
		else
		{
		//	vehicle3->place(Pos(position_x-5.0/*x, +x = to left, -x = to right*/,0.0/*y*/,0.0/*z*/));

			vehicle3->place(Pos(position_x/*x, +x = to left, -x = to right*/,0.0/*y*/,0.0/*z*/));


		//	vehicle3->place(Pos(position_x-5.0/*x, +x = to left, -x = to right*/,-3.0/*y*/,0.0/*z*/));

		}




		if(random_orientation)
		{
			/*******Generating Random Position****/
			//srand (time(NULL));
			int r = rand();
			std::cout << "random value 3: " << r << std::endl;
			random_or  = ((MAX_or-MIN_or)*((float)r/RAND_MAX))+MIN_or; // between -pi/3 (60 deg) and pi/3 (60 deg)
			/************************************/

			Pos pos(position_x-5.0/*5.5x, +x = to left, -x = to right*/,0.0/*y*/,0.0/*z*/);

		//	Pos pos(position_x/*5.5x, +x = to left, -x = to right*/,0.0/*y*/,0.0/*z*/);



			std::cout << "random_or2: " << random_or<<std::endl;

			//setting position and orientation
			vehicle3->place(osg::Matrix::rotate(random_or, 0, 0, 1) *osg::Matrix::translate(pos));
			//setting only position
			//vehicle3->place(Pos(position_x-5.0/*5.5x, +x = to left, -x = to right*/,0.0/*y*/,0.0/*z*/));

		}
		else
		{
			Pos pos(position_x-5.0/*5.5x, +x = to left, -x = to right*/,0.0/*y*/,0.0/*z*/);

		//	Pos pos(position_x/*5.5x, +x = to left, -x = to right*/,0.0/*y*/,0.0/*z*/);

			//setting position and orientation
			vehicle3->place(osg::Matrix::rotate(0.0, 0, 0, 1) *osg::Matrix::translate(pos));

		//	vehicle3->place(osg::Matrix::rotate(0, 0, 0, -1) *osg::Matrix::translate(pos));
			//setting only position
			//vehicle3->place(Pos(position_x-5.0/*5.5x, +x = to left, -x = to right*/,0.0/*y*/,0.0/*z*/));
		}


	//	if (obstacle_on && global_counter%10 == 0)
	//	{ PassiveBox* b1;
	//	double length = 6.5; //6.5 ; //20.0;//20.0; //4.5; //6.0 ; //4.5 /*for learning*/; //3.5 /*for*/;//testing//---------------------------------------------------------------------------TEST
	//	double width = 1.0;
	//	double height = 1.0;


	//	if(/*number_boxes >= 4*/ global_counter%5 == 0) {
					//int k = 0;
				//	while(k<=number_boxes)
				//	{
				//		b1 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(3.0, width, height /*size*/));
				//		b1->setColor(Color(1,0,0));
				//		random_position_S_x  = ((MAX_x-MIN_x)*((float)rand()/RAND_MAX))+MIN_x;
				//		random_position_S_y  = ((MAX_y-MIN_y)*((float)rand()/RAND_MAX))+MIN_y;
				//		b1->setPose(osg::Matrix::rotate(/*-0.5*(M_PI/4)*/ (M_PI/2), random_position_S_x /*0*/,0,1) * osg::Matrix::translate(/*i*4.5+*/random_position_S_x, random_position_S_y/*i*-pos_obstacle_y*2.0*//*0+i*/,height/2) /* pose*/);

				//		global.obstacles.push_back(b1);
				//									fixator.push_back(  new  FixedJoint(b1->getMainPrimitive(), global.environment));  //create pointer
				//									//fixator.at(i)->init(odeHandle, osgHandle);
				//									fixator.at(0)->init(odeHandle, osgHandle);

					//								k++;
				//	}


			//	}

	//	}



		///////////////Recreate Robot End//////////////////////////////////////////////////////////////////////////////////////
		//OdeAgent* agent6;
		//AbstractController* temp_controller = agent6->getController();

		//AC Lernen
		if (delete_controller){
			//qcontroller = new ACICOController();
			//qcontroller = new ACICOControllerV2();
			//qcontroller = new ACICOControllerV3();
			//qcontroller = new ACICOControllerV4();
			//qcontroller = new ACICOControllerV5();
			//qcontroller = new ACICOControllerV6();
			//qcontroller = new ACICOControllerV7();
			//qcontroller = new ACICOControllerV8();
			//qcontroller = new ACICOControllerV9();
			//qcontroller = new ACICOControllerV10();
			//qcontroller = new ACICOControllerV11();
			//qcontroller = new ACICOControllerV12();
			//qcontroller = new ACICOControllerV13();
			qcontroller = new AcIcoRcCriticController();

			qcontroller->setParam("eps", 0.1);//doesn't need to be changed if using previous controller
		} else {
			//qcontroller = (ACICOController*)temp_controller;
			//qcontroller = (ACICOControllerV2*)temp_controller;
			//qcontroller = (ACICOControllerV3*)temp_controller;
			//qcontroller = (ACICOControllerV4*)temp_controller;
			//qcontroller = (ACICOControllerV5*)temp_controller;
			//qcontroller = (ACICOControllerV6*)temp_controller;
			//qcontroller = (ACICOControllerV7*)temp_controller;
			//qcontroller = (ACICOControllerV8*)temp_controller;
			//qcontroller = (ACICOControllerV9*)temp_controller;
			//qcontroller = (ACICOControllerV10*)temp_controller;
			//qcontroller = (ACICOControllerV11*)temp_controller;
			//qcontroller = (ACICOControllerV12*)temp_controller;
			//qcontroller = (ACICOControllerV13*)temp_controller;
			qcontroller = (AcIcoRcCriticController*)temp_controller;
		}


		//Homeo + Q
		//qcontroller = new QLearningHomeokMController();
		//Q Lernen
		//qcontroller = new QLearningMController();

		global.configs.push_back(qcontroller);


		// create pointer to one2onewiring
		AbstractWiring*  wiring3 = new One2OneWiring(new ColorUniformNoise(0.1));

		// create pointer to agent

		plotoptions.push_back(PlotOption(NoPlot /*select "File" to save signals, "NoPlot" to not save*/));
		OdeAgent* agent3 = new OdeAgent(plotoptions);
		//OdeAgent* agent3 = new OdeAgent(global);
		agent3->init(qcontroller, vehicle3, wiring3);

//		if (shiftObstacle ==1 && obstacle_on == true)
//			{
//			   pos_obstacle_y = 2;
//
//			   shiftObstacle =0;
//
//			for (int i=0; i < number_boxes /*SET NUMBER OF OBSTACLES*/; i++){
//				b1 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(length, width, height /*size*/));
//				//b1->setTexture("dusty.rgb");
//				b1->setColor(Color(1,0,0));
//				//b1->setPosition(osg::Vec3(/*-4.5+*/i*4.5,3+i,0)); // Fixed robot position
//				//osg::Matrix pose;
//				//			pose.setTrans(osg::Vec3(/*-4.5+*/i*4.5,3+i,0));
//				//b1->setPose(osg::Matrix::rotate(0.5*(M_PI/2), 0,0, 1) * osg::Matrix::translate(/*-4.5+*/i*4.5,3+i,0.5) /* pose*/);
//				b1->setPose(osg::Matrix::rotate(/*-0.5*(M_PI/4)*/ (M_PI/2), 0,0, 1) * osg::Matrix::translate(/*i*4.5+*/pos_obstacle_x, pos_obstacle_y+i*-pos_obstacle_y*2.0/*0+i*/,height/2) /* pose*/);
//				global.obstacles.push_back(b1);
//				fixator.push_back(  new  FixedJoint(b1->getMainPrimitive(), global.environment));  //create pointer
//				fixator.at(i)->init(odeHandle, osgHandle);
//			    }
//
//	      }

		if(drawtrace_on)
								{
									TrackRobot* track3 = new TrackRobot(/*bool trackPos*/true,
											/*bool trackSpeed*/false,
											/*bool trackOrientation*/false,
											/*bool displayTrace*/true
									/*const char *scene="", int interval=1*/,"",60);
									agent3->setTrackOptions(*track3);
								}
		global.agents.push_back(agent3);

		std::cout << "\n end restart " << currentCycle << "\n";
		// restart!
		return true;

	}
	/****************************************************************************************************/


	/** optional additional callback function which is called every simulation step.
	      Called between physical simulation step and drawing.
	      @param draw indicates that objects are drawn in this timestep
	      @param pause always false (only called of simulation is running)
	      @param control indicates that robots have been controlled this timestep
	 */


	virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control)
	{
		// for demonstration: set simsteps for one cycle to 60.000/currentCycle (10min/currentCycle)
		// if simulation_time_reached is set to true, the simulation cycle is finished

		if(repeat_experiment)
		{
			/***********************Reset Function*********************************************/


			if (globalData.sim_step>=(time_factor*/*60.0*/60.0*200.000) /*|| qcontroller->distance <5 /*5*/ || qcontroller->failure_flag==1)//||qcontroller->Vt< -1000 ||qcontroller->Vt>1000)//(globalData.sim_step>=(time_factor*60.0*200.000) || qcontroller->distance <= 5 )// || qcontroller->distance > 250)// || qcontroller->failure_flag==1 /*parameter from acicocontroller.cpp*/)
			//if (globalData.sim_step>=(time_factor*60.0*200.000) || qcontroller->distance <5/*10*/|| qcontroller->failure_flag_ico==1)
			//if (globalData.sim_step>=(time_factor*60.0*200.000) || qcontroller->failure_flag_ico==1 || qcontroller->failure_flag==1)
			//if (globalData.sim_step>=(time_factor*100*60.0*200.000))
			{




				qcontroller->resetlearing(0);

				//qcontroller->setReset(repeat_number);//(10);
				simulation_time_reached=true;

			}
			/***********************************************************************************/

		}

	}


};


int main (int argc, char **argv)
{
	//srand( time( NULL));
	ThisSim sim;
	  sim.setGroundTexture("wood.ppm"/*green-artificial-grass-texture.jpg*/);//green_velour_wb.rgb
	  sim.setCaption("lpzrobots Simulator (NIMM4 robot)        Dasgupta 2012");
	return sim.run(argc, argv) ? 0 : 1;



}
