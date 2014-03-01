
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
//#include <fourwheeledrpos.h>


#include <selforg/trackrobots.h>
// used arena
#include <ode_robots/playground.h>
// used passive spheres
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>


#include <AmosOG.h> // ICO + AC 3 goals no obstacle
#include "amosII.h"

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

//OpenInvertNChannelController * ocontroller;

//select

AmosOG* qcontroller;

bool obstacle_on = false;
bool drawtrace_on = false;//true;//---------------------------------------------------------------------------TEST

bool random_positon_spheres = false;
bool random_positon = false;//true;
bool random_positon_frist = false;//-------------------------------------------------CH
bool random_orientation = false;

#define MAX_x   11.0//10.0 //11.0//9.0	//Max range-------------------------------------------------CH VERY IMPORTANT FACTOR
#define MIN_x   4.0  //5.0 (TEST) //Min range-------------------------------------------------CH VERY IMPORTANT FACTOR

#define MAX_y   2.5//5.0 // 9.0 	//Max range
#define MIN_y   -2.5//-5.0//-9.0 	//Min range
#define pos_obstacle_x 5.0 //7
#define pos_obstacle_y 3.0

#define MAX_or	M_PI/3 // 60 deg
#define MIN_or	-M_PI/3 // 60 deg

double random_or;
double random_position_S;
double random_position;
double random_positionx;
double random_positiony;


int number_spheres = 4;//2; // SET NUMBER of TARGET MAX = 4
int number_boxes = 2; // SET NUMBER of BOXES obstacles


bool repeat_experiment = true;//true; //if select true then set the follwing parameters
int repeat_number = 5000;// 1000;
double time_factor = 0.25/2;//0.25/2;

//0.25/4 = 7.5s
//0.25/2 = 15 s
// 0.25 = 30 s
// 0.5 = 1 min for each run,
// 1 = 2 mins for each run,
//1.5 = 3 mins for each run

double position_S = 0.0;
int position_x = 20;

bool delete_controller = false;// false if wants to repeat experiments but don't want to reset controller//DON'T delete controller if DON't want to reset controller!------------------------------------------------------(FRANK)


//***************************//

bool exploration_active = false;
Joint* fixator2;


std::vector<AbstractObstacle*> obst;
std::vector<FixedJoint*> fixator;







class ThisSim : public Simulation {
public:


	void generate_spheres(GlobalData& global)
	{
		PassiveSphere* s1;
		for (int i=0; i < number_spheres; i++){
			s1 = new PassiveSphere(odeHandle, osgHandle, 0.2);
			//Target 1
			//position_S = 0.0;// -2.0---------------------------------------------------------------------------TEST
			if (i==0) s1->setPosition(osg::Vec3(2/*position_S*/, position_S -2.5, 0.25/*0*/));
			//Target 2

			//if (i==1) s1->setPosition(osg::Vec3(15-11.0 /*position_S*/, position_S+10, 0.5/*0*/));
			if (i==1) s1->setPosition(osg::Vec3(9 /*position_S*/, position_S , 2/*0*/)); //Green Sphere

			//Target 3./s
			//if (i==2) s1->setPosition(osg::Vec3(15-11.0 /*position_S*/, position_S-10, 0.5/*0*/));
			if (i==2) s1->setPosition(osg::Vec3(1.0 /*position_S*/, position_S , 2/*0*/)); //Blue sphere

			//Target 4
			if (i==3) s1->setPosition(osg::Vec3(1.0 /*position_S*/, position_S - 8, 2/*0*/));
			//if (i==3) s1->setPosition(osg::Vec3(-10, 10,2/*2*/));

			s1->setTexture("Images/dusty.rgb");
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
	}
	void generate_box(GlobalData& global)
	{
		//7) - add passive box as OBSTACLES
		PassiveBox* b1;
		double length = 13.5/4.0 /*for learning*/; //3.5 /*for*/;//testing//---------------------------------------------------------------------------TEST
		double width = 0.4;
		double height = 0.5;
		//if(obstacle_on)
		{
			for (int i=0; i < number_boxes /*SET NUMBER OF OBSTACLES*/; i++){
				b1 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(length, width, height /*size*/));
				b1->setColor(Color(1,0,0));
				if (i == 0)
					b1->setPose(osg::Matrix::rotate(/*-0.5*(M_PI/4)*/ (M_PI/2), 0,0, 1) * osg::Matrix::translate(/*i*4.5+*/pos_obstacle_x, pos_obstacle_y+i*-pos_obstacle_y*2.0-8.5/*0+i*/,height/2) /* pose*/);
				else
					b1->setPose(osg::Matrix::rotate(/*-0.5*(M_PI/4)*/ (0), 0,0, 1) * osg::Matrix::translate(/*i*4.5+*/pos_obstacle_x - 1.5, pos_obstacle_y+i*-pos_obstacle_y*2.0-1/*0+i*/,height/2) /* pose*/);
				//obst.push_back(b1);
				global.obstacles.push_back(b1);
				fixator.push_back(  new  FixedJoint(b1->getMainPrimitive(), global.environment));  //create pointer
				fixator.at(i)->init(odeHandle, osgHandle);
			}
		}
	}
	void setup_Playground(GlobalData& global)
	{
		// odeHandle and osgHandle are global references
		// vec3 == length, width, height

		double length_pg = 25/4.0;//28;//0.0; //45, 32, 22
		double width_pg = 0.5/4.0;//0.0;  //0.2
		double height_pg = 0.5;//0.0; //0.5

		Playground* playground = new Playground(odeHandle, osgHandle.changeColor(Color(0.6,0.0,0.6)),
				osg::Vec3(length_pg /*length*/, width_pg /*width*/, height_pg/*height*/), /*factorxy = 1*/1, /*createGround=true*/true /*false*/);
		playground->setPosition(osg::Vec3(5, -4, 0.0)); // playground positionieren und generieren
		// register playground in obstacles list
		global.obstacles.push_back(playground);
	}

	OdeRobot* add_amos(const OdeHandle& odeHandle, const OsgHandle& osgHandle)
	{
		lpzrobots::AmosIIConf myAmosIIConf = lpzrobots::AmosII::getDefaultConf();
		myAmosIIConf.rubberFeet = true;
		for (int i=0; i < number_spheres; i++){
			myAmosIIConf.GoalSensor_references.push_back(obst.at(i)->getMainPrimitive());
		}
		lpzrobots::OdeHandle rodeHandle = odeHandle;
		  //rodeHandle.substance = lpzrobots::Substance(5.0, 0.0, .0, 0.99);
		lpzrobots::AmosII* amos = new lpzrobots::AmosII(
		  rodeHandle,
		  osgHandle.changeColor(lpzrobots::Color(0, 1, 1)),
		  myAmosIIConf, "AmosII");

		// define the usage of the individual legs
		amos->setLegPosUsage(amos->L0, amos->LEG);
		amos->setLegPosUsage(amos->L1, amos->LEG);
		amos->setLegPosUsage(amos->L2, amos->LEG);
		amos->setLegPosUsage(amos->R0, amos->LEG);
		amos->setLegPosUsage(amos->R1, amos->LEG);
		amos->setLegPosUsage(amos->R2, amos->LEG);

		return amos;
	}

	OdeRobot* vehicle3;
	// starting function (executed once at the beginning of the simulation loop)
	void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
	{

		// set initial camera position
		setCameraHomePos(Pos(-1.14383, 10.1945, 42.7865),  Pos(179.991, -77.6244, 0));
		//1) - set noise to 0.1
		global.odeConfig.noise= 0.05;//0.05;//0.05;//0.05;
		//2) - set controlinterval -> default = 1
		/*update frequency of the simulation ~> amos = 20*/
		global.odeConfig.setParam("controlinterval", 10);
		//3) - set simulation setp size
		global.odeConfig.setParam("simstepsize", 0.01); /*stepsize of the physical simulation (in seconds)*/
		//Update frequency of simulation = 1*0.01 = 0.01 s ; 100 Hz
		//4) - set gravity if not set then it uses -9.81 =earth gravity
		//global.odeConfig.setParam("gravity", -9.81);
		//5) - set Playground as boundary:
		setup_Playground(global);
		//generate spheres
		generate_spheres(global);
		generate_box(global);
		////////////////////////////////////Call this set up and Control//////////////////////////////////////////////
		bool ac_ico_robot = true;
		if (ac_ico_robot)
		{
			//1) Activate IR sensors
			//FourWheeledConf fconf = FourWheeledRPos::getDefaultConf();
			///2) relative sensors
			//for (int i=0; i < number_spheres; i++){
			//	fconf.rpos_sensor_references.push_back(obst.at(i)->getMainPrimitive());
			//}
			vehicle3 = add_amos(odeHandle, osgHandle);//new FourWheeledRPos(odeHandle, osgHandle, fconf);
			/****Initial position of Nimm4******/
			if(random_positon_frist)
			{
				/*******Generating Random Position****/
				// Input 0 (m) between -2.4 and 2.4
				random_position  = ((MAX_x-MIN_x)*((float)rand()/RAND_MAX))+MIN_x;
				/************************************/
				do
				{
					int r = rand();
					std::cout << "random value 1: " << r << std::endl;
					random_position  = ((MAX_x-MIN_x)*((float)r/RAND_MAX))+MIN_x;
				}while(abs(abs(position_S) - abs(random_position)) <= 4);
				std::cout<<"\n\n"<<"Inital Random X position"<<" = "<<random_position<<"\t"<<"Inital Random Y position"<<" = "<<-1*random_position<<"\n\n";
				//vehicle3->place(Pos(position_x-10.0 /* 20 random_position*/  /*+x = to left (sphere in front of robot), -x = to right sphere behind robot*/, 0.0 /*-1*random_position*/ /*+y = to up, -y = to down*/,0.0/*z*/));
				vehicle3->place(Pos(position_x-13.5 /* 20 random_position*/  /*+x = to left (sphere in front of robot), -x = to right sphere behind robot*/, 0.0 /*-1*random_position*/ /*+y = to up, -y = to down*/,0.0/*z*/));
			}
			else
			{
				/*******Generating Random Position****/
				int r = rand();
				std::cout << "random value 2: " << r << std::endl;
				random_or  = PI/2.0;//((MAX_or-MIN_or)*((float)r/RAND_MAX))+MIN_or; // between -pi/3 (60 deg) and pi/3 (60 deg)
				/************************************/
				//Pos pos(0/*, +x = to left, -x = to right*/,0/*y*/,0.0/*z*/);
				Pos pos(position_x-13.5/*, +x = to left, -x = to right*/,-6.5/*y*/,0.0/*z*/);
				std::cout << "random_or1: " << random_or << std::endl;
				//setting position and orientation
				vehicle3->place(osg::Matrix::rotate(random_or, 0, 0, 1) *osg::Matrix::translate(pos));
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
			qcontroller = new AmosOG(); ///////////////////// call this function NEW controller!!

			//((ACICOControllerV12*)qcontroller)->setCurrentCycle(currentCycle);
			//((ACICOControllerV13*)qcontroller)->setCurrentCycle(currentCycle);
			((AmosOG*)qcontroller)->setCurrentCycle(currentCycle);
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

			if(drawtrace_on)
			{
				TrackRobot* track3 = new TrackRobot(/*bool trackPos*/true,
						/*bool trackSpeed*/true,
						/*bool trackOrientation*/true,
						/*bool displayTrace*/true //,
				,"", 1);
				agent3->setTrackOptions(*track3);
			}

			agent3->init(qcontroller, vehicle3, wiring3);///////////// Initial controller!!!
			global.agents.push_back(agent3);


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
		bool manually_steered_robot = false;
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

		return false;

	}
	/****************************************************************************************************/


	/** optional additional callback function which is called every simulation step.
	      Called between physical simulation step and drawing.
	      @param draw indicates that objects are drawn in this timestep
	      @param pause always false (only called of simulation is running)
	      @param control indicates that robots have been controlled this timestep
	 */


	void reset_robot()
	{
		int r = rand();
		random_or  = PI/2.0;//((MAX_or-MIN_or)*((float)r/RAND_MAX))+MIN_or; // between -pi/3 (60 deg) and pi/3 (60 deg)
		/************************************/
		Pos pos(position_x-13.5/*, +x = to left, -x = to right*/,-6.5/*y*/,0.0/*z*/);
		//setting position and orientation
		vehicle3->place(osg::Matrix::rotate(random_or, 0, 0, 1) *osg::Matrix::translate(pos));
	}

	virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control)
	{
		if(repeat_experiment)
		{
			/***********************Reset Function*********************************************/

			//if (globalData.sim_step>=(time_factor*60.0*200.000) || qcontroller->distance <5/*10*/|| qcontroller->failure_flag==1)//||qcontroller->Vt< -1000 ||qcontroller->Vt>1000)//(globalData.sim_step>=(time_factor*60.0*200.000) || qcontroller->distance <= 5 )// || qcontroller->distance > 250)// || qcontroller->failure_flag==1 /*parameter from acicocontroller.cpp*/)
			//if (globalData.sim_step>=(time_factor*60.0*200.000) || qcontroller->distance <5/*10*/|| qcontroller->failure_flag_ico==1)
			//if (globalData.sim_step>=(time_factor*60.0*200.000) || qcontroller->failure_flag_ico==1 || qcontroller->failure_flag==1)
			//if (globalData.sim_step>=(time_factor*100*60.0*200.000))
			if (
					//qcontroller->distance <5/*10*/
				//||
				qcontroller->failure_flag==1)
			{
				//ofstream RBF_TEST;
				int timestep = this->currentCycle;
				//qcontroller->print_RBF_network_to_file(timestep);


				//qcontroller->setReset(repeat_number);//(10);
				reset_robot();



				//simulation_time_reached=true;
				qcontroller->resetlearing(0);
				if (qcontroller->terminate_now)
					simulation_time_reached=true;
				timestep++;

			}
			/***********************************************************************************/

		}

	}


};


int main (int argc, char **argv)
{
	//srand( time( NULL));
	ThisSim sim;

	return sim.run(argc, argv) ? 0 : 1;



}
