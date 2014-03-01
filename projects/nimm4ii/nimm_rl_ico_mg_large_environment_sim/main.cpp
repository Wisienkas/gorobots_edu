
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


#include <NimmLarge.h> // ICO + AC 3 goals no obstacle


// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

//OpenInvertNChannelController * ocontroller;

//select

NimmLarge* qcontroller;

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


int number_spheres = 14; // SET NUMBER of TARGET MAX = 4
int number_boxes = 6; // SET NUMBER of BOXES obstacles


bool repeat_experiment = true;//true; //if select true then set the follwing parameters
int repeat_number = 50000;// 1000;
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
		for (int i=0; i < number_spheres; i++) {
			s1 = new PassiveSphere(odeHandle, osgHandle, 0.5);
			if (i==0) {
				s1->setPosition(osg::Vec3(23.0 /*position_S*/, position_S - 4, 3/*0*/));
				s1->setColor(Color(1,0,0));
			} else if (i==2) {
				s1->setPosition(osg::Vec3(- 2.0 /*position_S*/, position_S - 9 , 3/*0*/));
				s1->setColor(Color(1,0,0));
			} else if (i==1) {
				s1->setPosition(osg::Vec3(10.0 /*position_S*/, position_S - 2 , 3/*0*/));
				s1->setColor(Color(1,0,0));
			} else if (i==3) {
				s1->setPosition(osg::Vec3(-15.0 /*position_S*/, position_S - 6, 3/*0*/));
				s1->setColor(Color(1,0,0));
			} else if (i == 4) {
				s1->setPosition(osg::Vec3(-30.0 /*position_S*/, position_S - 13, 3/*0*/));
				s1->setColor(Color(1,0,0));
			} else if (i == 5) {
				s1->setPosition(osg::Vec3(-45.0 /*position_S*/, position_S - 17, 3/*0*/));
				s1->setColor(Color(1,0,0));
			} else if (i == 6) {
				s1->setPosition(osg::Vec3(-54.0 /*position_S*/, position_S - 25, 3/*0*/));
				s1->setColor(Color(1,0,0));
			} else if (i == 7) {
				s1->setPosition(osg::Vec3(-66.0 /*position_S*/, position_S - 22, 3/*0*/));
				s1->setColor(Color(1,0,0));
			} else if (i == 8) {
				s1->setPosition(osg::Vec3(-69.0 /*position_S*/, position_S - 10, 3/*0*/));
				s1->setColor(Color(1,0,0));
			} else if (i == 9) {
				s1->setPosition(osg::Vec3(-65.0 /*position_S*/, position_S + 2, 3/*0*/));
				s1->setColor(Color(1,0,0));
			} else if (i == 11) {
				s1->setPosition(osg::Vec3(35.0 /*position_S*/, position_S - 33, 3/*0*/));
				s1->setColor(Color(1,1,1));
			} else if (i == 12) {
				s1->setPosition(osg::Vec3(-75/*-54.0 position_S*/, position_S - 33, 3/*0*/));
				s1->setColor(Color(1,1,1));
			} else if (i == 13) {
				s1->setPosition(osg::Vec3(-66.0 /*position_S*/, position_S + 15 , 3/*0*/));
				s1->setColor(Color(0,1,0));
			}
			else if (i == 10) {
				delete s1;
				s1 = new PassiveSphere(odeHandle, osgHandle, 1);
				s1->setPosition(osg::Vec3(-55.0 /*position_S*/, position_S + 10, 3/*0*/));
				s1->setColor(Color(0,0,0));
			}
			obst.push_back(s1);
			fixator2 = new  FixedJoint(s1->getMainPrimitive(), global.environment);
			fixator2->init(odeHandle, osgHandle);
		}
	}

	void generate_box(GlobalData& global)
	{
		PassiveBox* b1, *b2, *b3, *b4, *b5, *b6;
		double length = 23.5 /*for learning*/; //3.5 /*for*/;//testing//---------------------------------------------------------------------------TEST
		double width = 3.0;
		double height = 1.0;


		b1 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(length, width, height /*size*/));
		b1->setColor(Color(0,1,0));
		b2 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(length - 10 , width, height /*size*/));
		b2->setColor(Color(0.1,0.2,0.3));
		b3 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(width + 68, width , height /*size*/));
		b3->setColor(Color(0.1,0.2,0.3));
		b4 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(width + 20, width , height /*size*/));
		b4->setColor(Color(1,0,0));
		b5 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(length - 5 , width, height /*size*/));
		b5->setColor(Color(0.1,0.2,0.3));
		b6 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(width + 30, width + 10, height /*size*/));
		b6->setColor(Color(0.1,0.2,0.3));
		//if(obstacle_on)
		{
			for (int i=0; i < number_boxes /*SET NUMBER OF OBSTACLES*/; i++) {
				if (i == 0) {
					b1->setPose(osg::Matrix::rotate(/*-0.5*(M_PI/4)*/ (M_PI/3), 0,0, 1) *
					osg::Matrix::translate(/*i*4.5+*/pos_obstacle_x + 10, pos_obstacle_y+i*-pos_obstacle_y*2.0-20/*0+i*/,height/2)/* pose*/);
					global.obstacles.push_back(b1);
					fixator.push_back(  new  FixedJoint(b1->getMainPrimitive(), global.environment));  //create pointer
					fixator.at(i)->init(odeHandle, osgHandle);
				}
				else if (i == 1) {
					b2->setPose(osg::Matrix::rotate(/*-0.5*(M_PI/4)*/ (M_PI/3.5), 0,0, 1) *
					osg::Matrix::translate(/*i*4.5+*/pos_obstacle_x - 4, pos_obstacle_y+i*-pos_obstacle_y*2.0+3/*0+i*/,height/2)/* pose*/);
					global.obstacles.push_back(b2);
					fixator.push_back(  new  FixedJoint(b2->getMainPrimitive(), global.environment));  //create pointer
					fixator.at(i)->init(odeHandle, osgHandle);
				}
				else if (i == 2) {
					b3->setPose(osg::Matrix::rotate(/*-0.5*(M_PI/4)*/ (M_PI), 0,0, 1) *
							osg::Matrix::translate(/*i*4.5+*/pos_obstacle_x - 6 , pos_obstacle_y+i*-pos_obstacle_y*2.0+18
									/*0+i*/,height/2)/* pose*/);
					global.obstacles.push_back(b3);
					fixator.push_back(  new  FixedJoint(b3->getMainPrimitive(), global.environment));  //create pointer
					fixator.at(i)->init(odeHandle, osgHandle);
				} else if (i == 3) {
					b4->setPose(osg::Matrix::rotate(/*-0.5*(M_PI/4)*/ (M_PI), 0,0, 1) *
					osg::Matrix::translate(/*i*4.5+*/pos_obstacle_x - 10 , pos_obstacle_y+i*-pos_obstacle_y*2.0 /*0+i*/,height/2)/* pose*/);
					global.obstacles.push_back(b4);
					fixator.push_back(  new  FixedJoint(b4->getMainPrimitive(), global.environment));  //create pointer
					fixator.at(i)->init(odeHandle, osgHandle);
				} else if (i == 4) {
					b5->setPose(osg::Matrix::rotate(/*-0.5*(M_PI/4)*/ (M_PI/3.5), 0,0, 1) *
					osg::Matrix::translate(/*i*4.5+*/pos_obstacle_x - 30 , pos_obstacle_y+i*-pos_obstacle_y*2.0+20/*0+i*/,height/2)/* pose*/);
					global.obstacles.push_back(b5);
					fixator.push_back(  new  FixedJoint(b5->getMainPrimitive(), global.environment));
					fixator.at(i)->init(odeHandle, osgHandle);
				} else if (i == 5) {
					b6->setPose(osg::Matrix::rotate(/*-0.5*(M_PI/4)*/ (M_PI/3.5), 0,0, 1) *
					osg::Matrix::translate(/*i*4.5+*/pos_obstacle_x - 55 , pos_obstacle_y+i*-pos_obstacle_y*2.0+23/*0+i*/,height/2)/* pose*/);
					global.obstacles.push_back(b6);
					fixator.push_back(  new  FixedJoint(b6->getMainPrimitive(), global.environment));
					fixator.at(i)->init(odeHandle, osgHandle);
				}
			}
		}
	}
	void setup_Playground(GlobalData& global)
	{
		// odeHandle and osgHandle are global references
		// vec3 == length, width, height

		double length_pg = 110;//28;//0.0; //45, 32, 22
		double width_pg = 0.5;//0.0;  //0.2
		double height_pg = 0.5;//0.0; //0.5

		Playground* playground = new Playground(odeHandle, osgHandle.changeColor(Color(0.6,0.0,0.6)),
				osg::Vec3(length_pg /*length*/, width_pg /*width*/, height_pg/*height*/), /*factorxy = 1*/1, /*createGround=true*/true /*false*/);
		playground->setPosition(osg::Vec3(-20, 23, 0.0));
		// register playground in obstacles list
		global.obstacles.push_back(playground);
	}
	OdeRobot* vehicle3;
	// starting function (executed once at the beginning of the simulation loop)
	void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
	{
		// set initial camera position
		setCameraHomePos(Pos(-1.14383, 10.1945, 202.7865),  Pos(179.991, -77.6244, 0));
		//1) - set noise to 0.1
		global.odeConfig.noise= 0.05;//0.05;//0.05;//0.05;
		//2) - set controlinterval -> default = 1
		/*update frequency of the simulation ~> amos = 20*/
		global.odeConfig.setParam("controlinterval", 4);
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
			FourWheeledConf fconf = FourWheeledRPos::getDefaultConf();
			///2) relative sensors
			for (int i=0; i < number_spheres; i++){
				fconf.rpos_sensor_references.push_back(obst.at(i)->getMainPrimitive());
			}
			vehicle3 = new FourWheeledRPos(odeHandle, osgHandle, fconf);
			/****Initial position of Nimm4******/


			/*******Generating Random Position****/
			int r = rand();
			std::cout << "random value 2: " << r << std::endl;
			random_or  = -PI/2.0;//((MAX_or-MIN_or)*((float)r/RAND_MAX))+MIN_or; // between -pi/3 (60 deg) and pi/3 (60 deg)
			/************************************/
			Pos pos(position_x + 7/*, +x = to left, -x = to right*/,-15.0/*y*/,0.0/*z*/);
			//std::cout << "random_or1: " << random_or << std::endl;
			//setting position and orientation
			vehicle3->place(osg::Matrix::rotate(random_or, 0, 0, 1) *osg::Matrix::translate(pos));


			//AC Lernen
			qcontroller = new NimmLarge(); ///////////////////// call this function NEW controller!!

			//((ACICOControllerV12*)qcontroller)->setCurrentCycle(currentCycle);
			//((ACICOControllerV13*)qcontroller)->setCurrentCycle(currentCycle);
			((NimmLarge*)qcontroller)->setCurrentCycle(currentCycle);
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
						/*bool trackSpeed*/false,
						/*bool trackOrientation*/false,
						/*bool displayTrace*/true //,
				/*const char *scene="", int interval=1*/);
				agent3->setTrackOptions(*track3);
			}

			agent3->init(qcontroller, vehicle3, wiring3);///////////// Initial controller!!!
			global.agents.push_back(agent3);
		}

		////////////////////////////////////Another set up and Control//////////////////////////////////////////////
		bool random_controlled_robot =false;
//

		////////////////////////////////////Another set up and Control//////////////////////////////////////////////
		bool manually_steered_robot = false;


		showParams(global.configs);
	}

	//  add own key handling stuff here, just insert some case values
	virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
	{
		if (down) { // only when key is pressed, not when released
			std::vector<double> f;
			switch ( (char) key )
			{

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
			case 'w': // move forward
				qcontroller->setMC(0.6, 0.6);
				break;
			case 's': // move backward
				qcontroller->setMC(-0.6, -0.6);
				break;
			case 'a': // turn left
				qcontroller->setMC(-0.3, 0.3);
				break;
			case 'd': // turn right
				qcontroller->setMC(0.3, -0.3);
			break;
			case 'p': // turn right
				qcontroller->setMC(0, 0);
			break;
			case ' ': // stop exploration
				qcontroller->reduce_noise += 3;
			break;
			case 'e': // stop exploration
				qcontroller->reduce_noise -= 3;
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
		random_or  = -PI/2.0;
		Pos pos(position_x+6/*, +x = to left, -x = to right*/,-15.0/*y*/,0.0/*z*/);
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
					//qcontroller->distance <5/*10*/ ||
					qcontroller->failure_flag==1)
			{
				//ofstream RBF_TEST;
				int timestep = this->currentCycle;
				//qcontroller->print_RBF_network_to_file(timestep);



				//qcontroller->setReset(repeat_number);//(10);
				reset_robot();
				qcontroller->resetlearing(0);
				if (qcontroller->terminate_immediatly)
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