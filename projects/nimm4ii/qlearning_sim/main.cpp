
#include <stdio.h>

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

// used robot
#include <ode_robots/nimm2.h>
#include "nimm4.h"

#include <selforg/trackrobots.h>
// used arena
#include <ode_robots/playground.h>
// used passive spheres
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>

// used controller
//#include <selforg/invertnchannelcontroller.h>
//#include <selforg/invertmotorspace.h>
//#include <selforg/sinecontroller.h>
#include "randomcontroller.h"
#include "openinvertnchannelcontroller.h"
#include "qlearninghomeokmcontroller.h"
//#include "qlearningmcontroller.h"

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

OpenInvertNChannelController * ocontroller;
QLearningHomeokMController* qcontroller;
//QLearningMController* qcontroller;
bool exploration_active = false;

std::vector<AbstractObstacle*> obst;
std::vector<FixedJoint*> fixator;

class ThisSim : public Simulation {
public:

	// starting function (executed once at the beginning of the simulation loop)
	void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
	{
		// first: position(x,y,z) second: view(alpha,beta,gamma)
		// gamma=0;
		// alpha == horizontal angle
		// beta == vertical angle
		// "normal"
		//setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));
		// top view
		setCameraHomePos(Pos(-1.14383, 10.1945, 42.7865),  Pos(179.991, -77.6244, 0));
		// initialization
		// - set noise to 0.1
		global.odeConfig.noise=0.05;
		global.odeConfig.setParam("controlinterval", 10);
		global.odeConfig.setParam("realtimefactor", 0);

		// use Playground as boundary:
		// - create pointer to playground (odeHandle contains things like world and space the
		//   playground should be created in; odeHandle is generated in simulation.cpp)
		// - setting geometry for each wall of playground:
		//   setGeometry(double length, double width, double	height)
		// - setting initial position of the playground: setPosition(double x, double y, double z)
		// - push playground in the global list of obstacles(globla list comes from simulation.cpp)

		// odeHandle and osgHandle are global references
		// vec3 == length, width, height
		Playground* playground = new Playground(odeHandle, osgHandle.changeColor(Color(0.3,0.3,0.3)),
				osg::Vec3(32, 0.2, 0.5),  /*factorxy = 1*/1, /*createGround=true*/false);
		playground->setPosition(osg::Vec3(0,0,0.05)); // playground positionieren und generieren
		// register playground in obstacles list
		global.obstacles.push_back(playground);

		// add passive spheres as obstacles
		// - create pointer to sphere (with odehandle, osghandle and
		//   optional parameters radius and mass,where the latter is not used here) )
		// - set Pose(Position) of sphere
		// - set a texture for the sphere
		// - add sphere to list of obstacles
		// goals:
		int number_spheres = 4;
		PassiveSphere* s1;
		for (int i=0; i < number_spheres; i++){
			s1 = new PassiveSphere(odeHandle, osgHandle, 0.5);
			if (i==0) s1->setPosition(osg::Vec3(-10,-10,2));
			if (i==1) s1->setPosition(osg::Vec3( 10, 10,2));
			if (i==2) s1->setPosition(osg::Vec3( 10, -10,2));
			if (i==3) s1->setPosition(osg::Vec3(-10, 10,2));

			s1->setTexture("Images/dusty.rgb");
			if (i==0){ s1->setColor(Color(1,0,0)); }
			if (i==1){ s1->setColor(Color(0,1,0)); }
			if (i==2){ s1->setColor(Color(0,0,1)); }
			if (i==3){ s1->setColor(Color(1,1,0)); }
			obst.push_back(s1);
			global.obstacles.push_back(s1);
			// fix sphere (in actual position) to simulation
			fixator.push_back(  new  FixedJoint(s1->getMainPrimitive(), global.environment));  //create pointer
			fixator.at(i)->init(odeHandle, osgHandle);
		}

		// obstacles
			int number_boxes = 4;
			PassiveBox* b1;
			for (int i=0; i < number_boxes; i++){

				if (i==0){
					b1 = new PassiveBox(odeHandle, osgHandle,  osg::Vec3(3.0, 2.0, 1.0) /*,mass = 1.0*/);
					b1->setPosition(osg::Vec3(-10, 0, 0));
				}
				if (i==1){
					b1 = new PassiveBox(odeHandle, osgHandle,  osg::Vec3(3.0, 2.0, 1.0) /*,mass = 1.0*/);
					b1->setPosition(osg::Vec3(+10, 0, 0));
				}
				if (i==2){
					b1 = new PassiveBox(odeHandle, osgHandle,  osg::Vec3(2.0, 3.0, 1.0) /*,mass = 1.0*/);
					b1->setPosition(osg::Vec3(0, -10, 0));
				}
				if (i==3){
					b1 = new PassiveBox(odeHandle, osgHandle,  osg::Vec3(2.0, 3.0, 1.0) /*,mass = 1.0*/);
					b1->setPosition(osg::Vec3(0, +10, 0));
				}

				//b1->setTexture("Images/dusty.rgb");
				b1->setColor(Color(0.5,0.5,0.5));
				global.obstacles.push_back(b1);
				// fix box (in actual position) to simulation
				fixator.push_back(  new  FixedJoint(b1->getMainPrimitive(), global.environment));  //create pointer
				fixator.back()->init(odeHandle, osgHandle);
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


		bool random_controlled_robot =false;
		if (random_controlled_robot){
			// robot 1
			OdeRobot* vehicle = new Nimm4(odeHandle, osgHandle, "Nimm4");
			((Nimm4*)vehicle)->setBodyColor(Color(0.0,0.0,1.0));
			((Nimm4*)vehicle)->setBodyTexture("../../../../osg/data/Images/wood.rgb");
			((Nimm4*)vehicle)->setWheelTexture("../../../../osg/data/Images/chess.rgb");
			vehicle->place(Pos(0,1,0));

			// create pointer to controller
			AbstractController *controller = new RandomController();
			global.configs.push_back(controller);

			// create pointer to one2onewiring
			One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));

			// create pointer to agent
			OdeAgent* agent = new OdeAgent(plotoptions);
			agent->init(controller, vehicle, wiring);
			// show robot´s trace in simulation
			TrackRobot* track = new TrackRobot(/*bool trackPos*/false,
					/*bool trackSpeed*/false,
					/*bool trackOrientation*/false,
					/*bool displayTrace*/true //,
			/*const char *scene="", int interval=1*/);
			agent->setTrackOptions(*track);
			global.agents.push_back(agent);
		}
		// OPTION:
		bool manually_steered_robot = false;
		if (manually_steered_robot){
			// 2nd robot
			OdeRobot* vehicle2 = new Nimm4(odeHandle, osgHandle, "Nimm4",
					/*double size=*/1.0, /*double force =3*/1.5, /*double speed=*/15,
					/*bool sphereWheels =*/true);

			((Nimm4*)vehicle2)->setBodyColor(Color(1.0,0.0,0.0));
			((Nimm4*)vehicle2)->setBodyTexture("../../../../osg/data/Images/wood.rgb");
			((Nimm4*)vehicle2)->setWheelTexture("../../../../osg/data/Images/chess.rgb");
			vehicle2->place(Pos(1,1,0));
			ocontroller = new OpenInvertNChannelController(10);
			ocontroller->setParam("eps", 0.1);
			ocontroller->setBiasUpdateRule(no); //bias is always zero
			global.configs.push_back(ocontroller);

			// create pointer to one2onewiring
			AbstractWiring*  wiring2 = new One2OneWiring(new ColorUniformNoise(0.1));

			// create pointer to agent
			OdeAgent* agent2 = new OdeAgent(plotoptions);
      agent2->init(ocontroller, vehicle2, wiring2);
			TrackRobot* track2 = new TrackRobot(/*bool trackPos*/false,
					/*bool trackSpeed*/false,
					/*bool trackOrientation*/false,
					/*bool displayTrace*/true //,
			/*const char *scene="", int interval=1*/);
			agent2->setTrackOptions(*track2);
			global.agents.push_back(agent2);
		}

		bool qlearning_robot = true;
		if (qlearning_robot){
			// 3rd robot
			//	((Nimm4*)vehicle3)->setRelPosSensorReference(global.obstacles.back()->getMainPrimitive());
			Nimm4Conf nimm4conf = Nimm4::getDefaultConf();
			nimm4conf.rpos_sensor_references.push_back(obst.at(0)->getMainPrimitive());
			nimm4conf.rpos_sensor_references.push_back(obst.at(1)->getMainPrimitive());
			nimm4conf.rpos_sensor_references.push_back(obst.at(2)->getMainPrimitive());
			nimm4conf.rpos_sensor_references.push_back(obst.at(3)->getMainPrimitive());
			OdeRobot* vehicle3 = new Nimm4(odeHandle, osgHandle, "Nimm4",
					/*double size=*/1.0, /*double force =3*/1.5, /*double speed=*/15,
					/*bool sphereWheels =*/true, nimm4conf);

			((Nimm4*)vehicle3)->setBodyColor(Color(0.0,1.0,0.0));
			((Nimm4*)vehicle3)->setBodyTexture("../../../../osg/data/Images/wood.rgb");
			((Nimm4*)vehicle3)->setWheelTexture("../../../../osg/data/Images/chess.rgb");

			vehicle3->place(Pos(0,0,0));
			//Homeo + Q
			QLearningHomeokMControllerConf qconf= QLearningHomeokMController::getDefaultConf();
			qconf.number_qlearner=4;
			qcontroller = new QLearningHomeokMController(qconf);
			//Q Lernen
			//	qcontroller = new QLearningMController();
			qcontroller->setParam("eps", 0.1);
			global.configs.push_back(qcontroller);

			// create pointer to one2onewiring
			AbstractWiring*  wiring3 = new One2OneWiring(new ColorUniformNoise(0.1));

			// create pointer to agent

			plotoptions.push_back(PlotOption(File));
			OdeAgent* agent3 = new OdeAgent(plotoptions);
      agent3->init(qcontroller, vehicle3, wiring3);
			TrackRobot* track3 = new TrackRobot(/*bool trackPos*/true,
					/*bool trackSpeed*/false,
					/*bool trackOrientation*/false,
					/*bool displayTrace*/true //,
			/*const char *scene="", int interval=1*/);
			agent3->setTrackOptions(*track3);
			global.agents.push_back(agent3);
		}

		showParams(global.configs);
	}

	// add own key handling stuff here, just insert some case values
	virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
	{
		if (down) { // only when key is pressed, not when released
			std::vector<double> f;
			switch ( (char) key )
			{
			case 'a':
				if (qcontroller-> learn){
				qcontroller-> learn=false;
				std::cout <<"deactivated learning "<< std::endl;
				} else {
					qcontroller-> learn=true;
					std::cout <<"activated learning "<< std::endl;
				}
				break;
			case 't':
				//				qcontroller->homeokinetic_controller -> setBiasUpdateRule(org);
				break;
			case 'x':
				std::cout<<"dropping spheres"<< std::endl;
				for (int i=0; i<fixator.size(); i++){
					delete fixator.at(i);
				}
				fixator.clear();
				break;
			case 'l': // turn left
				std::cout <<"turn left"<< std::endl;
				f= ocontroller->getMotorCommandFactor();
				f.at(0)-=0.2;
				f.at(2)-=0.2;
				ocontroller->setMotorCommandFactor(f);
				std::cout<<"f= "<<f.at(0)<<"  "<<f.at(1)<<"  "<<f.at(2)<<"  "<<f.at(3)<<"  "<<std::endl;
				break;
			case 'r': // turn right
				f = ocontroller->getMotorCommandFactor();
				f.at(1)-=0.2;
				f.at(3)-=0.2;
				ocontroller->setMotorCommandFactor(f);
				std::cout<<"f= "<<f.at(0)<<"  "<<f.at(1)<<"  "<<f.at(2)<<"  "<<f.at(3)<<"  "<<std::endl;
				break;
			case 'n': // turn right
				f = ocontroller->getMotorCommandFactor();
				for (int i=0; i<4; i++){
					f.at(i)=1;
				}
				std::cout<<"f= "<<f.at(0)<<"  "<<f.at(1)<<"  "<<f.at(2)<<"  "<<f.at(3)<<"  "<<std::endl;
				ocontroller->setMotorCommandFactor(f);
				break;
			case 's': // activate steering control
				//qcontroller->setProgrammedSteeringControl(true);
				qcontroller->setLearnedSteeringControl(true);
				std::cout<<"steering activated"<<std::endl;
				break;
			case 'd': // deactivate steering control
				//qcontroller->setProgrammedSteeringControl(false);
				qcontroller->setLearnedSteeringControl(false);
				std::cout<<"steering deactivated"<<std::endl;
				break;
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
	//			qcontroller->setMC(-0.3, 0.3);
				qcontroller->setMC(0.5, 1.0);

				break;
			case 'k': // turn right
//				qcontroller->setMC(0.3, -0.3);
				qcontroller->setMC(1.0, 0.5);

				break;
			case ' ': // stop
				qcontroller->setMC(1,1);
				break;
			default:
				return false;
				break;
			}
		}
		return false;
	}



};


int main (int argc, char **argv)
{
	ThisSim sim;
	sim.setCaption("lpzrobots Simulator");
	return sim.run(argc, argv) ? 0 : 1;

}

