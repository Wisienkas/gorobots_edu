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
#include <ode_robots/mirmorph.h>
#include <mirmorphrpos.h>
#include <ode_robots/nimm2.h>
#include <ode_robots/nimm4.h>

#include <selforg/trackrobots.h>
// used arena
#include <ode_robots/playground.h>
#include <ode_robots/terrainground.h>
// used passive spheres
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>

#include "../controller/ico_controller.h"


//#include "emptycontroller.h"

// OR an example controller
//#include "samplecontroller.h"


// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;


EmptyController* qcontroller;


////Use this setup for the task ICO learning
//when select--> "emptycontroller_icolearning.h"

//// students set this parameter = "true" for ico learning, and set it to "false" for RL
//bool ico_learning_task = true;

// students set this to "true" for reset robot in ico learning, set to "false" for RL
// set this to "false" to control the robot by keyboard for checking the robot sensory signals
//bool repeat_experiment = true;


//Use this setup for the task RL & others
//when select--> "emptycontroller.h" or samplecontroller.h

// students set this parameter = "true" for ico learning, and set it to "false" for RL
bool ico_learning_task = true;
//
//// students set this to "true" for reset robot in ico learning, set to "false" for RL
//// set this to "false" to control the robot by keyboard for checking the robot sensory signals
bool repeat_experiment = true;


bool obstacle_on = true;
bool use_rough_ground = true;
bool drawtrace_on = true;//---------------------------------------------------------------------------TEST

bool random_positon_spheres = false;
bool random_positon = false;//true;
bool random_positon_frist = false;//-------------------------------------------------CH
bool random_orientation = true;

#define MAX_x   11.0//10.0 //11.0//9.0	//Max range-------------------------------------------------CH VERY IMPORTANT FACTOR
#define MIN_x   4.0  //5.0 (TEST) //Min range-------------------------------------------------CH VERY IMPORTANT FACTOR

#define MAX_y   2.5//5.0 // 9.0 	//Max range
#define MIN_y   -2.5//-5.0//-9.0 	//Min range

int pos_obstacle_x = 7.0;
int pos_obstacle_y = 0.0;

#define MAX_or	M_PI/3 // 60 deg
#define MIN_or	-M_PI/3 // 60 deg

double random_or;
double random_position_S;
double random_position;
double random_positionx;
double random_positiony;


int number_spheres = 1; // SET NUMBER of TARGET MAX = 4
int number_boxes = 1; // SET NUMBER of BOXES obstacles

int repeat_number = 100;// 1000;
double time_factor = 0.25*2;//0.25/2;

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

bool reboot_sim = false;
double x[2] = {0,0};
double y[2] = {0,0};
double traveled_distance = 0;

std::ofstream weights_log;

double _x[2];
double _y[2];
double _traveled_distance;
double _IR_max[8];
double _tilt[2];
std::vector<double> _tilt_vector;
std::vector<double> _speeds;
std::vector<std::vector<double>> _results;
std::vector<double> fitness_avg;

double rel_x, rel_y;

std::ofstream fitnessFile;

std::ofstream ballcorr, leftcorr, rightcorr;

class ThisSim : public Simulation {
public:

    // starting function (executed once at the beginning of the simulation loop)
    void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
    {
        weights_log.open("weights.csv");
        weights_log << "Repetition,w0,w1,w2" << std::endl;

        fitnessFile.open("fitness_function.csv", std::ios::trunc);
        fitnessFile << "Generation,Fitness value" << std::endl;

        ballcorr.open("Correlation/ball0.csv");
        leftcorr.open("Correlation/left0.csv");
        rightcorr.open("Correlation/right0.csv");

        // set initial camera position
        setCameraHomePos(Pos(-1.14383, 10.1945, 42.7865),  Pos(179.991, -77.6244, 0));


        // initialization simulation parameters
        //1) - set noise to 0.1
        global.odeConfig.noise= 0.00;//0.05;

        //2) - set controlinterval -> default = 1
        global.odeConfig.setParam("controlinterval", 1);/*update frequency of the simulation ~> amos = 20*/
        //3) - set simulation setp size
        global.odeConfig.setParam("simstepsize", 0.01); /*stepsize of the physical simulation (in seconds)*/
        //Update frequency of simulation = 1*0.01 = 0.01 s ; 100 Hz

        //4) - set gravity if not set then it uses -9.81 =earth gravity
        //global.odeConfig.setParam("gravity", -9.81);

        _x[0] = 15.0;
        _x[1] = 0.0;
        _y[0] = 0.0;
        _y[1] = 0.0;
        _traveled_distance = 0;
        for(uint i=0; i<8; i++){
            _IR_max[i] = 0.0;
        }
        _tilt[0] = -10.0;
        _tilt[1] = 10.0;
        _tilt_vector.clear();

        _results.resize(repeat_number);

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
        double height_pg = 2.0;//0.0; //0.5

        Playground* playground = new Playground(odeHandle, osgHandle.changeColor(Color(0.6,0.0,0.6)),
                                                osg::Vec3(length_pg /*length*/, width_pg /*width*/, height_pg/*height*/), /*factorxy = 1*/1, /*createGround=true*/true /*false*/);
        playground->setPosition(osg::Vec3(4,0,0.0)); // playground positionieren und generieren
        // register playground in obstacles list
        global.obstacles.push_back(playground);


        PassiveSphere* s1;

        double target_z_position = 1.5;

        if(ico_learning_task)
        {
            target_z_position = 3.0;
        }


        for (int i=0; i < number_spheres; i++){
            s1 = new PassiveSphere(odeHandle, osgHandle, 0.5);

            //Target 1
            if(random_positon_spheres)
            {
                /*******Generating Random Position****/
                random_position_S  = ((MAX_x-MIN_x)*((float)rand()/RAND_MAX))+MIN_x; // Input 0 (m) between -2.4 and 2.4
                std::cout<<"\n\n\n\n\n"<<"Inital Random Target X position"<<" = "<<-1*random_position_S<<"\t"<<"Inital Random Target Y position"<<" = "<<random_position_S<<"\n\n\n\n";
                /************************************/
                if (i==0) s1->setPosition(osg::Vec3(-1*random_position_S,random_position_S,target_z_position/*2*/));
            }
            else
            {
                //position_S = 0.0;// -2.0---------------------------------------------------------------------------TEST
                if (i==0) s1->setPosition(osg::Vec3(-3.0 /*position_S*/, position_S+5, target_z_position/*0*/));
            }

            //Target 2

            //if (i==1) s1->setPosition(osg::Vec3(15-11.0 /*position_S*/, position_S+10, 0.5/*0*/));
            if (i==1) s1->setPosition(osg::Vec3(0.0 /*position_S*/, position_S-5, target_z_position/*0*/));

            //Target 3
            //if (i==2) s1->setPosition(osg::Vec3(15-11.0 /*position_S*/, position_S-10, 0.5/*0*/));
            if (i==2) s1->setPosition(osg::Vec3(0.0 /*position_S*/, position_S-5, target_z_position/*0*/));


            //Target 4
            if (i==3) s1->setPosition(osg::Vec3(-10.0 /*position_S*/, position_S, target_z_position/*0*/));
            //if (i==3) s1->setPosition(osg::Vec3(-10, 10,2/*2*/));

            s1->setTexture("Images/dusty.rgb");
            //Target 1
            if (i==0){ s1->setColor(Color(0,1,0)); }
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

        PassiveBox* b1;
        double length = 6.5;//1.5 /*for learning*/; //3.5 /*for*/;//testing//---------------------------------------------------------------------------TEST
        double width = 1.0;
        double height = 2.0;

        if(obstacle_on)
        {
            random_position_S = 0.0;
            for (int i=0; i < number_boxes /*SET NUMBER OF OBSTACLES*/; i++){
                b1 = new PassiveBox(odeHandle, osgHandle, osg::Vec3(length, width, height /*size*/));
                //b1->setTexture("dusty.rgb");
                b1->setColor(Color(1,0,0));
                //b1->setPosition(osg::Vec3(/*-4.5+*/i*4.5,3+i,0)); // Fixed robot position
                //osg::Matrix pose;
                //			pose.setTrans(osg::Vec3(/*-4.5+*/i*4.5,3+i,0));
                //b1->setPose(osg::Matrix::rotate(0.3*(M_PI/2)*i, 0,0, 1) * osg::Matrix::translate(/*-4.5+*/i*5,5+i,0.5) /* pose*/);
                //b1->setPose(osg::Matrix::rotate(/*-0.5*(M_PI/4)*/ (M_PI/2), 0,0, 1) * osg::Matrix::translate(/*i*4.5+*/pos_obstacle_x, pos_obstacle_y+i*-pos_obstacle_y*2.0/*0+i*/,height/2) /* pose*/);
                b1->setPose(osg::Matrix::rotate(/*-0.5*(M_PI/4)*/ (M_PI/2), 0, 0, 1) * osg::Matrix::translate(/*i*4.5+*/pos_obstacle_x, pos_obstacle_y+random_position_S/*i*-pos_obstacle_y*2.0*//*0+i*/,height/2+0.45) /* pose*/);
                global.obstacles.push_back(b1);
                fixator.push_back(  new  FixedJoint(b1->getMainPrimitive(), global.environment));  //create pointer
                fixator.at(i)->init(odeHandle, osgHandle);

            }
        }


        // Add rough terrain
        if(use_rough_ground){
            Substance roughterrainSubstance(1.0, 0.1, 500.0, 0.0);
            OdeHandle oodeHandle = odeHandle;
            oodeHandle.substance = roughterrainSubstance;

            TerrainGround* terrainground = new TerrainGround(oodeHandle, osgHandle.changeColor(Color(83.0/255.0, 48.0/255.0, 0.0/255.0)), "rough6.ppm", "", 34.9, 34.9, 0.2);
            terrainground->setPose(osg::Matrix::translate(4, 0, 0.19));
            global.obstacles.push_back(terrainground);
        }

        ////////////////////////////////////Call this set up and Control//////////////////////////////////////////////
        bool ac_ico_robot = true;
        if (ac_ico_robot)
        {

            //0)

            //qcontroller->setReset(1);

            //1) Activate IR sensors
            MirmorphRPosConf fconf = MirmorphRPos::getDefaultConf();
            fconf.mirconf.length = escale(0.417363, 0.8, 3.2);
            fconf.mirconf.width = escale(0.100928, 0.6, 2.4);
            fconf.mirconf.height = escale(0.00995006, 0.36, 1.44);
            for(uint i=0; i<4; i++){
                fconf.mirconf.radius[i] = escale(0.81144, 0.2, 3*fconf.mirconf.length/8.0);
                fconf.mirconf.wheelthickness[i] = escale(0.339151, 0.1, 0.6);
                fconf.mirconf.wheelheights[i] = escale(0.938729, 0, fconf.mirconf.height/2.0);
            }
            fconf.frontIR_angle = escale(0.631452, -M_PI/2, M_PI/2);
            fconf.sideTopIR_angle = escale(0.868055, 0, M_PI);
            fconf.sideBottomIR_angle = escale(0.182207, 0, M_PI);
            fconf.backIR_angle = escale(0.694402, -M_PI/2, M_PI/2);

//            fconf.irRangeFront = 4;
//            fconf.irRangeSide = 4;
//            fconf.irRangeBack = 3;

            //2) relative sensors
            for (int i=0; i < number_spheres; i++){
                fconf.rpos_sensor_references.push_back(obst.at(i)->getMainPrimitive());
            }

            OdeRobot* vehicle3 = new MirmorphRPos(odeHandle, osgHandle, fconf, "mir");



            /****Initial position of Nimm4******/

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
                vehicle3->place(Pos(position_x-15.0 /* 20 random_position*/  /*+x = to left (sphere in front of robot), -x = to right sphere behind robot*/, 0.0 /*-1*random_position*/ /*+y = to up, -y = to down*/,0.5/*z*/));
            }
            else
            {

                /*******Generating Random Position****/
                //srand (time(NULL));
                int r = rand();
                std::cout << "random value 2: " << r << std::endl;
                random_or  = ((MAX_or-MIN_or)*((float)r/RAND_MAX))+MIN_or; // between -pi/3 (60 deg) and pi/3 (60 deg)
                /************************************/

                double robot_z = fconf.mirconf.height/2.0 + fconf.mirconf.radius[0] + 0.3;
                Pos pos(position_x-5.0/*, +x = to left, -x = to right*/,0.0/*y*/,robot_z/*z*/);

                std::cout << "random_or1: " << random_or << std::endl;

                //setting position and orientation
                vehicle3->place(osg::Matrix::rotate(random_or, 0, 0, 1) *osg::Matrix::translate(pos));
                //setting only position
                //vehicle3->place(Pos(position_x-5.0/*5.5x, +x = to left, -x = to right*/,0.0/*y*/,0.0/*z*/));


            }


            qcontroller = new EmptyController("1","1");

            qcontroller->learn = true;

            global.configs.push_back(qcontroller);

            // create pointer to one2onewiring
            AbstractWiring*  wiring3 = new One2OneWiring(new ColorUniformNoise(0.0));

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
                                                    /*bool displayTrace*/true //,
                                                    /*const char *scene="", int interval=1*/);
                agent3->setTrackOptions(*track3);
            }

        }

        //bool random_controlled_robot =false;

        //Initialize results vector size
        results.resize(repeat_number);


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
                for (unsigned int i=0; i<fixator.size(); i++){
                    delete fixator.at(i);
                }
                fixator.clear();
                break;
            case 'q': // print Q-table
                //				qcontroller->printQTable();
                break;
            case 'r':
                reboot_sim = true;


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
        std::vector<double> w0 = qcontroller->ico_controller[0]->getWeights();
        std::vector<double> w1 = qcontroller->ico_controller[1]->getWeights();
        std::vector<double> w2 = qcontroller->ico_controller[2]->getWeights();

        weights_log << currentCycle << "," << w0[1] << "," << w1[1] << "," << w2[1] << std::endl;

        std::cout << "\n begin restart " << currentCycle << "\n";

        std::cout<<"Current Cycle"<<this->currentCycle<<std::endl;

        ballcorr.close();
        leftcorr.close();
        rightcorr.close();

        //Temporary variable storing the pointer to the controller if the latter should not be resetted
        AbstractController* temp_controller;

        //OdeAgent* temp_agent;

        // We would like to have 10 runs!
        // after it we must clean all and return false because we don't want a new restart
        if (this->currentCycle == repeat_number) // This will be delete all after finish simulation!!!!!!
        {
            char a;
            std::cin >> a;

            double last_fitness = 0;
            for(uint i=0; i<fitness_avg.size(); i++){
                last_fitness += fitness_avg[i]*(i+1);
            }
            last_fitness /= 15;

            fitnessFile << currentCycle+1 << "," << last_fitness << std::endl;

            weights_log.close();
            //clean robots
            while (global.agents.size() > 0)
            {
                OdeAgent* agent = *global.agents.begin();
                AbstractController* controller = agent->getController();
                OdeRobot* robot = agent->getRobot();
                AbstractWiring* wiring = agent->getWiring();

                global.configs.erase(std::find(global.configs.begin(),
                                               global.configs.end(), controller));
                //                delete controller;///////////////////////////////////////////////This will finally delete which is OK

                //                delete robot;

                //                delete wiring;

                delete agent;
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
            std::cout << "end." << std::endl;

            fitnessFile.close();

            return false; // don't restart, just quit
        }

        // Now we must delete all robots and agents from the simulation and create new robots and agents.
        // BUT NOT CONTROLLER for learning
        while (global.agents.size() > 0)
        {
            //	  std::cout << "\n MAIN WHILE LOOP" << currentCycle << "\n";
            OdeAgent* agent = *global.agents.begin();


            AbstractController* controller = agent->getController();

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

            //delete (agent);


            global.agents.erase(global.agents.begin());
            //	  std::cout << "\n END OF MAIN WHILE LOOP" << currentCycle << "\n";

        }

        ///////////////Recreate Robot Start//////////////////////////////////////////////////////////////////////////////////////
        //1) Activate IR sensors
        MirmorphRPosConf fconf = MirmorphRPos::getDefaultConf();
        fconf.mirconf.length = escale(0.417363, 0.8, 3.2);
        fconf.mirconf.width = escale(0.100928, 0.6, 2.4);
        fconf.mirconf.height = escale(0.00995006, 0.36, 1.44);
        for(uint i=0; i<4; i++){
            fconf.mirconf.radius[i] = escale(0.81144, 0.2, 3*fconf.mirconf.length/8.0);
            fconf.mirconf.wheelthickness[i] = escale(0.339151, 0.1, 0.6);
            fconf.mirconf.wheelheights[i] = escale(0.938729, 0, fconf.mirconf.height/2.0);
        }
        fconf.frontIR_angle = escale(0.631452, -M_PI/2, M_PI/2);
        fconf.sideTopIR_angle = escale(0.868055, 0, M_PI);
        fconf.sideBottomIR_angle = escale(0.182207, 0, M_PI);
        fconf.backIR_angle = escale(0.694402, -M_PI/2, M_PI/2);

//        fconf.irRangeFront = 4;
//        fconf.irRangeSide = 4;
//        fconf.irRangeBack = 3;

        //2) relative sensors
        for (int i=0; i < number_spheres; i++)
        {
            fconf.rpos_sensor_references.push_back(obst.at(i)->getMainPrimitive());
        }

        OdeRobot* vehicle3 = new MirmorphRPos(odeHandle, osgHandle, fconf, "mir");


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

            vehicle3->place(Pos(random_positionx/*10    +x = to left, -x = to right*/, random_positiony /*  +y = to close, -y = to far*/,0.5/*z*/));
        }
        else
        {
            vehicle3->place(Pos(position_x-5.0/*x, +x = to left, -x = to right*/,0.0/*y*/,0.5/*z*/));

        }




        if(random_orientation)
        {
            /*******Generating Random Position****/
            //srand (time(NULL));
            int r = rand();
            std::cout << "random value 3: " << r << std::endl;
            random_or  = ((MAX_or-MIN_or)*((float)r/RAND_MAX))+MIN_or; // between -pi/3 (60 deg) and pi/3 (60 deg)
            /************************************/

            double robot_z = fconf.mirconf.height/2.0 + fconf.mirconf.radius[0] + 0.3;
            Pos pos(position_x-5.0/*5.5x, +x = to left, -x = to right*/,0.0/*y*/,robot_z/*z*/);

            std::cout << "random_or2: " << random_or<<std::endl;

            //setting position and orientation
            vehicle3->place(osg::Matrix::rotate(random_or, 0, 0, 1) *osg::Matrix::translate(pos));
            //setting only position
            //vehicle3->place(Pos(position_x-5.0/*5.5x, +x = to left, -x = to right*/,0.0/*y*/,0.0/*z*/));

        }
        else
        {
            double robot_z = fconf.mirconf.height/2.0 + fconf.mirconf.radius[0] + 0.3;
            Pos pos(position_x-5.0/*5.5x, +x = to left, -x = to right*/,0.0/*y*/,0.5/*z*/);

            //setting position and orientation
            vehicle3->place(osg::Matrix::rotate(0.0, 0, 0, 1) *osg::Matrix::translate(pos));
            //setting only position
            //vehicle3->place(Pos(position_x-5.0/*5.5x, +x = to left, -x = to right*/,0.0/*y*/,0.0/*z*/));
        }

        if(currentCycle > repeat_number-5){
            double robot_z = fconf.mirconf.height/2.0 + fconf.mirconf.radius[0] + 0.3;
            random_or = (MAX_OR-MIN_OR)*(5-(repeat_number-currentCycle)-1)/4+MIN_OR;
            Pos pos(position_x-5.0/*5.5x, +x = to left, -x = to right*/,0.0/*y*/,robot_z/*z*/);

            //setting position and orientation
            vehicle3->place(osg::Matrix::rotate(random_or, 0, 0, 1) *osg::Matrix::translate(pos));
        }


        if(repeat_experiment)
        {
            qcontroller = (EmptyController*)temp_controller;
        }

        global.configs.push_back(qcontroller);


        // create pointer to one2onewiring
        AbstractWiring*  wiring3 = new One2OneWiring(new ColorUniformNoise(0.0));

        // create pointer to agent

        plotoptions.push_back(PlotOption(NoPlot /*select "File" to save signals, "NoPlot" to not save*/));
        OdeAgent* agent3 = new OdeAgent(plotoptions);
        //OdeAgent* agent3 = new OdeAgent(global);
        agent3->init(qcontroller, vehicle3, wiring3);

        if(drawtrace_on)
        {
            TrackRobot* track3 = new TrackRobot(/*bool trackPos*/true/*true*/,
                                                /*bool trackSpeed*/true,
                                                /*bool trackOrientation*/false,
                                                /*bool displayTrace*/ true //,
                                                /*const char *scene="", int interval=1*/);
            agent3->setTrackOptions(*track3);
        }
        global.agents.push_back(agent3);

        qcontroller->stop = false;

        double fitness = 0;
        fitness += results[currentCycle-1][14]*log10(100/results[currentCycle-1][2])/2.8;
        fitness += results[currentCycle-1][14]*log10(1/results[currentCycle-1][11]);
        fitness += results[currentCycle-1][14]*log10(2/results[currentCycle-1][12])/1.3;

        fitnessFile << currentCycle << "," << fitness << std::endl;

        if(currentCycle > repeat_number-5-1){
            fitness_avg.push_back(fitness);
        }

        _x[0] = 15.0;
        _x[1] = 0.0;
        _y[0] = 0.0;
        _y[1] = 0.0;
        _traveled_distance = 0;
        _speeds.clear();
        for(int i=0; i<8; i++){
            _IR_max[i] = 0.0;
        }
        _tilt[0] = -10.0;
        _tilt[1] = 10.0;
        _tilt_vector.clear();

        ballcorr.open("Correlation/ball"+std::to_string(currentCycle)+".csv");
        leftcorr.open("Correlation/left"+std::to_string(currentCycle)+".csv");
        rightcorr.open("Correlation/right"+std::to_string(currentCycle)+".csv");

        std::cout << "\n end restart " << currentCycle << "\n";
        // restart!

        if(currentCycle <= repeat_number-5){
            qcontroller->learn = true;
        }
        else{
            qcontroller->learn = false;
        }

        qcontroller->stop = false;

        return true;

    }
    /****************************************************************************************************/

    double escale(double input, double min, double max){
        double output;

        output = input * (max-min) + min;

        return output;
    }


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

        // Calculate traveled distance
        _x[1] = _x[0];
        _y[1] = _y[0];
        Position pos = globalData.agents.back()->getRobot()->getPosition();
        _x[0] = pos.x;
        _y[0] = pos.y;
        _traveled_distance += sqrt(pow(_x[0]-_x[1],2)+pow(_y[0]-_y[1],2));

        rel_x = 0 - pos.x;
        rel_y = 5 - pos.y;

        bool reached = false;
        if(sqrt(pow(-3-pos.x,2)+pow(5-pos.y,2))<0.7){
            reached = true;
        }

        Position vel = globalData.agents.back()->getRobot()->getSpeed();
        _speeds.push_back(sqrt(pow(vel.x,2)+pow(vel.y,2)));
        double avg_speed = 0;
        for(uint i=0; i<_speeds.size(); i++){
            avg_speed += _speeds[i];
        }
        avg_speed /= _speeds.size();

        for(uint i=0; i<8; i++){
            if(qcontroller->fitnessIR.at(i) > _IR_max[i]){
                _IR_max[i] = qcontroller->fitnessIR.at(i);
            }
        }

        double tilt_value = average_filter(_tilt_vector, qcontroller->orientation.at(2), 5);
        if(tilt_value > _tilt[0]){
            _tilt[0] = tilt_value;
        }
        else if(tilt_value < _tilt[1]){
            _tilt[1] = tilt_value;
        }

        matrix::Matrix rotation = globalData.agents.back()->getRobot()->getOrientation();
        double angle_x = atan2(rotation.val(2,1), rotation.val(2,2));
        double angle_y = atan2(-rotation.val(2,0), sqrt(pow(rotation.val(2,1),2)+pow(rotation.val(2,2),2)));
        double angle_z = atan2(rotation.val(1,0), rotation.val(0,0));

        std::vector<double> w0 = qcontroller->ico_controller[0]->getWeights();
        std::vector<double> w1 = qcontroller->ico_controller[1]->getWeights();
        std::vector<double> w2 = qcontroller->ico_controller[2]->getWeights();

        ballcorr << qcontroller->predictive_signal_green << "," << qcontroller->reflexive_signal_green << "," << w0[1] << "," << qcontroller->xb << "," << qcontroller->yb << "," << qcontroller->zb << "," << qcontroller->input_distance_s << std::endl;
        leftcorr << qcontroller->AL << "," << qcontroller->ALs << "," << w2[1] << std::endl;
        rightcorr << qcontroller->AR << "," << qcontroller->ARs << "," << w1[1] << std::endl;

        if(repeat_experiment)
        {
            /***********************Reset Function*********************************************/

            //if (globalData.sim_step>=(time_factor*60.0*200.000) /*|| qcontroller->distance <5/ *10* /|| qcontroller->failure_flag==1*/)//||qcontroller->Vt< -1000 ||qcontroller->Vt>1000)//(globalData.sim_step>=(time_factor*60.0*200.000) || qcontroller->distance <= 5 )// || qcontroller->distance > 250)// || qcontroller->failure_flag==1 /*parameter from acicocontroller.cpp*/)
            if (globalData.sim_step>=(time_factor*60.0*100.000) /*if time = 15s*/ || reboot_sim || (angle_x<0.5 && angle_x>-0.5) || reached)
            {
                qcontroller->stop = true;
                simulation_time_reached=true;
                reboot_sim = false;

                std::vector<double> partial_result;
                partial_result.resize(15);

                partial_result[0] = min(globalData.sim_step/(100.0), 30);
                partial_result[1] = traveled_distance;
                partial_result[2] = max(qcontroller->distance/10, 0.3);
                partial_result[11] = 0;
                for(uint i=0; i<8; i++){
                    partial_result[i+3] = _IR_max[i];
                    if(_IR_max[i] > partial_result[11]){
                        partial_result[11] = _IR_max[i];
                    }
                }

                partial_result[12] = abs(_tilt[0]-_tilt[1]);

                partial_result[13] = 0;

                partial_result[14] = avg_speed;

                results[this->currentCycle-1] = partial_result;
            }
            /***********************************************************************************/
        }

    }

    double average_filter(std::vector<double> values, double new_value, int size){
        double average = 0;
        if(values.size() < size){
            values.push_back(new_value);

            for(uint i=0; i<values.size(); i++){
                average += values[i];
            }
            average /= values.size();
        }
        else{
            values.erase(values.begin());
            values.push_back(new_value);

            for(int i=0; i<size; i++){
                average += values[i];
            }
            average /= size;
        }

        return average;
    }


    std::vector<std::vector<double>> results;
};


int main (int argc, char **argv)
{
    ThisSim sim;

    bool answer = sim.run(argc, argv);

    //	for(int i=0; i<repeat_number; i++){
    //	  for(int j=0; j<3; j++){
    //	    std::cout << sim.results[i][j] << " ";
    //	  }
    //	  std::cout << std::endl;
    //	}

    return answer;

}
