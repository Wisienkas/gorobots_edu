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


/**
 * "Coordinated Locomotion for a Collaborative Task" experiment (P.68)
 * Multiple CPGs with Local Feedback Mechanisms for Locomotor Adaptation of Hexapod Robots
 * To enable sensory feedback mechanisms, you should follow the following instructions
 *	To enable phase reset and inhibition mechanisms for locomotor Adaptation, press Ctrl+e.
 * To enable continuous local sensory feedback mechanism for locomotor adaptation, press'C'.
 * To enable combined feedforward/continuous local sensory feedback mechanism
 * press 'M' then press one of the following '!', '@','#' or '$' to specify the gait, and enable Continuous Local Sensory
 * Feedback Mechanism by entering.
 *
 *
 * author: Subhi Shaker Barikhan
 * Date 27.05.2014
 */
// include simulation environment stuff
#include <ode_robots/simulation.h>
// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>
// playground
#include <ode_robots/playground.h>
// simple wiring
#include <selforg/one2onewiring.h>
// the robot
#include <ode_robots/amosII.h>
// the controller
//#include "amosIIcontrol.h"
#include "adaptivecontroller.h"
// joint needed for fixation of the robot in the beginning
#include <ode_robots/joint.h>
#include <ode_robots/octaplayground.h>
// add head file for creating a sphere by Ren ------------
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivemesh.h>
#include <ode_robots/passivebox.h>
#include <selforg/abstractcontroller.h>
#include <ode_robots/color.h>
#include <ode_robots/terrainground.h>

//#include <NeuralLocomotionControlAdaptiveClimbing.h>
#include <iostream>
using namespace std;

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;


std::vector<lpzrobots::AbstractObstacle*> obst;
//std::vector<lpzrobots::FixedJoint*> fixator;
// add head file for creating a sphere by Ren ------------

class ThisSim : public lpzrobots::Simulation {
  public:

  ThisSim(){
    addPaletteFile("colors/UrbanExtraColors.gpl");
    addColorAliasFile("colors/UrbanColorSchema.txt");
    // you can replace color mappings in your own file, see colors/UrbanColorSchema.txt
    // addColorAliasFile("myColorSchema.txt");
     obstweight=2.0;
   setGroundTexture("Images/whiteground.jpg"); // gets its color from the schema
    setTitle("Two Robots holding an Object");
    //setCaption("right aligned text");
  }

  /**
   * starting function (executed once at the beginning of the simulation loop)
   */
  virtual void start(const lpzrobots::OdeHandle& odeHandle,
      const lpzrobots::OsgHandle& osgHandle,
      lpzrobots::GlobalData& global) {
    // set initial camera position
    setCameraHomePos(
        lpzrobots::Pos(-0.0114359, 6.66848, 0.922832),
        lpzrobots::Pos(178.866, -7.43884, 0));

    // initialization
            // - set noise to 0.1
            // - register file chess.ppm as a texture called chessTexture (used for the wheels)
            global.odeConfig.noise=0.05;
            //    global.odeConfig.setParam("gravity", 0);
            global.odeConfig.setParam("controlinterval", 5);
            global.odeConfig.setParam("simstepsize", 0.01);

            //lpzrobots::AmosIIConf myAmosIIConf = lpzrobots::AmosII::getDefaultConf(1.0 /*_scale*/,1 /*_useShoulder*/,1 /*_useFoot*/,1 /*_useBack*/);

            bool use_amosii_version1 = false;
               bool use_amosii_version2 = true;

               if (use_amosii_version1 && !use_amosii_version2){
                     std::cout<<std::endl<<std::endl<<"AMOSII  VERSION 1 SELECTED!"<<std::endl<<std::endl;
                     // using amosii version 1
                     // Add amosII robot
                     myAmosIIConf = lpzrobots::AmosII::getAmosIIv1Conf(1.0 /*_scale*/,1 /*_useShoulder*/,1 /*_useFoot*/,0 /*_useBack*/, true /*for using foot feedback mechanism*/);
                     myAmosIIConf.rubberFeet = true;
                     myAmosIIConf2 = lpzrobots::AmosII::getAmosIIv1Conf(1.0 /*_scale*/,1 /*_useShoulder*/,1 /*_useFoot*/,0 /*_useBack*/, true /*for using foot feedback mechanism*/);
                                     myAmosIIConf2.rubberFeet = true;

                     amos = new lpzrobots::AmosII(
                         odeHandle,
                         osgHandle.changeColor(lpzrobots::Color(1, 1, 1)),
                         myAmosIIConf, "AmosIIv1_FrontRobot");

                     //// second amos
                     amos_Assistent = new lpzrobots::AmosII(
                                         odeHandle,
                                         osgHandle.changeColor(lpzrobots::Color(1, 1, 1)),
                                         myAmosIIConf2, "AmosIIv1_HindRobot");
                     ////

                     adaptiveCont = new AdaptiveController(/*amos version*/1);
                     adaptiveCont_2 = new AdaptiveController(/*amos version*/1);

                     obj = new lpzrobots::PassiveSphere(odeHandle, osgHandle,0.115,obstweight);

                      obj->setPosition(osg::Vec3(-4.5-(myAmosIIConf.size)/2-0.05, 0.0, 0.1));
                 //     obj->setTexture("Images/dusty.rgb");
                      obj->setColor(lpzrobots::Color(0.5,0.75,0.25));
                         obst.push_back(obj);
                         global.obstacles.push_back(obj);


                    // controller = new AmosIIControl();
                   } else {
                     std::cout<<"select only one version of AMOSII !"<<std::endl;
                     assert(use_amosii_version1 != use_amosii_version2);
                   }

                   if (use_amosii_version2 && !use_amosii_version1){
                     std::cout<<std::endl<<std::endl<<"AMOSII  VERSION 2 SELECTED!"<<std::endl<<std::endl;
                     // using amosii version 2
                     // Add amosII robot
                     myAmosIIConf = lpzrobots::AmosII::getAmosIIv2Conf(1.0 /*_scale*/,1 /*_useShoulder*/,1 /*_useFoot*/,0 /*_useBack*/,true /*for using foot feedback mechanism*/);
                 //    lpzrobots::AmosIIConf myAmosIIConf1 = lpzrobots::AmosII::getAmosIIv1Conf();
                     myAmosIIConf.rubberFeet = true;


                       myAmosIIConf2 = lpzrobots::AmosII::getAmosIIv2Conf(1.0 /*_scale*/,1 /*_useShoulder*/,1 /*_useFoot*/,0 /*_useBack*/,true /*for using foot feedback mechanism*/);
                       myAmosIIConf2.rubberFeet = true;
                       myAmosIIConf2.legContactSensorIsBinary=false;
                     amos = new lpzrobots::AmosII(
                         odeHandle,
                         osgHandle.changeColor(lpzrobots::Color(1, 1, 1)),
                         myAmosIIConf, "AmosIIv2_FrontRobot");
                     ///second amos
                     amos_Assistent = new lpzrobots::AmosII(
                                                         odeHandle,
                                                         osgHandle.changeColor(lpzrobots::Color(1, 1, 1)),
                                                         myAmosIIConf2, "AmosIIv2_HindRobot");
                     adaptiveCont = new AdaptiveController(/*amos version*/2);
                     adaptiveCont_2 = new AdaptiveController(/*amos version*/2);



                  obj = new lpzrobots::PassiveSphere(odeHandle, osgHandle,0.13,obstweight);
                   obj->setPosition(osg::Vec3(-4.5-(myAmosIIConf.size)/2-0.05, 0.0, 0.1));
              //     obj->setTexture("Images/dusty.rgb");
                   obj->setColor(lpzrobots::Color(0.5,0.75,0.25));
                      obst.push_back(obj);
                      global.obstacles.push_back(obj);

                   }
    //the second sphere
    lpzrobots::PassiveSphere* s2 = new lpzrobots::PassiveSphere(odeHandle, osgHandle, 0.1);
    s2->setPosition(osg::Vec3(0.0, 3.0, 0.1));
    s2->setTexture("Images/dusty.rgb");
    s2->setColor(lpzrobots::Color(0,1,0));
    obst.push_back(s2);
    global.obstacles.push_back(s2);
    lpzrobots::FixedJoint* fixator2 = new  lpzrobots::FixedJoint(s2->getMainPrimitive(), global.environment);
    fixator2->init(odeHandle, osgHandle);

    //the third sphere
    lpzrobots::PassiveSphere* s3 = new lpzrobots::PassiveSphere(odeHandle, osgHandle, 0.1);
    s3->setPosition(osg::Vec3(0.0, -3.0, 0.1));
    s3->setTexture("Images/dusty.rgb");
    s3->setColor(lpzrobots::Color(0,0,1));
    obst.push_back(s3);
    global.obstacles.push_back(s3);
    lpzrobots::FixedJoint* fixator3 = new  lpzrobots::FixedJoint(s3->getMainPrimitive(), global.environment);
    fixator3->init(odeHandle, osgHandle);

    //----------create a sphere as the target by Ren-----------------------------


    lpzrobots::OdeHandle rodeHandle = odeHandle;
    rodeHandle.substance = lpzrobots::Substance(3.0, 0.0, 50.0, 0.8);
    //------------------- Link the sphere to the Goal Sensor by Ren---------------
    for(unsigned int i = 0; i<obst.size(); i++)
    {
      myAmosIIConf.GoalSensor_references.push_back(obst.at(i)->getMainPrimitive());
    }


    // define the usage of the individual legs
    amos->setLegPosUsage(amos->L0, amos->LEG);
    amos->setLegPosUsage(amos->L1, amos->LEG);
    amos->setLegPosUsage(amos->L2, amos->LEG);
    amos->setLegPosUsage(amos->R0, amos->LEG);
    amos->setLegPosUsage(amos->R1, amos->LEG);
    amos->setLegPosUsage(amos->R2, amos->LEG);


    amos_Assistent->setLegPosUsage(amos_Assistent->L0, amos_Assistent->LEG);
    amos_Assistent->setLegPosUsage(amos_Assistent->L1, amos_Assistent->LEG);
    amos_Assistent->setLegPosUsage(amos_Assistent->L2, amos_Assistent->LEG);
    amos_Assistent->setLegPosUsage(amos_Assistent->R0, amos_Assistent->LEG);
    amos_Assistent->setLegPosUsage(amos_Assistent->R1, amos_Assistent->LEG);
    amos_Assistent->setLegPosUsage(amos_Assistent->R2, amos_Assistent->LEG);

    amos_Assistent->setColor(Color(1.0,1.0,0.0));


    // put amos a little bit in the air
    amos->place(osg::Matrix::translate(-4.5,0,0.07));//0.5
    amos_Assistent->place( osg::Matrix::translate(-4.5-myAmosIIConf.size-0.16,0,0.07));//  //-4.5-myAmosIIConf.size-0.225 amos I

 //   amos_Assistent->place(osg::Matrix::translate(-4.5,0.7,0.07));
    //empCont = new EmptyController();

    controller=adaptiveCont;
    controller_2=adaptiveCont_2;
    // create wiring
    One2OneWiring* wiring1 = new One2OneWiring(new ColorUniformNoise());
    One2OneWiring* wiring2 = new One2OneWiring(new ColorUniformNoise());
    // create agent and init it with controller, robot and wiring
    lpzrobots::OdeAgent* agent = new lpzrobots::OdeAgent(global);
    agent->init(controller, amos, wiring1);

    lpzrobots::OdeAgent* agent2 = new lpzrobots::OdeAgent(global);

    agent2->init( controller_2,  amos_Assistent, wiring2);
    // create a fixed joint to hold the robot in the air at the beginning

    robotfixator = new lpzrobots::FixedJoint(
        amos->getMainPrimitive(),
        global.environment);
    robotfixator->init(odeHandle, osgHandle, false);

   robotfixator_1=new lpzrobots::FixedJoint(
       amos_Assistent->getMainPrimitive(),
       global.environment);
    robotfixator_1->init(odeHandle, osgHandle, false);



    robotfixator_2=new lpzrobots::FixedJoint(
        amos->getShoulderPrimitive(AmosII::L2),
        amos_Assistent->getShoulderPrimitive(AmosII::L0)); //amos_Assistent->getTibiaPrimitive(AmosII::L0)
    robotfixator_2->init(odeHandle, osgHandle, false);

    robotfixator_3=new lpzrobots::FixedJoint(
        amos->getShoulderPrimitive(AmosII::R2),
           amos_Assistent->getShoulderPrimitive(AmosII::R0));
        robotfixator_3->init(odeHandle, osgHandle, false);



        adaptiveCont->ConnectTwoRobotsVert(1);
        adaptiveCont_2->ConnectTwoRobotsVert(2);
 //   delete robotfixator;
   // robotfixator = NULL;
    // inform global variable over everything that happened:
    global.configs.push_back(amos);
    global.agents.push_back(agent);
    global.configs.push_back(controller);


    global.configs.push_back(amos_Assistent);
       global.agents.push_back(agent2);
       global.configs.push_back(controller_2);

    std::cout << "\n\n"
        << "################################\n"
        << "#   Press x to free amosII!    #\n"
        << "################################\n"
        << "\n\n" << std::endl;

  }

  /**
   * add own key handling stuff here, just insert some case values
   */
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
            delete robotfixator_1;
            robotfixator = NULL;
            robotfixator_1 = NULL;
          }
            break;
          case 'e'://enable foot contacts sensors
        	  adaptiveCont->enableFootsensors();
            adaptiveCont_2->enableFootsensors();
            break;
          case 'd'://disable foot contacts sensors
        	  adaptiveCont->disableFootsensors();
            adaptiveCont_2->disableFootsensors();
            break;
          case 'q'://increase modulatory input (frequency) by 0.01
        	  adaptiveCont->increaseFrequency();
                   adaptiveCont_2->increaseFrequency();
                   break;
          case 's': //decrease modulatory input (frequency)by 0.01
        	  adaptiveCont->decreaseFrequency();
                 adaptiveCont_2->decreaseFrequency();
                  break;
          case 'l':
        	  adaptiveCont->enableLift();
                   adaptiveCont_2->enableLift();
                  break;
          case 'k':
        	  adaptiveCont->disableLift();
                  adaptiveCont_2->disableLift();
                  break;
          case 'E':
        	  adaptiveCont->enableCoxawithContactSensorSignal();
                     adaptiveCont_2->enableCoxawithContactSensorSignal();
                     break;
           case 'D':
        	   adaptiveCont->disableCoxawithContactSensorSignal();
                     adaptiveCont_2->disableCoxawithContactSensorSignal();
                     break;
          case 9://Ctrl+i  //increase weight of the carried object  (experiment about two connected robots)
                      obstweight+=0.2;
                      obj->getMainPrimitive()->setMass(obstweight);
                      obj->update();
                      adaptiveCont->printObjectWeight(obstweight);
                       break;
          case  10://Ctrl+j //decrease weight of the carried object  (experiment about two connected robots)
          			obstweight-=0.2;
                      obj->getMainPrimitive()->setMass(obstweight);
                      obj->update();
                      adaptiveCont->printObjectWeight(obstweight);
                      break;


           case 'W': //middle Leg Amputation
        	   adaptiveCont->enableLegAmputation(); //Leg Amputation Enable
                     adaptiveCont_2->enableLegAmputation(); //Leg Amputation Enable

                     break;
           case  'w':// intact robot
        	   adaptiveCont->disableLegAmputation();
                    adaptiveCont_2->disableLegAmputation();
                    break;
           case 'C':
        	   adaptiveCont->enableContactForceMech();
                    adaptiveCont_2->enableContactForceMech();
                     break;
           case  'c':
        	   adaptiveCont->disableContactForceMech();
                     adaptiveCont_2->disableContactForceMech();
                     break;
           case 'H': // push object missions (P.59)
        	   adaptiveCont->pushObject();
          	  	  break;
           case 'M'://enable oscillator coupling (fully connected network)
        	   adaptiveCont->enableOscillatorCoupling();
                   adaptiveCont_2->enableOscillatorCoupling();
                   break;
           case  'm': //disable oscillator coupling (fully connected network)
        	   adaptiveCont->disableOscillatorCoupling();
                   adaptiveCont_2->disableOscillatorCoupling();
                   break;
           case 'R': // oscillators are connected as ring
        	   adaptiveCont->enableOscillatorRingCoupling();
                   adaptiveCont_2->enableOscillatorRingCoupling();
                   break;
           case  'r':
        	   adaptiveCont->disableOscillatorRingCoupling();
                   adaptiveCont_2->disableOscillatorRingCoupling();
                   break;
           case  '#':// tetrapod
        	   adaptiveCont->enableTetrapodGait();
                       adaptiveCont_2->enableTetrapodGait();
                       break;
            case  '@': //wave
            	adaptiveCont->enableWaveGait();
                       adaptiveCont_2->enableWaveGait();
                    break;
            case  '!': //tripod
            	adaptiveCont->enableTripodGait();
                     adaptiveCont_2->enableTripodGait();
                     break;
            case  '$'://irregular
            	adaptiveCont->enableIrregularGait();
                     adaptiveCont_2->enableIrregularGait();
                     break;

           case 5://Ctrl+e // enable 'Phase Reset and Inhibition Mechanisms'
        	   adaptiveCont->enablePhaseResetInhibitionMech();
                   adaptiveCont_2->enablePhaseResetInhibitionMech();
                  break;
           case 4://Ctrl+d // disable 'Phase Reset and Inhibition Mechanisms'
        	   adaptiveCont->disablePhaseResetInhibitionMech();
                  adaptiveCont_2->disablePhaseResetInhibitionMech();
                   break;
           case 'P':// enable speed calculation
        	   adaptiveCont->calculateSpeed();
        	   adaptiveCont_2->calculateSpeed();
             break;
           case 'p': // disable speed calculation
        	   adaptiveCont->disableCalculateSpeed();
        	   adaptiveCont_2->disableCalculateSpeed();
               break;

          case  '%':// wave gait for the fore robot and tripod gait for the rear robot
        	  adaptiveCont->enableWaveGait();
                    adaptiveCont_2->enableTripodGait();
               break;
          case  '^': // wave gait for the fore robot and tetrapod gait for the rear robot
        	  adaptiveCont->enableWaveGait();
        	  adaptiveCont_2->enableTetrapodGait();
               break;

          default:
            return false;
            break;
        }
      }
      return false;
    }
  protected:
  lpzrobots::Joint* robotfixator;
  lpzrobots::Joint* robotfixator_1;
  lpzrobots::Joint* robotfixator_2;
  lpzrobots::Joint* robotfixator_3;
  lpzrobots::Joint* robotfixator_4;
  lpzrobots::Joint* robotfixator_5;
  AbstractController* controller;
  AbstractController* controller_2;
   double obstweight;
  lpzrobots::PassiveSphere* obj;

  AdaptiveController* adaptiveCont; // fore robot's controller
  AdaptiveController* adaptiveCont_2; //rear robot's controller
  lpzrobots::AmosII* amos;
  lpzrobots::AmosII* amos_Assistent;
  lpzrobots::AmosIIConf myAmosIIConf;
  lpzrobots::AmosIIConf myAmosIIConf2;
};

int main(int argc, char **argv)
{
  ThisSim sim;
//  sim.setGroundTexture("Images/greenground.rgb");
  return sim.run(argc, argv) ? 0 : 1;
}

