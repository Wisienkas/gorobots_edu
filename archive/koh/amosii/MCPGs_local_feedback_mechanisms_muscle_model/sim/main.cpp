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
 * press 'M' then press one of the following '!', '@','#' or '$' to specify the gait, and enable controllerinuous Local Sensory
 * Feedback Mechanism by entering.
 *
 *
 *
 * Adaptability to Morphological Change (P.60): press 'W' and enable one of the sensory feedback mechanisms which you
 * want to assess.
 *
 *	 Pushing an Object (P.59):
 *	 -set variable 'pushingobject' to true
 *	 -press 'H' and
 *	 -enable one of the sensory feedback mechanisms which you want to test.
 *
 *   Multiple stepping behavior if the front legs (P.57)
 *	 -press  'V':
 *	 -enable one of the sensory feedback mechanisms which you want to test.
 *
 *	 To deploy an irregular terrain, set IrregularTerrain to true. (P.57)
 *
 *
 *Utilization of muscle model:
 * -press 'U'
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
//#include <ode_robots/amosII.h>
#include "amosII.h"

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

    pushingobject= false;
    IrregularTerrain=false;//true;

    obstweight=2.1;// the weight of the carried object (experiment about changing the center of mass) // 1.7 hind legs //5.7 or 2.9 for front legs
   setGroundTexture("Images/whiteground.jpg"); // gets its color from the schema
    //setTitle("Local sensory feedback mechanisms");
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

            //lpzrobots::AmosIIConf myAmosIIConf = lpzrobots::AmosII::getDefaultConf(1.0 /*_scale*/,1 /*_useShoulder*/,1 /*_useFoot*/,1 /*_useBack*/,true /*for using foot feedback mechanism*/);


            // it is possible to choose between AMOSIIv1 and AMOSIIv2
               bool use_amosii_version1 = false;
               bool use_amosii_version2 =true ;

               if (use_amosii_version1 && !use_amosii_version2)//AMOSIIv1 is selected
				   {
						 std::cout<<std::endl<<std::endl<<"AMOSII  VERSION 1 SELECTED!"<<std::endl<<std::endl;
						 // using amosII version 1
						 // Add amosII robot
						 myAmosIIConf = lpzrobots::AmosII::getAmosIIv1Conf(1.0 /*_scale*/,1 /*_useShoulder*/,1 /*_useFoot*/,1 /*_useBack*/,true /*for using foot feedback mechanism*/);
						 myAmosIIConf.rubberFeet = true;

						 amos = new lpzrobots::AmosII(
							 odeHandle,
							 osgHandle.changeColor(lpzrobots::Color(1, 1, 1)),
							 myAmosIIConf, "AmosIIv1");
						 adaptiveCont = new AdaptiveController(/*amos version*/1);
						 //empCont_2 = new EmptyController(/*amos version*/1);
				   }
               else
                   {
                     std::cout<<"select only one version of AMOSII !"<<std::endl;
                     assert(use_amosii_version1 != use_amosii_version2);
                   }

                   if (use_amosii_version2 && !use_amosii_version1) //AMOSIIv2 is selected
                   {
                     std::cout<<std::endl<<std::endl<<"AMOSII  VERSION 2 SELECTED!"<<std::endl<<std::endl;
                       myAmosIIConf = lpzrobots::AmosII::getAmosIIv2Conf(1.0 /*_scale*/,1 /*_useShoulder*/,1 /*_useFoot*/,1 /*_useBack*/,true /*for using foot feedback mechanism*/);                     myAmosIIConf.rubberFeet = true;
                       myAmosIIConf.legContactSensorIsBinary=false;

                     amos = new lpzrobots::AmosII(
                         odeHandle,
                         osgHandle.changeColor(lpzrobots::Color(1, 1, 1)),
                         myAmosIIConf, "AmosIIv2");
                     adaptiveCont = new AdaptiveController(/*amos version*/2);



                     /************ determine the terrain************************/
                     // activate the following when you want to use another terrain.
                     if (IrregularTerrain)
                     {
                       //Using terrains in the folder
                       //TerrainGround* terrainground = new TerrainGround(odeHandle, osgHandle,"testTerrain.ppm","Images/pandafur.jpg",10.0,5.0,0.1);// terrains/terrain_bumpInDip128.ppm testTerrain5.ppm
                       //TerrainGround* terrainground = new TerrainGround(odeHandle, osgHandle,"testTerrain.ppm","Images/pandafur.jpg",10.0,5.0,0.1);// terrains/terrain_bumpInDip128.ppm testTerrain5.ppm

                       //Using terrain in Lpzrobots
                    	 TerrainGround* terrainground = new TerrainGround(odeHandle, osgHandle,"terrains/terrain_bumpInDip128.ppm","Images/pandafur.jpg",10.0,5.0,0.1);// terrains/terrain_bumpInDip128.ppm testTerrain5.ppm
                    	                      Substance* subst =new Substance();
                    	                  //  subst->toSnow(0.1);
                    	 terrainground->setPosition(osg::Vec3(0,0,-0.0)); // playground positionieren und generieren
                    	 global.obstacles.push_back(terrainground);
                    	                   //   terrainground->setSubstance(*subst);
                    	                    //  playground->setGroundTexture("Images/greenground.rgb");*/
                     }

                    /************ End************************/


                     /************ Playground as boundary************************/
					// use Playground as boundary:
                     /*  OctaPlayground* playground = new OctaPlayground(odeHandle, osgHandle, osg::Vec3(60, 0.2, 0.4), 4);

							playground->setGroundTexture("Images/greenground.rgb");
					playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
					global.obstacles.push_back(playground);*/
                     /************ End************************/
                   }



 /************ Pushing an object************************/
//This mission should be accompanied with adjusting the fore leg joints (see emptycontroller.h).

if (pushingobject)
{
 lpzrobots::PassiveBox* obj = new lpzrobots::PassiveBox(odeHandle, osgHandle, osg::Vec3(0.12, 0.5, 0.08));
 //  lpzrobots::PassiveSphere* obj = new lpzrobots::PassiveSphere(odeHandle, osgHandle,Spheresize,10.0);
 obj->setPosition(osg::Vec3(-4.5, 0.0, 0.02));
//     obj->setTexture("Images/dusty.rgb");
//   obj->setColor(lpzrobots::Color(0,1,0));
obst.push_back(obj);
global.obstacles.push_back(obj);
adaptiveCont->adjustJointanglesforpushing();
}
 //    double Spheresize = 0.18;

/************ENd (Pushing an object)************************/



 /************ change the center of mass by putting object on the robot************************/
 /** For better assessment, you should activate one of the following choices:
  note that, the position of the object depends on the position of the robot. Any change of the initial position of the robot requires
                   changing the position of the object.*/

 //****************************1) object on the hind part of the robot*****************************/
  /* obj = new lpzrobots::PassiveBox(odeHandle, osgHandle, osg::Vec3(0.05, 0.07,0.05),obstweight);// hind legs 0.05,0.07 ,0.05
     obj->setPosition(osg::Vec3(-5.07, 0.0, 0.9));*/


 //*****************************2) object on the fore part of the robot********************************/
  /* obj = new lpzrobots::PassiveBox(odeHandle, osgHandle, osg::Vec3(0.07, 0.058,0.02),obstweight);
     obj->setPosition(osg::Vec3(-4.765,0, 0.4));//-4.765*/

 //*****************************3) object on the middle part of the robot********************************/
 /*obj = new lpzrobots::PassiveBox(odeHandle, osgHandle, osg::Vec3(0.07, 0.058,0.02),obstweight);
     obj->setPosition(osg::Vec3(-4.89,0, 0.4));//-4.765*/



// whichever the choice was, you should activate the following code
  /*obj->setColor(lpzrobots::Color(1, 1, 1));
     obst.push_back(obj);
   global.obstacles.push_back(obj);*/


/************ End (change the center of mass by putting object on the robot)************************/




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

    // Add amosII robot
 //   lpzrobots::AmosIIConf myAmosIIConf = lpzrobots::AmosII::getDefaultConf(1.0 /*_scale*/,1 /*_useShoulder*/,1 /*_useFoot*/,1 /*_useBack*/);
  //  myAmosIIConf.rubberFeet = true;
    lpzrobots::OdeHandle rodeHandle = odeHandle;
    rodeHandle.substance = lpzrobots::Substance(3.0, 0.0, 50.0, 0.8);
    //------------------- Link the sphere to the Goal Sensor by Ren---------------
    for(unsigned int i = 0; i<obst.size(); i++)
    {
      myAmosIIConf.GoalSensor_references.push_back(obst.at(i)->getMainPrimitive());
    }
    //------------------- Link the sphere to the Goal Sensor by Ren---------------
   /* amos = new lpzrobots::AmosII(
        rodeHandle,
        osgHandle.changeColor(lpzrobots::Color(1, 1, 1)),
        myAmosIIConf, "AmosII");*/

    // define the usage of the individual legs
    amos->setLegPosUsage(amos->L0, amos->LEG);
    amos->setLegPosUsage(amos->L1, amos->LEG);
    amos->setLegPosUsage(amos->L2, amos->LEG);
    amos->setLegPosUsage(amos->R0, amos->LEG);
    amos->setLegPosUsage(amos->R1, amos->LEG);
    amos->setLegPosUsage(amos->R2, amos->LEG);



    // put amos a little bit in the air
    amos->place(osg::Matrix::translate(-4.9,0,0.1));//0.5 +-4.5 0.07


    controller=adaptiveCont;

    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise());

    // create agent and init it with controller, robot and wiring
    lpzrobots::OdeAgent* agent = new lpzrobots::OdeAgent(global);
    agent->init(controller, amos, wiring);



    // create a fixed joint to hold the robot in the air at the beginning
    robotfixator = new lpzrobots::FixedJoint(
        amos->getMainPrimitive(),
        global.environment);
    robotfixator->init(odeHandle, osgHandle, false);

    delete robotfixator;
   // robotfixator = NULL;
    // inform global variable over everything that happened:
    global.configs.push_back(amos);
    global.agents.push_back(agent);
    global.configs.push_back(controller);

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
            robotfixator = NULL;
          }
          break;
        case 'e'://enable foot contact sensors
        	adaptiveCont->enableFootsensors();
              break;
            case 'd'://disable foot contact sensors
            	adaptiveCont->disableFootsensors();
              break;
            case 'q'://increase modulatory input (frequency) by 0.01
            	adaptiveCont->increaseFrequency();
                     break;
            case 's'://decrease modulatory input (frequency) by 0.01
            	adaptiveCont->decreaseFrequency();
                    break;
            case 'l':
            	adaptiveCont->enableLift();
                    break;
            case 'k':
            	adaptiveCont->disableLift();
                    break;
            case 'E':
            	adaptiveCont->enableCoxawithContactSensorSignal();
                       break;
             case 'D':
            	 adaptiveCont->disableCoxawithContactSensorSignal();
                       break;
            case 9://Ctrl+i  //increase weight of object  (experiment about changing the center of mass)
                        obstweight+=0.2;
                        obj->getMainPrimitive()->setMass(obstweight);
                        obj->update();
                        adaptiveCont->printObjectWeight(obstweight);
                         break;
            case  10://Ctrl+j //decrease weight of object (experiment about changing the center of mass)
            			obstweight-=0.2;
                        obj->getMainPrimitive()->setMass(obstweight);
                        obj->update();
                        adaptiveCont->printObjectWeight(obstweight);
                        break;
            case  'v': //disable multiple stepping
            	adaptiveCont->disableMultiple_stepping();
                        break;
            case  'V': //enable multiple stepping
            	adaptiveCont->enableMultiple_stepping();
                        break;

             case 'W'://middle Leg Amputation
            	 adaptiveCont->enableLegAmputation(); //Leg Amputation Enable
                       break;
             case  'w':// intact robot
            	 adaptiveCont->disableLegAmputation();
                      break;
             case 'C'://enable continuous local sensory feedback mechanism
            	 adaptiveCont->enableContactForceMech();
                       break;
             case  'c'://disable continuous local sensory feedback mechanism
            	 adaptiveCont->disableContactForceMech();
                       break;
             case 'H':// push object missions (P.59), besides you should set variable 'pushingobject' to true
            	 adaptiveCont->pushObject();
            	  	  break;
             case 'M'://enable oscillator coupling (fully connected network)
            	 adaptiveCont->enableOscillatorCoupling();
                     break;
             case  'm'://disable oscillator coupling (fully connected network)
            	 adaptiveCont->disableOscillatorCoupling();
                     break;
             case 'U'://enable muscle model
            	 adaptiveCont->enableMuscleModel();
                              break;
             case  'u'://disable muscle model
            	 adaptiveCont->disableMuscleModel();
                              break;
             case 'R'://enable oscillator coupling (ring network)
            	 adaptiveCont->enableOscillatorRingCoupling();
                     break;
             case  'r'://disable oscillator coupling (ring network)
            	 adaptiveCont->disableOscillatorRingCoupling();
                     break;
             case  '#':// tetrapod
            	 adaptiveCont->enableTetrapodGait();
                         break;
              case  '@'://wave
            	  adaptiveCont->enableWaveGait();
                      break;
              case  '!'://tripod
            	  adaptiveCont->enableTripodGait();
                       break;
              case  '$'://irregular
            	  adaptiveCont->enableIrregularGait();
                       break;

             case 5://Ctrl+e // enable 'Phase Reset and Inhibition Mechanisms'
            	 adaptiveCont->enablePhaseResetInhibitionMech();
                    break;
             case 4://Ctrl+d // disable 'Phase Reset and Inhibition Mechanisms'
            	 adaptiveCont->disablePhaseResetInhibitionMech();
                     break;
             case 'P':// enable speed calculation
            	 adaptiveCont->calculateSpeed();
               break;
             case 'p':// disable speed calculation
            	 adaptiveCont->disableCalculateSpeed();
                 break;
             case 'Z':// The height of fore legs is greater than others
            	 adaptiveCont->ChangeHeightofForelegs();//
                  break;
                case 'z':// The height of hind legs is greater than others
                	adaptiveCont->ChangeHeightofHindlegs();
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

  lpzrobots::PassiveBox* obj;
  bool pushingobject;
  bool IrregularTerrain;

  AdaptiveController* adaptiveCont;
  AdaptiveController* adaptiveCont_2;
  lpzrobots::AmosII* amos;
  lpzrobots::AmosII* amos_Assistent;
  lpzrobots::AmosIIConf myAmosIIConf;
};

int main(int argc, char **argv)
{
  ThisSim sim;
//  sim.setGroundTexture("Images/greenground.rgb");
  return sim.run(argc, argv) ? 0 : 1;
}
