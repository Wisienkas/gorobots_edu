/***************************************************************************
 *   Copyright (C) 2012 by Timo Nachstedt                                  *
 *    nachstedt@physik3.gwdg.de                                            *
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
#include "controllers/amosIIcontrol.h"
// joint needed for fixation of the robot in the beginning
#include <ode_robots/joint.h>

#include <ode_robots/playground.h>

class ThisSim : public lpzrobots::Simulation {
  public:

  ThisSim(){
    addPaletteFile("colors/UrbanExtraColors.gpl");
    addColorAliasFile("colors/UrbanColorSchema.txt");
    setGroundTexture("Images/whiteground.jpg");
    setTitle("ANN Framework example");
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

    // set simulation parameters
    global.odeConfig.setParam("controlinterval", 10);
    global.odeConfig.setParam("simstepsize", 0.01);

    // select which version of AMOSII should be used
    bool use_amosii_version1 = true;
    bool use_amosii_version2 = false;

    if (use_amosii_version1 && !use_amosii_version2){
      std::cout<<std::endl<<std::endl<<"AMOSII  VERSION 1 SELECTED!"<<std::endl<<std::endl;
      // using amosii version 1
      // Add amosII robot
      lpzrobots::AmosIIConf myAmosIIConf = lpzrobots::AmosII::getAmosIIv1Conf();
      myAmosIIConf.rubberFeet = true;
      amos = new lpzrobots::AmosII(
          odeHandle,
          osgHandle.changeColor(lpzrobots::Color(1, 1, 1)),
          myAmosIIConf, "AmosIIv1");

      controller = new AmosIIControl(/*amos version*/1);
    } else {
      std::cout<<"select only one version of AMOSII !"<<std::endl;
      assert(use_amosii_version1 != use_amosii_version2);
    }

    if (use_amosii_version2 && !use_amosii_version1){
      std::cout<<std::endl<<std::endl<<"AMOSII  VERSION 2 SELECTED!"<<std::endl<<std::endl;
      // using amosii version 2
      // Add amosII robot
      lpzrobots::AmosIIConf myAmosIIConf = lpzrobots::AmosII::getAmosIIv2Conf();
      myAmosIIConf.rubberFeet = true;
      amos = new lpzrobots::AmosII(
          odeHandle,
          osgHandle.changeColor(lpzrobots::Color(1, 1, 1)),
          myAmosIIConf, "AmosIIv2");

      controller = new AmosIIControl(/*amos version*/2);
    } else {
      std::cout<<"select only one version of AMOSII !"<<std::endl;
      assert(use_amosii_version1 != use_amosii_version2);
    }

    // put amos a little bit in the air
    amos->place(osg::Matrix::translate(.0, .0, 0.5));



    // create wiring
    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise());

    // create agent and init it with controller, robot and wiring
    lpzrobots::OdeAgent* agent = new lpzrobots::OdeAgent(global);
    agent->init(controller, amos, wiring);

    // add playground
    lpzrobots::Playground* playground
    = new lpzrobots::Playground(odeHandle, osgHandle,
        osg::Vec3(10, 0.2, 0.3));
    playground->setPosition(osg::Vec3(0,0,0));
    global.obstacles.push_back(playground);

    // create a fixed joint to hold the robot in the air at the beginning
    robotfixator = new lpzrobots::FixedJoint(
        amos->getMainPrimitive(),
        global.environment);
    robotfixator->init(odeHandle, osgHandle, false);

    // inform global variable over everything that happened:
    global.configs.push_back(amos);
    global.agents.push_back(agent);
    global.configs.push_back(controller);

    std::cout << "\n\n"
        << "################################\n"
        << "#   Press x to free amosII!    #\n"
        << "################################\n"
        << "\n" << std::endl;


    std::cout<<"started with walking = false"<<std::endl<<std::endl;
    walking = false;

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
        case 's':
          std::cout<<"s pressed"<<std::endl;
          if (walking){
            controller->control_adaptiveclimbing->tripod->stopWalking();
            std::cout<<"stop now"<< std::endl;
            walking=false;
          } else{
            controller->control_adaptiveclimbing->tripod->startWalking();
            std::cout<<"start now"<< std::endl;
            walking=true;
          }
          break;
        case 'j':
          controller->control_adaptiveclimbing->tripod->lowerLeg();
          std::cout<<"j pressed"<<std::endl;
          std::cout<<"lower leg"<<std::endl;
          break;
        case 'b':
          std::cout<<"b pressed"<<std::endl;
          //          if (come_back){
          //            controller->control_adaptiveclimbing->tripod->stopComeBack();
          //            std::cout<<"stop come back"<<std::endl;
          //            come_back=false;
          //          } else {
          controller->control_adaptiveclimbing->tripod->comeBack();
          std::cout<<"lower leg and come back"<<std::endl;
          //        come_back=true;
          //        }
          break;
        case 'l': // Turn left
          std::cout<<"l pressed"<<std::endl;
          controller->control_adaptiveclimbing->tripod->turnLeft();
          std::cout<<"start now"<< std::endl;
          break;
        case 'r':  // Turn right
          std::cout<<"r pressed"<<std::endl;
          controller->control_adaptiveclimbing->tripod->turnRight();
          std::cout<<"start now"<< std::endl;
          break;
        case 'a': // left side ward
          std::cout<<"a pressed"<<std::endl;
          controller->control_adaptiveclimbing->tripod->sidewardleft();
          std::cout<<"start now"<< std::endl;
          break;
        case 'g': // right side ward
          std::cout<<"g pressed"<<std::endl;
          controller->control_adaptiveclimbing->tripod->sidewardright();
          std::cout<<"start now"<< std::endl;
          break;
        case 'y': // Turn right side ward // STILL NOT WORKING this function
          std::cout<<"y pressed"<<std::endl;
          controller->control_adaptiveclimbing->tripod->turnrightsideward();
          std::cout<<"start now"<< std::endl;
          break;
        case 'w': // Turn right side ward
          std::cout<<"w pressed"<<std::endl;
          controller->control_adaptiveclimbing->tripod->turnleftsideward();
          std::cout<<"start now"<< std::endl;
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
  AmosIIControl* controller;
  lpzrobots::AmosII* amos;
  bool walking;
};


int main(int argc, char **argv)
{
  ThisSim sim;
  sim.setGroundTexture("Images/greenground.rgb");
  return sim.run(argc, argv) ? 0 : 1;
}

