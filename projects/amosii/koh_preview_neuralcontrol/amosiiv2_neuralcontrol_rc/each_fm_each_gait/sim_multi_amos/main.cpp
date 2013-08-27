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
#include "amosIIcontrol.h"//"tripodgait18dof.h"
// joint needed for fixation of the robot in the beginning
#include <ode_robots/joint.h>

// add head file for creating a sphere by Ren ------------
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>
#include <selforg/abstractcontroller.h>
#include <ode_robots/color.h>
#include <iostream>
using namespace std;
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
    setGroundTexture("Images/whiteground.jpg"); // gets its color from the schema
    //setTitle("centered text");
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

    // set simulation parameters
    global.odeConfig.setParam("controlinterval", 10);
    global.odeConfig.setParam("simstepsize", 0.01);
    global.odeConfig.setParam("noise", 0.0);//0.02); // 0.02

    // add playground
//    lpzrobots::Playground* playground
//    = new lpzrobots::Playground(odeHandle, osgHandle,
//        osg::Vec3(10, 0.2, 0.3));
//    playground->setTexture(0,0,lpzrobots::TextureDescr("Images/wall_bw.jpg",-1.5,-3));
//    playground->setPosition(osg::Vec3(0,0,.0));
//    global.obstacles.push_back(playground);

    //----------create a sphere as the target by Ren-----------------------------
    //the first sphere
//    lpzrobots::PassiveSphere* s1 = new lpzrobots::PassiveSphere(odeHandle, osgHandle, 0.1);
//    s1->setPosition(osg::Vec3(3.0, 0.0, 0.1));
//    s1->setTexture("Images/dusty.rgb");
//    s1->setColor(lpzrobots::Color(1,0,0));
//    obst.push_back(s1);
//    global.obstacles.push_back(s1);
//    lpzrobots::FixedJoint* fixator1 = new  lpzrobots::FixedJoint(s1->getMainPrimitive(), global.environment);
//    fixator1->init(odeHandle, osgHandle);
//
//    //the second sphere
//    lpzrobots::PassiveSphere* s2 = new lpzrobots::PassiveSphere(odeHandle, osgHandle, 0.1);
//    s2->setPosition(osg::Vec3(0.0, 3.0, 0.1));
//    s2->setTexture("Images/dusty.rgb");
//    s2->setColor(lpzrobots::Color(0,1,0));
//    obst.push_back(s2);
//    global.obstacles.push_back(s2);
//    lpzrobots::FixedJoint* fixator2 = new  lpzrobots::FixedJoint(s2->getMainPrimitive(), global.environment);
//    fixator2->init(odeHandle, osgHandle);
//
//    //the third sphere
//    lpzrobots::PassiveSphere* s3 = new lpzrobots::PassiveSphere(odeHandle, osgHandle, 0.1);
//    s3->setPosition(osg::Vec3(0.0, -3.0, 0.1));
//    s3->setTexture("Images/dusty.rgb");
//    s3->setColor(lpzrobots::Color(0,0,1));
//    obst.push_back(s3);
//    global.obstacles.push_back(s3);
//    lpzrobots::FixedJoint* fixator3 = new  lpzrobots::FixedJoint(s3->getMainPrimitive(), global.environment);
//    fixator3->init(odeHandle, osgHandle);

    //----------create a sphere as the target by Ren-----------------------------


    // Add amosII robot 1
    lpzrobots::AmosIIConf myAmosIIConf = lpzrobots::AmosII::getDefaultConf(1.0 /*_scale*/,1 /*_useShoulder*/,1 /*_useFoot*/,1 /*_useBack*/);
    myAmosIIConf.rubberFeet = false;//true;
    myAmosIIConf.useLocalVelSensor = true;

    // Add amosII robot 2
    lpzrobots::AmosIIConf myAmosIIConf2 = lpzrobots::AmosII::getDefaultConf(1.0 /*_scale*/,1 /*_useShoulder*/,1 /*_useFoot*/,1 /*_useBack*/);
    myAmosIIConf2.rubberFeet = false;//true;
    myAmosIIConf2.useLocalVelSensor = true;

    // Add amosII robot 3
    lpzrobots::AmosIIConf myAmosIIConf3 = lpzrobots::AmosII::getDefaultConf(1.0 /*_scale*/,1 /*_useShoulder*/,1 /*_useFoot*/,1 /*_useBack*/);
    myAmosIIConf3.rubberFeet = false;//true;
    myAmosIIConf3.useLocalVelSensor = true;


    // Add amosII robot 4
    lpzrobots::AmosIIConf myAmosIIConf4 = lpzrobots::AmosII::getDefaultConf(1.0 /*_scale*/,1 /*_useShoulder*/,1 /*_useFoot*/,1 /*_useBack*/);
    myAmosIIConf4.rubberFeet = false;//true;
    myAmosIIConf4.useLocalVelSensor = true;


    // Add amosII robot 5
    lpzrobots::AmosIIConf myAmosIIConf5 = lpzrobots::AmosII::getDefaultConf(1.0 /*_scale*/,1 /*_useShoulder*/,1 /*_useFoot*/,1 /*_useBack*/);
    myAmosIIConf5.rubberFeet = false;//true;
    myAmosIIConf5.useLocalVelSensor = true;


    //lpzrobots::AmosIIConf myAmosIIConf = lpzrobots::AmosII::getAmosIIv1Conf(1.0 /*_scale*/,1 /*_useShoulder*/,1 /*_useFoot*/,1 /*_useBack*/);
    //myAmosIIConf.rubberFeet = true;

    //myAmosIIConf.legContactSensorIsBinary = true;
    lpzrobots::OdeHandle rodeHandle = odeHandle;

    //Change surface property (lpzrobots / ode_robots / osg / substance.cpp)
    rodeHandle.substance = lpzrobots::Substance(3.0/*3.0*//*roughness*/, 0.0/*0 slip*/, 50.0/*hardness*/, 0.8/*elasticity*/);
    //rodeHandle.substance = lpzrobots::Substance::getFoam(5/*roughness*/);
    //rodeHandle.substance = lpzrobots::Substance::getSnow(0.0/*_slip > 1.0 = high slip, <1.0 low slip*/);
    //rodeHandle.substance = lpzrobots::Substance::Substance::getRubber(50/*_hardness [5-50]*/);



    //------------------- Link the sphere to the Goal Sensor by Ren---------------
    for(int i = 0; i<obst.size(); i++)
    {
      // Add amosII robot 1
      myAmosIIConf.GoalSensor_references.push_back(obst.at(i)->getMainPrimitive());

      // Add amosII robot 2
      myAmosIIConf2.GoalSensor_references.push_back(obst.at(i)->getMainPrimitive());

      // Add amosII robot 3
      myAmosIIConf3.GoalSensor_references.push_back(obst.at(i)->getMainPrimitive());

      // Add amosII robot 4
      myAmosIIConf4.GoalSensor_references.push_back(obst.at(i)->getMainPrimitive());

      // Add amosII robot 5
      myAmosIIConf5.GoalSensor_references.push_back(obst.at(i)->getMainPrimitive());
    }
    //------------------- Link the sphere to the Goal Sensor by Ren---------------

    // Add amosII robot 1
    amos = new lpzrobots::AmosII(
        rodeHandle,
        osgHandle.changeColor(lpzrobots::Color(1, 1, 1)),
        myAmosIIConf, "AmosII");

    // define the usage of the individual legs
    amos->setLegPosUsage(amos->L0, amos->LEG);
    amos->setLegPosUsage(amos->L1, amos->LEG);
    amos->setLegPosUsage(amos->L2, amos->LEG);
    amos->setLegPosUsage(amos->R0, amos->LEG);
    amos->setLegPosUsage(amos->R1, amos->LEG);
    amos->setLegPosUsage(amos->R2, amos->LEG);

    // put amos a little bit in the air
    amos->place(osg::Matrix::translate(.0, .0, 0.5));


    // Add amosII robot 2
    amos2 = new lpzrobots::AmosII(
        rodeHandle,
        osgHandle.changeColor(lpzrobots::Color(1, 1, 1)),
        myAmosIIConf2, "AmosII");

    // define the usage of the individual legs
    amos2->setLegPosUsage(amos2->L0, amos2->LEG);
    amos2->setLegPosUsage(amos2->L1, amos2->LEG);
    amos2->setLegPosUsage(amos2->L2, amos2->LEG);
    amos2->setLegPosUsage(amos2->R0, amos2->LEG);
    amos2->setLegPosUsage(amos2->R1, amos2->LEG);
    amos2->setLegPosUsage(amos2->R2, amos2->LEG);

    // put amos a little bit in the air
    amos2->place(osg::Matrix::translate(.0, 1.0, 0.5));


    // Add amosII robot 3
    amos3 = new lpzrobots::AmosII(
        rodeHandle,
        osgHandle.changeColor(lpzrobots::Color(1, 1, 1)),
        myAmosIIConf3, "AmosII");

    // define the usage of the individual legs
    amos3->setLegPosUsage(amos3->L0, amos3->LEG);
    amos3->setLegPosUsage(amos3->L1, amos3->LEG);
    amos3->setLegPosUsage(amos3->L2, amos3->LEG);
    amos3->setLegPosUsage(amos3->R0, amos3->LEG);
    amos3->setLegPosUsage(amos3->R1, amos3->LEG);
    amos3->setLegPosUsage(amos3->R2, amos3->LEG);

    // put amos a little bit in the air
    amos3->place(osg::Matrix::translate(-0.5, -1.0, 0.5));

    // Add amosII robot 4
    amos4 = new lpzrobots::AmosII(
        rodeHandle,
        osgHandle.changeColor(lpzrobots::Color(1, 1, 1)),
        myAmosIIConf4, "AmosII");

    // define the usage of the individual legs
    amos4->setLegPosUsage(amos4->L0, amos4->LEG);
    amos4->setLegPosUsage(amos4->L1, amos4->LEG);
    amos4->setLegPosUsage(amos4->L2, amos4->LEG);
    amos4->setLegPosUsage(amos4->R0, amos4->LEG);
    amos4->setLegPosUsage(amos4->R1, amos4->LEG);
    amos4->setLegPosUsage(amos4->R2, amos4->LEG);

    // put amos a little bit in the air
    amos4->place(osg::Matrix::translate(1.0, -2.0, 0.5));

    // Add amosII robot 5
    amos5 = new lpzrobots::AmosII(
        rodeHandle,
        osgHandle.changeColor(lpzrobots::Color(1, 1, 1)),
        myAmosIIConf5, "AmosII");

    // define the usage of the individual legs
    amos5->setLegPosUsage(amos5->L0, amos5->LEG);
    amos5->setLegPosUsage(amos5->L1, amos5->LEG);
    amos5->setLegPosUsage(amos5->L2, amos5->LEG);
    amos5->setLegPosUsage(amos5->R0, amos5->LEG);
    amos5->setLegPosUsage(amos5->R1, amos5->LEG);
    amos5->setLegPosUsage(amos5->R2, amos5->LEG);

    // put amos a little bit in the air
    amos5->place(osg::Matrix::translate(-1.0, 2.0, 0.5));



    //Controller amos1
    controller = new AmosIIControl();//TripodGait18DOF();
    // create wiring
    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise());

    // create agent and init it with controller, robot and wiring
    lpzrobots::OdeAgent* agent = new lpzrobots::OdeAgent(global);
    agent->init(controller, amos, wiring);



    //Controller amos2
    controller2 = new AmosIIControl();//TripodGait18DOF();
    // create wiring
    One2OneWiring* wiring2 = new One2OneWiring(new ColorUniformNoise());

    // create agent and init it with controller, robot and wiring
    lpzrobots::OdeAgent* agent2 = new lpzrobots::OdeAgent(global);
    agent2->init(controller2, amos2, wiring2);


    //Controller amos3
    controller3 = new AmosIIControl();//TripodGait18DOF();
    // create wiring
    One2OneWiring* wiring3 = new One2OneWiring(new ColorUniformNoise());

    // create agent and init it with controller, robot and wiring
    lpzrobots::OdeAgent* agent3 = new lpzrobots::OdeAgent(global);
    agent3->init(controller3, amos3, wiring3);

    //Controller amos4
    controller4 = new AmosIIControl();//TripodGait18DOF();
    // create wiring
    One2OneWiring* wiring4 = new One2OneWiring(new ColorUniformNoise());

    // create agent and init it with controller, robot and wiring
    lpzrobots::OdeAgent* agent4 = new lpzrobots::OdeAgent(global);
    agent4->init(controller4, amos4, wiring4);

    //Controller amos5
    controller5 = new AmosIIControl();//TripodGait18DOF();
    // create wiring
    One2OneWiring* wiring5 = new One2OneWiring(new ColorUniformNoise());

    // create agent and init it with controller, robot and wiring
    lpzrobots::OdeAgent* agent5 = new lpzrobots::OdeAgent(global);
    agent5->init(controller5, amos5, wiring5);



    // create a fixed joint to hold the robot in the air at the beginning
    robotfixator = new lpzrobots::FixedJoint(
        amos->getMainPrimitive(),
        global.environment);
    robotfixator->init(odeHandle, osgHandle, false);


    // create a fixed joint to hold the robot in the air at the beginning
    robotfixator2 = new lpzrobots::FixedJoint(
        amos2->getMainPrimitive(),
        global.environment);
    robotfixator2->init(odeHandle, osgHandle, false);



    // inform global variable over everything that happened:
    global.configs.push_back(amos);
    global.agents.push_back(agent);
    global.configs.push_back(controller);


    // inform global variable over everything that happened:
    global.configs.push_back(amos2);
    global.agents.push_back(agent2);
    global.configs.push_back(controller2);

    // inform global variable over everything that happened:
    global.configs.push_back(amos3);
    global.agents.push_back(agent3);
    global.configs.push_back(controller3);

    // inform global variable over everything that happened:
    global.configs.push_back(amos4);
    global.agents.push_back(agent4);
    global.configs.push_back(controller4);

    // inform global variable over everything that happened:
    global.configs.push_back(amos5);
    global.agents.push_back(agent5);
    global.configs.push_back(controller5);

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

        case 'y':
            if (robotfixator2) {
              std::cout << "dropping robot2" << std::endl;
              delete robotfixator2;
              robotfixator2 = NULL;
            }
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
  AbstractController* controller;
  lpzrobots::AmosII* amos;

  lpzrobots::Joint* robotfixator2;
  AbstractController* controller2;
  lpzrobots::AmosII* amos2;

  lpzrobots::Joint* robotfixator3;
  AbstractController* controller3;
  lpzrobots::AmosII* amos3;

  lpzrobots::Joint* robotfixator4;
  AbstractController* controller4;
  lpzrobots::AmosII* amos4;

  lpzrobots::Joint* robotfixator5;
  AbstractController* controller5;
  lpzrobots::AmosII* amos5;


//  lpzrobots::Joint* robotfixator7[10];
//  AbstractController* controller7[10];
//  lpzrobots::AmosII* amos7[10];


};

int main(int argc, char **argv)
{
  ThisSim sim;
  sim.setGroundTexture("Images/greenground.rgb");
  return sim.run(argc, argv) ? 0 : 1;
}

