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

//Include header fiels from /ode_robots/include/ode_robots XXX

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


// add terrains
#include <ode_robots/terrainground.h>

// add speaker for sound sensor (1)
#include <ode_robots/speaker.h>


using namespace std;
std::vector<lpzrobots::AbstractObstacle*> obst;
//std::vector<lpzrobots::FixedJoint*> fixator;
// add head file for creating a sphere by Ren ------------

//Select only one of these
bool gapcrossing_experiment_setup = false;
bool add_playground = false;
bool add_rough_terrain = true;

class ThisSim : public lpzrobots::Simulation {
  public:


    // add speaker for sound sensor (2)
    lpzrobots::Speaker* speaker1;
    lpzrobots::Speaker* speaker2;
    double value1, value2;

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
      if(add_playground){
        lpzrobots::Playground* playground
        = new lpzrobots::Playground(odeHandle, osgHandle,
            osg::Vec3(10, 0.2, 0.3));
        playground->setTexture(0,0,lpzrobots::TextureDescr("Images/wall_bw.jpg",-1.5,-3));
        playground->setPosition(osg::Vec3(0,0,.0));
        global.obstacles.push_back(playground);
      }

      //----------create gap by KOH-----------------------------
      lpzrobots::OdeHandle playgroundHandle = odeHandle;
      playgroundHandle.substance = lpzrobots::Substance(0.5/* 3.0 roughness*/, 0.0/*0 slip*/, 100.0/* 50 hardness*/, 0.0/*elasticity*/); //substance for playgrounds (NON-SLIPPERY!!!)

      //EXPERIMENTAL SETUP 1: SINGLE OBSTACLE (Adaption to different obstacle altitudes and walking gaits)
      int obtacle_no = 2; // set to 1 for learning and set to 2 for two platforms
      double gap_distance = 2.23;//2.23;//2.14 /*amosiiv1*/; //2.23 /*amosiiv2*/, 2.3 fail!!
      double size = 1.0; // set to 100 for learning and set to 1.0 for gap crossing experiment

      for(int i=0;i<obtacle_no;i++){
        double obstacle_height = 0.01+0.01*i;
        double obstacle_distance = 0.01;
        if (gapcrossing_experiment_setup) {
          lpzrobots::Playground* playground = new lpzrobots::Playground(playgroundHandle, osgHandle, osg::Vec3(obstacle_distance, size/*size*/,
              0.1/*obstacle_height*/), 1, false);
          playground->setTexture(0, 0, lpzrobots::TextureDescr("Images/wall_bw.jpg" /*ode_robots/osg/data/images/*/, -0.5, -3));
          playground->setPosition(osg::Vec3(gap_distance*i/*distance between platforms*/, 0, .0));
          global.obstacles.push_back(playground);
        }
      }
      //----------create gap by KOH-----------------------------



      //----------Added terrains----------------------------------

      //Changing terrains, e.g., labyrinth.ppm, rough1.ppm, rough6.ppm, rough6flatstart.ppm test.ppm
      if(add_rough_terrain){
        lpzrobots::TerrainGround* terrainground = new lpzrobots::TerrainGround(odeHandle,
            osgHandle, "rough1.ppm" /* "terrains/3potential_texture.ppm"-->ode_robots/osg/data/terrains/terrain_bumpInDip128.ppm*/," " /*ode_robots/osg/data/images/*/,10.0,10.0,0.025/*0.15, height*/);//3.0,3.0,0.05/*0.15, height*/);
        //TerrainGround* terrainground = TerrainGround(odeHandle,
        //osgHandle,"../../../pmanoonpong-gorobots-fork/projects/amosii/koh_preview_neuralcontrol/amosiiv1v2_neuralcontrol_rc/one_fm_multi_gaits/sim/rough1.ppm" /*terrains/macrospheresTex_256.ppm ode_robots/osg/data/terrains/terrain_bumpInDip128.ppm*/,"Images/pandafur.jpg" /*ode_robots/osg/data/images/*/,10.0,5.0,0.6/*0.15, height*/);
        //   Substance* subst =new Substance();

        terrainground->setPosition(osg::Vec3(0,0,0.01)); // playground positionieren und generieren
        global.obstacles.push_back(terrainground);
      }
      //----------Added terrains----------------------------------




      //----------create a sphere as the target by Ren-----------------------------
      //the first sphere
      lpzrobots::PassiveSphere* s1 = new lpzrobots::PassiveSphere(odeHandle, osgHandle, 0.1);
      s1->setPosition(osg::Vec3(3.0, 0.0, 2.0/*0.1 = above ground in z direction*/));
      s1->setTexture("Images/dusty.rgb");
      s1->setColor(lpzrobots::Color(1,0,0));
      obst.push_back(s1);
      global.obstacles.push_back(s1);
      lpzrobots::FixedJoint* fixator1 = new  lpzrobots::FixedJoint(s1->getMainPrimitive(), global.environment);
      fixator1->init(odeHandle, osgHandle);

      //the second sphere
      lpzrobots::PassiveSphere* s2 = new lpzrobots::PassiveSphere(odeHandle, osgHandle, 0.1);
      s2->setPosition(osg::Vec3(0.0, 3.0, 2.0/*0.1 = above ground in z direction*/));
      s2->setTexture("Images/dusty.rgb");
      s2->setColor(lpzrobots::Color(0,1,0));
      obst.push_back(s2);
      global.obstacles.push_back(s2);
      lpzrobots::FixedJoint* fixator2 = new  lpzrobots::FixedJoint(s2->getMainPrimitive(), global.environment);
      fixator2->init(odeHandle, osgHandle);

      //the third sphere
      lpzrobots::PassiveSphere* s3 = new lpzrobots::PassiveSphere(odeHandle, osgHandle, 0.1);
      s3->setPosition(osg::Vec3(0.0, -3.0, 2.0/*0.1 = above ground in z direction*/));
      s3->setTexture("Images/dusty.rgb");
      s3->setColor(lpzrobots::Color(0,0,1));
      obst.push_back(s3);
      global.obstacles.push_back(s3);
      lpzrobots::FixedJoint* fixator3 = new  lpzrobots::FixedJoint(s3->getMainPrimitive(), global.environment);
      fixator3->init(odeHandle, osgHandle);

      //----------create a sphere as the target by Ren-----------------------------

      // add speaker for sound sensor (3)
      //Add speaker 1 for sound sensor by KOH
      lpzrobots::PassiveSphere* sp1 = new lpzrobots::PassiveSphere(odeHandle, osgHandle,0.3);
      sp1->setPose(osg::Matrix::translate(0,-1,0.5));
      sp1->setColor(lpzrobots::Color(1,0,0));
      global.obstacles.push_back(sp1);
      speaker1 = new lpzrobots::Speaker(10);
      speaker1->init(sp1->getMainPrimitive());
      value1=0.5;
      speaker1->set(&value1,1);
      lpzrobots::FixedJoint* fixatorsp1 = new  lpzrobots::FixedJoint(sp1->getMainPrimitive(), global.environment);
      fixatorsp1->init(odeHandle, osgHandle);


      //Add speaker 2 for sound sensor by KOH
      lpzrobots::PassiveSphere* sp2 = new lpzrobots::PassiveSphere(odeHandle, osgHandle,0.3);
      sp2->setPose(osg::Matrix::translate(0,1,0.5));
      sp2->setColor(lpzrobots::Color(0,0.5,1));
      global.obstacles.push_back(sp2);
      speaker2 = new lpzrobots::Speaker(10);
      speaker2->init(sp2->getMainPrimitive());
      value2=1.5;
      speaker2->set(&value2,1);
      lpzrobots::FixedJoint* fixatorsp2 = new  lpzrobots::FixedJoint(sp2->getMainPrimitive(), global.environment);
      fixatorsp2->init(odeHandle, osgHandle);
      //-------------------------------------------------------------------//

      // select which version of AMOSII should be used
      bool use_amosii_version1 = false;
      bool use_amosii_version2 = true;

      //AMOS V1
      if (use_amosii_version1 && !use_amosii_version2){
        std::cout<<std::endl<<std::endl<<"AMOSII  VERSION 1 SELECTED!"<<std::endl<<std::endl;
        // using amosii version 1
        // Add amosII robot

        lpzrobots::AmosIIConf myAmosIIConf = lpzrobots::AmosII::getAmosIIv1Conf(1.0 /*_scale*/,1 /*_useShoulder*/,1 /*_useFoot*/,1 /*_useBack*/);
        myAmosIIConf.rubberFeet = false;//true;
        myAmosIIConf.useLocalVelSensor = true;
        //myAmosIIConf.legContactSensorIsBinary = true;

        lpzrobots::OdeHandle rodeHandle = odeHandle;

        //Change surface property (lpzrobots / ode_robots / osg / substance.cpp)
        rodeHandle.substance = lpzrobots::Substance(3.0/*3.0*//*3.0*//*roughness*/, 0.0/*0 slip*/, 50/*50.0*//*hardness*/, 0.8/*elasticity*/);
        //rodeHandle.substance = lpzrobots::Substance::getFoam(5/*roughness*/);
        //rodeHandle.substance = lpzrobots::Substance::getSnow(0.0/*_slip > 1.0 = high slip, <1.0 low slip*/);
        //rodeHandle.substance = lpzrobots::Substance::Substance::getRubber(50/*_hardness [5-50]*/);

        //------------------- Link the sphere to the Goal Sensor by Ren---------------
        for(int i = 0; i<obst.size(); i++)
        {
          myAmosIIConf.GoalSensor_references.push_back(obst.at(i)->getMainPrimitive());
        }
        //------------------- Link the sphere to the Goal Sensor by Ren---------------

        amos = new lpzrobots::AmosII(
            odeHandle,
            osgHandle.changeColor(lpzrobots::Color(1, 1, 1)),
            myAmosIIConf, "AmosIIv1");

        //controller = new AmosIIControl(/*amos version 1*/);

        // define the usage of the individual legs
        amos->setLegPosUsage(amos->L0, amos->LEG);
        amos->setLegPosUsage(amos->L1, amos->LEG);
        amos->setLegPosUsage(amos->L2, amos->LEG);
        amos->setLegPosUsage(amos->R0, amos->LEG);
        amos->setLegPosUsage(amos->R1, amos->LEG);
        amos->setLegPosUsage(amos->R2, amos->LEG);

        // put amos a little bit in the air
        amos->place(osg::Matrix::translate(.0, .0, 0.5));

        controller = new AmosIIControl(/*amos version1 */);//TripodGait18DOF();

      } else {
        std::cout<<"select only one version of AMOSII !"<<std::endl;
        assert(use_amosii_version1 != use_amosii_version2);
      }

      //AMOS V2
      if (use_amosii_version2 && !use_amosii_version1){
        std::cout<<std::endl<<std::endl<<"AMOSII  VERSION 2 SELECTED!"<<std::endl<<std::endl;
        // using amosii version 2
        // Add amosII robot

        lpzrobots::AmosIIConf myAmosIIConf = lpzrobots::AmosII::getDefaultConf(1.0 /*_scale*/,1 /*_useShoulder*/,1 /*_useFoot*/,1 /*_useBack*/);
        myAmosIIConf.rubberFeet = false;
        myAmosIIConf.useLocalVelSensor = true;
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
          myAmosIIConf.GoalSensor_references.push_back(obst.at(i)->getMainPrimitive());
        }
        //------------------- Link the sphere to the Goal Sensor by Ren---------------


        amos = new lpzrobots::AmosII(
            odeHandle,
            osgHandle.changeColor(lpzrobots::Color(1, 1, 1)),
            myAmosIIConf, "AmosIIv2");

        //controller = new AmosIIControl(/*amos version 2*/);

        // define the usage of the individual legs
        amos->setLegPosUsage(amos->L0, amos->LEG);
        amos->setLegPosUsage(amos->L1, amos->LEG);
        amos->setLegPosUsage(amos->L2, amos->LEG);
        amos->setLegPosUsage(amos->R0, amos->LEG);
        amos->setLegPosUsage(amos->R1, amos->LEG);
        amos->setLegPosUsage(amos->R2, amos->LEG);

        // put amos a little bit in the air
        amos->place(osg::Matrix::translate(.0, .0, 0.5));

        controller = new AmosIIControl(/*amos version 2*/);//TripodGait18DOF();

      } else {
        std::cout<<"select only one version of AMOSII !"<<std::endl;
        assert(use_amosii_version1 != use_amosii_version2);
      }


      //    // Add amosII robot
      //    lpzrobots::AmosIIConf myAmosIIConf = lpzrobots::AmosII::getDefaultConf(1.0 /*_scale*/,1 /*_useShoulder*/,1 /*_useFoot*/,1 /*_useBack*/);
      //    myAmosIIConf.rubberFeet = false;//true;
      //    myAmosIIConf.useLocalVelSensor = true;
      //
      //
      //    //lpzrobots::AmosIIConf myAmosIIConf = lpzrobots::AmosII::getAmosIIv1Conf(1.0 /*_scale*/,1 /*_useShoulder*/,1 /*_useFoot*/,1 /*_useBack*/);
      //    //myAmosIIConf.rubberFeet = true;
      //
      //    //myAmosIIConf.legContactSensorIsBinary = true;
      //    lpzrobots::OdeHandle rodeHandle = odeHandle;
      //
      //    //Change surface property (lpzrobots / ode_robots / osg / substance.cpp)
      //    rodeHandle.substance = lpzrobots::Substance(3.0/*3.0*//*roughness*/, 0.0/*0 slip*/, 50.0/*hardness*/, 0.8/*elasticity*/);
      //    //rodeHandle.substance = lpzrobots::Substance::getFoam(5/*roughness*/);
      //    //rodeHandle.substance = lpzrobots::Substance::getSnow(0.0/*_slip > 1.0 = high slip, <1.0 low slip*/);
      //    //rodeHandle.substance = lpzrobots::Substance::Substance::getRubber(50/*_hardness [5-50]*/);
      //
      //
      //
      //    //------------------- Link the sphere to the Goal Sensor by Ren---------------
      //    for(int i = 0; i<obst.size(); i++)
      //    {
      //      myAmosIIConf.GoalSensor_references.push_back(obst.at(i)->getMainPrimitive());
      //    }
      //    //------------------- Link the sphere to the Goal Sensor by Ren---------------

      //    amos = new lpzrobots::AmosII(
      //        rodeHandle,
      //        osgHandle.changeColor(lpzrobots::Color(1, 1, 1)),
      //        myAmosIIConf, "AmosII");


      //    // put amos a little bit in the air
      //    amos->place(osg::Matrix::translate(.0, .0, 0.5));
      //
      //
      //
      //    // create wiring
      //    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise());
      //
      //    // create agent and init it with controller, robot and wiring
      //    lpzrobots::OdeAgent* agent = new lpzrobots::OdeAgent(global);
      //    agent->init(controller, amos, wiring);
      //
      //    // add playground
      //    lpzrobots::Playground* playground
      //    = new lpzrobots::Playground(odeHandle, osgHandle,
      //        osg::Vec3(10, 0.2, 0.3));
      //    playground->setPosition(osg::Vec3(0,0,0));
      //    global.obstacles.push_back(playground);
      //
      //    // create a fixed joint to hold the robot in the air at the beginning
      //    robotfixator = new lpzrobots::FixedJoint(
      //        amos->getMainPrimitive(),
      //        global.environment);
      //    robotfixator->init(odeHandle, osgHandle, false);
      //
      //    // inform global variable over everything that happened:
      //    global.configs.push_back(amos);
      //    global.agents.push_back(agent);
      //    global.configs.push_back(controller);
      //
      //    std::cout << "\n\n"
      //        << "################################\n"
      //        << "#   Press x to free amosII!    #\n"
      //        << "################################\n"
      //        << "\n" << std::endl;
      //
      //
      //    std::cout<<"started with walking = false"<<std::endl<<std::endl;
      //    //walking = false;


      ////////////////////////////////////////////////////////////////////
      //    // define the usage of the individual legs
      //    amos->setLegPosUsage(amos->L0, amos->LEG);
      //    amos->setLegPosUsage(amos->L1, amos->LEG);
      //    amos->setLegPosUsage(amos->L2, amos->LEG);
      //    amos->setLegPosUsage(amos->R0, amos->LEG);
      //    amos->setLegPosUsage(amos->R1, amos->LEG);
      //    amos->setLegPosUsage(amos->R2, amos->LEG);
      //
      //    // put amos a little bit in the air
      //    amos->place(osg::Matrix::translate(.0, .0, 0.5));
      //
      //    controller = new AmosIIControl();//TripodGait18DOF();
      // create wiring
      One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise());

      // create agent and init it with controller, robot and wiring
      lpzrobots::OdeAgent* agent = new lpzrobots::OdeAgent(global);
      agent->init(controller, amos, wiring);


      // Possibility to add tracking for robot
      bool track = false;
      if (track)
        agent->setTrackOptions(TrackRobot(true, false, false, true, "", 60)); // Display trace
      //if(track) agent->setTrackOptions(TrackRobot(false,false,false, false, ""));

      // create a fixed joint to hold the robot in the air at the beginning
      //    robotfixator = new lpzrobots::FixedJoint(
      //        amos->getMainPrimitive(),
      //        global.environment);
      //    robotfixator->init(odeHandle, osgHandle, false);


      //    // add playground
      //    lpzrobots::Playground* playground
      //    = new lpzrobots::Playground(odeHandle, osgHandle,
      //        osg::Vec3(10, 0.2, 0.3));
      //    playground->setPosition(osg::Vec3(0,0,0));
      //    global.obstacles.push_back(playground);

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
          << "\n\n" << std::endl;

      /////////////////////////////////////////


    }

    virtual void addCallback(lpzrobots::GlobalData& globalData, bool draw, bool pause, bool control) {
      // add speaker for sound sensor (4)
      if(control)
      {
        speaker1->act(globalData);
        speaker2->act(globalData);
      }
      //------------------------------//
    };

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

            // add speaker for sound sensor (5)
          case 'j':
            value1+=.1;
            speaker1->set(&value1,1);
            std::cout << "louder" << std::endl;;
            break;
          case 'J':
            value1-=.1;
            speaker1->set(&value1,1);
            std::cout << "quiter" << std::endl;
            break;

          case 'k':
            value2+=.1;
            speaker2->set(&value2,1);
            std::cout << "louder" << std::endl;;
            break;
          case 'K':
            value2-=.1;
            speaker2->set(&value2,1);
            std::cout << "quiter" << std::endl;
            break;
            //-------------------------------//

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


    //  lpzrobots::Joint* robotfixator;
    //  AmosIIControl* controller;
    //  lpzrobots::AmosII* amos;


};

int main(int argc, char **argv)
{
  ThisSim sim;
  sim.setGroundTexture("Images/greenground.rgb");
  return sim.run(argc, argv) ? 0 : 1;
}

