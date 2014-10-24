/***************************************************************************
 *   Copyright (C) 2012 by Robot Group Goettingen                          *
 *                                    									   *
 *    fhesse@physik3.gwdg.de     			                               *
 *    xiong@physik3.gwdg.de                  	                           *
 *    poramate@physik3.gwdg.de                                             *
 *   LICENSE:                                                              *
 *   This work is licensed under the Creative Commons                      *
 *   Attribution-NonCommercial-ShareAlike 2.5 License. To view a copy of   *
 *   this license, visit http://creativecommons.org/licenses/by-nc-sa/2.5/ *
 *   or send a letter to Creative Commons, 543 Howard Street, 5th Floor,   *
 *   San Francisco, California, 94105, USA.                                *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  *
 *                                                                         *
 *   AMOSII v2 has now only 18 sensors                                     *
 ***************************************************************************/
//
// Main.cpp
//  This is the program for moving the real robot!!
//  The codes to be changed for the experiments are mentioned as "#comment 1-3 brabrabra"
//
//  2014.08.13 start to design
//      方針：とりあえず、変なリスクを負いたくないので、クラスを使わずにすべてメイン関数内ですべてが住むようにデザイン
//      　　：見た目が汚すぎるけど、多分何とかなるんじゃないかな？ 無駄な時間を使いたくない
//
//
//
#include <selforg/agent.h>
#include <selforg/abstractrobot.h>
#include <selforg/one2onewiring.h>
#include <selforg/sinecontroller.h>
//#include <ode_robots/amosiistdscalingwiring.h>

//#include "ode_robots/hexabot.h"
#include <hexabotController.h>
#include <amosControllerAdptHexabot.hpp>
//#include "../../../lpzrobots/real_robots/robots/amosii/amosIIserialv1.h"  //serial interface to AMOSII version 1
//#include "../../../lpzrobots/real_robots/robots/amosii/globaldata.h"
//#include "lpzrobots/amosIIserialv2.h"  //serial interface to AMOSII version 2

#include <hxPeriodAnalysis.h>
#include <itfHexaSimulation.h>

#include <findsolution_analyzestability.hpp>
#include <findSolution.hpp>

#include <runLogSimulation.hpp>
#include <timedChangePrmAgts.hpp>

#include <repeatAnalysis.hpp>
#include <repeatModulators.hpp>

#include <array>
#include <cmath>
#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // terminal control definitions
#include <time.h>   // time calls
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <curses.h>

using namespace lpzrobots;
using namespace std;

// grobal parameters
bool stop=0;
double noise=.0;
double realtimefactor=1;
bool   singleline = true; // whether to display the state in a single line

// about Experiments
const int S_CONTROL_FREQ = 30;//25;//3200; // 3200 in thesis
const int time_wait = 10;// (s) time to wait

// about file
// #comment 1
//   DETAIL_FILE_POS is the pass for the folder which you want to put files
//   FILE_NAME is the name of the file
#define DETAIL_FILE_POS  "//home//ambe//Dropbox//AshigaruShare//hexData//amos//realExp//"
//#define FILE_NAME "Backward_leg_sw000_ST10_fixedPs" // For Backward
#define FILE_NAME "Trial1_leg_sw008_ST5_d06" // Name of file

// debug mode
#define DEBUG_MAIN 1


AmosIIConf getAmosIIv2Conf(double _scale, bool _useShoulder, bool _useFoot, bool _useBack) {

  AmosIIConf c;

  // "Internal" variable storing the currently used version
  c.amos_version = 2;
  // use shoulder (fixed joint between legs and trunk)
  c.useShoulder = _useShoulder;
  c.useTebiaJoints = 0;
  //create springs at the end of the legs
  c.useFoot = _useFoot;
  //create a joint in the back
  c.useBack = _useBack;
  c.rubberFeet = false;
  c.useLocalVelSensor = 0;
  c.legContactSensorIsBinary = false;

  // the trunk length. this scales the whole robot! all parts' sizes,
  // masses, and forces will be adapted!!
  c.size = 0.43 * _scale;
  //trunk width
  c.width = 7.0 / 43.0 * c.size;
  //trunk height
  c.height = 6.5 / 43.0 * c.size;
  c.frontLength = 12.0 / 43.0 * c.size;
  // we use as density the original trunk weight divided by the original
  // volume

  //Change mass by KOH to 3.0
  const double density = 3.0 / (0.43 * 0.07 * 0.065); //2.2 / (0.43 * 0.07 * 0.065);

  c.trunkMass = density * c.size * c.width * c.height;
  // use the original trunk to total mass ratio
  const double mass = 5.758 / 2.2 * c.trunkMass;
  c.frontMass = c.trunkMass * c.frontLength / c.size;
  // distribute the rest of the weight like this for now */
  c.shoulderMass = (mass - c.trunkMass) / (6 * (3.0 + c.useShoulder)) * (20.0 - c.useFoot) / 20.0;
  c.coxaMass = c.shoulderMass;
  c.secondMass = c.shoulderMass;
  c.tebiaMass = c.shoulderMass;
  // foot gets 3 or 4 times 1/20 of shoulderMass (divide and multiply by
  // 3 or 4)
  c.footMass = (mass - c.trunkMass) / 6 * c.useFoot / 20.0;

  //As real robot!!
  const double shoulderHeight_cm = 6.5;
  //shoulder height "4.5 wrong" --> correct=6.5 cm from rotating point
  c.shoulderHeight = shoulderHeight_cm / 6.5 * c.height;

  // distance between hindlegs and middle legs
  c.legdist1 = 19.0 / 43.0 * c.size;
  // distance between middle legs and front legs
  c.legdist2 = 15.0 / 43.0 * c.size;

  // configure the wheels (if used). They don't have any counterpart in
  // reality, so the chosen values are arbitrary
  c.wheel_radius = 0.10 * c.size;
  c.wheel_width = 0.04 * c.size;
  c.wheel_mass = (mass - c.trunkMass) / 6.0;

  // -----------------------
  // 1) Biomechanics
  // Manual setting adjustable joint positions at the body
  // -----------------------

  // amosII has a fixed but adjustable joint that decides how the legs
  // extend from the trunk. Here you can adjust these joints

  // ------------- Front legs -------------
  // angle (in rad) around vertical axis at leg-trunk fixation 0:
  // perpendicular
  // => forward/backward
  c.fLegTrunkAngleV = 0.0;
  // angle around horizontal axis at leg-trunk fixation 0: perpendicular
  // => upward/downward
  c.fLegTrunkAngleH = 0.0;
  // rotation of leg around own axis 0: first joint axis is vertical
  // => till
  c.fLegRotAngle = 0.0;

  // ------------- Middle legs ----------------
  // => forward/backward
  c.mLegTrunkAngleV = 0.0;
  // => upward/downward
  c.mLegTrunkAngleH = 0.0;
  // => till
  c.mLegRotAngle = 0.0;

  // ------------- Rear legs ------------------
  // => forward/backward
  c.rLegTrunkAngleV = 0.0;
  // => upward/downward
  c.rLegTrunkAngleH = 0.0;
  // => till
  c.rLegRotAngle = 0.0;

  // be careful changing the following dimension, they may break the
  // simulation!! (they shouldn't but they do)
  const double shoulderLength_cm = 4.5;
  c.shoulderLength = shoulderLength_cm / 43.0 * c.size;
  c.shoulderRadius = .03 * c.size;

  const double coxaLength_cm = 3.5;
  c.coxaLength = coxaLength_cm / 43.0 * c.size;
  c.coxaRadius = .04 * c.size;

  const double secondLength_cm = 6.0;
  c.secondLength = secondLength_cm / 43.0 * c.size;
  c.secondRadius = .03 * c.size;
  c.tebiaRadius = 1.3 / 43.0 * c.size;

  const double tebiaLength_cm = 11.5; // 3)
  c.tebiaLength = tebiaLength_cm / 43.0 * c.size;

  // this determines the limit of the footspring
  c.footRange = .2 / 43.0 * c.size;
  c.footRadius = 1.5 / 43.0 * c.size;

  // -----------------------
  // 2) Joint Limits
  // Setting Max, Min of each joint with respect to real
  // -----------------------

  //Similar to real robot
  //-45 deg; downward (+) MIN
  c.backJointLimitD = M_PI / 180 * 45.0;
  // 45 deg; upward (-) MAX
  c.backJointLimitU = -M_PI / 180 * 45.0;

  // 70 deg; forward (-) MAX --> normal walking range 60 deg MAX
  c.fcoxaJointLimitF = -M_PI / 180.0 * 70.0;
  //-70 deg; backward (+) MIN --> normal walking range -10 deg MIN
  c.fcoxaJointLimitB = M_PI / 180.0 * 70.0;

  //60 deg; forward (-) MAX --> normal walking range 30 deg MAX
  c.mcoxaJointLimitF = -M_PI / 180.0 * 60.0;
  //60 deg; backward (+) MIN --> normal walking range -40 deg MIN
  c.mcoxaJointLimitB = M_PI / 180 * 60.0;

  //70 deg; forward (-) MAX --> normal walking range 60 deg MAX
  c.rcoxaJointLimitF = -M_PI / 180.0 * 70.0;
  //70 deg; backward (+) MIN --> normal walking range -10 deg MIN
  c.rcoxaJointLimitB = M_PI / 180.0 * 70.0;

  // 70 deg; downward (+) MIN
  c.secondJointLimitD = M_PI / 180.0 * 75.0;
  // 70 deg upward (-) MAX
  c.secondJointLimitU = -M_PI / 180.0 * 75.0;

  //130 deg downward; (+) MIN
  c.tebiaJointLimitD = M_PI / 180.0 * 130.0;
  // 20 deg  downward; (+) MAX
  c.tebiaJointLimitU = M_PI / 180.0 * 20.0;

  // -----------------------
  // 3) Motors
  // Motor power and joint stiffness
  // -----------------------

  c.footSpringPreload = 8.0 / 43.0 * c.size;
  // negative is downwards (spring extends)
  c.footSpringLimitD = c.footSpringPreload;
  c.footSpringLimitU = c.footSpringPreload + c.footRange;

  const double backPower_scale = 30.0;
  const double coxaPower_scale = 10.0;
  const double springstiffness = 350.0;

  // use an original radius and mass and scale original torque by their
  // new values to keep acceleration constant
  c.backPower = backPower_scale * (1.962 / (0.035 * 2.2)) * c.coxaLength * c.trunkMass;
  // torque in Nm
  c.coxaPower = coxaPower_scale * (1.962 / (0.035 * 2.2)) * c.coxaLength * c.trunkMass;
  c.secondPower = c.coxaPower;
  c.tebiaPower = c.coxaPower;
  // this is the spring constant. To keep  acceleration for the body
  // constant, we use the above unscaled preload of 0.08 and original
  // trunkMass to and then multiply by the new ones
  c.footPower = (springstiffness * 0.08 / 2.2) * c.trunkMass / c.footSpringPreload;

  c.backDamping = 0.0;
  // Georg: no damping required for new servos
  c.coxaDamping = 0.0;
  c.secondDamping = 0.0;
  c.tebiaDamping = 0.01;
  c.footDamping = 0.05; // a spring has no damping??

  //Increasing MaxVel by KOH to a factor of 1.7
  c.backMaxVel = 1.7 * 1.961 * M_PI;
  // The speed calculates how it works
  c.coxaMaxVel = 1.7 * 1.961 * M_PI;
  c.secondMaxVel = 1.7 * 1.961 * M_PI;
  c.tebiaMaxVel = 1.7 * 1.961 * M_PI;
  c.footMaxVel = 1.7 * 1.961 * M_PI;

  c.usRangeFront = 0.93 * c.size;
  c.irRangeLeg = 0.03 * c.size;

  //Values by Dennis
  // 1 is parallel, -1 is antiparallel
  c.usParallel = false;
  c.usAngleX = 0.25;
  c.usAngleY = 1;

  c.texture = "Images/whiteground.rgb";
  c.bodyTexture = "Images/stripes.rgb";

  return c;
}
AmosIIConf getAmosIIv1Conf(double _scale, bool _useShoulder, bool _useFoot, bool _useBack) {
  // Take basic configuration from amosiiv2
  // and then make necessary changes
  AmosIIConf c = getAmosIIv2Conf(_scale, _useShoulder, _useFoot, _useBack);

  // "Internal" variable storing the currently used version
  c.amos_version = 1;

  //trunk height
  c.height = /*6.5*/8.5 / 43.0 * c.size; //---------------------------------------------------AMOSIIv1
  // -----------------------
  // 1) Biomechanics
  // Manual setting adjustable joint positions at the body
  // -----------------------

  // amosII has a fixed but adjustable joint that decides how the legs
  // extend from the trunk. Here you can adjust these joints

  // ------------- Front legs -------------
  // angle (in rad) around vertical axis at leg-trunk fixation 0:
  // perpendicular
  // => forward/backward
  c.fLegTrunkAngleV = 0.0;
  // angle around horizontal axis at leg-trunk fixation 0: perpendicular
  // => upward/downward
  c.fLegTrunkAngleH = 3.1416 / 6; //---------------------------------------------------AMOSIIv1
  // rotation of leg around own axis 0: first joint axis is vertical
  // => till
  c.fLegRotAngle = 0.0;

  // ------------- Middle legs ----------------
  // => forward/backward
  c.mLegTrunkAngleV = 0.0;
  // => upward/downward
  c.mLegTrunkAngleH = 3.1416 / 6; //---------------------------------------------------AMOSIIv1
  // => till
  c.mLegRotAngle = 0.0;

  // ------------- Rear legs ------------------
  // => forward/backward
  c.rLegTrunkAngleV = 0.0;
  // => upward/downward
  c.rLegTrunkAngleH = 3.1416 / 6; //---------------------------------------------------AMOSIIv1
  // => till
  c.rLegRotAngle = 0.0;

  // -----------------------
  // 2) Joint Limits
  // Setting Max, Min of each joint with respect to real
  // -----------------------
  //
  //Similar to real robot
  //-45 deg; downward (+) MIN
  c.backJointLimitD = M_PI / 180 * 45.0; //---------------------------------------------------AMOSIIv1
  // 45 deg; upward (-) MAX
  c.backJointLimitU = -M_PI / 180 * 45.0;

  // 45 deg; forward (-) MAX --> normal walking range 25 deg MAX
  c.fcoxaJointLimitF = -M_PI / 180.0 * 45.0;
  //-45 deg; backward (+) MIN --> normal walking range -30 deg MIN
  c.fcoxaJointLimitB = M_PI / 180.0 * 45.0;

  //45 deg; forward (-) MAX --> normal walking range 25 deg MAX
  c.mcoxaJointLimitF = -M_PI / 180.0 * 45.0;
  //45 deg; backward (+) MIN --> normal walking range -30 deg MIN
  c.mcoxaJointLimitB = M_PI / 180 * 45.0;

  //45 deg; forward (-) MAX --> normal walking range 25 deg MAX
  c.rcoxaJointLimitF = -M_PI / 180.0 * 45.0;
  //45 deg; backward (+) MIN --> normal walking range -30 deg MIN
  c.rcoxaJointLimitB = M_PI / 180.0 * 45.0;

  // 30 deg; downward (+) MIN --> normal walking range 65 deg MIN
  c.secondJointLimitD = M_PI / 180.0 * 30.0;
  // 100 deg upward (-) MAX --> normal walking range 115 deg MAX
  c.secondJointLimitU = -M_PI / 180.0 * 100.0;

  //140 deg downward; (+) MIN --> normal walking range 140 deg MIN
  c.tebiaJointLimitD = M_PI / 180.0 * 140.0;
  //15 deg  downward; (+) MAX --> normal walking range 120 deg MAX
  c.tebiaJointLimitU = M_PI / 180.0 * 15.0;

  return c;
}

// Parameter settings
void setConfigs(AmosIIConf& amConf, HexabotControllerConf& hcConf, runLogSimulation::runLogSimConf& rlConf){
    // set configuration of the parameters
    //double phase1 = 0.;
    //double phase2 = M_PI;

    //======= Robot ==============================================
    // using amosii version 1
    amConf = getAmosIIv1Conf(1.,1,1,0);

    // ======= Controller ==================================================
    hcConf = HexabotController::getDefaultConf();

    // set the Freq, Duty rate and inhiibi Coef
    for(int i = 0; i < 6; i++){
        hcConf.oscConf[i].freq = 1./ (5. / 0.45);//1./ (3. / 0.4);
        hcConf.oscConf[i].dutyRate = 0.55;
        hcConf.oscConf[i].inhibiCoef = 2;//1.; //0.5 1.5; //1.5; //0.9
    }
    hcConf.controlFreq = (double)S_CONTROL_FREQ;
    hcConf.twoCpgCoef = 10.; //10. //2.5; //0.3; //0.05; //0.1;

    // set initial Freq Difference
    hcConf.oscConf[0].initialPhase = 0;//3/2. * M_PI;
    hcConf.oscConf[1].initialPhase = M_PI;//3/2. * M_PI;
    hcConf.oscConf[2].initialPhase = 0;//3/2. * M_PI;
    hcConf.oscConf[3].initialPhase = M_PI;//3/2. * M_PI;
    hcConf.oscConf[4].initialPhase = 0;//3/2. * M_PI;
    hcConf.oscConf[5].initialPhase = M_PI;//3/2. * M_PI;

    // to use phase reset or not
    hcConf.PC_MODE = PC_PR_LPASS;//PC_NONE;//PC_PR_LPASS;//PC_PR_LPASS;//PC_IN_IN; // PC_PR_PR
    hcConf.TFactor = 1.;

    // to use one side phase modulation or not
    hcConf.is_oneSidePhaseModulation = false;//true; //true;//false;

    // set for poincre analysis
    //  the time when we start the phase reset mechanism after start the simulation.
    hcConf.biasStartTime = 3.;

    // set the leg trajectory
    for(int i=0;i<3;i++){
        hcConf.trjConf[i].centerTrj = osg::Vec3d( 0.11 , 0 , -0.105);
    }
    for(int i=3;i<6;i++){
        hcConf.trjConf[i].centerTrj = osg::Vec3d( 0.11 , 0 , -0.105);
    }
    // fore legs
    hcConf.trjConf[0].centerTrj = osg::Vec3d( 0.11, 0.0, -0.102);//-0.05
    hcConf.trjConf[3].centerTrj = osg::Vec3d( 0.11, -0.0, -0.102);
    // middle legs
    hcConf.trjConf[1].centerTrj = osg::Vec3d( 0.11, -0.04, -0.102);//-0.105
    hcConf.trjConf[4].centerTrj = osg::Vec3d( 0.11, 0.04, -0.102);// 0.04
    // hind legs
    hcConf.trjConf[2].centerTrj = osg::Vec3d( 0.11, 0.0, -0.102);//-0.05
    hcConf.trjConf[5].centerTrj = osg::Vec3d( 0.11, -0.0, -0.102);

    bool is_leg_upDown = true;
    if(is_leg_upDown){
        for(int i=0;i<6;i++){
            hcConf.trjConf[i].walkLength = 0.08;//0.10; //0.06;//0.04;
            hcConf.trjConf[i].swingHeight = 0.05;//6
        }
    }

    // ====== Modules =============================================================
    //******* RunLogSimulation **************************
    rlConf = runLogSimulation::getDefaultConf();
    rlConf.poincarePhase = 0.;
    rlConf.poincarePhaseLegNum = 1;
    rlConf.waitTime = 30;

    // #comment 2
    //  We set the initial value (phase difference betwn front and middle, middle and rear leg) here
    // Candidates to check are following values
    // At each duty factor, we conduct 8 trials

    // Experiments for duty 0.575
    rlConf.initialState = osg::Vec2d(1.2, 2.4);// For Trv d0575
    //rlConf.initialState = osg::Vec2d(2.2, 2.4);// For Trv d0575
    //rlConf.initialState = osg::Vec2d(1.7, 1.9);// For Trv d0575
    //rlConf.initialState = osg::Vec2d(1.7, 2.9);// For Trv d0575

    //rlConf.initialState = osg::Vec2d(4.4, 4.7);// For Bck d0575
    //rlConf.initialState = osg::Vec2d(3.4, 4.7);// For Bck d0575
    //rlConf.initialState = osg::Vec2d(3.9, 5.2);// For Bck d0575
    //rlConf.initialState = osg::Vec2d(3.9, 4.2);// For Bck d0575

    // We should choose duty 0.525 or 0.625 (Better result is OK)
    // Experiments for duty 0.525
    //rlConf.initialState = osg::Vec2d(1.5, 2.7);// For Trv d0525
    //rlConf.initialState = osg::Vec2d(2.5, 2.7);// For Trv d0525
    //rlConf.initialState = osg::Vec2d(2., 2.2);// For Trv d0525
    //rlConf.initialState = osg::Vec2d(2., 3.);// For Trv d0525

    //rlConf.initialState = osg::Vec2d(4.2, 4.5);// For Bck d0525
    //rlConf.initialState = osg::Vec2d(3.4, 4.5);// For Bck d0525
    //rlConf.initialState = osg::Vec2d(3.7, 5.);// For Bck d0525
    //rlConf.initialState = osg::Vec2d(3.7, 4.);// For Bck d0525

    // Experiments for duty 0.625
    //rlConf.initialState = osg::Vec2d(1.1, 2.0);// For Trv d0625
    //rlConf.initialState = osg::Vec2d(1.9, 2.0);// For Trv d0625
    //rlConf.initialState = osg::Vec2d(1.4, 1.5);// For Trv d0625
    //rlConf.initialState = osg::Vec2d(1.4, 2.5);// For Trv d0625

    //rlConf.initialState = osg::Vec2d(4.6, 4.9);// For Bck d0625
    //rlConf.initialState = osg::Vec2d(3.6, 4.9);// For Bck d0625
    //rlConf.initialState = osg::Vec2d(4.1, 5.2);// For Bck d0625
    //rlConf.initialState = osg::Vec2d(4.1, 4.4);// For Bck d0625


    //rlConf.initialState = osg::Vec2d(4.2, 4.8);// For Backward d06
    //rlConf.initialState = osg::Vec2d(3.7, 4.8);// For Back d06
    //rlConf.initialState = osg::Vec2d(4., 4.);// For Back d06

    //rlConf.initialState = osg::Vec2d(1.5, 1.7);// For Trv d06
    //rlConf.initialState = osg::Vec2d(2., 2.2);// For Trv d06
    //rlConf.initialState = osg::Vec2d(1.5, 2.7);// For Trv d06
    //rlConf.initialState = osg::Vec2d(1.5, 2.2);// For Trv d06

    // duty 055
    //rlConf.initialState = osg::Vec2d(2.5, 2.6);// For Trv d055
    //rlConf.initialState = osg::Vec2d(1.5, 2.6);// For Trv d055
    //rlConf.initialState = osg::Vec2d(2., 2.1);// For Trv d055
    //rlConf.initialState = osg::Vec2d(2., 3.1);// For Trv d055

    //rlConf.initialState = osg::Vec2d(4.3, 4.9);// For Bck d055
    //rlConf.initialState = osg::Vec2d(3.3, 4.9);// For Bck d055
    //rlConf.initialState = osg::Vec2d(3.8, 5.4);// For Bck d055
    //rlConf.initialState = osg::Vec2d(3.8, 4.4);// For Bck d055

    //rlConf.initialState = osg::Vec2d(2, 2);// For Trv d05
    //rlConf.initialState = osg::Vec2d(3.8, 4.9);// For Bck d05


    //rlConf.initialState = osg::Vec2d(4., 4.); // For Backward d05 1
    //rlConf.initialState = osg::Vec2d(4, 3.8); // For Backward d05 2
    //rlConf.initialState = osg::Vec2d(4., 4.4); // For Backward d05 3
    //rlConf.initialState = osg::Vec2d(3.5, 3.5); // For Backward d05 4
    //rlConf.initialState = osg::Vec2d(4.5, 3.5); // For Backward d05 5
    //rlConf.initialState = osg::Vec2d(3.5, 4.5); // For Backward d05 6

    //rlConf.initialState = osg::Vec2d(5, 5); // For Backward d06 1
    //rlConf.initialState = osg::Vec2d(5., 4.25); // For Backward d06 2
    //rlConf.initialState = osg::Vec2d(4.25, 5.); // For Backward d06 3
    //rlConf.initialState = osg::Vec2d(3.5, 3.5); // For Backward d06 4
    //rlConf.initialState = osg::Vec2d(5., 3.5); // For Backward d06 5
    //rlConf.initialState = osg::Vec2d(3.5, 5.); // For Backward d06 6

    //rlConf.initialState = osg::Vec2d(2., 2.); // For Travelling d05 1
    //rlConf.initialState = osg::Vec2d(2., 2.5); // For Travelling d05 2
    //rlConf.initialState = osg::Vec2d(2.5, 2.); // For Travelling d05 3
    //rlConf.initialState = osg::Vec2d(3., 3.); // For Travelling d05 4
    //rlConf.initialState = osg::Vec2d(2., 3.); // For Travelling d05 5
    //rlConf.initialState = osg::Vec2d(3., 2.); // For Travelling d05 6

    //rlConf.initialState = osg::Vec2d(1.5, 1.5); // For Travelling d06 1
    //rlConf.initialState = osg::Vec2d(1.5, 2.25); // For Travelling d06 2
    //rlConf.initialState = osg::Vec2d(2.25, 1.5); // For Travelling d06 3
    //rlConf.initialState = osg::Vec2d(3., 3.); // For Travelling d06 4
    //rlConf.initialState = osg::Vec2d(1.5, 3.); // For Travelling d06 5
    //rlConf.initialState = osg::Vec2d(3., 1.5); // For Travelling d06 6

    // rlConf.initialState = osg::Vec2d(2.2, 2.2); // For Travelling d06
    // rlConf.initialState = osg::Vec2d(2., 2.); // For Travelling d065

    return;
}

// log configs
void logConfigs(bool logFlag_, AmosIIConf& amConf, HexabotControllerConf& hcConf, runLogSimulation::runLogSimConf& rlConf){
    // without log
    if(!logFlag_) return;

    // file out
    std::ofstream paramOfs;
    // log mode
    std::stringstream str;
    str << DETAIL_FILE_POS << "Configs_" << FILE_NAME << ".dat";
    paramOfs.open(str.str());


        paramOfs << "# This is the file contains the all config parameters of the robot and controller" << std::endl;
        paramOfs << "# Main.Ver :: real " << ",  Cntr.Ver :: " << VERSION_HCON << ",  Robot.Ver :: " << VERSION_HPA << std::endl;
        paramOfs << std::endl;

        paramOfs << "***** Ssimulation ****************************" << std::endl;
        paramOfs << " SimFreq : Real"  << ",  CtrFreq : " << S_CONTROL_FREQ << std::endl;
        paramOfs << std::endl;

        paramOfs << "***** Amos robot **************************" << std::endl;
        HexabotConf hrConf = AmosController_adptHexabot::changeConf(amConf);
        paramOfs << "Amos Version :" << amConf.amos_version << std::endl;
        paramOfs << " size : " << std::endl;
        paramOfs << "   body length : " << amConf.size + amConf.frontLength << ", body width : "<< amConf.width << ", y_TC_TC : " << hrConf.jLength.length_y_TCJ_to_TCJ << ", x_centr_TC : " << hrConf.jLength.length_x_center_to_TCJ << std::endl;
        paramOfs << "   joint length (sholder)1-2-3(foot): " <<     hrConf.jLength.length_TCJ_to_CTJ << ",  " << hrConf.jLength.length_CTJ_to_FTJ <<",  "<< hrConf.jLength.length_FTJ_to_Toe << std::endl;
        paramOfs << " mass : " << std::endl;
        paramOfs << "   whole : "<< hrConf.wholeMass << ", body : " << amConf.trunkMass + amConf.frontMass << ", joint 1-2-3 : " <<  amConf.coxaMass << ", " <<  amConf.secondMass << ", "  << amConf.tebiaMass <<  std::endl;
        paramOfs << " COM : " << "center" << std::endl;
        paramOfs << std::endl;

        paramOfs << "***** Hexabot controller **************************" << std::endl;
        paramOfs << " General Param " << std::endl;
        //paramOfs << "   PD gain P-D-I : " << hrConf.servoParam.power << ", " << hrConf.servoParam.damp << ", " << hrConf.servoParam.integ << std::endl;
        paramOfs << "   Freq - Duty   : " << hcConf.oscConf[0].freq << ", " << hcConf.oscConf[0].dutyRate  << std::endl;
        paramOfs << "   Inhibi Coef   : " << hcConf.oscConf[0].inhibiCoef << std::endl;
        paramOfs << "   Left and Right CPGs Relation Coef : " << hcConf.twoCpgCoef << std::endl;

        paramOfs << " Phase Modulation  " << std::endl;
        paramOfs << "   Modulation Mode : " << hcConf.PC_MODE << "  (0:Phasereset + inhibi, 1:Phasereset only on swing phase, 2:PhaseReset+PhaseReset, 3:Inhibi+inhibi )" << std::endl;
        paramOfs << "   Modulation Use one side : " << hcConf.is_oneSidePhaseModulation << std::endl;

        paramOfs << " Leg Trajectory" << std::endl;
        paramOfs << "   stance length : " << hcConf.trjConf[0].walkLength <<
                    ", swing height : " << hcConf.trjConf[0].swingHeight << std::endl;

        paramOfs << std::endl;
        paramOfs << "***** One proc simulators **************************" << std::endl;
        paramOfs << "*****    Run log simulation  **************************" << std::endl;
        paramOfs << " Initial val : ( " << rlConf.initialState.x() << ", " << rlConf.initialState.y() << ")" << std::endl;

      if(paramOfs.is_open()) paramOfs.close();

    return;
}

// Helper
int contains(char **list, int len,  const char *str){
    for(int i=0; i<len; i++){
        if(strcmp(list[i],str) == 0) return i+1;
    }
    return 0;
};

int main(int argc, char** argv){
    // おまじないと思っている
    list<PlotOption> plotoptions;
    int port = 1;
    int index = contains(argv,argc,"-g");
    if(index >0 && argc>index) {
        plotoptions.push_back(PlotOption(GuiLogger, atoi(argv[index])));
    }
    if(contains(argv,argc,"-f")!=0) plotoptions.push_back(PlotOption(File));
    if(contains(argv,argc,"-n")!=0) plotoptions.push_back(PlotOption(MatrixViz));
    if(contains(argv,argc,"-l")!=0) singleline=false;
    index = contains(argv,argc,"-p");
    if(index >0 && argc>index)
        port=atoi(argv[index]);
    if(contains(argv,argc,"-h")!=0) {
        printf("Usage: %s [-g N] [-f] [-n] [p PORT]\n",argv[0]);
        printf("\t-g N\tstart guilogger with interval N\n\t-f\twrite logfile\n");
        printf("\t-n\tstart neuronviz\n");
        printf("\t-p PORT\t COM port number, default 1\n");
        printf("\t-h\tdisplay this help\n");
        exit(0);
    }
    initializeConsole();

// 変数の宣言など(特にポインタなど) **************
    // grobal data
    GlobalData globaldata;
    // Agent
    Agent* agent = 0;
    // base system
    itfSimulation<HexabotController> *itfSim = 0; // just refarence
    // system which we use
    repeatAnalysis<HexabotController> *repAnalSim = 0;
    //One proc simulators ***************************
    // runLogSimulation
    runLogSimulation *runLog_onePSim = 0;
    //Repeat modulators ****************************
    itfRepeatModulator *pRepMod = 0;
    //TimedChangePram ******************************
     itfTimedChangePrmAgent *pTCPA = 0;
     itfTimedChangePrmAgent *pTCPA_sub1 = 0;
     itfTimedChangePrmAgent *pTCPA_sub2 = 0;

    // robot
    //Hexabot* hexabot;
    AmosIISerialV1* robot = 0;
    // controller
    AbstractController* controller = 0;
    HexabotController* tController = 0;
    AmosController_adptHexabot* aController = 0;

    // configs
    HexabotControllerConf hcConf;
    //HexabotConf hrConf;
    AmosIIConf amConf;
    // configs of one proc simulator
    runLogSimulation::runLogSimConf rlConf;
    // simNum
    int simNum = 1;

    //STATUS
    enum STATUS_ {IDLE, WAIT_INITIALIZE, PROC_SIMULATION, ERROR};
    enum STATUS_ STATUS = IDLE;

// 初期設定 ****************************
   // パラメタ設定とか、変数の変化に何を使うとかいろいろ
    // set configs of the robot
    setConfigs(amConf, hcConf, rlConf);
    // set the simulation program
    pRepMod = new cNoParamRepMod(1);
    // timed param Changing Agent
    cLinearFunc_changeDuty_keepST_tpa::paramSet prm2(hcConf.oscConf[0].dutyRate, hcConf.oscConf[1].dutyRate, hcConf.oscConf[2].dutyRate, hcConf.oscConf[3].dutyRate, hcConf.oscConf[4].dutyRate, hcConf.oscConf[5].dutyRate,
                                                           hcConf.oscConf[0].freq, hcConf.oscConf[1].freq, hcConf.oscConf[2].freq, hcConf.oscConf[3].freq, hcConf.oscConf[4].freq, hcConf.oscConf[5].freq);
    // #comment 3
    //  To change the duty factor, we should change the 3rd term of these two constructors
    //  ex) cLinearFunc_changeDuty_keepST_tpa(prm2, 5, 0.525 <- this is the duty factor, 0.0);
    pTCPA_sub1 = new cLinearFunc_changeDuty_keepST_tpa(prm2, 5, 0.575, 0.0);
    pTCPA_sub2 = new cLinearFunc_changeDuty_keepST_tpa(prm2, 5, 0.575, -0.0004);
    pTCPA = new cCombineTwoAgents_tpa(pTCPA_sub1, pTCPA_sub2, 180);
    // oneProcSimulation class
    runLog_onePSim = new runLogSimulation(rlConf, pTCPA, 180, true, true, DETAIL_FILE_POS, FILE_NAME, S_CONTROL_FREQ);
    // simulation class
    repAnalSim = new repeatAnalysis<HexabotController>( (itfOneProcSimulation<HexabotController>&)(*runLog_onePSim), (itfRepeatModulator&)(*pRepMod) );
    // set itfSim
    itfSim = repAnalSim;

// 実験開始準備 *********************
    // simulation start
    itfSim->start_simulation();
    // update configs before making robots
    itfSim->request_updateConfigs();
    // log configs
    logConfigs(true, amConf, hcConf, rlConf);
    // change status
    STATUS = WAIT_INITIALIZE;

// ロボット、コントローラ作成 ****************
    // make controller
    std::ostringstream ost2;
    ost2 << "amosii" ;
    std::string hexName = ost2.str();
    char hexName2[30];
    strcpy(hexName2,hexName.c_str());

    aController = new AmosController_adptHexabot(hexName2, amConf, hcConf, false, DETAIL_FILE_POS, FILE_NAME, simNum, false);
    //tController = new HexabotController(hexName2, hrConf, hcConf, false);//  TestController::TestController(confy); // debug
    tController = aController;
    controller = aController;

    // one2onewiring gives full range of robot actuators
    AbstractWiring* wiring = new One2OneWiring(new WhiteUniformNoise(),true);

    // make a robot
    //robot         = new AmosIISerialV2("/dev/ttyS0");     // using serial port
    robot         = new AmosIISerialV1("/dev/ttyUSB0");   // using USB-to-serial adapter

    // make an agent
    agent         = new Agent(plotoptions);
    agent->init(controller, robot, wiring);
    // if you like, you can keep track of the robot with the following line.
    // this assumes that your robot returns its position, speed and orientation.
    // agent->setTracMkOptions(TrackRobot(true,false,false, false,"systemtest"));

    globaldata.agents.push_back(agent);
    globaldata.configs.push_back(robot);
    globaldata.configs.push_back(controller);

    showParams(globaldata.configs);
    printf("\nPress c to invoke parameter input shell\n");
    printf("The output of the program is more fun then useful ;-).\n");
    printf(" The number are the sensors and the position there value.\n");
    printf(" You probably want to use the guilogger with e.g.: -g 10\n");

    cmd_handler_init();
    //initscr();
    cbreak();
    noecho();
    intrflush(stdscr,FALSE);
    keypad(stdscr,TRUE);
    nodelay(stdscr,TRUE);

// Main loop to proceed
    long int t=0;
    double startTime;
    clock_t start,end;
    while(!stop){
        // proceed the robot experiments
        agent->step(noise,t);

        // procedure for itfSimulator
        // get time
        int nowStep = t;
        double time = (double)nowStep * (double)(1./ (double)S_CONTROL_FREQ);

        //Time for the simulation
        // do something depending on situation
        if(STATUS == IDLE){
            //nothing
        }
        else if(STATUS == WAIT_INITIALIZE){
            // wait for PD control converging
            if(time > time_wait){
                // change status to do simulation
                STATUS = PROC_SIMULATION;

                // simulation start
                itfSim->start_oneProc(simNum, *tController);
                startTime = time;
            }
        }
        else if(STATUS == PROC_SIMULATION){
            // proc the simulation
            itfSim->calcStep(*tController, time - startTime);

            // detect the finish
            if(itfSim->isFinished_oneProc(*tController, time - startTime)){
                STATUS = IDLE;
                // stop the simulation
                itfSim->finish_oneProc(*tController, time - startTime);
                std::cout << " ----> finish one Proc simulation" << std::endl;
                stop = 1;
            }
        }

        if(control_c_pressed()){

            if(!handleConsole(globaldata)){
                stop=1;
            }
            cmd_end_input();
        }

        t++;

        if(t % 50 == 1){
            // memorize a time
            end = clock();
            // output -->
            std::cout << "$$$ OneProcTime = " << (double)(end-start)/CLOCKS_PER_SEC / 50. * 1000. << " [ms]" << std::endl;
            // memorize a time
            start = clock();
        }

        // temporally stop command
        //KEYBOARD BJC OPTION
        // B: stop temporally, A: start again
        int key=0;
        key = wgetch (stdscr);
        //if(key != 0){std::cout << "Detect Key : " << key << std::endl;}
        if (key==98){ //B
            std::cout << "@@@ Stop temporally (Press A to restart):  time =" << (double)t / S_CONTROL_FREQ << " [s]" << std::endl;
            while(1){
                key = wgetch (stdscr);
                if (key == 97){  //A
                    break;
                }
            }
        }
    };

// Terminate the programs
    if( !itfSim->isFinished_simulation(simNum)){
        std::cout << "Programs want to continue but we do not have any choice but stopping ---> finish simulation" << std::endl;
    }
    // We do not have any choice except fot stopping ---> finish simulation
    itfSim->finish_simulation(simNum);
    std::cout << "Simulation has finished!!  Step:" << simNum <<  std::endl;
    // finish procedure

    delete robot;
    delete agent;
    closeConsole();
    fprintf(stderr,"terminating\n");
    // should clean up but what costs the world
    return 0;
}

