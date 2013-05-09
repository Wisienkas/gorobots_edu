/***************************************************************************
 *   Copyright (C) 2010 by
 *    Martin Biehl <mab@physik3.gwdg.de>                                   *
 *    Guillaume de Chambrier <s0672742@sms.ed.ac.uk>                       *
 *    martius@informatik.uni-leipzig.de                                    *
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
 **************************************************************************/

#ifndef __AMOSII_H
#define __AMOSII_H

#include <selforg/inspectable.h>

#include <ode_robots/oderobot.h>
#include <ode_robots/raysensorbank.h>
#include <ode_robots/speedsensor.h>
//----------- Add contact sensor ---- by Ren
#include <ode_robots/contactsensor.h>
#include "spring.h"

// rotation and translation matrixes (to make the code shorter)
#define ROTM osg::Matrix::rotate
#define TRANSM osg::Matrix::translate

namespace lpzrobots {
  
  class Primitive;
  class Joint;
  class OneAxisServo;
  class TwoAxisServo;
  class Spring;
  
  typedef struct Shoulder {
      Shoulder() {
        trans = 0;
        should = 0;
      }
      Primitive* trans;
      Primitive* should;
  } Shoulder;
  
  typedef struct {
    public:
      
      bool useShoulder; ///< fix the shoulder element to the trunk.
      bool useTebiaJoints; /// whether to use joints at the knees or fix them
      bool useFoot; ///< use spring foot
      bool useBack; ///< use the hinge joint in the back
      bool useWhiskers;
      bool rubberFeet; ///< if true, rubber substance is used for feet instead of the substance used for the rest of the robot
      bool useContactSensors; ///< use foot contact sensors
      bool useLocalVelSensor; ///< get velocity vector in local coordinates and pass it as sensorvalues
      
      double size; ///< scaling factor for robot (length of body)
      double width; ///< trunk width
      double height; ///< trunk height
      double frontLength; ///< length of the front of the body (if back joint is used)
      double density; ///< homogenous density used to scale the whole robot
      double mass; ///< total mass
      
      double trunkMass; ///<  trunk mass
      double frontMass;
      double shoulderMass;
      double coxaMass;
      double secondMass;
      double tebiaMass;
      double footMass;

      double shoulderHeight; ///< fix legs to trunk at this distance from bottom of trunk
      double legdist1; ///< distance between hindlegs and middle legs
      double legdist2; ///< distance between middle legs and front legs
      //amosII has a fixed but adjustable joint that decides how the legs extend from the trunk
      //here you can adjust these joints, if shoulder = 0 this still influences how the legs
      //extend (the coxa-trunk connection in that case)
      
      double fLegTrunkAngleV; ///< angle in rad around vertical axis at leg-trunk fixation 0: perpendicular
      double fLegTrunkAngleH; ///< angle around horizontal axis at leg-trunk fixation 0: perpendicular
      double fLegRotAngle; ///< rotation of leg around own axis 0: first joint axis is vertical
      double mLegTrunkAngleV; ///< middle legs and so on
      double mLegTrunkAngleH;
      double mLegRotAngle;
      double rLegTrunkAngleV;
      double rLegTrunkAngleH;
      double rLegRotAngle;

      //the zero position of the joints depends on the placement of the two bodies at
      //the time of initialisation. specify in the following at which angle the leg
      //part should extend from the previous leg part (in rad). this sets the 0 angle!
      
      double fcoxaZero; //front, negative is forward
      double fsecondZero; //positive is down
      double ftebiaZero; //positive is down
      
      //the lengths and radii of the individual leg parts
      
      double legLength;
      double shoulderLength;
      double shoulderRadius;
      double coxaLength;
      double coxaRadius;
      double secondLength;
      double secondRadius;
      double tebiaLength; ///< length of tebia including  fully extended foot spring (if used)
      double tebiaRadius;
      double footRange; ///< range of the "foot" spring
      double footRadius; ///< choose different from tebiaRadius
      
      //set limits for each joint
      
      double backJointLimitD; ///< smaller limit, positive is down
      double backJointLimitU;

      //TR, TL
      //--Front leg
      double fcoxaJointLimitF; ///< forward limit, negative is forward (zero was specified above)
      double fcoxaJointLimitB; ///< backward limit
      //--Middle leg
      double mcoxaJointLimitF; ///< forward limit, negative is forward (zero was specified above)
      double mcoxaJointLimitB; ///< backward limit
      //--Rear leg
      double rcoxaJointLimitF; ///< forward limit, negative is forward (zero was specified above)
      double rcoxaJointLimitB; ///< backward limit
      
      //--CR,CL
      double secondJointLimitD; ///< lower limit, positive is down
      double secondJointLimitU; ///< upper limit
      //--FR,FL
      double tebiaJointLimitD; ///< lower limit, positive is down
      double tebiaJointLimitU;

      double footSpringPreload; ///< preload spring
      double footSpringLimitU; ///<upper limit negative is up
      double footSpringLimitD;

      double backPower;
      double coxaPower; ///< maximal force for at hip joint motors
      double secondPower;
      double tebiaPower; ///< spring strength in the knees
      double footPower;

      double backDamping;
      double coxaDamping; ///< damping of hip joint servos
      double secondDamping;
      double tebiaDamping; ///< damping in the knees
      double footDamping;

      double backMaxVel;
      double coxaMaxVel; ///< speed of the hip servo
      double secondMaxVel;
      double tebiaMaxVel;
      double footMaxVel;

      double whiskerLength; ///< length of whisker
      
      double T; ///< T is the for the time for calculating the cost of transport over time
      double *v;

      matrix::Matrix m;
      bool *legContacts;
      double irSensors;
      bool irFront;
      bool irBack;
      bool irLeft;
      bool irRight;
      double irRangeFront;
      double irRangeBack;
      double irRangeLeft;
      double irRangeRight;

      //Added by Dennis for IR sensor
      double iranglex; // angle of the x axis
      double irangley; // angle of the y axis
      double irparallel;
  } AmosIIConf;
  
  typedef struct {
    public:
      int legID;
      dGeomID geomid;
      dBodyID bodyID;
      //mabe add all geoms and bodies here?
      
      dJointID coxajoint;
      dJointID secondjoint;
      dJointID tebiajoint;
      
  } Leg;
  
  class AmosII : public OdeRobot, public Inspectable {
    public:
      
      /**
       * constructor of VierBeiner robot
       * @param odeHandle data structure for accessing ODE
       * @param osgHandle ata structure for accessing OSG
       * @param conf configuration object
       */
      AmosII(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const AmosIIConf& conf, const std::string& name);

      virtual ~AmosII() {
        destroy();
      }
      ;

      //%%%%%%%%%%%%%%%%%%%%%%%%%Default CONFIGURATION as Real robot%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
      static AmosIIConf getDefaultConf(double _scale = 1.0, bool _useShoulder = 1, bool _useFoot = 1,
          bool _useBack = 1) {
        AmosIIConf c;
        
        //--------------Begin Don not change------------------------------------------------//
        //use shoulder (fixed joint between legs and trunk)
        c.useShoulder = _useShoulder;
        c.useTebiaJoints = 0;
        c.useFoot = _useFoot; //create springs at the end of the legs
        c.useBack = _useBack; //create a joint in the back
        c.useWhiskers = 0;
        c.rubberFeet = true; //false = 0; // true = 1; // Change!!
        c.useContactSensors = true;
        c.useLocalVelSensor = 0;
        
        // the trunk length. this scales the whole robot! all parts' sizes, masses, and forces will be adapted!!
        c.size = 0.43 * _scale;
        c.width = 7.0 / 43.0 * c.size; //16.5/43.0; //1.0/3.0; //1.0/1.5  //trunk width
        c.height = 6.5 / 43.0 * c.size; //1.0/4.0 //trunk height
        c.frontLength = 12.0 / 43.0 * c.size;
        c.legLength = 1.0 / 2.2; //length of upper and lower limb combined without the tip (will be scaled by c.size)
        //c.density    =  2.2/(0.43 * 0.07 * 0.065)/((pow(_scale,3))); /*we use as density the original trunk weight divided by the original volume*/
        c.density = 2.2 / (0.43 * 0.07 * 0.065);
        c.trunkMass = c.density * c.size * c.width * c.height; //2.200/5.758;
        std::cout << "trunkMass " << c.trunkMass << "\n";
        c.mass = 5.758 / 2.2 * c.trunkMass; //use the original trunk to total mass ratio
        std::cout << "Mass " << c.mass << "\n";
        c.v = new double[1];
        c.frontMass = c.trunkMass * c.frontLength / c.size;
        c.shoulderMass = (c.mass - c.trunkMass) //distribute the rest of the weight like this for now
        / (6 * (3.0 + c.useShoulder)) * (20.0 - c.useFoot) / 20.0;
        c.coxaMass = c.shoulderMass;
        c.secondMass = c.shoulderMass;
        c.tebiaMass = c.shoulderMass;
        std::cout << "shoulderMass " << c.shoulderMass << "\n";
        c.footMass = (c.mass - c.trunkMass) //foot gets 3 or 4 times 1/20 of shoulderMass (divide and multiply by 3 or 4)
        / 6 * c.useFoot / 20.0;
        std::cout << "footMass " << c.footMass << "\n";
        
        //As real robot!!
        double shoulderHeight_cm = 6.5;
        c.shoulderHeight = shoulderHeight_cm / 6.5 * c.height; //0) shoulder height "4.5 wrong" --> correct=6.5 cm from rotating point
            
        c.legdist1 = 19.0 / 43.0 * c.size; //distance between hindlegs and middle legs
        c.legdist2 = 15.0 / 43.0 * c.size; //distance between middle legs and front legs
            
        //--------------End Don not change------------------------------------------------//
        
        //-1)--Begin-Biomechanics--Manual setting adjustable joint positions at the body----//
        
        //amosII has a fixed but adjustable joint that decides how the legs extend from the trunk
        //here you can adjust these joints
        
        //--------Front legs
        //--Forward/Backward
        c.fLegTrunkAngleV = 0.0; ///< angle (in rad) around vertical axis at leg-trunk fixation 0: perpendicular
            
        //--Upward/Downward
        c.fLegTrunkAngleH = 0.0; ///< angle around horizontal axis at leg-trunk fixation 0: perpendicular
            
        //--Till
        c.fLegRotAngle = 0.0; ///< rotation of leg around own axis 0: first joint axis is vertical
            
        //--------Middle legs
        //--Forward/Backward
        c.mLegTrunkAngleV = 0.0; ///< middle legs and so on
            
        //--Upward/Downward
        c.mLegTrunkAngleH = 0.0;
        
        //--Till
        c.mLegRotAngle = 0.0;
        
        //--------Rear legs
        //--Forward/Backward
        c.rLegTrunkAngleV = 0.0;
        
        //--Upward/Downward
        c.rLegTrunkAngleH = 0.0;
        
        //--Till
        c.rLegRotAngle = 0.0;
        
//    	c.fLegTrunkAngleH = 0.15;
//    	c.mLegTrunkAngleH = 0.15;
//    	c.rLegTrunkAngleH = 0.15;
        
//        c.fLegRotAngle = 0.85;
//        c.mLegRotAngle = 0.85;
//        c.rLegRotAngle = 0.85;
        
        //---End-Biomechanics--Manual setting adjustable joint positions at the body----//
        
        //the zero position of the joints depends on the placement of the two bodies at
        //the time of initialisation. specify in the following at which angle the leg
        //part should extend from the previous leg part (in rad). this sets the 0 angle!
        //don't use this, it is usually not necessary and might be removed. it was intended
        //to talk to ode's servo motors directly instead of using lpzrobots set motor functions
        
        c.fcoxaZero = 0.0; //front, negative is forward
        c.fsecondZero = 0.0; //positive is down
        c.ftebiaZero = 0.0; //positive is down
            
        //be careful changing the following dimension, they may break the simulation!! (they shouldn't but they do)
        double shoulderLength_cm = 4.5;
        c.shoulderLength = shoulderLength_cm / 43.0 * c.size; //4.5
        c.shoulderRadius = .03 * c.size;
        
        double coxaLength_cm = 3.5;
        c.coxaLength = coxaLength_cm / 43.0 * c.size; //first length = 3.5 cm
        c.coxaRadius = .04 * c.size;
        
        double secondLength_cm = 6.0;
        c.secondLength = secondLength_cm / 43.0 * c.size; //second length = 6.0 cm
        c.secondRadius = .03 * c.size;
        c.tebiaRadius = 1.3 / 43.0 * c.size;
        
        //c.tebiaLength = 11.5/43.0*c.size; //KOH change
        
        double tebiaLength_cm = 11.5; // 3)
        //tebiaLength_setup = (11.2-1.3-0.2); // as real robot (9.7, while real robot = 11.5)
        c.tebiaLength = tebiaLength_cm / 43.0 * c.size; //10.2/43.0*c.size;
            
        //c.tebiaLength = (11.2-2.0)/43.0*c.size;//10.2/43.0*c.size;
        
        c.footRange = .2 / 43.0 * c.size; //this determines the limit of the footspring
        c.footRadius = 1.5 / 43.0 * c.size;
        
        //-2)--Begin------Setting Max, Min of each joint with respect to real robot----//
        
        //Similar to real robot
        c.backJointLimitD = M_PI / 180 * 45.0; //-45 deg; downward (+) MIN
        c.backJointLimitU = -M_PI / 180 * 45.0; //45 deg; upward (-) MAX
            
        c.fcoxaJointLimitF = -M_PI / 180.0 * 70.0; //70 deg; forward (-) MAX --> normal walking range 60 deg MAX
        c.fcoxaJointLimitB = M_PI / 180.0 * 70.0; //-70 deg; backward (+) MIN --> normal walking range -10 deg MIN
            
        c.mcoxaJointLimitF = -M_PI / 180.0 * 60.0; //60 deg; forward (-) MAX --> normal walking range 30 deg MAX
        c.mcoxaJointLimitB = M_PI / 180 * 60.0; //60 deg; backward (+) MIN --> normal walking range -40 deg MIN
            
        c.rcoxaJointLimitF = -M_PI / 180.0 * 70.0; //70 deg; forward (-) MAX --> normal walking range 60 deg MAX
        c.rcoxaJointLimitB = M_PI / 180.0 * 70.0; //70 deg; backward (+) MIN --> normal walking range -10 deg MIN
            
        c.secondJointLimitD = M_PI / 180.0 * 75.0; // 70 deg; downward (+) MIN
        c.secondJointLimitU = -M_PI / 180.0 * 75.0; // 70 deg upward (-) MAX
            
        c.tebiaJointLimitD = M_PI / 180.0 * 130.0; //130 deg downward; (+) MIN
        c.tebiaJointLimitU = M_PI / 180.0 * 20.0; // 20 deg  downward; (+) MAX
            
        //-2)--End------Setting Max, Min of each joint with respect to real robot----//
        
        //-3)--Begin- Motor power and joint stiffness----//
        
        c.footSpringPreload = 8.0 / 43.0 * c.size;
        c.footSpringLimitD = c.footSpringPreload; //negative is downwards (spring extends)
        c.footSpringLimitU = c.footSpringPreload + c.footRange;
        
        double backPower_scale = 30.0;
        double coxaPower_scale = 10.0; //15.0;
        double springstiffness = 350.0;
        
        // c.backPower = 2.0*(1.962/(0.035*2.2))*c.coxaLength*c.trunkMass;/*use an original radius and mass and scale original torque by their new values to keep acceleration constant*/
        c.backPower = backPower_scale * (1.962 / (0.035 * 2.2)) * c.coxaLength * c.trunkMass;
        std::cout << "backPower " << c.backPower << "\n";
        //c.coxaPower  = 1.0*(1.962/(0.035*2.2))*c.coxaLength*c.trunkMass;// c.backPower; //torque in Nm
        c.coxaPower = coxaPower_scale * (1.962 / (0.035 * 2.2)) * c.coxaLength * c.trunkMass; // c.backPower; //15 factor torque in Nm
        c.secondPower = c.coxaPower;
        c.tebiaPower = c.coxaPower;
        c.footPower = (/*150.0*/springstiffness * 0.08 / 2.2) * c.trunkMass / c.footSpringPreload; /*this is the spring constant
         to keep  acceleration for the body
         constant, we use the above unscaled
         preload of 0.08 and original trunkMass
         to and then multiply by the new ones*/
        
        c.backDamping = 0.0; //5;
        c.coxaDamping = 0.0; // Georg: no damping required for new servos
        c.secondDamping = 0.0;
        c.tebiaDamping = 0.01;
        c.footDamping = 0.05; // a spring has no damping??
            
        c.backMaxVel = 1.961 * M_PI;
        c.coxaMaxVel = 1.961 * M_PI; // The speed calculates how it works
        c.secondMaxVel = 1.961 * M_PI;
        c.tebiaMaxVel = 1.961 * M_PI;
        c.footMaxVel = 1.961 * M_PI;
        c.T = 1.0;
        
        c.whiskerLength = 4.0 / 43.0 * c.size; //scale
            
        c.legContacts = new bool[6];
        c.irSensors = false;
        c.irFront = false;
        c.irBack = false;
        c.irLeft = false;
        c.irRight = false;
        c.irRangeFront = 3;
        c.irRangeBack = 2;
        c.irRangeLeft = 2;
        c.irRangeRight = 2;
        
        //Values by Dennis
        c.irparallel = -1; // 1 is parallel, -1 is antiparallel
        c.iranglex = 0.5;
        c.irangley = 1; //1;
        //      c.elasticity = 10;
        return c;
      }
      
      //%%%%%%%%%%%%%%%%%%%%%%%%%CONFIGURATION%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
      
      //%%%%%%%%%%%%%%%%%%%%%%%%%Other CONFIGURATIONS NOT USE%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
      
      //joint limits differ, density is reduced with scale
      static AmosIIConf getExperimentConf(double _scale = 1.0, bool _useShoulder = 1, bool _useFoot = 1, bool _useBack =
          0) {
        AmosIIConf c;
        
        //use shoulder (fixed joint between legs and trunk)
        c.useShoulder = _useShoulder;
        c.useTebiaJoints = 0;
        c.useFoot = _useFoot; //create springs at the end of the legs
        c.useBack = _useBack; //create a joint in the back
        c.useWhiskers = 0;
        c.rubberFeet = false;
        c.useContactSensors = 0;
        c.useLocalVelSensor = 0;
        
        // the trunk length. this scales the whole robot! all parts' sizes, masses, and forces will be adapted!!
        c.size = 0.43 * _scale;
        c.width = 7.0 / 43.0 * c.size; //16.5/43.0; //1.0/3.0; //1.0/1.5  //trunk width
        c.height = 6.5 / 43.0 * c.size; //1.0/4.0 //trunk height
        c.frontLength = 12.0 / 43.0 * c.size;
        c.legLength = 1.0 / 2.2; //length of upper and lower limb combined without the tip (will be scaled by c.size)
        c.density = 2.2 / (0.43 * 0.07 * 0.065) / (pow(_scale, 3)); /*we use as density the original trunk weight
         divided by the original volume*/
        c.trunkMass = c.density * c.size * c.width * c.height; //2.200/5.758;
        std::cout << "trunkMass " << c.trunkMass << "\n";
        c.mass = 5.758 / 2.2 * c.trunkMass; //use the original trunk to total mass ratio
        std::cout << "Mass " << c.mass << "\n";
        c.v = new double[1];
        c.frontMass = c.trunkMass * c.frontLength / c.size;
        c.shoulderMass = (c.mass - c.trunkMass) //distribute the rest of the weight like this for now
        / (6 * (3.0 + c.useShoulder)) * (20.0 - c.useFoot) / 20.0;
        c.coxaMass = c.shoulderMass;
        c.secondMass = c.shoulderMass;
        c.tebiaMass = c.shoulderMass;
        std::cout << "shoulderMass " << c.shoulderMass << "\n";
        c.footMass = (c.mass - c.trunkMass) //foot gets 3 or 4 times 1/20 of shoulderMass (divide and multiply by 3 or 4)
        / 6 * c.useFoot / 20.0;
        std::cout << "footMass " << c.footMass << "\n";
        
        c.shoulderHeight = 4.5 / 6.5 * c.height;
        c.legdist1 = 19.0 / 43.0 * c.size; //distance between hindlegs and middle legs
        c.legdist2 = 15.0 / 43.0 * c.size; //distance between middle legs and front legs
            
        //amosII has a fixed but adjustable joint that decides how the legs extend from the trunk
        //here you can adjust these joints
        
        c.fLegTrunkAngleV = 0.0; ///< angle (in rad) around vertical axis at leg-trunk fixation 0: perpendicular
        c.fLegTrunkAngleH = 0.0; ///< angle around horizontal axis at leg-trunk fixation 0: perpendicular
        c.fLegRotAngle = 0.0; ///< rotation of leg around own axis 0: first joint axis is vertical
        c.mLegTrunkAngleV = 0.0; ///< middle legs and so on
        c.mLegTrunkAngleH = 0.0;
        c.mLegRotAngle = 0.0;
        c.rLegTrunkAngleV = 0.0;
        c.rLegTrunkAngleH = 0.0;
        c.rLegRotAngle = 0.0;
        
        //the zero position of the joints depends on the placement of the two bodies at
        //the time of initialisation. specify in the following at which angle the leg
        //part should extend from the previous leg part (in rad). this sets the 0 angle!
        //don't use this, it is usually not necessary and might be removed. it was intended
        //to talk to ode's servo motors directly instead of using lpzrobots set motor functions
        
        c.fcoxaZero = 0.0; //front, negative is forward
        c.fsecondZero = 0.0; //positive is down
        c.ftebiaZero = 0.0; //M_PI/2.0;  //positive is down
            
        //be careful changing the following dimension, they may break the simulation!! (they shouldn't but they do)
        
        c.shoulderLength = 4.5 / 43.0 * c.size;
        c.shoulderRadius = .03 * c.size;
        c.coxaLength = 3.5 / 43.0 * c.size;
        c.coxaRadius = .04 * c.size;
        c.secondLength = 6.0 / 43.0 * c.size;
        c.secondRadius = .03 * c.size;
        c.tebiaRadius = 1.3 / 43.0 * c.size;
        c.tebiaLength = 11.2 / 43.0 * c.size;
        c.footRange = .2 / 43.0 * c.size; //this determines the limit of the footspring
        c.footRadius = 1.5 / 43.0 * c.size;
        
        c.backJointLimitD = M_PI / 4.0;
        c.backJointLimitU = -M_PI / 4.0;
        
        c.fcoxaJointLimitF = -M_PI / 180.0 * 50.0;
        c.fcoxaJointLimitB = 0.0;
        c.mcoxaJointLimitF = -M_PI / 180.0 * 10.0;
        c.mcoxaJointLimitB = M_PI / 180 * 30.0;
        c.rcoxaJointLimitF = M_PI / 180.0 * 10.0;
        c.rcoxaJointLimitB = M_PI / 180.0 * 57.0;
        
        c.secondJointLimitD = M_PI / 4; //-M_PI/180.0 * 30; ///< angle range for vertical direction of legs (this is lower limit)
        c.secondJointLimitU = -M_PI / 4; //-M_PI/180.0 * 75;
        c.tebiaJointLimitD = 3 * M_PI / 4; //6.5 * M_PI/8.0;
        c.tebiaJointLimitU = M_PI / 4; //5.5 * M_PI/8.0;
            
        c.footSpringPreload = 8.0 / 43.0 * c.size;
        c.footSpringLimitD = c.footSpringPreload; //negative is downwards (spring extends)
        c.footSpringLimitU = c.footSpringPreload + c.footRange;
        
        c.backPower = (1.962 / (0.035 * 2.2)) * c.coxaLength * c.trunkMass;/*use an original radius and mass and
         scale original torque by their new values
         to keep acceleration constant*/
        std::cout << "backPower " << c.backPower << "\n";
        c.coxaPower = c.backPower; //torque in Nm
        c.secondPower = c.coxaPower;
        c.tebiaPower = c.coxaPower;
        c.footPower = (/*150.0*/200.0 * 0.08 / 2.2) * c.trunkMass / c.footSpringPreload; /*this is the spring constant
         to keep  acceleration for the body
         constant, we use the above unscaled
         preload of 0.08 and original trunkMass
         to and then multiply by the new ones*/
        
        c.backDamping = 0.05;
        c.coxaDamping = 0.0; // Georg: no damping required for new servos
        c.secondDamping = 0.0;
        c.tebiaDamping = 0.0;
        c.footDamping = 0.05; // a spring has no damping??
            
        c.backMaxVel = 1.961 * M_PI;
        c.coxaMaxVel = 1.961 * M_PI; // The speed calculates how it works
        c.secondMaxVel = 1.961 * M_PI;
        c.tebiaMaxVel = 1.961 * M_PI;
        c.footMaxVel = 1.961 * M_PI;
        c.T = 1.0;
        
        c.whiskerLength = 4.0 / 43.0 * c.size; //scale
            
        c.legContacts = new bool[6];
        c.irSensors = false;
        c.irFront = false;
        c.irBack = false;
        c.irLeft = false;
        c.irRight = false;
        c.irRangeFront = 3;
        c.irRangeBack = 2;
        c.irRangeLeft = 2;
        c.irRangeRight = 2;
        //      c.elasticity = 10;
        return c;
      }
      
      static AmosIIConf getFixedConf(double _scale = 1.0, bool _useShoulder = 1, bool _useFoot = 1, bool _useBack = 0) {
        AmosIIConf c;
        
        //use shoulder (fixed joint between legs and trunk)
        c.useShoulder = _useShoulder;
        c.useTebiaJoints = true;
        c.useFoot = _useFoot; //create springs at the end of the legs
        c.useBack = _useBack; //create a joint in the back
        c.useWhiskers = 0;
        c.rubberFeet = false;
        c.useContactSensors = 0;
        c.useLocalVelSensor = 0;
        
        // the trunk length. this scales the whole robot! all parts' sizes will be adapted if this is changed!!
        c.size = 0.43;
        c.width = 8.5 / 43.0 * c.size; //16.5/43.0; //1.0/3.0; //1.0/1.5  //trunk width
        c.height = 7.0 / 43.0 * c.size; //1.0/4.0 //trunk height
        c.frontLength = 10.0 / 43.0;
        c.legLength = 1.0 / 2.2; //length of upper and lower limb combined without the tip (will be scaled by c.size)
        c.density = 2.2 / (c.size * c.width * c.height); /*we use as density the original trunk dimension
         divided by its volume*/
        c.trunkMass = c.density * c.size * c.width * c.height; //2.200/5.758;
        c.mass = 5.758 / 2.2 * c.trunkMass; //use the original trunk to total mass ratio
        c.v = new double[1];
        c.frontMass = c.trunkMass * c.frontLength / c.size;
        c.shoulderMass = (c.mass - c.trunkMass) //distribute the rest of the weight like this for now
        / (6 * (3.0 + c.useShoulder)) * (20.0 - c.useFoot) / 20.0;
        c.coxaMass = c.shoulderMass;
        c.secondMass = c.shoulderMass;
        c.tebiaMass = c.shoulderMass;
        c.footMass = (c.mass - c.trunkMass) //foot gets 3 or 4 times 1/20 of shoulderMass (divide and multiply by 3 or 4)
        / 6 * c.useFoot / 20.0;
        
        c.shoulderHeight = 4.5 / 6.5 * c.height;
        c.legdist1 = 19.0 / 43.0 * c.size; //distance between hindlegs and middle legs
        c.legdist2 = 15.0 / 43.0 * c.size; //distance between middle legs and front legs
            
        //amosII has a fixed but adjustable joint that decides how the legs extend from the trunk
        //here you can adjust these joints
        
        c.fLegTrunkAngleV = 0.0; ///< angle (in rad) around vertical axis at leg-trunk fixation 0: perpendicular
        c.fLegTrunkAngleH = 0.0; ///< angle around horizontal axis at leg-trunk fixation 0: perpendicular
        c.fLegRotAngle = 0.0; ///< rotation of leg around own axis 0: first joint axis is vertical
        c.mLegTrunkAngleV = 0.0; ///< middle legs and so on
        c.mLegTrunkAngleH = 0.0;
        c.mLegRotAngle = 0.0;
        c.rLegTrunkAngleV = 0.0;
        c.rLegTrunkAngleH = 0.0;
        c.rLegRotAngle = 0.0;
        
        //the zero position of the joints depends on the placement of the two bodies at
        //the time of initialisation. specify in the following at which angle the leg
        //part should extend from the previous leg part (in rad). this sets the 0 angle!
        //don't use this, it is usually not necessary and might be removed. it was intended
        //to talk to ode's servo motors directly instead of using lpzrobots set motor functions
        
        c.fcoxaZero = 0.0; //front, negative is forward
        c.fsecondZero = 0.0; //positive is down
        c.ftebiaZero = 0.0; //positive is down
            
        //be careful changing the following dimension, they may break the simulation!! (they shouldn't but they do)
        
        c.shoulderLength = 4.5 / 43.0 * c.size;
        c.shoulderRadius = .03 * c.size;
        c.coxaLength = 3.5 / 43.0 * c.size;
        c.coxaRadius = .04 * c.size;
        c.secondLength = 6.0 / 43.0 * c.size;
        c.secondRadius = .03 * c.size;
        c.tebiaRadius = 1.3 / 43.0 * c.size;
        c.tebiaLength = 11.2 / 43.0 * c.size;
        c.footRange = .1 / 43.0 * c.size; //this determines the limit of the footspring
        c.footRadius = 1.0 / 43.0 * c.size;
        
        c.backJointLimitD = M_PI / 30;
        c.backJointLimitU = -M_PI / 30;
        
        c.fcoxaJointLimitF = -M_PI / 180.0 * 50.0;
        c.fcoxaJointLimitB = 0.0;
        c.mcoxaJointLimitF = -M_PI / 180.0 * 10.0;
        c.mcoxaJointLimitB = M_PI / 180 * 30.0;
        c.rcoxaJointLimitF = M_PI / 180.0 * 10.0;
        c.rcoxaJointLimitB = M_PI / 180.0 * 57.0;
        
        c.secondJointLimitD = M_PI / 30.0; ///< angle range for vertical direction of legs (this is lower limit)
        c.secondJointLimitU = -M_PI / 30;
        c.tebiaJointLimitD = 16.0 * M_PI / 30.0;
        c.tebiaJointLimitU = 14.0 * M_PI / 30.0;
        
        c.footSpringPreload = 8.0 / 43.0 * c.size;
        c.footSpringLimitD = c.footSpringPreload; //negative is downwards (spring extends)
        c.footSpringLimitU = c.footSpringPreload + c.footRange;
        
        c.backPower = (1.962 / (0.35 * 2.2)) * c.coxaLength * c.trunkMass;/*use an original radius and mass and
         scale original torque by their new values
         to keep acceleration constant*/
        c.coxaPower = c.backPower; //torque in Nm
        c.secondPower = c.coxaPower;
        c.tebiaPower = c.coxaPower;
        c.footPower = (150.0 * 0.08 / 2.2) * c.trunkMass / c.footSpringPreload; /*this is the spring constant
         to keep  acceleration for the body
         constant, we use the above unscaled
         preload of 0.08 and original trunkMass
         to and then multiply by the new ones*/
        
        c.backDamping = 0.05;
        c.coxaDamping = 0.0; // Georg: no damping required for new servos
        c.secondDamping = 0.0;
        c.tebiaDamping = 0.01;
        c.footDamping = 0.05; // a spring has no damping
            
        c.backMaxVel = 1.961 * M_PI;
        c.coxaMaxVel = 1.961 * M_PI; // The speed calculates how it works
        c.secondMaxVel = 1.961 * M_PI;
        c.tebiaMaxVel = 1.961 * M_PI;
        c.footMaxVel = 1.961 * M_PI;
        c.T = 1.0;
        
        c.whiskerLength = 4.0 / 43.0 * c.size; //scale
            
        c.legContacts = new bool[6];
        c.irSensors = false;
        c.irFront = false;
        c.irBack = false;
        c.irLeft = false;
        c.irRight = false;
        c.irRangeFront = 3;
        c.irRangeBack = 2;
        c.irRangeLeft = 2;
        c.irRangeRight = 2;
        //      c.elasticity = 10;
        return c;
      }
      
      static AmosIIConf getFranksDefaultConf(double _scale = 5.0, bool _useShoulder = 1, bool _useFoot = 1,
          bool _useBack = 0) {
        AmosIIConf c;
        
        //use shoulder (fixed joint between legs and trunk)
        c.useShoulder = _useShoulder;
        c.useTebiaJoints = 0;
        c.useFoot = _useFoot; //create springs at the end of the legs
        c.useBack = _useBack; //create a joint in the back
        c.useWhiskers = 0;
        c.rubberFeet = false;
        c.useContactSensors = 0;
        c.useLocalVelSensor = 0;
        
        // the trunk length. this scales the whole robot! all parts' sizes, masses, and forces will be adapted!!
        c.size = 0.43 * _scale;
        c.width = 7.0 / 43.0 * c.size; //16.5/43.0; //1.0/3.0; //1.0/1.5  //trunk width
        c.height = 6.5 / 43.0 * c.size; //1.0/4.0 //trunk height
        c.frontLength = 12.0 / 43.0 * c.size;
        c.legLength = 1.0 / 2.2; //length of upper and lower limb combined without the tip (will be scaled by c.size)
        c.density = 2.2 / (0.43 * 0.07 * 0.065) / (pow(_scale, 3)); /*we use as density the original trunk weight
         divided by the original volume*/
        c.trunkMass = c.density * c.size * c.width * c.height; //2.200/5.758;
        std::cout << "trunkMass " << c.trunkMass << "\n";
        c.mass = 5.758 / 2.2 * c.trunkMass; //use the original trunk to total mass ratio
        std::cout << "Mass " << c.mass << "\n";
        c.v = new double[1];
        c.frontMass = c.trunkMass * c.frontLength / c.size;
        c.shoulderMass = (c.mass - c.trunkMass) //distribute the rest of the weight like this for now
        / (6 * (3.0 + c.useShoulder)) * (20.0 - c.useFoot) / 20.0;
        c.coxaMass = c.shoulderMass;
        c.secondMass = c.shoulderMass;
        c.tebiaMass = c.shoulderMass;
        std::cout << "shoulderMass " << c.shoulderMass << "\n";
        c.footMass = (c.mass - c.trunkMass) //foot gets 3 or 4 times 1/20 of shoulderMass (divide and multiply by 3 or 4)
        / 6 * c.useFoot / 20.0;
        std::cout << "footMass " << c.footMass << "\n";
        
        c.shoulderHeight = 4.5 / 6.5 * c.height;
        c.legdist1 = 19.0 / 43.0 * c.size; //distance between hindlegs and middle legs
        c.legdist2 = 15.0 / 43.0 * c.size; //distance between middle legs and front legs
            
        //amosII has a fixed but adjustable joint that decides how the legs extend from the trunk
        //here you can adjust these joints
        
        c.fLegTrunkAngleV = 0.0; ///< angle (in rad) around vertical axis at leg-trunk fixation 0: perpendicular
        c.fLegTrunkAngleH = 0.0; ///< angle around horizontal axis at leg-trunk fixation 0: perpendicular
        c.fLegRotAngle = 0.0; ///< rotation of leg around own axis 0: first joint axis is vertical
        c.mLegTrunkAngleV = 0.0; ///< middle legs and so on
        c.mLegTrunkAngleH = 0.0;
        c.mLegRotAngle = 0.0;
        c.rLegTrunkAngleV = 0.0;
        c.rLegTrunkAngleH = 0.0;
        c.rLegRotAngle = 0.0;
        
        //the zero position of the joints depends on the placement of the two bodies at
        //the time of initialisation. specify in the following at which angle the leg
        //part should extend from the previous leg part (in rad). this sets the 0 angle!
        //don't use this, it is usually not necessary and might be removed. it was intended
        //to talk to ode's servo motors directly instead of using lpzrobots set motor functions
        
        c.fcoxaZero = 0.0; //front, negative is forward
        c.fsecondZero = 0.0; //positive is down
        c.ftebiaZero = 0.0; //positive is down
            
        //be careful changing the following dimension, they may break the simulation!! (they shouldn't but they do)
        
        c.shoulderLength = 4.5 / 43.0 * c.size;
        c.shoulderRadius = .03 * c.size;
        c.coxaLength = 3.5 / 43.0 * c.size;
        c.coxaRadius = .04 * c.size;
        c.secondLength = 6.0 / 43.0 * c.size;
        c.secondRadius = .03 * c.size;
        c.tebiaRadius = 1.3 / 43.0 * c.size;
        c.tebiaLength = 11.2 / 43.0 * c.size;
        c.footRange = .2 / 43.0 * c.size; //this determines the limit of the footspring
        c.footRadius = 1.5 / 43.0 * c.size;
        
        c.backJointLimitD = M_PI / 4.0;
        c.backJointLimitU = -M_PI / 4.0;
        
        c.fcoxaJointLimitF = -M_PI / 180.0 * 50.0;
        c.fcoxaJointLimitB = 0.0;
        c.mcoxaJointLimitF = -M_PI / 180.0 * 10.0;
        c.mcoxaJointLimitB = M_PI / 180 * 30.0;
        c.rcoxaJointLimitF = M_PI / 180.0 * 10.0;
        c.rcoxaJointLimitB = M_PI / 180.0 * 57.0;
        
        c.secondJointLimitD = -M_PI / 180.0 * 30; ///< angle range for vertical direction of legs (this is lower limit)
        c.secondJointLimitU = -M_PI / 180.0 * 75;
        c.tebiaJointLimitD = 6.5 * M_PI / 8.0;
        c.tebiaJointLimitU = 5.5 * M_PI / 8.0;
        
        c.footSpringPreload = 8.0 / 43.0 * c.size;
        c.footSpringLimitD = c.footSpringPreload; //negative is downwards (spring extends)
        c.footSpringLimitU = c.footSpringPreload + c.footRange;
        
        c.backPower = (1.962 / (0.035 * 2.2)) * c.coxaLength * c.trunkMass;/*use an original radius and mass and
         scale original torque by their new values
         to keep acceleration constant*/
        std::cout << "backPower " << c.backPower << "\n";
        c.coxaPower = c.backPower; //torque in Nm
        c.secondPower = c.coxaPower;
        c.tebiaPower = c.coxaPower;
        c.footPower = (150.0 * 0.08 / 2.2) * c.trunkMass / c.footSpringPreload; /*this is the spring constant
         to keep  acceleration for the body
         constant, we use the above unscaled
         preload of 0.08 and original trunkMass
         to and then multiply by the new ones*/
        
        c.backDamping = 0.05;
        c.coxaDamping = 0.0; // Georg: no damping required for new servos
        c.secondDamping = 0.0;
        c.tebiaDamping = 0.01;
        c.footDamping = 0.05; // a spring has no damping??
            
        c.backMaxVel = 1.961 * M_PI;
        c.coxaMaxVel = 1.961 * M_PI; // The speed calculates how it works
        c.secondMaxVel = 1.961 * M_PI;
        c.tebiaMaxVel = 1.961 * M_PI;
        c.footMaxVel = 1.961 * M_PI;
        c.T = 1.0;
        
        c.whiskerLength = 4.0 / 43.0 * c.size; //scale
            
        c.legContacts = new bool[6];
        c.irSensors = false;
        c.irFront = false;
        c.irBack = false;
        c.irLeft = false;
        c.irRight = false;
        c.irRangeFront = 3;
        c.irRangeBack = 2;
        c.irRangeLeft = 2;
        c.irRangeRight = 2;
        //      c.elasticity = 10;
        return c;
      }
      
      //%%%%%%%%%%%%%%%%%%%%%%%%%Other CONFIGURATIONS NOT USE%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
      
      /**
       * updates the OSG nodes of the vehicle
       */
      virtual void update();

      /** sets the pose of the vehicle
       @param pose desired pose matrix
       */
      virtual void place(const osg::Matrix& pose);

      /** returns actual sensorvalues
       @param sensors sensors scaled to [-1,1]
       @param sensornumbAmosII::getDefaultConf()er length of the sensor array
       @return number of actually written sensors
       */
      virtual int getSensors(sensor* sensors, int sensornumber);

      /** sets actual motorcommands
       @param motors motors scaled to [-1,1]
       @param motornumber length of the motor array
       */
      virtual void setMotors(const motor* motors, int motornumber);

      /** returns number of sensors
       */
      virtual int getSensorNumber();

      /** returns number of motors
       */
      virtual int getMotorNumber();
      /** checks for internal collisions and treats them.
       *  In case of a treatment return true (collision will be ignored by other objects
       *  and the default routine)  else false (collision is passed to other objects and
       *  (if not treated) to the default routine).
       */
      virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2);

      /** this function is called in each timestep. It should perform robot-internal checks,
       like space-internal collision detection, sensor resets/update etc.
       @param globalData structure that contains global data from the simulation environment
       */
      virtual void doInternalStuff(GlobalData& globalData);

      // virtual void AmosII::updateLegTouch(int);
      
      /**
       * calculates the total energy consumption of all servos.
       */
      double round(double, int);

      virtual double energyConsumption();

      virtual double energyConsumpThroughtHeatLoss(const dReal *torques);

      virtual double outwardMechanicalPower(const dReal *torques, const dReal *angularV);

      virtual double costOfTransport(double E, double W, double V, double T);

      virtual double getMassOfRobot();

      // Configurable Interface
      virtual bool setParam(const paramkey& key, paramval val);

      /** the main object of the robot, which is used for position and speed tracking */
      virtual Primitive* getMainPrimitive() const {
        return objects[0];
      }
    protected:
      
      /** creates vehicle at desired pose
       @param pose 4x4 pose matrix
       */
      virtual void create(const osg::Matrix& pose);

      /** destroys vehicle and space
       */
      virtual void destroy();

      Shoulder shoulder[6];
      AmosIIConf conf;
      double legmass; // leg mass
      int countt;
      bool created; // true if robot was created
      RaySensorBank irSensorBank; // a collection of ir sensors
      SpeedSensor* speedsensor;

    public:
      double costOfTran;
      double* energyOneStep; ///< energy consumption for one time step
      double Power; ///< energy consumption over a period t;
      bool recordGait;
      double *heights;
      double *angles;
//    double motorsold[12];
    private:
      double hcorrection;
      bool *dones;
      bool check;
      unsigned int t;
      FILE* f;
      double timeCounter;
      double *pos1d;
      const dReal *pos1;
      const dReal *pos2;
      dMass *massOfobject;
      bool getPos1;
      double distance;
      double time;
      int avg;

      std::vector<Leg> legContact;
      Leg* legContactArray;
//    std::vector<dGeomID> footIDs;
    protected:
      // some objects explicitly needed for ignored collision pairs
      Primitive *trunk, /**irbox, *bigboxtransform, *headtrans,*/*front, *center;
      std::vector<Primitive*> legs; //all moving parts of the legs
      std::vector<Primitive*> coxas;
      std::vector<Primitive*> seconds;
      std::vector<Primitive*> tebias;
      std::vector<Primitive*> feet;
      std::vector<Primitive*> ignoredPairs;
//    std::vector<Pos> coxaPos;
//    std::vector<Pos> secondPos;
//    std::vector<Pos> tebiaPos;
//    std::vector<Pos> footPos;
      std::vector<Primitive*> objects; // all the objects
      std::vector<Joint*> joints; // joints legs
      //  std::vector <TwoAxisServo*> hipservos; // motor
      OneAxisServo* backservo;
      std::vector<OneAxisServo*> coxaservos;
      std::vector<OneAxisServo*> secondservos;
      std::vector<OneAxisServo*> tebiaservos;
      std::vector<OneAxisServo*> footsprings;
      std::vector<OneAxisServo*> whiskersprings;
      //----------- Add contact sensor ---- by Ren
      ContactSensor* ContactSens[];
      
  };

}

#endif
