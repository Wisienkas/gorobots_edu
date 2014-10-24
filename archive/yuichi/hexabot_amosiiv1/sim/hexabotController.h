/***************************************************************************
 *   Copyright (C) 2010 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *                                                                         *
 *   ANY COMMERCIAL USE FORBIDDEN!                                         *
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
 *                                            *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/
#ifndef __HEXABOTCONTROLLER_H
#define __HEXABOTCONTROLLER_H

//! Version Control
#define VERSION_HCON "HCON_Ver 1.21_20140912"
/*
Ver 1.21_20140912
  I modified the bugs in Phase resetting that happens when the leg touches twice...

Ver 1.20_20140701
  I vanish the phase reset in stance phase. (Big change)

Ver 1.11_20140623
  I fixed the bugs that i fogot to initialize the value "iCoef" (dangerous bug!!)

Ver 1.10_20140613
  I add low pass phase reset as the Phase Modulation method (important change)

Ver 1.04_20140605
  I changed the invKinematics Func to the vertual func to change it on the derived class (amosAdopter)

Ver 1.03_20140524
  I changed the step() to call HexabotController::stepNoLearning

Ver 1.02_20140516
I changed the robot and controller conf to be ref val to change it from main
Main controll the parameter

Ver 1.01_20140414
  We modify the trajectory making algorithm to move the legs at constant speed on stance phase

Ver 1.00_20140207
 This is the first version of Hexabot controller

***************************************************************************

Ver3.01_20131202
I changed the period that the phase inhibition lasts for.
It becomes T/6

Ver3.00_20130705
I changes the calculation of the CPG by using 4th RungeKutta
Because of this, the accuracy of the simulation was very daughtful.
I have changed it, and it seems work well

I add also the Flag about phase reset to reset the oscillation!!

::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
Ver2.03_20130628
I fixed bugs about iterating CPG phase.
I missed to add the "dt" time step value to calculate next step's value

Ver2.02_20130312
I add the some functions to change the phase reset ON_OFF
and to set the phase directly from the main Function

Ver2.01_20130307
I changed the phase inhibition duration
Phase inhibition is applied everywhere in the stance phase.
The converged point is keenly depend on this parameters and Control Freq

Ver2.00_20130305
THis is the version 2.00.
I start the version control system !!
*/


#include <iostream>
#include <fstream>

#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>
#include <selforg/configurable.h>
#include "ode_robots/hexabotsensormotordefinition.h"

//#include "oderobot.h"
#include <osg/Matrix>
#include <assert.h>
#include <cmath>
#include <stdlib.h>
#include <string.h>

#include <selforg/matrix.h>

#define CPGNUM 6
//#define DEBUG


using namespace HEXABOT;
//using namespace lpzrobots;

// Circle Buff Class
class CirBuff{
private:
	std::vector<double> buff;
	int currentBuffNum;
	int maxBuff;
public:
	// constructor
	CirBuff();//1000 Buf was made
	CirBuff(int _maxBuff);
	// destructor
	~CirBuff();

	// addBuff
	void addBuff(double value);
	// getCurrentBuff
	double getCurBuff(void) const;
	// getPrvBuffer
	double getPrvBuff(int prvCount) const;
	// getMaxBuff
	int getMaxBuffNum(void) const{
		return maxBuff;
	}
};

// I think it was not good at some point, but I set the all parameter from this conf class
// If you want to make a robot control, you should make new class function to change value!
typedef struct {
	double freq; // Frequency of the main CPG (Hz)
	double dutyRate; // Duty Rate that means the rate of duration when leg is stance phase
	double initialPhase; // initial phase of the oscillator (rad)
	double inhibiCoef; // Coefficience of the inhibit
} OscConf;

typedef struct {
	osg::Vec3d centerTrj; // Center point of the trajectory on leg base coordinate (m)
	osg::Vec3d moveDirection; // The move direction of the leg on stance phase on leg base coordinate (m)
							  // notice that magnitude should be 1!! and it is valid for x-y plane(of course it can be allowed to contain z axis value but this method does not be optimized in this situation)
	double walkLength; // The distance which the leg move in stance phase (m)
	double swingHeight; // The maximam height that leg reach on swing situation (m)
	double stanceHeight; // The minimum height (absolute value) that leg reaches in stance phase (m)
} TrjConf;

typedef enum{
    PC_PR_IN = 0, // Phasereset + inhibi
    PC_PR = 1, // Phasereset only on swing phase
    PC_PR_PR = 2, // PhaseReset + PhaseReset
    PC_IN_IN = 3, // Inhibi + inhibi
    PC_PR_LPASS = 4, // Phase reset by low Pass
    PC_NONE = 5 // without PC
} PeriodChanging_MODE;

typedef struct {	
	// the param for oscilation
	OscConf oscConf[CPGNUM];

	// the param for trajectory generation module
	TrjConf trjConf[CPGNUM];

	// control Frequency (Hz)
	double controlFreq;

	// twoCPG relation coefficience
 	double twoCpgCoef;
	// twoCPG relation angle]
 	double twoCPGAngleDiff;

	// The flag which determine whether will we use foot contact changing period method or not!!
    //bool useFootBias;
	// Foot Bias start time (s)
	double biasStartTime;

    // Period Changing Mode
    PeriodChanging_MODE PC_MODE;

    // one side phase modulation moed
    bool is_oneSidePhaseModulation;

    // Time factor for 1 demensional delay system
    double TFactor;

} HexabotControllerConf;

/**
 *
 * class for hexapod tripodgait using 18 DOF
 *
 */
class HexabotController : public AbstractController {

  public:
    HexabotController(std::string name, HexabotConf& _aConf, HexabotControllerConf& _conf, bool _fileFlag = false);

    virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

    virtual ~HexabotController();

    /// returns the name of the object (with version number)
    //virtual paramkey getName() const {
    //  return name;
   	//}
    /// returns the number of sensors the controller was initialised with or 0
    /// if not initialised
    virtual int getSensorNumber() const {
      return number_channels;
    }
    /// returns the mumber of motors the controller was initialised with or 0 if
    // not initialised
    virtual int getMotorNumber() const {
      return number_channels;
    }

    /// performs one step (includes learning).
    /// Calulates motor commands from sensor inputs.
    virtual void step(const sensor*, int number_sensors, motor*,
        int number_motors);

    /// performs one step without learning. Calculates motor commands from sensor
    /// inputs.
    virtual void stepNoLearning(const sensor*, int number_sensors,
        motor*, int number_motors);

    /***** STOREABLE ****/
    /** stores the controller values to a given file. */
    virtual bool store(FILE* f) const;
    /** loads the controller values from a given file. */
    virtual bool restore(FILE* f);

    // default config
    static HexabotControllerConf getDefaultConf();

    // straight connected config
    //static HexabotControllerConf getStraightCaseConf();

    // For Phase Diff investigation
    bool setObserverMode();

    // tell the address of this controller
    bool addObject(HexabotController* obj);


  protected:
    unsigned short number_channels;

    // ############ The parameter for obs mode ###################
    // through this parameter, we observe the movement
    bool obsMode;
    std::vector<HexabotController*> mObject;

    //##############   Parameter time  ###########################
    // time step (not true time)
    int t;

	// ################# values for Simple one dimensional oscilation CPG ########################
  public:
    // value set
    typedef struct{
    	// Phase for CPG
    	double cpgPhase;
        // Phase of CPG when the leg touches ground
        double touchDownPhase;
    	// foot Bias
    	double footBias;// = 0.;
        // Filted Phase (pass through the post processing with delayed system)
        double filtPhase;
    	// Output Phase (pass through the post processing)
    	double outPhase;// = 0.;
    	// values for contact
        bool prvTouchF;
        // count
        long count;
        // inhibition parameter (it has changed depends on inhibit timing)
        double iCoef;
        // time to contact prv
        double prvTouchTime;
    }oscSet;

    // control set
    typedef struct{
    	// leg toe position for controll in leg coordinate. "l_" represents it is in Leg coordinate. Unit:m
    	osg::Vec3d l_toePos;
    	// command angle for servo (rad)
    	double tAngle; // 1st joint tibia
    	double cAngle; // 2nd joint c
    	double fAngle; // 3rd joint f

    	// inverse kinematics success ??
    	bool isSuccessInvKinema;
    }ctrSet;

  protected:
    // oscillation set
    oscSet osc[CPGNUM];

    // controll set for leg trajectry
    ctrSet ctr[CPGNUM];

    // Phase Reset Flag
    bool phaseResetFlag[CPGNUM];

    // we apply phase modulation only one side
    //  It is for analitic solution
    //bool is_oneSidePhaseModulation;


    // ############### sensor Data set ########################
  public:
    // joint data set
    typedef struct{
    	// sensed angle
    	double tAngle;
    	double cAngle;
    	double fAngle;

    	// sensed torque
    	double tTorque;
		double cTorque;
		double fTorque;
    }sensedJointData;

    // sensor data set
    typedef struct{
    	// each joint data
    	sensedJointData jData[CPGNUM];
    	// foot contact Data
    	double forceData[CPGNUM];

    	// pose data roll, pitch, yaw (rad)
    	osg::Vec3d pose;
    	// angular Vel data w
    	osg::Vec3d angVel;
    	// grobal position of the center of the robot(center of the robot exactly)
    	//  we should notice that the ground has 0.1 m thickness, so if you want to get height of the robot from ground, you should substract thickness.
    	osg::Vec3d g_roboPos;
    	// grobal speed of the center of the robot
    	osg::Vec3d g_roboSpd;
    	// grobal pos of COG
    	osg::Vec3d g_cogPos;
    	// grobal pos of leg toe
    	osg::Vec3d g_legToePos[CPGNUM];

    }sensedDataSet;

  protected:
    sensedDataSet sDataSet;

    // ############### robot config ###########################
    // hexabot controller configuration
    HexabotControllerConf &conf;

    // hexabot configuration
    HexabotConf &robotConf;

    // ############### file output ############################
    bool fileFlag;// if true log!!

    std::ofstream ofs;
    std::ofstream ofs2;
    std::ofstream ofs3;


  protected:
    // inverse kinematics-----------
    // get the angle data from leg toe position in Leg coordinate
    // toePos : in leg coordinate (m) , angle data : (rad)
    // the definition is written in another paper
    virtual bool clacInvKinematics(osg::Vec3d toePos, double& tAng, double& cAng, double& fAng);


  public:
   // This is the eFunction to analyze this oscillation from external viewer
     // sendPhaseData
     double eGetPhase(int ch);

     // get oscillation data
     oscSet eGetOscSet(int ch);

     // get control data
     ctrSet eGetCtrSet(int ch);

     // get sensed data
     sensedDataSet eGetSensedData(void);

     // get configuration of Hexabot controller
     HexabotControllerConf eGetHexabotCtrConf(void);

     // check the phase that is in the duration (0 <= phase < 2 PI) and if it does not fulfill we change the phase to fulfill
     static void checkPhase(double& phase);

     // set fileFlag
     //bool setFileFlag(bool flag){fileFlag = flag;};

     // check the leg just touches ground or not
     bool eIsLegContact(int ch);

     // get the phase that the leg i touches down
     double eGetTouchDownPhase(int ch);

  public:
   // This is the function to set some parameters for simullation.
     // activate and deactivate PhaseReset of all CPGs
     bool eActivatePhaseReset_all(bool flag);

     // activate and deactivate PhaseReset of selected CPGs
     bool eActivatePhaseReset_each(bool flag, int ch);

     // set Phase
     bool eSetPhase(double phase, int ch);

     // set the one side phase modulation mode
     //bool eActivateOneSidePhaseModulation(bool flag);


    /*
    virtual paramval getParam(const paramkey& key) const {
      if (key == "WeightH1_H1")
        return conf.WeightH1_H1;
      else if (key == "WeightH2_H2")
        return conf.WeightH2_H2;
      else if (key == "WeightH1_H2")
        return conf.WeightH1_H2;
      else if (key == "WeightH2_H1")
        return conf.WeightH2_H1;
      else if (key == "fact")
        return conf.fact;
      else if (key == "direction")
        return conf.direction;
      else if (key == "bias")
        return conf.bias;
      else
        return AbstractController::getParam(key);
    }

    virtual bool setParam(const paramkey& key, paramval val) {
      if (key == "WeightH1_H1")
        conf.WeightH1_H1 = val;
      else if (key == "WeightH2_H2")
        conf.WeightH2_H2 = val;
      else if (key == "WeightH1_H2")
        conf.WeightH1_H2 = val;
      else if (key == "WeightH2_H1")
        conf.WeightH2_H1 = val;
      else if (key == "fact")
        conf.fact = val;
      else if (key == "direction")
        conf.direction = val;
      else if (key == "bias")
        conf.bias = val;
      else
        return false;
      return true;
    }

    virtual paramlist getParamList() const {
      paramlist list;
      list.push_back(
          std::pair<paramkey, paramval>("WeightH1_H1", conf.WeightH1_H1));
      list.push_back(
          std::pair<paramkey, paramval>("WeightH2_H2", conf.WeightH2_H2));
      list.push_back(
          std::pair<paramkey, paramval>("WeightH1_H2", conf.WeightH1_H2));
      list.push_back(
          std::pair<paramkey, paramval>("WeightH2_H1", conf.WeightH2_H1));
      list.push_back(
          std::pair<paramkey, paramval>("fact", conf.fact));
      list.push_back(
          std::pair<paramkey, paramval>("direction", conf.direction));
      list.push_back(
          std::pair<paramkey, paramval>("bias", conf.bias));
      return list;
    }
    */
};

#endif
