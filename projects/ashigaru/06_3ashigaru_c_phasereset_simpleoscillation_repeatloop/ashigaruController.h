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
#ifndef __ASHIGARUCONTROLLER_H
#define __ASHIGARUCONTROLLER_H

#include <iostream>
#include <fstream>

#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>
#include <selforg/configurable.h>
#include "ode_robots/ashigarusensormotordefinition.h"

//#include "oderobot.h"
#include <osg/Matrix>
#include <assert.h>
#include <cmath>
#include <stdlib.h>
#include <string.h>

#include <selforg/matrix.h>

#define CPGNUM 3
//#define DEBUG


using namespace ASHIGARU;
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

typedef struct {
	// The information of connection -- at this time I did not use it, so it is ok that you don't consider
	// I think I have to think more about this representation way but temporally I made like this
	//  Each arrangement represent the number ID of Leg or Body of Ashigaru as follows
	//                            Body0
	//                    Leg2  /      \  Leg1
	//                         <Ashigaru>
	//                    Body1 \      /  Body2
	//                            Leg0                     |  x
	//                                                    \/
	// Is Leg i used for connection?? and what body position does it connect to
	int isLegUsedCnct[CPGNUM];
		// -1 : no connection
		// 0  : connect to Body 0
		// 1  : connect to Body 1
		// 2  : connect to Body 2

	// Is Body i is connected by a leg of another Ashigaru??
	int isBodyCncted[CPGNUM];
		// -1 : no connection
		// 0  : connect to Leg 0
		// 1  : connect to Leg 1
		// 2  : connect to Leg 2

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
	bool useFootBias;
	// Foot Bias start time (s)
	double biasStartTime;

	// to disable the leg
    //bool disLeg;

} AshigaruControllerConf;

/**
 *
 * class for hexapod tripodgait using 18 DOF
 *
 */
class AshigaruController : public AbstractController {

  public:
    AshigaruController(std::string name, const AshigaruConf& _aConf, const AshigaruControllerConf& _conf = getDefaultConf());

    virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

    virtual ~AshigaruController();

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
    static AshigaruControllerConf getDefaultConf();

    // straight connected config
    static AshigaruControllerConf getStraightCaseConf();

    // For Phase Diff investigation
    bool setObserverMode();

    // tell the address of this controller
    bool addObject(AshigaruController* obj);


  protected:
    unsigned short number_channels;

    // ############ The parameter for obs mode ###################
    // through this parameter, we observe the movement
    bool obsMode;
    std::vector<AshigaruController*> mObject;

    //##############   Parameter time  ###########################
    // time step (not true time)
    int t;

	// ################# values for Simple one dimensional oscilation CPG ########################
  public:
    // value set
    typedef struct{
    	// Phase for CPG
    	double cpgPhase;
    	// foot Bias
    	double footBias;// = 0.;
    	// Output Phase (pass through the post processing)
    	double outPhase;// = 0.;
    	// values for contact
        bool prvTouchF;
        // count
        int count;
        // inhibition parameter (it has changed depends on inhibit timing)
        double iCoef;
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
    	// grobal position of the center of the robot
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
    // ashigaru controller configuration
    AshigaruControllerConf conf;

    // ashigaru configuration
    AshigaruConf robotConf;

    // ############### file output ############################
    std::ofstream ofs;
    std::ofstream ofs2;
    std::ofstream ofs3;


  protected:
    // inverse kinematics-----------
    // get the angle data from leg toe position in Leg coordinate
    // toePos : in leg coordinate (m) , angle data : (rad)
    // the definition is written in another paper
    bool clacInvKinematics(osg::Vec3d toePos, double& tAng, double& cAng, double& fAng);


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

     // get configuration of Ashigaru controller
     AshigaruControllerConf eGetAshigaruCtrConf(void);

     // check the phase that is in the duration (0 <= phase < 2 PI) and if it does not fulfill we change the phase to fulfill
     static void checkPhase(double& phase);


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
