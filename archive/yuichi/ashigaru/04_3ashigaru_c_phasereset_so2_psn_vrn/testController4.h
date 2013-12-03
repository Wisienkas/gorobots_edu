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
 *   $Log: tripodgait18dof.h,v $
 *                                                                         *
 ***************************************************************************/
#ifndef __TESTCONTROLLER4_H
#define __TESTCONTROLLER4_H

//debug mode
//#define DEBUG_MODE;

#include <iostream>
#include <fstream>

#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>
#include <selforg/configurable.h>
#include "ode_robots/ashigarusensormotordefinition.h"

#include <assert.h>
#include <cmath>
#include <stdlib.h>
#include <string.h>

#include <selforg/matrix.h>

using namespace ASHIGARU;

typedef struct TestController4Conf {
    double WeightH1_H1;
    double WeightH2_H2;
    double WeightH1_H2;
    double WeightH2_H1;
    double fact;
    double direction;
    double bias;
    bool disLeg;
} TestController4Conf;

// Circle Buff
class CirBuff{
private:
	std::vector<double> buff;
	int currentBuffNum;
	int maxBuff;
public:
	// constructor
	CirBuff();
	CirBuff(int _maxBuff);
	// destructor
	~CirBuff();

	// addBuff
	void addBuff(double value);
	// getCurrentBuff
	double getCurBuff(void);
	// getPrvBuffer
	double getPrvBuff(int prvCount);
};


/**
 *
 * class for hexapod tripodgait using 18 DOF
 *
 */
class TestController4 : public AbstractController {

  public:
    TestController4(std::string name, double ini, const TestController4Conf& conf = getDefaultConf());

    virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

    virtual ~TestController4();

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

    /// performs one step without learning. Calulates motor commands from sensor
    /// inputs.
    virtual void stepNoLearning(const sensor*, int number_sensors,
        motor*, int number_motors);

    /***** STOREABLE ****/
    /** stores the controller values to a given file. */
    virtual bool store(FILE* f) const;
    /** loads the controller values from a given file. */
    virtual bool restore(FILE* f);

    // default config
    static TestController4Conf getDefaultConf();

  protected:
    unsigned short number_channels;

    //##############   Parameter time  ###########################
    int t;
#define CPGNUM 3


    // ################# value for Debug ###########################
#ifdef DEBUG_MODE
    double debug[CPGNUM];
#endif


	// ################# values for improved SO(2) CPG ########################

 	 // Control value which determine the freq
	double ContM[CPGNUM];
	 // synapsis strength
	double w00[CPGNUM];
	double w11[CPGNUM];
	double w01[CPGNUM];
	double w10[CPGNUM];

	 // normal Bias
	double normalBias[CPGNUM];// = {0.01, ;

	 // foot Bias
	double footBiasH1[CPGNUM];// = 0.;
	double footBiasH2[CPGNUM];// = 0.;

	//Activity
	double activityH1[CPGNUM];
	double activityH2[CPGNUM];

	// Output
	double outputH1[CPGNUM];// = 0.;
	double outputH2[CPGNUM];// = 0.;

	// ################## values for contact #################################
    bool prvTouchF[CPGNUM];
    int count[CPGNUM];

    // Mechanism to compensate the delay of the system
    std::vector <CirBuff> fBuff;
    //CirBuff fBuff[CPGNUM];
    int delayTime[CPGNUM];
    bool countFlag[CPGNUM];
    int timeCount[CPGNUM];

	// ################# values for POST PROCESSING ###########################
	double pcpg_step[CPGNUM][2];
	double setold[CPGNUM][2];
	double set[CPGNUM][2];
	double countupold[CPGNUM][2];
	double countup[CPGNUM][2];
	double countdownold[CPGNUM][2];
	double countdown[CPGNUM][2];
	double diffset[CPGNUM][2];
	double deltaxup[CPGNUM][2];
	double deltaxdown[CPGNUM][2];
	double xup[CPGNUM][2];
	double xdown[CPGNUM][2];
	double yup[CPGNUM][2];
	double ydown[CPGNUM][2];
	double pcpg_output[CPGNUM][2];

	// ############### values for PSN ########################################
	//---Input neurons
    std::vector<double> input;

    std::vector< std::vector<double> > psn_activity; 	//PSN neural activities
    std::vector< std::vector<double> > psn_output;		//PSN neural outputs
    std::vector< std::vector<double> > psn_w;			//PSN neural weights
    std::vector<double> psn_bias;						//PSN bias
    std::vector< std::vector<double> > psn_input2_w;
    std::vector< std::vector<double> > psn_pcpg_w;

    // ############### values for VRN ########################################
    std::vector< std::vector<double> > vrn_activity; 	//VRN neural activities
    std::vector< std::vector<double> > vrn_output;		//VRN neural outputs
    std::vector< std::vector<double> > vrn_w; 			//VRN neural weights
    double vrn_bias;
    std::vector< std::vector<double> > vrn_psn_w; 		//VRN neural weights
    double vrn_input3_w;
    double vrn_input4_w;

    // ############### value of output #######################################
    std::vector<double> t_activity;
    std::vector<double> t_output;

    std::vector<double> c_activity;
    std::vector<double> c_output;
    std::vector<double> c_outputold;
    std::vector<double> c_output_post;

    std::vector<double> f_activity;
    std::vector<double> f_output;



    TestController4Conf conf;

    std::ofstream ofs;
    std::ofstream ofs2;


  public:
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
};

#endif