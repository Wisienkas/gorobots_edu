/***************************************************************************
 *   Copyright (C) 2008 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
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
 *                                                                         *
 *   Meta controller allowing to choose between "manual", random and       *
 *   homeokinetic                                                          *
 *   control for each of the 19 degrees of freedom.                        *
 *                                                                         *
 *   $Log: amosIImetacontroller.h,v $                                *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/
#ifndef __QLEARNINGHOMEOKMCONTROLLER_H
#define __QLEARNINGHOMEOKMCONTROLLER_H

#include <selforg/abstractcontroller.h>

#include <assert.h>
#include <cmath>

#include <selforg/matrix.h>
#include <selforg/noisegenerator.h>

#include <vector>

#include "openinvertnchannelcontroller.h"
//#include "../amosII2legsII/openinvertnchannelcontroller.h"
#include "qlearning.h"

struct QLearningHomeokMControllerConf {
	int number_qlearner;
} ;


/**
 * class for robot controller 
 * using several controllers, see comments for the configuration above
 */
class QLearningHomeokMController : public AbstractController {

public:
	QLearningHomeokMController(const QLearningHomeokMControllerConf& conf = getDefaultConf());
	virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

	virtual ~QLearningHomeokMController();

	/// returns the number of sensors the controller was initialised with or 0 if not initialised
	virtual int getSensorNumber() const { return number_sensors; }
	/// returns the mumber of motors the controller was initialised with or 0 if not initialised
	virtual int getMotorNumber() const  { return number_motors; }

	/// performs one step (includes learning).
	/// Calulates motor commands from sensor inputs.
	virtual void step(const sensor* , int number_sensors, motor* , int number_motors);

	/// performs one step without learning. Calulates motor commands from sensor inputs.
	//
	//  ?? calls step for calculation of motor commands?
	//
	virtual void stepNoLearning(const sensor* , int number_sensors,
			motor* , int number_motors);



	/************** CONFIGURABLE ******************************** /
  virtual paramval getParam(const paramkey& key) const;
  virtual bool setParam(const paramkey& key, paramval val);
  virtual paramlist getParamList() const;
	 */

	/**** STOREABLE ****/
	/** stores the controller values to a given file. */
	virtual bool store(FILE* f) const { return true; };
	/** loads the controller values from a given file. */
	virtual bool restore(FILE* f){ return true;};




	static QLearningHomeokMControllerConf getDefaultConf(){
		QLearningHomeokMControllerConf c;
		c.number_qlearner =1;
		return c;
	}

	void setProgrammedSteeringControl(bool b){
		preprogrammed_steering_control_active = b;
	}

	void setLearnedSteeringControl(bool b){
		learned_steering_control_active = b;
	}

	void printQTable(){
		for (int i=0; i< conf.number_qlearner; i++){
			std::cout<<"Q-Learner "<<i;
			if (active_qlearner==i) {std::cout<<"(active)";}
			std::cout<<std::endl;
			qlearner.at(i)->printQTable();
			std::cout<<std::endl<<std::endl;
		}
	}

	void setExplorationActive(bool b){
		for (int i=0; i< conf.number_qlearner; i++){
			qlearner.at(i)->setExplorationActive(b);
		}
	}

	void setMC(double left, double right){
		mc[0]=left;
		mc[1]=right;
		mc[2]=left;
		mc[3]=right;
	}

	std::vector<OpenInvertNChannelController*> homeokinetic_controller;
	bool learn;
protected:

	virtual unsigned int calc_current_state(double alpha, double distance, std::vector<double> x_hom);
	virtual double calc_current_reward(double alpha, double old_alpha,
			double distance, double old_distance, std::vector<double> x_hom);

	int goalcounter, goalcounter2;

	unsigned short number_sensors;
	unsigned short number_motors;
	unsigned short number_motors_homeokinetic;
	unsigned short number_sensors_homeokinetic;

	QLearningHomeokMControllerConf conf;
	int t;
	bool preprogrammed_steering_control_active;
	bool learned_steering_control_active;

	bool one_controller;
bool manual_hom_control;

	std::vector<unsigned int> old_state;
	unsigned int old_action;
	double old_reward;

	int active_qlearner;

	std::vector<double> old_distance, old_alpha;

	std::vector<QLearning*> qlearner;
	double mc[4];

	double summed_reward;
	int number_reward_summations;
};

#endif
