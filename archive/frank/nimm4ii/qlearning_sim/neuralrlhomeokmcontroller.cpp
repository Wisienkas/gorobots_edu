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
 *   Meta controller allowing to choose between "manual" and homeokinetic  *
 *   control for each of the 19 degrees of freedom.                        *
 *                                                                         *
 *   $Log: hexapod12dofmetacontroller.cpp,v $                              *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/

#include "neuralrlhomeokmcontroller.h"
#include <selforg/controller_misc.h>


using namespace matrix;
using namespace std;



NeuralRLHomeokMController::NeuralRLHomeokMController( const NeuralRLHomeoMControllerConf& _conf)
: AbstractController("QLearningHomeokMController", "$Id: "), conf(_conf)
{
	//// Martin: (check if i have the same prob)
	//	//as the following pointers were not zero when i ran the simulation with the -nographics option
	//	//(else not) i set them zero here. i am clueless as to why.
	//	random_controller = 0;
	//	gait_controller = 0;
	////	homeokinetic_controllers = 0;

	preprogrammed_steering_control_active =false;
	t=0;

	summed_reward=0;
	number_reward_summations=0;
	learn=true;

	//for manual steering
	mc[0]=0;
	mc[1]=0;
	mc[2]=0;
	mc[3]=0;


};


NeuralRLHomeokMController::~NeuralRLHomeokMController()
{
	if(homeokinetic_controller){
		delete homeokinetic_controller;
	}
}


void NeuralRLHomeokMController::init(int sensornumber, int motornumber, RandGen* randGen)
{
	if(!randGen) {
		this->randGen=new RandGen();
	} else {
		this->randGen=randGen;
	}

	number_sensors = sensornumber;
	number_motors = motornumber;

	number_sensors_homeokinetic = 4;  // for nimm4 robot
	number_motors_homeokinetic = 4;

	//create and initialize controllers with the appropriate numbers of dofs

	homeokinetic_controller = new OpenInvertNChannelController(/*buffersize*/100);  //add new controller
	homeokinetic_controller -> init(number_sensors_homeokinetic, number_motors_homeokinetic, randGen);
	homeokinetic_controller -> setBiasUpdateRule(no); //bias is always zero

	old_reward=0;

	old_distance.resize(conf.number_qlearner,0);
	old_alpha.resize(conf.number_qlearner,0);
	old_state.resize(conf.number_qlearner,0);
	old_action=0;

	lweight.resize(conf.number_qlearner,0);
	rweight.resize(conf.number_qlearner,0);


	unsigned int number_states  = 3; // left, in front/nothing, right
	// Test with 3 actions first: left, nothing, right
	unsigned int number_actions = 3;
	//	unsigned int number_actions = 16;
	//	// no brake,
	//	// brake 1, brake 2, brake 3, brake 4,
	//	// brake 1 and 2, brake 1 and 3, brake 1 and 4,
	//	// brake 2 and 3, brake 2 and 4, brake 3 and 4,
	//	// brake 1,2,3, brake 1,2,4, brake 1,3,4,  brake 2,3,4
	//	// brake 1,2,3,4

	active_qlearner = 0;

	for (int i=0; i<conf.number_qlearner; i++){
		qlearner.push_back( new QLearning(number_states, number_actions));
	}
}

void NeuralRLHomeokMController::step(const sensor* x_, int number_sensors, motor* y_, int number_motors){

	sensor x_homeok[number_sensors_homeokinetic];
	std::vector<double> x_hom; // same as above, just as vector for qlearnig stuff below
	// do not consider e.g. IR sensors and so on
	// sensors belonging to motors are in the beginning of x_
	for (unsigned int j =0; j < number_sensors_homeokinetic; j++){
		x_homeok[j]=x_[j];
		x_hom.push_back(x_[j]);
	}

	std::vector<double> alpha;
	std::vector<double> distance;

	// calculate angle and distance for each of the goals (for each of the controllers)
	for (int i=0; i<conf.number_qlearner; i++){
		//x_[4] => forward/backward
		//x_[5] => left/right
		alpha.push_back(sign(x_[3+i*3+1]) * atan (x_[3+i*3+2]/x_[3+i*3+1]) * 180 / M_PI); // angle in degrees
		distance.push_back( sqrt(pow(x_[3+i*3+1],2)+pow(x_[3+i*3+2],2)) );
		//		// calculate angle (deviationof goal from front)
		//		double alpha =  sign(x_[4]) * atan (x_[5]/x_[4]) * 180 / M_PI; // angle in degrees
		//	    double distance = sqrt(pow(x_[4],2)+pow(x_[5],2));
	}


	//learned_steering_control_active=true;
	if (learned_steering_control_active) {


		// change to next goal, if near the current goal
		if (distance.at(active_qlearner)<1){
			active_qlearner = ((active_qlearner+1)%conf.number_qlearner);
			std::cout<<"ACTIVE LEARNER CHANGED TO "<<active_qlearner<<std::endl;
		}
		std::vector<unsigned int> current_state;
		std::vector<double> reward;

		// calculate for all QLearner, so all can learn, even though they did not determine the action!
		for (int i=0; i< conf.number_qlearner; i++){
			current_state.push_back(calc_current_state(alpha.at(i), distance.at(i),x_hom));
		}

		for (int i=0; i< conf.number_qlearner; i++){
			reward.push_back(calc_current_reward(
					alpha.at(i), old_alpha.at(i), distance.at(i), old_distance.at(i), x_hom));
		}

	/*	double LS=0, RS=0; //= state 1
		if (state==0) {
			LS=1;
		}
		if (state==2) {
			RS=1;
		}

		//parameter adaptation
		for (int i=0; i< conf.number_qlearner; i++){
			double eps=0.1;
			lweight.at(i) + =eps * LS * old.action * reward.at(i);
			rweight.at(i) + =eps * RS * old.action * reward.at(i);
		}

		// action generation
		double l_factor=0, r_factor=0;
		double lrtmp = randGen->rand();
		double noise = randGen->rand();
		//if
		double laction= 1- 0.5 * (LS * lweight.at(active_qlearner) + noise);
		double raction= 1- 0.5 * (LS * lweight.at(active_qlearner) + noise);

*/


	}
	// assuming here that homeokinesis controls all sensors
	// QLearning just modulates amplitudes/setpoints of motor commands in openinvertnchannelcontroller

	homeokinetic_controller->step(x_homeok , number_sensors_homeokinetic,y_ , number_motors_homeokinetic);

	// manual steering
	/*
	for (int i=0;i<4;i++)
		y_[i]=mc[i];
	std::cout<<y_[0]<<"  "<<y_[1]<<"  "<<y_[2]<<"  "<<y_[3]<<"  "<<std::endl;
	 */
}

void NeuralRLHomeokMController::stepNoLearning(const sensor* x_, int number_sensors,
		motor* y_, int number_motors)
{

}


unsigned int NeuralRLHomeokMController::calc_current_state(
		double alpha, double distance, std::vector<double> x_hom){


	unsigned int current_state=1; // nothing seen or in front
	if ( /*(alpha>-45) &&*/ (alpha<-3) ) {//left
		current_state=0;
		//if (x_[4]<0) current_state =3/*2*/; //if behind robot
	}
	if ( /*(alpha<45) &&*/ (alpha> 3) ) {//right
		current_state=2;
		//if (x_[4]<0) current_state =3/*0*/; //if behind robot
	}

	return current_state;
}


double NeuralRLHomeokMController::calc_current_reward(
		double alpha, double old_alpha, double distance, double old_distance, std::vector<double> x_hom){

	double reward = 0;

	if (abs(alpha) > 3){
		reward -= 0.1;
	}
	return reward;
}
