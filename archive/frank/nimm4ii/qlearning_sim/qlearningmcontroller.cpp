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

#include "qlearningmcontroller.h"
#include <selforg/controller_misc.h>


using namespace matrix;
using namespace std;



QLearningMController::QLearningMController( const QLearningMControllerConf& _conf)
: AbstractController("QLearningMController", "$Id: "), conf(_conf)
{
	//// Martin: (check if i have the same prob)
	//	//as the following pointers were not zero when i ran the simulation with the -nographics option
	//	//(else not) i set them zero here. i am clueless as to why.
	//	random_controller = 0;
	//	gait_controller = 0;
	////	homeokinetic_controllers = 0;

	preprogrammed_steering_control_active =false;
	t=0;

};


QLearningMController::~QLearningMController()
{
}


void QLearningMController::init(int sensornumber, int motornumber, RandGen* randGen)
{

	number_sensors = sensornumber;
	number_motors = motornumber;


	old_state=0;
	old_action=0;

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
	qlearner = new QLearning(number_states, number_actions);

}

void QLearningMController::step(const sensor* x_, int number_sensors, motor* y_, int number_motors){

	// calculate angle (deviationof goal from front)
	double alpha =  sign(x_[4]) * atan (x_[5]/x_[4]) * 180 / M_PI; // angle in degrees

	//learned_steering_control_active=true;
	if (learned_steering_control_active) {


		// states based on angle
		// 3 states assumed: goal to the left, goal to the right, goal straight on (= no goal)
		unsigned int current_state=1; // nothing seen  //directly in front
		if ( (alpha<-10) ) {//left
			current_state=0;
			//if (x_[4]<0) current_state =3/*2*/; //if behind robot
		}
		if ( (alpha> 10) ) {//right
			current_state=2;
			//if (x_[4]<0) current_state =3/*0*/; //if behind robot
		}
//		if ( (alpha<10) && (alpha> -10) ) {//directly in front
//			current_state=1;
//		}



//		// calculate current state
//		// 3 states assumed: goal to the left, goal to the right, goal straight on (= no goal)
//		unsigned int current_state=1; // nothing seen or seen directly in front
//		if (x_[5]<-0.1) current_state=0; //left
//		if (x_[5]> 0.1) current_state=2; //right



		//if (current_state!=old_state) { //only when state changed
		t++;
		if ((t%20)==0) { //every 100 timesteps
			//std::cout<<t<<std::endl;

			//calculate reward
			double reward;
/*
			// reward based on sensors
			bool sensor_based_reward =true;
			if (sensor_based_reward){
				//double reward = 1/x_[4] + 1/x_[5];
				reward = abs(1/x_[5]); // only steering
				if (old_action==1) reward+=5;
			}
*/
			// reward based on state
			bool state_based_reward =true;
			if (state_based_reward){
				reward=0;
				if ((old_action==1) && (old_state == 1))	reward = 1; // goal is in front, or no goal and robot does not steer
			}



			// update Q-table
			double learned_reward = qlearner->learn (old_state, old_action, current_state, reward);

			// select action
			int current_action = qlearner->selectAction(current_state);


			if (current_action == 1){
				y_[0]=0.6;
				y_[1]=0.6;
				y_[2]=0.6;
				y_[3]=0.6;
			}
			if (current_action == 0){
				y_[0]=-0.6;
				y_[1]=0.6;
				y_[2]=-0.6;
				y_[3]=0.6;
				//				std::cout<<"links:   "<<std::endl;
			}
			if (current_action == 2){
				y_[0]=0.6;
				y_[1]=-0.6;
				y_[2]=0.6;
				y_[3]=-0.6;
			}

			std::cout<<"s0 = "<<old_state<<"   ";
			std::cout<<"a0 = "<<old_action<<"   ";
			std::cout<<"s1 = "<<current_state<<"   ";
			std::cout<<"a1 = "<<current_action<<"   ";
			std::cout<<"r = "<<reward<<"   ";
			std::cout<<"lr = "<<learned_reward<<std::endl;



			old_state = current_state;
			old_action = current_action;

		}

	}

}

void QLearningMController::stepNoLearning(const sensor* x_, int number_sensors,
		motor* y_, int number_motors)
{

}




