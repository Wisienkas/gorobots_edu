/*
 * NimmExp1RobotState.cpp
 *
 *  Created on: Jul 24, 2013
 *      Author: bassel
 */


//NimmExp1RobotState
/*************************************************************************
 *  NimmExp1RobotState includes functions to support RL in the multiple goal scenario.
 *  It controls subgoals related system's functionalities.
 * (e.g. reward signals when the robot reaches a subgoal)
 * It controls mainly:
 * 			1) the reward positive signals when the robot reaches a right subgoal (called by reward function in acicoRCcontroller)
 * 			2) the exploration signals (when turned off and when turned on)
 * 			3) The part of the reward function which gives positive rewards when the robot reaches a right subgoal
 */

#include "NimmExp1RobotState.h"



#ifndef clip
#define	clip(x,l ,u )( ((x)<(l)) ? (l) : ((x)>(u)) ? (u) : (x))
#endif

#ifndef min
#define min(x, y) ((x <= y) ? x : y)
#endif

#ifndef max
#define max(x, y) ((x >= y) ? x : y)
#endif



int counter = 0;
#define FINAL_GOAL 3
#define absorbing_time 100


#define WRITE_AVERAGE_NUMBER 0

NimmExp1RobotState::NimmExp1RobotState() {
	current_goal = 0;
	reward_absorbing_counter = 0;
	epoch_number = 0;
	final_goal_checked = false;

	for (int i=0; i<NUMBER_OF_LANDMARKS; i++) {
		exploration_array[i] = true;
		vmax.push_back(0.001);
		vmin.push_back(-0.001);
	}

	////////////////////exploration init
	exploration_old = gauss() / 2.0;
}



double NimmExp1RobotState::get_state_array() {
	//double* taken_rewards = new double [NUMBER_OF_LANDMARKS];
	return current_goal;

	/*for (int i=0; i<NUMBER_OF_LANDMARKS; i++)
		if (i < current_goal)
			taken_rewards[i] = 1;
		else
			taken_rewards[i] = 0;

	return taken_rewards;*/
}

bool NimmExp1RobotState::Final_goal_is_reached()
{
	return final_goal_checked;
}
//detect a new learning phase (for learning phase definition see thesis)
void NimmExp1RobotState::reach_new_state() {
	if (current_goal < (FINAL_GOAL - 1)) {
			current_goal++;
		reward_absorbing_counter = 0;
		exploration_old = gauss() / 2.0;
	} else {
		final_goal_checked = true;
	}
}

//positive reward signals when the robot hits a right subgoal
double NimmExp1RobotState::get_reward(double* distances) {

	if (reward_absorbing_counter > absorbing_time)
			reach_new_state();

	if ((distances[current_goal] < 0.02) && (reward_absorbing_counter <= absorbing_time)) {
			reward_absorbing_counter++;
			return 1;
		}
	return 0;
}


bool printed = false;

bool NimmExp1RobotState::reset_robot_state() {
	epoch_number++;
	/////////////////////////////////
	/////print section

	std::cout<<"epoch_number"<<epoch_number<<" - "<<std::endl;
	for (int i=0; i<NUMBER_OF_LANDMARKS; i++)
		std::cout<<exploration_array[i]<<"-"<<std::endl;


	/////////////////////////////////
	if (((epoch_number-1) % TEST_TRY )== 0)
		set_test_trial();
	current_goal = 0;
	reward_absorbing_counter = 0;

	//////////exploration section
	exploration_old = gauss() / 2.0;
	counter = 0;

	final_goal_checked = false;



	if (epoch_number > 1500)
		return true;
	//for process termination
	for (int i=0; i<NUMBER_OF_LANDMARKS; i++)
		if (exploration_array[i])
			return false;
	/////print results

	if ((!printed) && (WRITE_AVERAGE_NUMBER)) {
		std::ofstream myfile;
		myfile.open ("Results_exp1.txt", std::ios::app);
		myfile<< "Epochs Number = "<<epoch_number<<std::endl;
		myfile.close();
	}
	printed = true;

	//////////////////////////
	return true;
}

void NimmExp1RobotState::set_test_trial()
{
	if (Final_goal_is_reached()) {
		for (int i=0; i<NUMBER_OF_LANDMARKS; i++)
			exploration_array[i] = false;
		return;
	}

	for (int i=0; i<NUMBER_OF_LANDMARKS; i++)
		if (i < current_goal)
			exploration_array[i] = false;
		else
			exploration_array[i] = true;
}
//final output unit
double NimmExp1RobotState::bind_exploration(double actor_out, double &exploration)
{
	if (((epoch_number % TEST_TRY) == 0) || (!exploration_array[current_goal])) {
		if ((counter == 0) && ((epoch_number % TEST_TRY) == 0)) {
			std::cout<<"test trial :)"<<std::endl; counter++;
		}
		exploration = 0;
		return actor_out;
	}
	else
		if ((epoch_number % 2) == 0)
			return exploration;//actual_out + exploration;
		else
			return actor_out + exploration;
}


NimmExp1RobotState::~NimmExp1RobotState() {
	// TODO Auto-generated destructor stub
}
//exploration function
double NimmExp1RobotState::get_exploration(double Vt, double gain)
{
	double y;
	double V1, V0;
	double max_value, min_value;
	//double amplified_value;
	double scale;

	//double gain =  0.9997;//0.99;//0.6;

	if (Vt > vmax[current_goal])
		vmax[current_goal] = Vt;
	if (Vt < vmin[current_goal])
		vmin[current_goal] = Vt;


	double low_pass_value = ((1-gain)*gauss()) + ((gain)*(exploration_old));
	//save old value


	V1 = vmax[current_goal];// / 2.0;//0.1;
	V0 = 0;//vmin[current_goal] / 2.0;//-0.1;


	y = (V1-Vt)/(V1-V0);
	max_value = max(0, y);
	min_value = min(/*0.1*/1 ,max_value); //changing from 1 to 0.1



	//std::cout<<"min_value"<<min_value<<std::endl;

	scale = 0.2*min_value;//perfect 0.08;
	//amplified_value = scale*min_value;

	exploration_old = low_pass_value;//*min_value;


	return scale*low_pass_value;
}

double NimmExp1RobotState::gauss()
{
	double	sum;
	int 	i;

	for( sum = i = 0; i < 12; i++)
		sum += (1.0*(double)rand()/(RAND_MAX));
	return(sum - 6.0);
}


