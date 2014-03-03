/*
 * AmosMgRobotState.cpp
 *
 *  Created on: Jul 24, 2013
 *      Author: bassel Zeidan
 */
//AmosMgRobotState AMOS - Multiple goal
/*************************************************************************
 *  AmosMgRobotState includes functions to support RL in the multiple goal scenario.
 *  It controls subgoals related system's functionalities.
 * (e.g. reward signals when the robot reaches a subgoal)
 * It controls mainly:
 * 			1) the reward positive signals when the robot reaches a right subgoal (called by reward function in acicoRCcontroller)
 * 			2) the exploration signals (when turned off and when turned on)
 * 			3) The part of the reward function which gives positive rewards when the robot reaches a right subgoal
 * 			4) ICO learning functions (output plus learning) (related to subgoals' positions)
 */
#include "AmosMgRobotState.h"


#ifndef clip
#define	clip(x,l ,u )( ((x)<(l)) ? (l) : ((x)>(u)) ? (u) : (x))
#endif

#ifndef min
#define min(x, y) ((x <= y) ? x : y)
#endif

#ifndef max
#define max(x, y) ((x >= y) ? x : y)
#endif

#ifndef absolute
#define absolute(x) (((x) < 0) ? -(x) : (x))
#endif

int counter = 0;
#define absorbing_time 40
#define PHASE_MAX_TIME 1300//1600

#define REFLEX_RANGE 0.07
#define PREDICTION_SIGNAL 0.13

#define WRITE_AVERAGE 0

AmosMgRobotState::AmosMgRobotState() {
	 //vmax = 0.05;
	 //vmin = -0.05;
	old_index = -2;
	current_goal = 0;
	reward_absorbing_counter = 0;
	epoch_number = 0;
	final_goal_checked = false;

	for (int i=0; i<NUMBER_OF_LANDMARKS; i++) {
		exploration_array[i] = true;
		vmax.push_back(0.001);
		vmin.push_back(-0.001);
	}

	phase_counter = 0;
	ignore_landmark = false;

	////////////////////exploration init
	exploration_old = gauss() / 2.0;
	for (int i=0; i<NUM_INPUT_ICO; i++)
			ico_weights[i]=0;
}



//////////////////////////////////////////////Reinforcement learning part//////////////////////////


double AmosMgRobotState::get_state_array() {
	//double* taken_rewards = new double [NUMBER_OF_LANDMARKS];
	phase_counter++;
	return current_goal;

	/*for (int i=0; i<NUMBER_OF_LANDMARKS; i++)
		if (i < current_goal)
			taken_rewards[i] = 1;
		else
			taken_rewards[i] = 0;

	return taken_rewards;*/
}

bool AmosMgRobotState::Final_goal_is_reached()
{
	return final_goal_checked;
}

bool AmosMgRobotState::Is_end_of_phase()
{
	//std::cout<<"phase_counter"<<phase_counter<<std::endl;
	if (phase_counter > PHASE_MAX_TIME)
			return true;
	return false;
}

void AmosMgRobotState::reach_new_state() {
	if (current_goal < (FINAL_GOAL - 1)) {
			current_goal++;
		reward_absorbing_counter = 0;
		exploration_old = gauss() / 2.0;
		phase_counter = 0;
	} else {
		final_goal_checked = true;
	}
}


double AmosMgRobotState::get_reward(double* distances) {

	//std::cout<<"phase_counter"<<phase_counter<<std::endl;

	if (reward_absorbing_counter > absorbing_time)
			reach_new_state();


	//std::cout<<"dis="<<distances[current_goal]<<"current_goal"<<current_goal<<std::endl;
	if ((distances[current_goal] < 0.04) && (reward_absorbing_counter <= absorbing_time)) {
			reward_absorbing_counter++;
			return 1;
		}

	/*for (int i=0; i<NUMBER_OF_LANDMARKS; i++)
		if ((distances[i] < 0.02) && (i != current_goal)) {
					return -0.1;
				}*/

	return 0;
}

bool printed = false;

bool AmosMgRobotState::reset_robot_state() {
	epoch_number++;
	///////////////////////////////
	/*if (epoch_number > 000) {
		std::ofstream myfile;
		myfile.open ("Results_ENV1.txt", std::ios::app);
		myfile<< "Fail to lean!!!! 1000 iterations "<<std::endl;
		myfile.close();
		printed = true;
		return true;
	}*/
	if ((epoch_number > 2000) && (WRITE_AVERAGE)) {
		std::ofstream myfile;
		myfile.open ("AVG_trials.txt", std::ios::app);
		myfile<< "Fail to learn!!!! 2000 iterations "<<std::endl;
		myfile.close();
		printed = true;
		return true;
	}


	/////////////////////////////////
	/////print section
	phase_counter = 0;

	std::cout<<"epoch_number"<<epoch_number<<" - "<<std::endl;
	for (int i=0; i<NUMBER_OF_LANDMARKS; i++)
		std::cout<<exploration_array[i]<<"-"<<std::endl;
	std::cout<<"current goal = "<<current_goal<<std::endl;


	/////////////////////////////////
	if (((epoch_number-1) % TEST_TRY )== 0)
		set_test_trial();
	current_goal = 0;
	reward_absorbing_counter = 0;

	//////////exploration section
	exploration_old = gauss() / 2.0;
	counter = 0;

	final_goal_checked = false;

	//for process termination
	for (int i=0; i<NUMBER_OF_LANDMARKS; i++)
		if (exploration_array[i])
			return false;
	/////print results

	if ((!printed) && (WRITE_AVERAGE)) {
		std::ofstream myfile;
		myfile.open ("AVG_trials.txt", std::ios::app);
		myfile<< "Epochs Number = "<<epoch_number<<std::endl;
		myfile.close();
	}
	printed = true;

	//////////////////////////
	return true;
}

void AmosMgRobotState::set_test_trial()
{
	if (Final_goal_is_reached()) { //current_goal does not exceed the value of the target
		for (int i=0; i<NUMBER_OF_LANDMARKS; i++)
			exploration_array[i] = false;
		//std::cout<<"DONE-DONE-DONE-DONE-DONE-DONE-DONE-DONE-DONE-DONE-DONE-DONE-DONE-DONE-DONE-DONE-DONE-DONE-DONE-DONE-DONE-";
		//exit(-1);
		return;
	}

	for (int i=0; i<NUMBER_OF_LANDMARKS; i++)
		if (i < current_goal)
			exploration_array[i] = false;
		else
			exploration_array[i] = true;
}


//final output unit's job
double AmosMgRobotState::bind_exploration(double actor_out, double &exploration, bool ico_active)
{
	if (((epoch_number % TEST_TRY) == 0) || (!exploration_array[current_goal])) {
		if ((counter == 0) && ((epoch_number % TEST_TRY) == 0)) {
			std::cout<<"test trial :)"<<std::endl; counter++;
		}
		exploration = 0;
		return actor_out;
	}
	else {

		if (((epoch_number % 2) == 0) || (ico_active))
			return exploration;//actual_out + exploration;
		else
			return actor_out + exploration;
	}
}


AmosMgRobotState::~AmosMgRobotState() {
	// TODO Auto-generated destructor stub
}

int AmosMgRobotState::get_target_number() {
	return current_goal;
}
//exploration function
double AmosMgRobotState::get_exploration(double Vt, double gain)
{
	bool lowpassnoise = true;

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


	V1 = vmax[current_goal] / 2.0;//0.1;
	V0 = vmin[current_goal] / 2.0;//-0.1;


	y = (V1-Vt)/(V1-V0);
	max_value = max(0, y);
	min_value = min(/*0.1*/1 ,max_value); //changing from 1 to 0.1



	//std::cout<<"min_value"<<min_value<<std::endl;

	scale = 0.4*min_value;//perfect 0.08;
	//amplified_value = scale*min_value;

	exploration_old = low_pass_value;//*min_value;
	return scale*low_pass_value;
}

double AmosMgRobotState::gauss()
{
	double	sum;
	int 	i;

	for( sum = i = 0; i < 12; i++)
		sum += (1.0*(double)rand()/(RAND_MAX));
	return(sum - 6.0);
}
//////////////////////////////////////////////ICO learning part//////////////////////////
void AmosMgRobotState::check_landmark_lock(double min_dis, int min_index) {
	if (!ignore_landmark) {
		old_index = min_index;
		if (min_dis < 0.04)
			ignore_landmark = true;
	} else {
		if (min_index != old_index)
			ignore_landmark = false;
	}
}

int AmosMgRobotState::get_nearest_signal_index(double* distances) {
	//for (int i=0; i<11; i++)
	//	std::cout<<"distances["<<i<<"]="<<distances[i]<<std::endl;


	double min_index = -1;
	double min = 9999999999;

	for (int i=0; i<NUMBER_OF_LANDMARKS; i++)
		if ((min > distances[i]) && (distances[i] < PREDICTION_SIGNAL)
				//&& (i >= current_goal) //(exploration_array[i] == true)//ignore acheived goals
				//&& (!ignore_landmark)
			) {
			min = distances[i];
			min_index = i;
		}

	check_landmark_lock(min, min_index);

	return min_index;
}

//ICO output
double AmosMgRobotState::get_ICO_out(double signal_value, double* distances, int index)
{
	if (ignore_landmark)
			return 0;

	double sum = 0;

	for (int i=0; i<NUM_INPUT_ICO; i++)
		sum += signal_value*ico_weights[i];


	if (distances[index] < REFLEX_RANGE)
		sum += signal_value;//Reflex_signal

	return sum;//Reflex_signal
}

bool AmosMgRobotState::is_test_trial()
{
	return ((epoch_number % TEST_TRY) == 0);
}

bool AmosMgRobotState::Is_reflex_active(double* distances, int index) {
	if (distances[index] < REFLEX_RANGE)
			return true;
	return false;
}
//ICO learning
double* AmosMgRobotState::learn_ico(double angle, double old_angle, double* distances, int index) {
	//ico learning
	if (ignore_landmark)
			return ico_weights;

	if ((distances[index] < REFLEX_RANGE)
		&& (absolute(angle) > 0.2) //ignore small updates
		&& (index >= current_goal)
		&& (exploration_array[index])
		&& (!is_test_trial())
		)  {//&& (distance > REFLEX_RANGE)) {
		std::cout<<"/////////////////////////////////////////////////////////////"<<std::endl;
		std::cout<<"distances["<<index<<"] = "<< distances[index]<<std::endl;
		std::cout<<"angle = "<<angle<<std::endl;
		std::cout<<"absolute(angle - old_angle) = "<<absolute(angle - old_angle)<<std::endl;
		std::cout<<"step number = "<<step_number<<std::endl;
		std::cout<<"/////////////////////////////////////////////////////////////"<<std::endl;

		double learning_rate = 0.6;
		for (int i=0; i<NUM_INPUT_ICO; i++)
			ico_weights[i] += learning_rate*absolute(angle - old_angle)*(angle);
	}
	return ico_weights;
}

void AmosMgRobotState::submit_step_number(int step) {
	this->step_number = step;
}



