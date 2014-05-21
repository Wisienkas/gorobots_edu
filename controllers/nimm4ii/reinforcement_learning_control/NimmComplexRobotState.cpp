/*
 * RobotState.cpp
 *
 *  Created on: Jul 24, 2013
 *      Author: bassel
 */
//RobotState Nimm - complex environment
/*************************************************************************
 *  RobotState includes functions to support RL in the multiple goal scenario.
 *  It controls subgoals related system's functionalities.
 * (e.g. reward signals when the robot reaches a subgoal)
 * It controls mainly:
 * 			1) the reward positive signals when the robot reaches a right subgoal (called by reward function in acicoRCcontroller)
 * 			2) the exploration signals (when turned off and when turned on)
 * 			3) The part of the reward function which gives positive rewards when the robot reaches a right subgoal
 * 			4) ICO learning functions (output plus learning) (related to subgoals' positions)
 */

#include "NimmComplexRobotState.h"


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

#define absorbing_time 50
#define PHASE_MAX_TIME 3500//1600

#define REFLEX_RANGE 0.025
#define PREDICTION_SIGNAL 0.055

#define WRITE_AVERAGE 0

#define SCALER 0.5

namespace {
  int counter = 0;
  bool printed = false;
}

NimmComplexRobotState::NimmComplexRobotState() {
	 //vmax = 0.05;
	 //vmin = -0.05;

	current_goal = 0;
	reward_absorbing_counter = 0;
	epoch_number = 0;
	final_goal_checked = false;
	ignore_landmark = false;

	for (int i=0; i<NUMBER_OF_LANDMARKS; i++) {
		exploration_array[i] = true;
		vmax.push_back(0.001);
		vmin.push_back(-0.001);
	}

	phase_counter = 0;

	////////////////////exploration init
	exploration_old = gauss()  * SCALER;
	for (int i=0; i<NUM_INPUT_ICO; i++)
			ico_weights[i]=0;
}



//////////////////////////////////////////////Reinforcement learning part//////////////////////////


double NimmComplexRobotState::get_state_array() {
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

bool NimmComplexRobotState::Final_goal_is_reached()
{
	return final_goal_checked;
}

bool NimmComplexRobotState::Is_end_of_phase()
{
	if (phase_counter > PHASE_MAX_TIME)
			return true;
	return false;
}

void NimmComplexRobotState::reach_new_state() {
	if (current_goal < (FINAL_GOAL - 1)) {
			current_goal++;
		reward_absorbing_counter = 0;
		exploration_old = gauss()  * SCALER;
		phase_counter = 0;
	} else {
		final_goal_checked = true;
	}
}
//reward function (related to the landmarks)
double NimmComplexRobotState::get_reward(double* distances) {

	//std::cout<<"phase_counter"<<phase_counter<<std::endl;
	//std::cout<<"dis="<<distances[current_goal]<<std::endl;
	if (reward_absorbing_counter > absorbing_time)
			reach_new_state();

	if ((distances[current_goal] < 0.007) && (reward_absorbing_counter <= absorbing_time)) {
			reward_absorbing_counter++;
			return 1;
		}

	/*for (int i=0; i<NUMBER_OF_LANDMARKS; i++)
		if ((distances[i] < 0.02) && (i != current_goal)) {
					return -0.1;
				}*/

	return 0;
}

bool NimmComplexRobotState::reset_robot_state() {
	epoch_number++;
	///////////////////////////////
	if ((epoch_number > 2000) && (WRITE_AVERAGE)) {
		std::ofstream myfile;
		myfile.open ("Results_ENV2.txt", std::ios::app);
		myfile<< "Fail to learn!!!! 4000 iterations "<<std::endl;
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


	this->samples.clear();

	/////////////////////////////////
	if (((epoch_number-1) % TEST_TRY )== 0)
		set_test_trial();
	current_goal = 0;
	reward_absorbing_counter = 0;

	//////////exploration section
	exploration_old = gauss()  * SCALER;
	counter = 0;

	final_goal_checked = false;

	//for process termination
	for (int i=0; i<NUMBER_OF_LANDMARKS; i++)
		if (exploration_array[i])
			return false;
	/////print results

	if ((!printed) && (WRITE_AVERAGE)) {
		std::ofstream myfile;
		myfile.open ("Results_ENV2.txt", std::ios::app);
		myfile<< "Epochs Number = "<<epoch_number<<std::endl;
		myfile.close();
	}
	printed = true;

	//////////////////////////
	return true;
}

void NimmComplexRobotState::set_test_trial()
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
double NimmComplexRobotState::bind_exploration(double actor_out, double &exploration, bool ico_active)
{
	if (((epoch_number % TEST_TRY) == 0) || (!exploration_array[current_goal])) {
		if ((counter == 0) && ((epoch_number % TEST_TRY) == 0)) {
			std::cout<<"test trial :)"<<std::endl; counter++;
		}
		exploration = 0;
		return actor_out;
	}
	else {

		if (((epoch_number % 2) == 0) || (ico_active)) {
			if (!ico_active)
				return clip(exploration, -0.04, 0.04);//actual_out + exploration;
			else
				return exploration;
		}
		else
			return actor_out + clip(exploration, -0.01, 0.01);
	}
}


NimmComplexRobotState::~NimmComplexRobotState() {
	// TODO Auto-generated destructor stub
}

int NimmComplexRobotState::get_target_number() {
	return current_goal;
}
//exploration function
double NimmComplexRobotState::get_exploration(double Vt, double gain)
{
	bool lowpassnoise = true;

	double y;
	double V1, V0;
	double max_value, min_value;
	//double amplified_value;
	double scale;

	//double gain =  0.9997;//0.99;//0.6;




	//save old value


	V1 = vmax[current_goal] * 0.8;// * 0.9;// * 0.7 ;//0.1;
	V0 = vmin[current_goal] * 0.8;// * 0.9;// * 0.7;//-0.1;


	y = (V1-Vt)/(V1-V0);
	max_value = max(0, y);
	min_value = min(/*0.1*/1 ,max_value); //changing from 1 to 0.1


	//double min_value_new = max_value;
	//min_value_new = clip(min_value_new, 0.95, 1);

	double gain1 = 0.98*min_value;//0.96;//*min_value*min_value;//1.002*min_value;
	gain1 = clip(gain1, 0.95, 1);//clip(gain1, 0.955, 1);

	double low_pass_value = ((1-gain1)*gauss()) + ((gain1)*(exploration_old));

	//std::cout<<"min_value"<<min_value<<std::endl;

	scale = 23*min_value*min_value;//perfect 0.08;
	//amplified_value = scale*min_value;

	exploration_old = low_pass_value;//*min_value;

	if (Vt > vmax[current_goal])
		vmax[current_goal] = Vt;
	if (Vt < vmin[current_goal])
		vmin[current_goal] = Vt;

	return scale*low_pass_value;
}

double NimmComplexRobotState::gauss()
{
	double	sum;
	int 	i;

	for( sum = i = 0; i < 12; i++)
		sum += (1.0*(double)rand()/(RAND_MAX));
	return(sum - 6.0);
}


void NimmComplexRobotState::submit_sample(double* inputs, double td_v, double td_actor, double perror) {
	Sample s;
	for (int i=0; i<4; i++)
		s.inputs.push_back(inputs[i]);
	s.tds.push_back(td_v);
	s.tds.push_back(td_actor);
	s.perror = perror;
	this->samples.push_back(s);
}

std::vector<Sample> NimmComplexRobotState::get_samples() {
	return this->samples;
}

//////////////////////////////////////////////ICO learning part//////////////////////////
void NimmComplexRobotState::check_landmark_lock(double min_dis, int min_index) {
	if (!ignore_landmark) {
		if (min_dis < 0.006)
			ignore_landmark = true;
	} else {
		if (min_index == -1)
			ignore_landmark = false;
	}
}

/////////////////////////ICO exploration unit

int NimmComplexRobotState::get_nearest_signal_index(double* distances) {
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
double NimmComplexRobotState::get_ICO_out(double signal_value, double* distances, int index)
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

bool NimmComplexRobotState::is_test_trial()
{
	return ((epoch_number % TEST_TRY) == 0);
}
//ICO learning
double* NimmComplexRobotState::learn_ico(double angle, double old_angle, double* distances, int index) {
	if (ignore_landmark)
		return ico_weights;
	//ico learning
	if ((distances[index] < REFLEX_RANGE)
		&& (absolute(angle) > 0.1) //ignore small updates
		&& (index >= current_goal)
		&& (exploration_array[index])
		&& (!is_test_trial())
		)  {//&& (distance > REFLEX_RANGE)) {
		double learning_rate = 0.6;
		for (int i=0; i<NUM_INPUT_ICO; i++)
			ico_weights[i] += learning_rate*absolute(angle - old_angle)*(angle);
	}
	return ico_weights;
}




