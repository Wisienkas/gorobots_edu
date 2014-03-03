/*
 * AmosMgRobotState.h
 *
 *  Created on: Jul 24, 2013
 *      Author: bassel Zeidan
 */

#ifndef AmosMgRobotState_H_
#define AmosMgRobotState_H_
#define NUMBER_OF_LANDMARKS 5
#define FINAL_GOAL 5
#define TEST_TRY 10

#include <iostream>
#include <cstdlib>



#include <fstream>
#include <vector>





class AmosMgRobotState {
private:
	bool exploration_array [NUMBER_OF_LANDMARKS];
	int current_goal;
	int reward_absorbing_counter;
	int epoch_number;
	bool final_goal_checked;
	std::vector<double> vmax;
	std::vector<double> vmin;
	int step_number;




	/////////////exploration calculation
	double exploration_old;
	bool ignore_landmark;
	int phase_counter;


	/////////////////////////////////ICO leaning parameters

#define NUM_INPUT_ICO 1
	double ico_weights [NUM_INPUT_ICO];
	int old_index;
public:
	////////////////////////////////members section///////////////////



	AmosMgRobotState();
	double get_state_array();
	double get_reward(double* distances);
	void reach_new_state();
	bool reset_robot_state();
	void set_test_trial();
	double bind_exploration(double actual_out, double &exploration, bool ICO_active);
	bool Final_goal_is_reached();
	bool Is_end_of_phase();

	int get_target_number();

	virtual ~AmosMgRobotState();

	bool Is_reflex_active(double* distances, int index);
	void submit_step_number(int step);


	double get_exploration(double Vt, double gain);
	double gauss();

	//std::vector<Sample> get_samples();
	bool is_test_trial();
	//////////////////////////////////////////////ICO learning part//////////////////////////
	int get_nearest_signal_index(double* distances);
	double get_ICO_out(double signal_value, double* distances, int index);
	double* learn_ico(double angle, double old_angle, double* distances, int index);
	void check_landmark_lock(double min_dis, int min_index);
};

#endif /* AmosMgRobotState_H_ */
