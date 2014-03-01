/*
 * RobotState.h
 *
 *  Created on: Jul 24, 2013
 *      Author: bassel
 */

#ifndef ROBOTSTATE_H_
#define ROBOTSTATE_H_
#define NUMBER_OF_LANDMARKS 3
#define TEST_TRY 10

#include <iostream>
#include <cstdlib>



#include <fstream>
#include <vector>





class RobotState {
private:
	bool exploration_array [NUMBER_OF_LANDMARKS];
	int current_goal;
	int reward_absorbing_counter;
	int epoch_number;
	bool final_goal_checked;
	bool ignore_landmark;

	/////////////exploration calculation
	double exploration_old;
	std::vector<double> vmax;
	std::vector<double> vmin;

	int phase_counter;


	/////////////////////////////////ICO leaning parameters

#define NUM_INPUT_ICO 1
	double ico_weights [NUM_INPUT_ICO];

public:
	////////////////////////////////members section///////////////////



	RobotState();
	double get_state_array();
	double get_reward(double* distances);
	void reach_new_state();
	bool reset_robot_state();
	void set_test_trial();
	double bind_exploration(double actual_out, double &exploration, bool ICO_ACTIVE);
	bool Final_goal_is_reached();
	bool Is_end_of_phase();
	void check_landmark_lock(double min_dis, int min_index);

	int get_target_number();

	virtual ~RobotState();


	double get_exploration(double Vt, double gain);
	double gauss();

	//////////////////////////////////////////////ICO learning part//////////////////////////
	int get_nearest_signal_index(double* distances);
	double get_ICO_out(double signal_value, double* distances, int index);
	double* learn_ico(double angle, double old_angle, double* distances, int index);

};

#endif /* ROBOTSTATE_H_ */
