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

	/////////////exploration calculation
	double exploration_old;
	std::vector<double> vmax;
	std::vector<double> vmin;


public:
	////////////////////////////////members section///////////////////



	RobotState();
	double get_state_array();
	double get_reward(double* distances);
	void reach_new_state();
	bool reset_robot_state();
	void set_test_trial();
	double bind_exploration(double actual_out, double &exploration);
	bool Final_goal_is_reached();

	virtual ~RobotState();


	double get_exploration(double Vt, double gain);
	double gauss();


};

#endif /* ROBOTSTATE_H_ */
