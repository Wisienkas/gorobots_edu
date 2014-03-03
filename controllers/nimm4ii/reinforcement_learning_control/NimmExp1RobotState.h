/*
 * NimmExp1RobotState.h
 *
 *  Created on: Jul 24, 2013
 *      Author: bassel
 */

#ifndef NimmExp1RobotState_H_
#define NimmExp1RobotState_H_
#define NUMBER_OF_LANDMARKS 3
#define TEST_TRY 10

#include <iostream>
#include <cstdlib>
#include <fstream>
#include <vector>




class NimmExp1RobotState {
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



	NimmExp1RobotState();
	double get_state_array();
	double get_reward(double* distances);
	void reach_new_state();
	bool reset_robot_state();
	void set_test_trial();
	double bind_exploration(double actual_out, double &exploration);
	bool Final_goal_is_reached();

	virtual ~NimmExp1RobotState();


	double get_exploration(double Vt, double gain);
	double gauss();


};

#endif /* NimmExp1RobotState_H_ */
