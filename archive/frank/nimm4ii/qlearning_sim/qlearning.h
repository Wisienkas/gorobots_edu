/*
 * qlearning.h
 *
 *  Created on: May 4, 2011
 *      Author: fhesse
 */

#ifndef QLEARNING_H_
#define QLEARNING_H_

#include <selforg/matrix.h>
#include <selforg/randomgenerator.h>

class QLearning{
public:
	QLearning(unsigned int number_states, unsigned int number_actions);
	~QLearning();


	/** selection of action given current state.
	      The policy is to take the actions with the highest value,
	      or a random action at the rate of exploration
	 */
	virtual unsigned int selectAction (unsigned int state);


	  /* performs learning and returns current expected reward.
	     \param state current state
	     \param action we select in current state
	     \param reward reinforcement we obtain in this state
	  */
	  virtual double learn (unsigned int state0, unsigned int action,
				unsigned int state1, double reward);


	/** tells the q learning that the agent was reset, so that it
	      forgets it memory. please note, that updating the Q-table is
	      one step later, so in case of a reward you should call learn one
	      more time before reset.
	 */
	virtual void reset(){};

	void printQTable();

	void setExplorationActive(bool b){
		exploration_active=b;
	}

	double exploration; //exploration rate (typically 0.02)
	double eps;

	matrix::Matrix Q; /// < Q table (mxn) == (states x actions)
	matrix::Matrix Qvisits; /// < visits of Q table (mxn) == (states x actions)

protected:
	//int tau;       ///< time horizont for averaging the reward
	double discount;  // (gamma)
    unsigned int number_states, number_actions;
    bool exploration_active;

    RandGen* randGen;


};

#endif /* QLEARNING_H_ */
