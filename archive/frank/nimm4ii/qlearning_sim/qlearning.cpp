/*
 * qlearning.cpp
 *
 *  Created on: May 4, 2011
 *      Author: fhesse
 */

#include "qlearning.h"
#include <selforg/controller_misc.h>

QLearning::QLearning(unsigned int number_states_, unsigned int number_actions_){
	eps=0.1;//0.1;      // learning rate (typically 0.1)
	discount=/*0.1*/0.9;//0.5;//0.1; //0.9; // discount discount factor for Q-values (typically 0.9)
	exploration=0.9/*0.5*/;//0.2; //exploration rate (typically 0.02)
	number_states=	number_states_;
	number_actions = number_actions_;
	Q.set(number_states,number_actions,0);// < Q table (mxn) == (states x actions)
	Q.val(0,0)=1;
	Q.val(1,1)=1;
	Q.val(2,2)=1;
	Qvisits.set(number_states,number_actions,0);// < visits of Q table (mxn) == (states x actions)

	if(!randGen) randGen = new RandGen(); // this gives a small memory leak
	this->randGen=randGen;

	exploration_active=false;
}

QLearning::~QLearning(){

};


unsigned int QLearning::selectAction (unsigned int state){
	//assert(initialised);
	assert(state < Q.getM());

	matrix::Matrix as = Q.row(state);
	//as += as.mapP(randGen, random_minusone_to_one)*0.1; // this is like random
	// walk if we know nothing
	int a= argmax(as);
	// TODO activate exploration

	// exploration
	if (exploration_active){
		double r = randGen->rand();
		if(r<exploration){
			a = (int)(randGen->rand()*(double)Q.getN());
		}
	}
	return a;
}


double QLearning::learn (unsigned int state0, unsigned int action,
		unsigned int state1, double reward){
	assert (state0 < number_states);
	assert (state1 < number_states);
	assert (action < number_actions);

	//calculate total reward (= achieved reward + maximum Q-value of future state)
	double total_reward = reward + discount * max(Q.row(state1));

	// update Q-value for executed state action pair
	Q.val(state0, action) += eps * (total_reward - Q.val(state0, action));
	Qvisits.val(state0, action) ++;
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


	//assert(initialised);
	//  actions[t%ringbuffersize] = action;
	//  states[t%ringbuffersize]  = state;
	//  rewards[t%ringbuffersize] = reward;
	//  collectedReward -= longrewards[t%tau];
	//  longrewards[t%tau] = reward;
	//  collectedReward += reward;
	// Todo: eligibility traces are wrong! see the book of sutton how to do it!
	// learn for previous eligibility steps
	//  double e_factor = 1; // learning factor for eligibility
	//  for(int i=1; i<=eligibility; i++){
	//    if(t-i <0) break;
	//    int a_t     = actions[(t-i)   %ringbuffersize];
	//    int a_tp1   = actions[(t-i+1) %ringbuffersize];
	//    int s_t     = states [(t-i)   %ringbuffersize];
	//    int s_tp1   = states [(t-i+1) %ringbuffersize];
	//    double r_t  = rewards[(t-i)   %ringbuffersize];
	//    double e    = eps*e_factor * learnRateFactor;  // local learning rate
	//    if(useSARSA)
	//      Q.val(s_t,a_t) += e*(r_t + discount*Q.val(s_tp1,a_tp1) - Q.val(s_t,a_t));
	//    else
	//      Q.val(s_t,a_t) += e*(r_t + discount*max(Q.row(s_tp1)) - Q.val(s_t,a_t));
	//    e_factor -= 1.0/eligibility;
	////  }
	//  t++;
	return Q.val(state0,action);
}


void QLearning::printQTable(){
	std::cout<<"           turn left     straight     turn right"<<std::endl;
	for (unsigned int m=0; m<Q.getM(); m++){
		if (m==0) {	std::cout<<"goal left   ";}
		if (m==1) {	std::cout<<"no/in front ";}
		if (m==2) {	std::cout<<"goal right  ";}
		for (unsigned int n=0; n<Q.getN(); n++){
			std::cout<<"   "<<Q.val(m,n)<<" ("<<Qvisits.val(m,n)<<")  ";
		}
		std::cout<<std::endl;
	}
}
