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

#include "qlearninghomeokmcontroller.h"
#include <selforg/controller_misc.h>
#include <iostream>
#include <fstream>

using namespace matrix;
using namespace std;



QLearningHomeokMController::QLearningHomeokMController( const QLearningHomeokMControllerConf& _conf)
: AbstractController("QLearningHomeokMController", "$Id: "), conf(_conf)
{
	//// Martin: (check if i have the same prob)
	//	//as the following pointers were not zero when i ran the simulation with the -nographics option
	//	//(else not) i set them zero here. i am clueless as to why.
	//	random_controller = 0;
	//	gait_controller = 0;
	////	homeokinetic_controllers = 0;


	preprogrammed_steering_control_active =false;



	t=0;

	summed_reward=0;
	number_reward_summations=0;
	learn=true;

	//for manual steering
	mc[0]=1;
	mc[1]=1;
	mc[2]=1;
	mc[3]=1;

	goalcounter=0;
	goalcounter2=0;

};


QLearningHomeokMController::~QLearningHomeokMController()
{
	for (int i=0; i< homeokinetic_controller.size(); i++){
		if(homeokinetic_controller.at(i)){
			delete homeokinetic_controller.at(i);
		}
	}
}


void QLearningHomeokMController::init(int sensornumber, int motornumber, RandGen* randGen)
{

	number_sensors = sensornumber;
	number_motors = motornumber;

	number_sensors_homeokinetic = 4;  // for nimm4 robot
	number_motors_homeokinetic = 4;

	//create and initialize controllers with the appropriate numbers of dofs
	//one controller
	one_controller = false;
	if (one_controller){
		homeokinetic_controller.resize(1);
		for (int i=0; i< homeokinetic_controller.size(); i++){
			homeokinetic_controller.at(i) = new OpenInvertNChannelController(/*buffersize*/100);  //add new controller
			homeokinetic_controller.at(i) -> init(number_sensors_homeokinetic,number_sensors_homeokinetic, randGen);
			homeokinetic_controller.at(i) -> setBiasUpdateRule(/*org*/no); //bias is always zero
			homeokinetic_controller.at(i) -> setParam("eps", 0.1);

			addInspectableMatrix("A", &homeokinetic_controller.at(i) ->A, false, "model matrix");
			addInspectableMatrix("C", &homeokinetic_controller.at(i) ->C, false, "controller matrix");
			addInspectableMatrix("h", &homeokinetic_controller.at(i) ->h, false, "controller bias");

			addParameter("eps",&homeokinetic_controller.at(i) -> eps);
			addParameter("rho",&homeokinetic_controller.at(i) -> rho);
			addParameter("s4delay",&homeokinetic_controller.at(i) ->s4delay);
			addParameter("s4avg",&homeokinetic_controller.at(i) -> s4avg);
			addParameter("delta",&homeokinetic_controller.at(i) ->delta);
			addParameter("factor_a",&homeokinetic_controller.at(i) ->factor_a);
			addParameter("desens",&homeokinetic_controller.at(i) -> desens);
			addParameter("number_it",&homeokinetic_controller.at(i) ->number_it);
			addParameter("epsilon_it",&homeokinetic_controller.at(i) ->epsilon_it);
			addParameter("damping_c",&homeokinetic_controller.at(i) -> damping_c);

		}
	} else {
		//several controllers
		homeokinetic_controller.resize(number_sensors_homeokinetic);
		for (int i=0; i< homeokinetic_controller.size(); i++){
			homeokinetic_controller.at(i) = new OpenInvertNChannelController(/*buffersize*/100);  //add new controller
			homeokinetic_controller.at(i) -> init(1,1, randGen);
			homeokinetic_controller.at(i) -> setBiasUpdateRule(/*org*/no); //bias is always zero
			homeokinetic_controller.at(i) -> setParam("eps", 0.1);

			addInspectableMatrix("A", &homeokinetic_controller.at(i) ->A, false, "model matrix");
			addInspectableMatrix("C", &homeokinetic_controller.at(i) ->C, false, "controller matrix");
			addInspectableMatrix("h", &homeokinetic_controller.at(i) ->h, false, "controller bias");

			addParameter("eps",&homeokinetic_controller.at(i) -> eps);
			addParameter("rho",&homeokinetic_controller.at(i) -> rho);
			addParameter("s4delay",&homeokinetic_controller.at(i) ->s4delay);
			addParameter("s4avg",&homeokinetic_controller.at(i) -> s4avg);
			addParameter("delta",&homeokinetic_controller.at(i) ->delta);
			addParameter("factor_a",&homeokinetic_controller.at(i) ->factor_a);
			addParameter("desens",&homeokinetic_controller.at(i) -> desens);
			addParameter("number_it",&homeokinetic_controller.at(i) ->number_it);
			addParameter("epsilon_it",&homeokinetic_controller.at(i) ->epsilon_it);
			addParameter("damping_c",&homeokinetic_controller.at(i) -> damping_c);

		}
	}

	old_reward=0;

	old_distance.resize(conf.number_qlearner,0);
	old_alpha.resize(conf.number_qlearner,0);
	old_state.resize(conf.number_qlearner,0);
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

	active_qlearner = 0;

	for (int i=0; i<conf.number_qlearner; i++){
		qlearner.push_back( new QLearning(number_states, number_actions));
	}


	// ------------ select learner -----------------------------------------------------


	//	// pure homeok control
	//	learned_steering_control_active=false;
	//	setExplorationActive(false);

		// homeok + RL control
		learned_steering_control_active=true;
		setExplorationActive(true);


	// TODO:
	// what about pure Q-Learners ?!

	// ------------ select learner -----------------------------------------------------


}

void QLearningHomeokMController::step(const sensor* x_, int number_sensors, motor* y_, int number_motors){

	sensor x_homeok[number_sensors_homeokinetic];
	std::vector<double> x_hom; // same as above, just as vector for qlearnig stuff below
	// do not consider e.g. IR sensors and so on
	// sensors belonging to motors are in the beginning of x_
	for (unsigned int j =0; j < number_sensors_homeokinetic; j++){
		x_homeok[j]=x_[j];
		x_hom.push_back(x_[j]);
	}


	if (t> 13001){

		x_hom[1]=-x_hom[1];
		x_homeok[1]=-x_homeok[1];
//		x_hom[2]=-x_hom[2];
//		x_homeok[2]=-x_homeok[2];
	}





	std::vector<double> alpha;
	std::vector<double> distance;

	// calculate angle and distance for each of the goals (for each of the controllers)
	for (int i=0; i<conf.number_qlearner; i++){
		//x_[4] => forward/backward
		//x_[5] => left/right
		if (sign(x_[0])==sign(x_[4+i*3])){ // goal is in front
			alpha.push_back(sign(x_[3+i*3+1]) * atan (x_[3+i*3+2]/x_[3+i*3+1]) * 180 / M_PI); // angle in degrees
		} else{ // behind robot angles "continue", directly behind robot is 180 degrees, left is negative, right is positive
			double alpha_tmp= sign(x_[3+i*3+1]) * atan (x_[3+i*3+2]/x_[3+i*3+1]) * 180 / M_PI; // angle in degrees
			if (alpha_tmp<=0) { // left
				alpha.push_back(-90 + (-90-alpha_tmp));
			} else { // right
				alpha.push_back( 90 + ( 90-alpha_tmp));
			}

		}
		distance.push_back( sqrt(pow(x_[3+i*3+1],2)+pow(x_[3+i*3+2],2)) );
		//		// calculate angle (deviation of goal from front)
		//		double alpha =  sign(x_[4]) * atan (x_[5]/x_[4]) * 180 / M_PI; // angle in degrees
		//	    double distance = sqrt(pow(x_[4],2)+pow(x_[5],2));
	}

	t++;
	//learned_steering_control_active=true;
	int time_for_start = 600/*0*/;

	// change to next goal, if near the current goal
	if ((distance.at(active_qlearner)</*1.3*/5) || ((t<(6000/*0*/+time_for_start))&&((t%500/*0*/)==0))){
		active_qlearner = ((active_qlearner+1)%conf.number_qlearner);
		if ( ((6000/*0*/+time_for_start)<t) && (t<(12000/*0*/+time_for_start))){ // 10 minuten nach dem lernen
			goalcounter++;
			std::cout<<goalcounter<<" GOAL(s) achieved "<<std::endl;
		} else{
			if ((13000/*0*/+time_for_start)>t){
				std::cout<<"ACTIVE LEARNER CHANGED TO "<<active_qlearner<<std::endl;
			}
		}

		if ( ((13000/*0*/+time_for_start)<t) && (t<(19000/*0*/+time_for_start))){ // 10 minuten nach dem lernen
			goalcounter2++;
			std::cout<<goalcounter2<<" GOAL(s) achieved "<<std::endl;
		}
	}

	std::vector<unsigned int> current_state;
	std::vector<double> reward;

	// for analysis
	if (t==19500/*0*/+time_for_start) {
		printQTable();
		std::cout << " goals (normal):"<<goalcounter<<std::endl;
		std::cout << " goals (defect):"<<goalcounter2<<std::endl;
		// ---------- print results to file ------------------------------------
		ofstream myfile;
		myfile.open ("goals.log");
		for (int i=0; i< conf.number_qlearner; i++){
			myfile<<"Q-Learner "<<i<<std::endl;
			myfile<<"           turn left     straight     turn right"<<std::endl;
			for (unsigned int m=0; m<qlearner.at(i)->Q.getM(); m++){
				if (m==0) {	myfile<<"goal left   ";}
				if (m==1) {	myfile<<"no/in front ";}
				if (m==2) {	myfile<<"goal right  ";}
				for (unsigned int n=0; n<qlearner.at(i)->Q.getN(); n++){
					myfile<<"   "<<qlearner.at(i)->Q.val(m,n)<<" ("<<qlearner.at(i)->Qvisits.val(m,n)<<")  ";
				}
				myfile<<std::endl;
			}
			myfile<<std::endl<<std::endl;
		}
		myfile << " goals (normal):"<<goalcounter<<std::endl;
		myfile << " goals (defect):"<<goalcounter2<<std::endl;
		myfile.close();

		ofstream myfile2;
		myfile2.open ("results_table.dat",ios::out | ios::app);
		myfile2 <<goalcounter<<"     ";
		myfile2 <<goalcounter2<<std::endl;
		myfile2.close();

		// --------------------------------------------------------------------
	}

	if ((learned_steering_control_active) && (t>time_for_start)) {
		if (t==time_for_start){std::cout<<"QLEARNING STARTED"<<std::endl;}


		if ( ((t%2/*0*/)==0) ){ //every 100 timesteps
			//std::cout<<t<<std::endl;



			// calculate for all QLearner, so all can learn, even though they did not determine the action!
			for (int i=0; i< conf.number_qlearner; i++){
				current_state.push_back(calc_current_state(alpha.at(i), distance.at(i),x_hom));
			}

			for (int i=0; i< conf.number_qlearner; i++){
				//			if (i==active_qlearner) {
				reward.push_back(calc_current_reward(
						alpha.at(i), old_alpha.at(i), distance.at(i), old_distance.at(i), x_hom));
				//			} else{
				//				reward.push_back(-99);
				//			}
			}

			/*
		//calculate reward -----------------------------------------------


		//		if (reward>=old_reward){
		// TODO: adapt to vectors
		summed_reward += reward.at(0);
		number_reward_summations ++;
		//			learn=false;
		//	}

		//end calculate reward -----------------------------------------------
			 */


			//---------------------decrease learning rate and exploration rate ------------------
			// as long as t<15000 (2,5 mins) eps and exploration are const (0.9)
			// after decrease they are 0
			for (int i=0; i< conf.number_qlearner; i++){
				if ( ((3000/*0*/+time_for_start)<t) && (t<(6000/*0*/+time_for_start))){ // 2.5 mins in which eps decreases;
					double eps_factor = 1.0 - ( ((double)t-(3000/*0.0*/+time_for_start))/3000/*0.0*/);
					//			if ( (15000<t) && (t<30000)){ // 2.5 mins in which eps decreases;
					//				double eps_factor = 1.0 - ( ((double)t-15000.0)/15000.0);
					qlearner.at(i)->exploration = 0.5 * eps_factor;
					qlearner.at(i)->eps = 0.1 * eps_factor;
					//	std::cout<<"t"<<t<<"  eps_factor="<<eps_factor<<std::endl;
				}
				if (t>(5999/*9*/+time_for_start)) {qlearner.at(i)->exploration = 0; qlearner.at(i)->eps = 0;} // to ensure 0 lerning rate and exploration
				// we have modulo (t%20) before ;-)
			}
			if (t==(3000/*0*/+time_for_start)) {std::cout<<"5 mins: EPS CHANGED TO: "<<qlearner.at(active_qlearner)->eps<<std::endl;}
			if (t== (6000/*0*/+time_for_start)) {std::cout<<"10 mins: EPS CHANGED TO: "<<qlearner.at(active_qlearner)->eps<<std::endl;}
			//---------------------------------------------------------------------------------------

			//if (current_state!=old_state) { //only when state changed



			//std::cout<<"sr="<<summed_reward<<"  steps="<<number_reward_summations<<"  normalized="<<summed_reward/((double)number_reward_summations)<<std::endl;
			// update Q-table
			// only for the active one so far
			double learned_reward=0;

			//	if ( (learn) && (number_reward_summations>0) ){ //it should be lerned
			//	if ( (learn) /* &&  (current_state!=old_state)*/){ //it should be lerned
			//			if ( (learn)  &&  (distance>1) && (sign(x_[0])==sign(x_[4])) /*&&  (current_state!=old_state)*/ ){ //it should be lerned
			//	if (x_[4]>1){ // do not learn if near to goal (driving away after pushing it will decrease reward)
			//				learned_reward = qlearner.at(0)->learn (old_state, old_action, current_state, summed_reward/((double)number_reward_summations));
			//		learned_reward = qlearner->learn (old_state, old_action, current_state, reward);
			//			}
			//			} else{
			//	learn=true;  // learn next time
			//			}

			// all QLearner are updated:  (old_action is the same for all)

			// !!!!!!!!!!!!!!!!!!!!!!!!!
			// dann werden aber auch sehr oft die aktionen der anderen gelernt




			if ( (learn) ){ // to have old_actions etc. filled
				for (int i=0; i<conf.number_qlearner; i++){
//					if (sign(x_[0])==sign(x_[4+i*3])){ // goal is in front
						int tmp= qlearner.at(i)->learn (
								old_state.at(i), old_action, current_state.at(i), reward.at(i));
						if (i==active_qlearner){ learned_reward = tmp; }// to typeout learned reward of active learner
//					} else {  // giving 0 reward instead
//						int tmp= qlearner.at(i)->learn (
//								old_state.at(i), old_action, current_state.at(i), /*reward.at(i)*/0);
//						if (i==active_qlearner){ learned_reward = tmp; }// to typeout learned reward of active learner
//					}
				}
			}



			/*
			// !!!!!!!!!!!!!!!!!!!!!!!!!!
			// erstmal nur den aktuellen lernen
			if ((t>40) && (learn) ){
				if (sign(x_[0])==sign(x_[4+active_qlearner*3])){
					learned_reward =  qlearner.at(active_qlearner)->learn (
							old_state.at(active_qlearner), old_action, current_state.at(active_qlearner), reward.at(active_qlearner));
				}
			}
			 */

			// select action (only the active QLearner)
			int current_action = qlearner.at(active_qlearner)->selectAction(current_state.at(active_qlearner));

			//			std::cout<<"state="<<old_state.at(active_qlearner)<<"  action="<<old_action<<std::endl;
			//
			//			std::cout<<"alpha="<<alpha.at(active_qlearner)<<"  distance="<<distance.at(active_qlearner);
			//			if (reward.at(active_qlearner)==0){
			//				std::cout<<"  r="<<0<<"  r+="<<0<<std::endl<<std::endl;
			//			} else{
			//			std::cout<<"  r="<<reward.at(active_qlearner)<<std::endl;
			//			}


			//			if ((t>40) && (learn) ){ // to have old_actions etc. filled
			//			//TODO: print what required
			//			std::cout<<"s0 = "<<old_state.at(active_qlearner)<<"   ";
			//			std::cout<<"a0 = "<<old_action<<"   ";
			//			std::cout<<"s1 = "<<current_state.at(active_qlearner)<<"   ";
			//			std::cout<<"a1 = "<<current_action<<"   ";
			//			std::cout<<"r = "<<reward.at(active_qlearner)<<"  ";
			//			//				std::cout<<"sr = "<<summed_reward/number_reward_summations<<"   ";
			//			std::cout<<"lr = "<<learned_reward<<std::endl;
			//			}
			manual_hom_control=false;
			if (manual_hom_control){
				if (one_controller){
					std::cout<< "here"<<std::endl;
					std::vector<double> f;
					f.push_back(mc[0]);
					f.push_back(mc[1]);
					f.push_back(mc[2]);
					f.push_back(mc[3]);
					homeokinetic_controller.at(0)->setMotorCommandFactor(f);
				} else {
					std::cout<< "or here"<<std::endl;
					std::vector<double> f0,f1,f2,f3;
					f0.push_back(mc[0]);
					f1.push_back(mc[1]);
					f2.push_back(mc[2]);
					f3.push_back(mc[3]);
					homeokinetic_controller.at(0)->setMotorCommandFactor(f0);
					homeokinetic_controller.at(1)->setMotorCommandFactor(f1);
					homeokinetic_controller.at(2)->setMotorCommandFactor(f2);
					homeokinetic_controller.at(3)->setMotorCommandFactor(f3);
				}

			}else{

				if (current_action == 1){
					if (one_controller){
						std::vector<double> f;
						f.push_back(1.0);
						f.push_back(1.0);
						f.push_back(1.0);
						f.push_back(1.0);
						homeokinetic_controller.at(0)->setMotorCommandFactor(f);
					} else {
						std::vector<double> f1;
						f1.push_back(1.0);
						homeokinetic_controller.at(0)->setMotorCommandFactor(f1);
						homeokinetic_controller.at(1)->setMotorCommandFactor(f1);
						homeokinetic_controller.at(2)->setMotorCommandFactor(f1);
						homeokinetic_controller.at(3)->setMotorCommandFactor(f1);
					}
					//				std::cout<<"gerade:   "<<std::endl;
				}
				if (current_action == 0){
					if (one_controller){
						std::vector<double> f;
						f.push_back(0.5);
						f.push_back(1.0);
						f.push_back(0.5);
						f.push_back(1.0);
						homeokinetic_controller.at(0)->setMotorCommandFactor(f);
					} else{
						std::vector<double> f0, f1;
						f1.push_back(1.0);
						f0.push_back(0.5);
						homeokinetic_controller.at(0)->setMotorCommandFactor(f0);
						homeokinetic_controller.at(1)->setMotorCommandFactor(f1);
						homeokinetic_controller.at(2)->setMotorCommandFactor(f0);
						homeokinetic_controller.at(3)->setMotorCommandFactor(f1);
					}
					//				std::cout<<"links:   "<<std::endl;
				}
				if (current_action == 2){
					if (one_controller){
						std::vector<double> f;
						f.push_back(1.0);
						f.push_back(0.5);
						f.push_back(1.0);
						f.push_back(0.5);
						homeokinetic_controller.at(0)->setMotorCommandFactor(f);
					} else {
						std::vector<double> f0, f1;
						f1.push_back(1.0);
						f0.push_back(0.5);
						homeokinetic_controller.at(0)->setMotorCommandFactor(f1);
						homeokinetic_controller.at(1)->setMotorCommandFactor(f0);
						homeokinetic_controller.at(2)->setMotorCommandFactor(f1);
						homeokinetic_controller.at(3)->setMotorCommandFactor(f0);
					}
					//				std::cout<<"rechts:   "<<std::endl;
				}
			}
			//test
			//std::cout<<x_[4]<<"   "<<x_[5]<<std::endl;

			//			summed_reward=0;
			//			number_reward_summations=0;

			old_state = current_state;
			old_action = current_action;
			old_reward = reward.at(active_qlearner);
			old_distance = distance;
			old_alpha = alpha;


		}

	}


	//		// manually defined steering
	//		if (preprogrammed_steering_control_active){ // manually defined steering
	//			if (x_[5]>0.1){ //Kugel rechts ?!
	//				std::vector<double> f;// = homeokinetic_controller->getMotorCommandFactor();
	//				f.push_back(1.0);
	//				f.push_back(0.5);
	//				f.push_back(1.0);
	//				f.push_back(0.5);
	//				homeokinetic_controller->setMotorCommandFactor(f);
	//				std::cout<<"rechts:   ";
	//			} else {
	//				if (x_[5]<-0.1){ //Kugel links ?!
	//					std::vector<double> f;
	//					f.push_back(0.5);
	//					f.push_back(1.0);
	//					f.push_back(0.5);
	//					f.push_back(1.0);
	//					homeokinetic_controller->setMotorCommandFactor(f);
	//					std::cout<<"links:   ";
	//				} else {
	//					std::vector<double> f;// = homeokinetic_controller->getMotorCommandFactor();
	//					f.push_back(1.0);
	//					f.push_back(1.0);
	//					f.push_back(1.0);
	//					f.push_back(1.0);
	//					homeokinetic_controller->setMotorCommandFactor(f);
	//					std::cout<<"gerade:   ";
	//				}
	//			}
	//		} else {
	//			std::vector<double> f;// = homeokinetic_controller->getMotorCommandFactor();
	//			f.push_back(1.0);
	//			f.push_back(1.0);
	//			f.push_back(1.0);
	//			f.push_back(1.0);
	//			homeokinetic_controller->setMotorCommandFactor(f);
	//			std::cout<<"gerade (no steering control)   ";
	//
	//		}


	//	// manually defined steering
	//	int state = calc_current_state(alpha.at(active_qlearner), distance.at(active_qlearner),x_hom);
	//		if (state==2){ //Kugel rechts ?!
	//			std::vector<double> f;// = homeokinetic_controller->getMotorCommandFactor();
	//			f.push_back(1.0);
	//			f.push_back(0.5);
	//			f.push_back(1.0);
	//			f.push_back(0.5);
	//			homeokinetic_controller->setMotorCommandFactor(f);
	//	//		std::cout<<"rechts:   ";
	//		} else {
	//			if (state==0){ //Kugel links ?!
	//				std::vector<double> f;
	//				f.push_back(0.5);
	//				f.push_back(1.0);
	//				f.push_back(0.5);
	//				f.push_back(1.0);
	//				homeokinetic_controller->setMotorCommandFactor(f);
	//		//		std::cout<<"links:   ";
	//			} else {
	//				std::vector<double> f;// = homeokinetic_controller->getMotorCommandFactor();
	//				f.push_back(1.0);
	//				f.push_back(1.0);
	//				f.push_back(1.0);
	//				f.push_back(1.0);
	//				homeokinetic_controller->setMotorCommandFactor(f);
	//			//	std::cout<<"gerade:   ";
	//			}
	//		}






/*
	// pure homeokinetic control (pre- and post-processing factor =1)
	std::vector<double> f1;
	f1.push_back(1.0);
	homeokinetic_controller.at(0)->setMotorCommandFactor(f1);
	homeokinetic_controller.at(1)->setMotorCommandFactor(f1);
	homeokinetic_controller.at(2)->setMotorCommandFactor(f1);
	homeokinetic_controller.at(3)->setMotorCommandFactor(f1);
*/




	// assuming here that homeokinesis controls all sensors
	// QLearning just modulates amplitudes/setpoints of motor commands in openinvertnchannelcontroller

	if (one_controller) {

		homeokinetic_controller.at(0)->step(x_homeok,number_sensors_homeokinetic , y_, number_sensors_homeokinetic);
	} else {

		for (int i=0; i<(int)homeokinetic_controller.size(); i++){
			homeokinetic_controller.at(i)->step(&x_homeok[i], 1, &y_[i], 1);
		}
	}

//	std::cout<<y_[0]<<"  "<<y_[1]<<"  "<<y_[2]<<"  "<<y_[3]<<"  "<<std::endl;


/*
	ofstream myfile3;
	myfile3.open ("motor_commands.dat",ios::out | ios::app);
	myfile3 <<abs(y_[0])<<std::endl;
	myfile3.close();
*/


	// manual steering
	/*
	for (int i=0;i<4;i++)
		y_[i]=mc[i];
	std::cout<<y_[0]<<"  "<<y_[1]<<"  "<<y_[2]<<"  "<<y_[3]<<"  "<<std::endl;
	 */


/*

		// manually defined steering
		int state = calc_current_state(alpha.at(active_qlearner), distance.at(active_qlearner),x_hom);
			if (state==2){ //Kugel rechts ?!
				y_[0]=0.6;
				y_[1]=0.3;
				y_[2]=0.6;
				y_[3]=0.3;
		//		std::cout<<"rechts:   ";
			} else {
				if (state==0){ //Kugel links ?!
					y_[0]=0.3;
					y_[1]=0.6;
					y_[2]=0.3;
					y_[3]=0.6;
			//		std::cout<<"links:   ";
				} else {
					y_[0]=0.6;
					y_[1]=0.6;
					y_[2]=0.6;
					y_[3]=0.6;
				//	std::cout<<"gerade:   ";
				}
			}
*/


	if (t> 13000){
//		std::cout<<"inversion"<<std::endl;
		y_[1]=-y_[1];
		//y_[2]=-y_[2];
	}

}

void QLearningHomeokMController::stepNoLearning(const sensor* x_, int number_sensors,
		motor* y_, int number_motors)
{

}


unsigned int QLearningHomeokMController::calc_current_state(
		double alpha, double distance, std::vector<double> x_hom){

	//	//TODO: put this into state (forward/backward)
	//	(sign(x_[0])==sign(x_[4]))

	/*
	// calculate current state
	// 3 states assumed: goal to the left, goal to the right, goal straight on (= no goal)
	unsigned int current_state=1; // nothing seen or seen directly in front
	if (x_[5]<-10/*0.1* /) current_state=0; //left
	if (x_[5]> 10/*0.1* /) current_state=2; //right
	 */

	// states based on angle
	// 3 states assumed: goal to the left, goal to the right, goal straight on (= no goal)

	// mit 2 states klappt es gut ;-)
	// unsigned int current_state=0; // nothing seen

	unsigned int current_state=1; // nothing seen
	if ( /*(alpha>-45) &&*/ (alpha</*-3*/-5) ) {//left
		current_state=0;
		//if (x_[4]<0) current_state =3/*2*/; //if behind robot
	}
	if ( /*(alpha<45) &&*/ (alpha> /*3*/5) ) {//right
		current_state=2;
		//if (x_[4]<0) current_state =3/*0*/; //if behind robot
	}
	//		if ( (alpha<=10) && (alpha>= -10) ) {//directly in front
	//			current_state=1;
	//		}

	return current_state;
}

double QLearningHomeokMController::calc_current_reward(
		double alpha, double old_alpha, double distance, double old_distance, std::vector<double> x_hom){

	double reward=0;

	// reward based on sensors
	bool sensor_based_reward =true;
	if (sensor_based_reward){
		//reward = abs(1/alpha);
		//reward=0.1;
		if ((alpha < /*5*/3)&&(alpha > /*-5*/-3)){
			/*
			reward = 1;
			//reward+= 1/ sqrt(x_[4]*x_[4]*x_[5]*x_[5]);
			if (old_action==1){
				reward+=1.0;
			}
			 */
			//TEST  	reward=(old_distance-distance);// >0 if robot comes closer

			//reward=1;

			//	std::vector<double> f = homeokinetic_controller->getMotorCommandFactor();
			//    reward+= pow((abs(x_hom[0]*f.at(0)+x_hom[1]*f.at(1)+x_hom[2]*f.at(2)+x_hom[3]*f.at(3))/4 -0.6),2);

			//reward+=1;
			//			if (abs(old_alpha-alpha)<5){
			//				reward+=1.0;
			//			}
			// add abs(sum of all measured wheel velocities (x_homeo)) => should be highest for straight movement
			//			std::vector<double> f = homeokinetic_controller->getMotorCommandFactor();
			//			reward+=abs(x_hom[0]*f.at(0)+x_hom[1]*f.at(1)+x_hom[2]*f.at(2)+x_hom[3]*f.at(3))/(4*0.6);
			//reward += 1/(old_alpha-alpha);
			//			if (sign(old_alpha)<3){
			//				reward+=1;
			//			}
			//			if (/*(old_action==1) &&*/ (sign(alpha)<2) && (sign(old_alpha)<2)){
			//				reward+=1;
			//			}

			//reward+=1/(sign(alpha)+sign(old_alpha)+0.1);

			//  reward+= - 0.1* (abs(alpha))/3;
		} else{
			//reward+= - (abs(alpha)-3);
			// reward+= -0.5;

		}
		//reward+=abs(x_hom[0]+x_hom[1]+x_hom[2]+x_hom[3])/(4*0.6);
		//if (reward>1) reward=0; //don't consider if too close

		//GOOD
		//		std::vector<double> f = homeokinetic_controller->getMotorCommandFactor();
		//		reward+= 0.1*(abs(x_hom[0]*f.at(0)+x_hom[1]*f.at(1)+x_hom[2]*f.at(2)+x_hom[3]*f.at(3))/4 -0.6);


		//reward+=old_distance-distance;

		//			// just adding 1/sensors
		//			reward = /*abs(1/x_[4]) +*/ abs(1/x_[5]);
		//			if (abs(x_[5])<1) reward=0; //don't consider if too close
		////std::cout<<"reward="<<reward<<std::endl;

		/*// truncating
		double r1 = abs(1/x_[4]); // distance
		if (r1>10) r1=10;
		double r2 = abs(1/x_[5]); // only steering
		if (r2>10) r2=10;
		reward = r1 + r2;
		 */

		/*//
		reward=0;
		if ( (abs(x_[5])<3) &&  (old_action == 1) ){
			reward = 1;
		}
		 */
	}
	/*
	// reward based on state
	bool state_based_reward =false;
	if (state_based_reward){
		reward=0;
		if (current_state == 1)	reward = 1; // goal is in front, or no goal and robot does not steer
	}
	 */

	//	if ( (old_state.at(active_qlearner)==1) && (old_action==1)){
	//		reward=1;
	//	} else {
	//		reward=-0.1;
	//	}

	if ((abs(alpha)<3) && (abs(old_alpha)<3) ){
		reward=1.0;
		//std::cout<<std::endl<<alpha<<"  "<<old_alpha<<"  "<<reward<<std::endl<<std::endl;
	}


	if (alpha>6){
		if (alpha>old_alpha){
			reward-=1;//0.5;//0.5;
		}
	}

	if (alpha<-6){
		if (alpha<old_alpha){
			reward-=1;//0.5;//0.5;
		}
	}



	return reward;
}
