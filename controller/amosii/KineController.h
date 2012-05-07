/*
 * KineController.h
 *
 *  Created on: Oct 24, 2011
 *      Author: Ren Guanjiao
 */

#ifndef KINECONTROLLER_H_
#define KINECONTROLLER_H_


#include <vector>
#include <cmath>

//#include "sensor_motor_definition.h"
#include <ode_robots/amosiisensormotordefinition.h>

#include <assert.h>
#include <stdlib.h>

//Save files
#include <iostream>
#include <fstream>
#include <string.h>
using namespace std;
//Save files

#define PI 3.14159265
#define STRIDE 100
#define LEGH 30
#define MAXNUM 30
#define STEP 10

// Class for ChaosControl and PostProcessing
class KineController
{
public:
	KineController(int amos_version);
	~KineController();

	double pA_pD_Z_RF;
	double pA_pD_Z_RM;
	double pA_pD_Z_RH;
	double pA_pD_Z_LF;
	double pA_pD_Z_LM;
	double pA_pD_Z_LH;

	std::vector<double> out;

	void Kine();
	std::vector<double> step_nlm(const std::vector<double> in0);


	void startWalking(){
		std::cout<<"start function"<<std::endl;
		walking = true;
		not_walking = false;
		lift_leg = false;
		lower_leg = false;
		come_back = false;
		not_come_back = false;

		//----
		turn_right = false;
		turn_left = false;
		sideward_right = false;
		sideward_left = false;
		turnright_sideward = false;
		turnleft_sideward = false;
		backward_left = false;
		backward_right = false;
	}
	void stopWalking(){
		walking = false;
		not_walking = true;
		lift_leg = false;
		lower_leg = false;
		come_back = false;
		not_come_back = false;

		//----
		turn_right = false;
		turn_left = false;
		sideward_right = false;
		sideward_left = false;
		turnright_sideward = false;
		turnleft_sideward = false;
		backward_left = false;
		backward_right = false;
	}

	void liftLeg(){
		walking = false;
		not_walking = false;
		lift_leg = true;
		lower_leg = false;
		come_back = false;
		not_come_back = false;

		//----
		turn_right = false;
		turn_left = false;
		sideward_right = false;
		sideward_left = false;
		turnright_sideward = false;
		turnleft_sideward = false;
		backward_left = false;
		backward_right = false;
	}

	void lowerLeg(){
		walking = false;
		not_walking = false;
		lift_leg = false;
		lower_leg = true;
		come_back = false;
		not_come_back = false;

		//----
		turn_right = false;
		turn_left = false;
		sideward_right = false;
		sideward_left = false;
		turnright_sideward = false;
		turnleft_sideward = false;
		backward_left = false;
		backward_right = false;

	}

	void comeBack(){ // lowering leg and walk backwards
		walking = false;
		not_walking = false;
		lift_leg = false;
		lower_leg = false;
		come_back = true;
		not_come_back = false;

		//----
		turn_right = false;
		turn_left = false;
		sideward_right = false;
		sideward_left = false;
		turnright_sideward = false;
		turnleft_sideward = false;
		backward_left = false;
		backward_right = false;
	}

	void stopComeBack(){ // lowering leg and walk backwards
		walking = false;
		not_walking = false;
		lift_leg = false;
		lower_leg = false;
		come_back = false;
		not_come_back = true;

		//----
		turn_right = false;
		turn_left = false;
		sideward_right = false;
		sideward_left = false;
		turnright_sideward = false;
		turnleft_sideward = false;
		backward_left = false;
		backward_right = false;
	}

	//------------------new
	void turnLeft(){ // lowering leg and walk backwards
		walking = false;
		not_walking = false;
		lift_leg = false;
		lower_leg = false;
		come_back = false;
		not_come_back = false;
		//----
		turn_right = false;
		turn_left = true;
		sideward_right = false;
		sideward_left = false;
		turnright_sideward = false;
		turnleft_sideward = false;
		backward_left = false;
		backward_right = false;
	}

	void turnRight(){ // lowering leg and walk backwards
		walking = false;
		not_walking = false;
		lift_leg = false;
		lower_leg = false;
		come_back = false;
		not_come_back = false;

		//----
		turn_right = true;
		turn_left = false;
		sideward_right = false;
		sideward_left = false;
		turnright_sideward = false;
		turnleft_sideward = false;
		backward_left = false;
		backward_right = false;
	}

	void sidewardright(){ // lowering leg and walk backwards
		walking = false;
		not_walking = false;
		lift_leg = false;
		lower_leg = false;
		come_back = false;
		not_come_back = false;

		//----
		turn_right = false;
		turn_left = false;
		sideward_right = true;
		sideward_left = false;
		turnright_sideward = false;
		turnleft_sideward = false;
		backward_left = false;
		backward_right = false;
	}

	void sidewardleft(){ // lowering leg and walk backwards
		walking = false;
		not_walking = false;
		lift_leg = false;
		lower_leg = false;
		come_back = false;
		not_come_back = false;

		//----
		turn_right = false;
		turn_left = false;
		sideward_right = false;
		sideward_left = true;
		turnright_sideward = false;
		turnleft_sideward = false;
		backward_left = false;
		backward_right = false;
	}

	void turnrightsideward(){ // lowering leg and walk backwards
		walking = false;
		not_walking = false;
		lift_leg = false;
		lower_leg = false;
		come_back = false;
		not_come_back = false;

		//----
		turn_right = false;
		turn_left = false;
		sideward_right = false;
		sideward_left = false;
		turnright_sideward = true;
		turnleft_sideward = false;
		backward_left = false;
		backward_right = false;
	}

	void turnleftsideward(){ // lowering leg and walk backwards
		walking = false;
		not_walking = false;
		lift_leg = false;
		lower_leg = false;
		come_back = false;
		not_come_back = false;

		//----
		turn_right = false;
		turn_left = false;
		sideward_right = false;
		sideward_left = false;
		turnright_sideward = false;
		turnleft_sideward = true;
		backward_left = false;
		backward_right = false;
	}

	void backwardleft(){ // lowering leg and walk backwards
		walking = false;
		not_walking = false;
		lift_leg = false;
		lower_leg = false;
		come_back = false;
		not_come_back = false;

		//----
		turn_right = false;
		turn_left = false;
		sideward_right = false;
		sideward_left = false;
		turnright_sideward = false;
		turnleft_sideward = false;
		backward_left = true;
		backward_right = false;
	}

	void backwardright(){ // lowering leg and walk backwards
		walking = false;
		not_walking = false;
		lift_leg = false;
		lower_leg = false;
		come_back = false;
		not_come_back = false;

		//----
		turn_right = false;
		turn_left = false;
		sideward_right = false;
		sideward_left = false;
		turnright_sideward = false;
		turnleft_sideward = false;
		backward_left = false;
		backward_right = true;
	}

protected:
	double l1,l2,l3;
	double H; //height of body
	int n; //number of steps
	int count;
	int step;
	int step2;
	bool swingflag;

	bool walking;
	bool not_walking;
	bool lift_leg;
	bool lower_leg;
	bool come_back;
	bool not_come_back;
	bool turnleft;

	//----
	bool turn_right;
	bool turn_left;
	bool sideward_right;
	bool sideward_left;
	bool turnright_sideward;
	bool turnleft_sideward;
	bool backward_left;
	bool backward_right;

	typedef struct
	{
		double X,Y,Z;
	}Point;
	Point legend_RF,legend_RM,legend_RH,legend_LF,legend_LM,legend_LH;
	Point legend_pre_RF,legend_pre_RM,legend_pre_RH,legend_pre_LF,legend_pre_LM,legend_pre_LH;
	Point desire;

	typedef struct
	{
		double TC,CTr,FTi;
	}legangle;
	legangle RF,RM,RH,LF,LM,LH;
	legangle RF_ini,RM_ini,RH_ini,LF_ini,LM_ini,LH_ini; 		//initial angle of different legs
  legangle RF_old,RM_old,RH_old,LF_old,LM_old,LH_old;

	void SingleLegInverKine_RF(double x,double y,double z);
	void SingleLegInverKine_RM(double x,double y,double z);
	void SingleLegInverKine_RH(double x,double y,double z);
	void SingleLegInverKine_LF(double x,double y,double z);
	void SingleLegInverKine_LM(double x,double y,double z);
	void SingleLegInverKine_LH(double x,double y,double z);
};

#endif //KINECONTROLLER_H_
