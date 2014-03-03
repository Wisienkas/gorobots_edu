




#include "hexapod_neurocontroller.h"
#include <cmath>
#include <iostream>
#include <ode_robots/amosiisensormotordefinition.h>

using namespace std;


HexapodNeuroMotionGenerator::HexapodNeuroMotionGenerator()
{
	WeightH1_H1 = 1.5;
	WeightH2_H2 = 1.5;
	WeightH1_H2 = 0.4;
	WeightH2_H1 = -0.4;
	fact = 0.3; //0.7;
	direction = -1;
	bias = 0.0; //negative is legs up


	outputH1 = 0.001;
	outputH2 = 0.001;
}


void HexapodNeuroMotionGenerator::motor_command_interpreter(double motor_comamnd, double* y_)
{
	double activityH1 = WeightH1_H1 * outputH1 + WeightH1_H2 * outputH2
		  + 0.01;
	double activityH2 = WeightH2_H2 * outputH2 + WeightH2_H1 * outputH1
	      + 0.01;

	outputH1 = tanh(activityH1);
	outputH2 = tanh(activityH2);

	double wave_bais = outputH2 * fact + bias;
	double wave_direction = outputH1 * fact * direction;


	double Rscale = 1;
	double Lscale = 1;

	//cout<<"motor command:"<<motor_comamnd<<endl;
	if (motor_comamnd > 0)
		Rscale -= (motor_comamnd);
	else if (motor_comamnd < 0)
			Lscale += (motor_comamnd);

	  // generate motor commands
	  // right rear coxa (knee) forward-backward joint (back is positive)
	  y_[TR2_m] = Rscale*wave_bais;
	  y_[CR2_m] = Rscale*wave_direction;
	  y_[FR2_m] = -Rscale*y_[1];
	  //left rear coxa (knee) forward-backward joint
	  y_[TL2_m] = -Lscale*wave_bais;
	  y_[CL2_m] = -Lscale*wave_direction;
	  y_[FL2_m] = -Lscale*y_[4];
	  //right middle coxa (knee) forward-backward joint
	  y_[TR1_m] = -Rscale*wave_bais;
	  y_[CR1_m] = -Rscale*wave_direction;
	  y_[FR1_m] = -Rscale*y_[7];
	  //left middle coxa (knee) forward-backward joint
	  y_[TL1_m] = Lscale*wave_bais;
	  y_[CL1_m] = Lscale*wave_direction;
	  y_[FL1_m] = -Lscale*y_[10];
	  //right front coxa (knee) forward-backward joint
	  y_[TR0_m] = Rscale*wave_bais;
	  y_[CR0_m] = Rscale*wave_direction;
	  y_[FR0_m] = -Rscale*y_[13];
	  //left front coxa (knee) forward-backward joint
	  y_[TL0_m] = -Lscale*wave_bais;
	  y_[CL0_m] = -Lscale*wave_direction;
	  y_[FL0_m] = -Lscale*y_[16];
}
