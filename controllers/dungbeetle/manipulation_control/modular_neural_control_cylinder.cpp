#include "modular_neural_control_cylinder.h"

Modular_neural_control_cylinder::Modular_neural_control_cylinder(): AbstractController("1", "1"){
	
	//Clipping varibles
	psn4_low = -0.2544;
	vrn1_low = -0.4897;
	
	//Moving Average variables
	movingAverageL2.resize(20,0.0);
	movingAverageR2.resize(20,0.0);
	sum = 0.0;
	
	//Counters
	startCounter = 0;
	counter = 0;
	timestepcount = 0;
	timestepcount2 = 0;

	//Changeable joints
	BJ = 0;
	TL0 = 0;
	TR0 = 0;
	TL1 = 0;
	TR1 = 0;
	TL2 = 0;
	TR2 = 0;
	CL0 = 0;
	CR0 = 0;
	CL1 = 0;
	CR1 = 0;
	CL2 = 0;
	CR2 = 0;
	FL0 = 0;
	FR0 = 0;
	FL1 = 0;
	FR1 = 0;
	FL2 = 0;
	FR2 = 0;

	//CPG variables
	outputH1 = 0.001;
	outputH2 = 0.001;
	outputH2reverse = 0;
	lastoutputH2 = 1;
	lastoutputH2reverse = -1;
	activityH1 = 0;
	activityH2 = 0;
	biasH1 = 0.01;
	biasH2 = 0.01;
	weightH1_H1 = 1.5;
	weightH2_H2 = 1.5;
	weightH1_H2 = 0.8;
	weightH2_H1 = -0.8;

	//Booleans
	start = false;
	legstart = true;
	push = false;
	finished = false;
	bump = false;
	climbed = false;
	push_over_step = false;
	stationary_push = false;
	customise = false;
	boxing = false;
	
	//Changeable parameters for joints
	addParameter("BJ", &BJ,-1.0,1.0);
	addParameter("TL0", &TL0,-1.0,1.0);
	addParameter("TR0", &TR0,-1.0,1.0);
	addParameter("TL1", &TL1,-1.0,1.0);
	addParameter("TR1", &TR1,-1.0,1.0);
	addParameter("TL2", &TL2,-1.0,1.0);
	addParameter("TR2", &TR2,-1.0,1.0);
	addParameter("CL0", &CL0,-1.0,1.0);
	addParameter("CR0", &CR0,-1.0,1.0);
	addParameter("CL1", &CL1,-1.0,1.0);
	addParameter("CR1", &CR1,-1.0,1.0);
	addParameter("CL2", &CL2,-1.0,1.0);
	addParameter("CR2", &CR2,-1.0,1.0);
	addParameter("FL0", &FL0,-1.0,1.0);
	addParameter("FR0", &FR0,-1.0,1.0);
	addParameter("FL1", &FL1,-1.0,1.0);
	addParameter("FR1", &FR1,-1.0,1.0);
	addParameter("FL2", &FL2,-1.0,1.0);
	addParameter("FR2", &FR2,-1.0,1.0);

  //Input variables
  inputs.resize(5);
  inputs.at(1) = 0;
  inputs.at(2) = 1;
  inputs.at(3) = 0.0;
  inputs.at(4) = 0.0;
  
  /*********** Initialize VRN ************/
  
  vrn1_activity.resize(14);
  vrn2_activity.resize(14);
  vrn1_output.resize(14);
  vrn2_output.resize(14);

  vrn1_w.resize(14);
  vrn2_w.resize(14);
  for(unsigned int i=0; i<vrn1_w.size(); i++)
  {
    vrn1_w.at(i).resize(14);
	vrn2_w.at(i).resize(14);
  }
  vrn1_bias.resize(3);
  vrn2_bias.resize(2);
  
  //---VRN weights	
  vrn1_input0_w = -0.57;
  vrn2_input0_w = 0.3;
  vrn2_input1_w = 1;
  
  vrn1_w.at(4).at(0) =    1.7246;
  vrn1_w.at(4).at(1) =    1.7246;
  vrn1_w.at(5).at(0) =   -1.7246;
  vrn1_w.at(5).at(1) =   -1.7246;
  vrn1_w.at(6).at(0) =    1.7246;
  vrn1_w.at(6).at(1) =   -1.7246;
  vrn1_w.at(7).at(0) =   -1.7246;
  vrn1_w.at(7).at(1) =    1.7246;
  vrn1_w.at(8).at(2) =    1.7246;
  vrn1_w.at(8).at(3) =    1.7246;
  vrn1_w.at(9).at(2) =   -1.7246;
  vrn1_w.at(9).at(3) =   -1.7246;
  vrn1_w.at(10).at(2) =   1.7246;
  vrn1_w.at(10).at(3) =  -1.7246;
  vrn1_w.at(11).at(2) =  -1.7246;
  vrn1_w.at(11).at(3) =   1.7246;
  vrn1_w.at(12).at(4) =   0.5;
  vrn1_w.at(12).at(5) =   0.5;
  vrn1_w.at(12).at(6) =  -0.5;
  vrn1_w.at(12).at(7) =  -0.5;
  vrn1_w.at(13).at(8) =   0.5;
  vrn1_w.at(13).at(9) =   0.5;
  vrn1_w.at(13).at(10) = -0.5;
  vrn1_w.at(13).at(11) = -0.5;
  
  vrn2_w.at(4).at(0) =    1.7246;
  vrn2_w.at(4).at(1) =    1.7246;
  vrn2_w.at(5).at(0) =   -1.7246;
  vrn2_w.at(5).at(1) =   -1.7246;
  vrn2_w.at(6).at(0) =    1.7246;
  vrn2_w.at(6).at(1) =   -1.7246;
  vrn2_w.at(7).at(0) =   -1.7246;
  vrn2_w.at(7).at(1) =    1.7246;
  vrn2_w.at(8).at(2) =    1.7246;
  vrn2_w.at(8).at(3) =    1.7246;
  vrn2_w.at(9).at(2) =   -1.7246;
  vrn2_w.at(9).at(3) =   -1.7246;
  vrn2_w.at(10).at(2) =   1.7246;
  vrn2_w.at(10).at(3) =  -1.7246;
  vrn2_w.at(11).at(2) =  -1.7246;
  vrn2_w.at(11).at(3) =   1.7246;
  vrn2_w.at(12).at(4) =   0.5;
  vrn2_w.at(12).at(5) =   0.5;
  vrn2_w.at(12).at(6) =  -0.5;
  vrn2_w.at(12).at(7) =  -0.5;
  vrn2_w.at(13).at(8) =   0.5;
  vrn2_w.at(13).at(9) =   0.5;
  vrn2_w.at(13).at(10) = -0.5;
  vrn2_w.at(13).at(11) = -0.5;
  
  //network bias
  vrn1_bias.at(2) = -2.48285;
  vrn2_bias.at(0) = 0.7;
  vrn2_bias.at(1) = -2.48285;
  
  /*********** Initialize PSN ************/
  
  psn1_inputs.resize(3);
  psn2_inputs.resize(3);
  psn3_inputs.resize(3);
  psn4_inputs.resize(3);
  psn1_activity.resize(12);
  psn2_activity.resize(12);
  psn3_activity.resize(12);
  psn4_activity.resize(12);
  psn1_output.resize(12);
  psn2_output.resize(12);
  psn3_output.resize(12);
  psn4_output.resize(12);
  
  psn1_control_w.resize(2);
  psn2_control_w.resize(2);
  psn3_control_w.resize(2);
  psn4_control_w.resize(2);
  
  psn1_input_w.resize(14);
  psn2_input_w.resize(14);
  psn3_input_w.resize(14);
  psn4_input_w.resize(14);
  for(unsigned int i=0; i<psn1_input_w.size(); i++)
  {
    psn1_input_w.at(i).resize(14);
	psn2_input_w.at(i).resize(14);
	psn3_input_w.at(i).resize(14);
	psn4_input_w.at(i).resize(14);
  }
  
  psn1_w.resize(12);
  psn2_w.resize(12);
  psn3_w.resize(12);
  psn4_w.resize(12);
  for(unsigned int i=0; i<psn1_w.size(); i++)
  {
    psn1_w.at(i).resize(12);
	psn2_w.at(i).resize(12);
	psn3_w.at(i).resize(12);
	psn4_w.at(i).resize(12);
  }
  psn1_bias.resize(4);
  psn2_bias.resize(4);
  psn3_bias.resize(4);
  psn4_bias.resize(4);
  
  //---PSN weights   
  psn1_input_w.at(2).at(0) = 0.5;
  psn1_input_w.at(3).at(1) = 0.5;
  psn1_input_w.at(4).at(1) = 0.5;
  psn1_input_w.at(5).at(0) = 0.5;
  psn2_input_w.at(2).at(0) = 0.5;
  psn2_input_w.at(3).at(1) = 0.5;
  psn2_input_w.at(4).at(1) = 0.5;
  psn2_input_w.at(5).at(0) = 0.5;
  psn3_input_w.at(2).at(0) = 0.5;
  psn3_input_w.at(3).at(1) = 0.5;
  psn3_input_w.at(4).at(1) = 0.5;
  psn3_input_w.at(5).at(0) = 0.5;
  psn4_input_w.at(2).at(0) = 0.5;
  psn4_input_w.at(3).at(1) = 0.5;
  psn4_input_w.at(4).at(1) = 0.5;
  psn4_input_w.at(5).at(0) = 0.5;
  
  psn1_w.at(2).at(0) = -5.0;
  psn1_w.at(3).at(1) = -5.0;
  psn1_w.at(4).at(0) = -5.0;
  psn1_w.at(5).at(1) = -5.0;
  psn1_w.at(6).at(2) =  0.5;
  psn1_w.at(7).at(3) =  0.5;
  psn1_w.at(8).at(4) =  0.5;
  psn1_w.at(9).at(5) =  0.5;
  psn1_w.at(10).at(6) = 3.0;
  psn1_w.at(10).at(7) = 3.0;
  psn1_w.at(11).at(8) = 3.0;
  psn1_w.at(11).at(9) = 3.0;
  psn2_w.at(2).at(0) = -5.0;
  psn2_w.at(3).at(1) = -5.0;
  psn2_w.at(4).at(0) = -5.0;
  psn2_w.at(5).at(1) = -5.0;
  psn2_w.at(6).at(2) =  0.5;
  psn2_w.at(7).at(3) =  0.5;
  psn2_w.at(8).at(4) =  0.5;
  psn2_w.at(9).at(5) =  0.5;
  psn2_w.at(10).at(6) = 3.0;
  psn2_w.at(10).at(7) = 3.0;
  psn2_w.at(11).at(8) = 3.0;
  psn2_w.at(11).at(9) = 3.0;
  psn3_w.at(2).at(0) = -5.0;
  psn3_w.at(3).at(1) = -5.0;
  psn3_w.at(4).at(0) = -5.0;
  psn3_w.at(5).at(1) = -5.0;
  psn3_w.at(6).at(2) =  0.5;
  psn3_w.at(7).at(3) =  0.5;
  psn3_w.at(8).at(4) =  0.5;
  psn3_w.at(9).at(5) =  0.5;
  psn3_w.at(10).at(6) = 3.0;
  psn3_w.at(10).at(7) = 3.0;
  psn3_w.at(11).at(8) = 3.0;
  psn3_w.at(11).at(9) = 3.0;
  psn4_w.at(2).at(0) = -5.0;
  psn4_w.at(3).at(1) = -5.0;
  psn4_w.at(4).at(0) = -5.0;
  psn4_w.at(5).at(1) = -5.0;
  psn4_w.at(6).at(2) =  0.5;
  psn4_w.at(7).at(3) =  0.5;
  psn4_w.at(8).at(4) =  0.5;
  psn4_w.at(9).at(5) =  0.5;
  psn4_w.at(10).at(6) = 3.0;
  psn4_w.at(10).at(7) = 3.0;
  psn4_w.at(11).at(8) = 3.0;
  psn4_w.at(11).at(9) = 3.0;
  
  //network bias
  psn1_bias.at(0) =  1;
  psn1_bias.at(1) =  1;
  psn1_bias.at(2) =  0.5;
  psn1_bias.at(3) = -1.35;
  psn2_bias.at(0) =  1;
  psn2_bias.at(1) =  1;
  psn2_bias.at(2) =  0.5;
  psn2_bias.at(3) = -1.35;
  psn3_bias.at(0) =  1;
  psn3_bias.at(1) =  1;
  psn3_bias.at(2) =  0.5;
  psn3_bias.at(3) = -1.35;
  psn4_bias.at(0) =  1;
  psn4_bias.at(1) =  1;
  psn4_bias.at(2) =  0.5;
  psn4_bias.at(3) = -1.35;
  
  //Inputs to PSN
  psn1_inputs.at(0) = &inputs.at(1);
  psn1_inputs.at(1) = &outputH2reverse;
  psn1_inputs.at(2) = &outputH1;
 
  psn2_inputs.at(0) = &inputs.at(1);
  psn2_inputs.at(1) = &outputH2reverse;
  psn2_inputs.at(2) = &outputH1;
 
  psn3_inputs.at(0) = &inputs.at(1);
  psn3_inputs.at(1) = &psn1_output.at(11);
  psn3_inputs.at(2) = &psn1_output.at(10);
 
  psn4_inputs.at(0) = &inputs.at(1);
  psn4_inputs.at(1) = &psn1_output.at(11);
  psn4_inputs.at(2) = &psn1_output.at(10);
  
  //inputs to VRN 
  vrn1_input0 = &inputs.at(3);
  vrn1_input1_0 = &outputH2reverse;
  vrn1_input1_1 = &psn1_output.at(10);

  vrn2_input0 = &inputs.at(2);
  vrn2_input1 = &psn2_output.at(10);
 }

void Modular_neural_control_cylinder::init(int sensornumber, int motornumber, RandGen* randGen){
	number_sensors = sensornumber;
    number_motors = motornumber;
}

double Modular_neural_control_cylinder::jointAngleScaling(double new_min, double new_max, double old_min, double old_max, double old_value){

	double old_range = old_max - old_min;
	double new_range = new_max - new_min;
	
	double new_value = ( ( (old_value - old_min) * new_range) / old_range) + new_min;
	return (new_value);
}

double Modular_neural_control_cylinder::setJointDegrees(double degree){

	double radians = M_PI / 180 * degree;
	return (radians);
}

void Modular_neural_control_cylinder::setStandConfiguration(motor* motors){

	if (customise)
	{
		//left front
		motors[TL0_m] = TL0;
		motors[CL0_m] = CL0;
		motors[FL0_m] = FL0;

		//right front
		motors[TR0_m] = TR0;
		motors[CR0_m] = CR0;
		motors[FR0_m] = FR0;

		//left middle
		motors[TL1_m] = TL1;
		motors[CL1_m] = CL1;
		motors[FL1_m] = FL1;

		//right middle
		motors[TR1_m] = TR1;
		motors[CR1_m] = CR1;
		motors[FR1_m] = FR1;

		// left back
		motors[TL2_m] = TL2;
		motors[CL2_m] = CL2;
		motors[FL2_m] = FL2;

		// right back
		motors[TR2_m] = TR2;
		motors[CR2_m] = CR2;
		motors[FR2_m] = FR2;

		// backbone joint points downwards
		motors[BJ_m] = BJ;
	}
	else
	{
		//left front
		motors[TL0_m] = setJointDegrees(-15);
		motors[CL0_m] = setJointDegrees(-5);
		motors[FL0_m] = setJointDegrees(35);

		//right front
		motors[TR0_m] = setJointDegrees(-15);
		motors[CR0_m] = setJointDegrees(-5);
		motors[FR0_m] = setJointDegrees(35);

		//left middle
		motors[TL1_m] = setJointDegrees(40);
		motors[CL1_m] = setJointDegrees(20);
		motors[FL1_m] = setJointDegrees(0);

		//right middle
		motors[TR1_m] = setJointDegrees(40);
		motors[CR1_m] = setJointDegrees(20);
		motors[FR1_m] = setJointDegrees(0);

		// left back
		motors[TL2_m] = setJointDegrees(40);
		motors[CL2_m] = setJointDegrees(-20);
		motors[FL2_m] = setJointDegrees(25);

		// right back
		motors[TR2_m] = setJointDegrees(40);
		motors[CR2_m] = setJointDegrees(-20);
		motors[FR2_m] = setJointDegrees(25);
	
		// backbone joint points downwards
		motors[BJ_m] = M_PI / 180 * 10.0;
	}
}

void Modular_neural_control_cylinder::MovingAverage(const sensor* sensors){
	
	sum = 0.0;

	movingAverageL2.erase(movingAverageL2.begin());
	movingAverageL2.push_back(sensors[FL0_as]); 	//Look at angle of front left febia
	movingAverageR2.erase(movingAverageR2.begin());
	movingAverageR2.push_back(sensors[FR0_as]); 	//Look at angle of front right febia
	
	for (float i = 0; i < (float)movingAverageL2.size(); i++)
	{
		sum += movingAverageL2[i] + movingAverageR2[i];
	}
	
	sum /= (float)movingAverageL2.size();
	
}

void Modular_neural_control_cylinder::UpdateCPG(){
	
	activityH1 = weightH1_H1 * outputH1 + weightH1_H2 * outputH2 + biasH1;
	activityH2 = weightH2_H2 * outputH2 + weightH2_H1 * outputH1 + biasH2;

	outputH1 = tanh(activityH1);
	outputH2 = tanh(activityH2);

	outputH2reverse = outputH1 * -1;
}

void Modular_neural_control_cylinder::UpdateVRN(){
	
	// Update VRN1
	vrn1_bias.at(0) = 1;
	vrn1_input1_1_w = (((inputs.at(2)*inputs.at(2) - inputs.at(1)*inputs.at(1)) 
					* (inputs.at(2)*inputs.at(2) - inputs.at(1)*inputs.at(1)))
					+ ((inputs.at(1) * inputs.at(2) * (inputs.at(2) + inputs.at(1)) / 2)
					* (inputs.at(1) * inputs.at(2) * (inputs.at(2) + inputs.at(1)) / 2))) * 2;
	vrn1_input1_0_w = 1 - (vrn1_input1_1_w/2.0);
	vrn1_bias.at(1) = 0.02 * (vrn1_input1_1_w/2.0);
	
	vrn1_activity.at(0) = vrn1_input0_w * *vrn1_input0 + vrn1_bias.at(0); 
	vrn1_activity.at(1) = vrn1_input1_0_w * *vrn1_input1_0 + vrn1_input1_1_w * *vrn1_input1_1 + vrn1_bias.at(1);

	vrn1_activity.at(4) = vrn1_w.at(4).at(0) * vrn1_output.at(0) + vrn1_w.at(4).at(1) * vrn1_output.at(1) + vrn1_bias.at(2);
	vrn1_activity.at(5) = vrn1_w.at(5).at(0) * vrn1_output.at(0) + vrn1_w.at(5).at(1) * vrn1_output.at(1) + vrn1_bias.at(2);
	vrn1_activity.at(6) = vrn1_w.at(6).at(0) * vrn1_output.at(0) + vrn1_w.at(6).at(1) * vrn1_output.at(1) + vrn1_bias.at(2);
	vrn1_activity.at(7) = vrn1_w.at(7).at(0) * vrn1_output.at(0) + vrn1_w.at(7).at(1) * vrn1_output.at(1) + vrn1_bias.at(2);

	vrn1_activity.at(12) = vrn1_w.at(12).at(4) * vrn1_output.at(4) + vrn1_w.at(12).at(5) * vrn1_output.at(5) 
						+ vrn1_w.at(12).at(6) * vrn1_output.at(6) + vrn1_w.at(12).at(7) * vrn1_output.at(7); 

	for(unsigned int i=0; i<vrn1_output.size();i++)
	{
		vrn1_output.at(i) = tanh(vrn1_activity.at(i));
	}

	
	// Update VRN2	
	vrn2_activity.at(0) = vrn2_input0_w * *vrn2_input0 + vrn2_bias.at(0);
	vrn2_activity.at(1) = vrn2_input1_w * *vrn2_input1;

	vrn2_activity.at(4) = vrn2_w.at(4).at(0) * vrn2_output.at(0) + vrn2_w.at(4).at(1) * vrn2_output.at(1) + vrn2_bias.at(1);
	vrn2_activity.at(5) = vrn2_w.at(5).at(0) * vrn2_output.at(0) + vrn2_w.at(5).at(1) * vrn2_output.at(1) + vrn2_bias.at(1);
	vrn2_activity.at(6) = vrn2_w.at(6).at(0) * vrn2_output.at(0) + vrn2_w.at(6).at(1) * vrn2_output.at(1) + vrn2_bias.at(1);
	vrn2_activity.at(7) = vrn2_w.at(7).at(0) * vrn2_output.at(0) + vrn2_w.at(7).at(1) * vrn2_output.at(1) + vrn2_bias.at(1);

	vrn2_activity.at(12) = vrn2_w.at(12).at(4) * vrn2_output.at(4) + vrn2_w.at(12).at(5) * vrn2_output.at(5) 
						+ vrn2_w.at(12).at(6) * vrn2_output.at(6) + vrn2_w.at(12).at(7) * vrn2_output.at(7); 

	for(unsigned int i=0; i<vrn2_output.size();i++)
	{
		vrn2_output.at(i) = tanh(vrn2_activity.at(i));
	}
}

double Modular_neural_control_cylinder::getVRN1output(){	
	return vrn1_output.at(12);
}

double Modular_neural_control_cylinder::getVRN2output(){	
	return vrn2_output.at(12);
}

void Modular_neural_control_cylinder::UpdatePSN(){
	
	// Update PSN1
	psn1_control_w.at(0) = inputs.at(1);
	psn1_control_w.at(1) = -inputs.at(1);
  
	psn1_bias.at(0) =  -(inputs.at(1)*inputs.at(1)*inputs.at(2)*(inputs.at(2)+1))/2;
	psn1_bias.at(1) =  1 + (inputs.at(1)*inputs.at(1)*inputs.at(2)*(inputs.at(2)+1))/2;
	
	psn1_activity.at(0) = psn1_control_w.at(0) * *psn1_inputs.at(0) + psn1_bias.at(0);
	psn1_activity.at(1) = psn1_control_w.at(1) * *psn1_inputs.at(0) + psn1_bias.at(1);
 
	psn1_activity.at(2) = psn1_input_w.at(2).at(0) * *psn1_inputs.at(1) + psn1_w.at(2).at(0) * psn1_output.at(0);
	psn1_activity.at(3) = psn1_input_w.at(3).at(1) * *psn1_inputs.at(2) + psn1_w.at(3).at(1) * psn1_output.at(1);
	psn1_activity.at(4) = psn1_input_w.at(4).at(1) * *psn1_inputs.at(2) + psn1_w.at(4).at(0) * psn1_output.at(0);
	psn1_activity.at(5) = psn1_input_w.at(5).at(0) * *psn1_inputs.at(1) + psn1_w.at(5).at(1) * psn1_output.at(1);

	psn1_activity.at(6) = psn1_w.at(6).at(2) * psn1_output.at(2) + psn1_bias.at(2);
	psn1_activity.at(7) = psn1_w.at(7).at(3) * psn1_output.at(3) + psn1_bias.at(2);
	psn1_activity.at(8) = psn1_w.at(8).at(4) * psn1_output.at(4) + psn1_bias.at(2);
	psn1_activity.at(9) = psn1_w.at(9).at(5) * psn1_output.at(5) + psn1_bias.at(2);

	psn1_activity.at(10) = psn1_w.at(10).at(6) * psn1_output.at(6) + psn1_w.at(10).at(7) * psn1_output.at(7) + psn1_bias.at(3); 
	psn1_activity.at(11) = psn1_w.at(11).at(8) * psn1_output.at(8) + psn1_w.at(11).at(9) * psn1_output.at(9) + psn1_bias.at(3); 

	for(unsigned int i=0; i<psn1_output.size();i++)
	{
		psn1_output.at(i) = tanh(psn1_activity.at(i));
	}
	
	
	// Update PSN2
	psn2_control_w.at(0) = -inputs.at(1);
	psn2_control_w.at(1) = inputs.at(1);
	
	psn2_activity.at(0) = psn2_control_w.at(0) * *psn2_inputs.at(0) + psn2_bias.at(0);
	psn2_activity.at(1) = psn2_control_w.at(1) * *psn2_inputs.at(0);
	 
	psn2_activity.at(2) = psn2_input_w.at(2).at(0) * *psn2_inputs.at(1) + psn2_w.at(2).at(0) * psn2_output.at(0);
	psn2_activity.at(3) = psn2_input_w.at(3).at(1) * *psn2_inputs.at(2) + psn2_w.at(3).at(1) * psn2_output.at(1);
	psn2_activity.at(4) = psn2_input_w.at(4).at(1) * *psn2_inputs.at(2) + psn2_w.at(4).at(0) * psn2_output.at(0);
	psn2_activity.at(5) = psn2_input_w.at(5).at(0) * *psn2_inputs.at(1) + psn2_w.at(5).at(1) * psn2_output.at(1);

	psn2_activity.at(6) = psn2_w.at(6).at(2) * psn2_output.at(2) + psn2_bias.at(2);
	psn2_activity.at(7) = psn2_w.at(7).at(3) * psn2_output.at(3) + psn2_bias.at(2);
	psn2_activity.at(8) = psn2_w.at(8).at(4) * psn2_output.at(4) + psn2_bias.at(2);
	psn2_activity.at(9) = psn2_w.at(9).at(5) * psn2_output.at(5) + psn2_bias.at(2);

	psn2_activity.at(10) = psn2_w.at(10).at(6) * psn2_output.at(6) + psn2_w.at(10).at(7) * psn2_output.at(7) + psn2_bias.at(3); 
	psn2_activity.at(11) = psn2_w.at(11).at(8) * psn2_output.at(8) + psn2_w.at(11).at(9) * psn2_output.at(9) + psn2_bias.at(3); 

	for(unsigned int i=0; i<psn2_output.size();i++)
	{
		psn2_output.at(i) = tanh(psn2_activity.at(i));
	}
	
	
	// Update PSN3
	psn3_control_w.at(0) = -inputs.at(1);
	psn3_control_w.at(1) = inputs.at(1);
	
	psn3_activity.at(0) = psn3_control_w.at(0) * *psn3_inputs.at(0) + psn3_bias.at(0);
	psn3_activity.at(1) = psn3_control_w.at(1) * *psn3_inputs.at(0);
 
	psn3_activity.at(2) = psn3_input_w.at(2).at(0) * *psn3_inputs.at(1) + psn3_w.at(2).at(0) * psn3_output.at(0);
	psn3_activity.at(3) = psn3_input_w.at(3).at(1) * *psn3_inputs.at(2) + psn3_w.at(3).at(1) * psn3_output.at(1);
	psn3_activity.at(4) = psn3_input_w.at(4).at(1) * *psn3_inputs.at(2) + psn3_w.at(4).at(0) * psn3_output.at(0);
	psn3_activity.at(5) = psn3_input_w.at(5).at(0) * *psn3_inputs.at(1) + psn3_w.at(5).at(1) * psn3_output.at(1);

	psn3_activity.at(6) = psn3_w.at(6).at(2) * psn3_output.at(2) + psn3_bias.at(2);
	psn3_activity.at(7) = psn3_w.at(7).at(3) * psn3_output.at(3) + psn3_bias.at(2);
	psn3_activity.at(8) = psn3_w.at(8).at(4) * psn3_output.at(4) + psn3_bias.at(2);
	psn3_activity.at(9) = psn3_w.at(9).at(5) * psn3_output.at(5) + psn3_bias.at(2);

	psn3_activity.at(10) = psn3_w.at(10).at(6) * psn3_output.at(6) + psn3_w.at(10).at(7) * psn3_output.at(7) + psn3_bias.at(3); 
	psn3_activity.at(11) = psn3_w.at(11).at(8) * psn3_output.at(8) + psn3_w.at(11).at(9) * psn3_output.at(9) + psn3_bias.at(3); 

	for(unsigned int i=0; i<psn3_output.size();i++)
	{
		psn3_output.at(i) = tanh(psn3_activity.at(i));
	}
	
	
	// Update PSN4
	psn4_control_w.at(0) = inputs.at(1);
	psn4_control_w.at(1) = -inputs.at(1);
  
	psn4_bias.at(0) =  -(inputs.at(1)*inputs.at(1)*inputs.at(2)*(inputs.at(2)+1))/2;
	psn4_bias.at(1) =  1 + (inputs.at(1)*inputs.at(1)*inputs.at(2)*(inputs.at(2)+1))/2;
 
	psn4_activity.at(0) = psn4_control_w.at(0) * *psn4_inputs.at(0) + psn4_bias.at(0);
	psn4_activity.at(1) = psn4_control_w.at(1) * *psn4_inputs.at(0) + psn4_bias.at(1);
 
	psn4_activity.at(2) = psn4_input_w.at(2).at(0) * *psn4_inputs.at(1) + psn4_w.at(2).at(0) * psn4_output.at(0);
	psn4_activity.at(3) = psn4_input_w.at(3).at(1) * *psn4_inputs.at(2) + psn4_w.at(3).at(1) * psn4_output.at(1);
	psn4_activity.at(4) = psn4_input_w.at(4).at(1) * *psn4_inputs.at(2) + psn4_w.at(4).at(0) * psn4_output.at(0);
	psn4_activity.at(5) = psn4_input_w.at(5).at(0) * *psn4_inputs.at(1) + psn4_w.at(5).at(1) * psn4_output.at(1);

	psn4_activity.at(6) = psn4_w.at(6).at(2) * psn4_output.at(2) + psn4_bias.at(2);
	psn4_activity.at(7) = psn4_w.at(7).at(3) * psn4_output.at(3) + psn4_bias.at(2);
	psn4_activity.at(8) = psn4_w.at(8).at(4) * psn4_output.at(4) + psn4_bias.at(2);
	psn4_activity.at(9) = psn4_w.at(9).at(5) * psn4_output.at(5) + psn4_bias.at(2);

	psn4_activity.at(10) = psn4_w.at(10).at(6) * psn4_output.at(6) + psn4_w.at(10).at(7) * psn4_output.at(7) + psn4_bias.at(3); 
	psn4_activity.at(11) = psn4_w.at(11).at(8) * psn4_output.at(8) + psn4_w.at(11).at(9) * psn4_output.at(9) + psn4_bias.at(3); 

	for(unsigned int i=0; i<psn4_output.size();i++)
	{
		psn4_output.at(i) = tanh(psn4_activity.at(i));
	}
	
}

std::vector<double> Modular_neural_control_cylinder::getPSN1output(){	
	std::vector<double> output;
	output.resize(2);
	
	output.at(0) = psn1_output.at(10);
	output.at(1) = psn1_output.at(11);
	
	return output;
}

std::vector<double> Modular_neural_control_cylinder::getPSN2output(){	
	std::vector<double> output;
	output.resize(2);
	
	output.at(0) = psn2_output.at(10);
	output.at(1) = psn2_output.at(11);
	
	return output;
}

std::vector<double> Modular_neural_control_cylinder::getPSN3output(){	
	std::vector<double> output;
	output.resize(2);
	
	output.at(0) = psn3_output.at(10);
	output.at(1) = psn3_output.at(11);
	
	return output;
}

std::vector<double> Modular_neural_control_cylinder::getPSN4output(){	
	std::vector<double> output;
	output.resize(2);
	
	output.at(0) = psn4_output.at(10);
	output.at(1) = psn4_output.at(11);
	
	return output;
}

void Modular_neural_control_cylinder::Clipping(){
	
	if (last_psn4_output_left > getPSN4output().at(0)) psn4_CTr_left = psn4_low;
	else if (last_psn4_output_left < getPSN4output().at(0)) 
	{
		psn4_CTr_left = getPSN4output().at(0);
	}

	if (last_psn4_output_right > getPSN4output().at(1)) psn4_CTr_right = psn4_low;
	else if (last_psn4_output_right < getPSN4output().at(1))
	{
		psn4_CTr_right = getPSN4output().at(1);
	}
	
	if (inputs.at(2) != -1)
	{
		
		if (last_vrn1_output_left > getVRN1output()) vrn1_CTr_left = vrn1_low;
		else if (last_vrn1_output_left < getVRN1output())
		{
			vrn1_CTr_left = getVRN1output();
		}

		if (last_vrn1_output_right > (-getVRN1output())) vrn1_CTr_right = vrn1_low;
		else if (last_vrn1_output_right < (-getVRN1output()))
		{
			vrn1_CTr_right = (-getVRN1output());
		}
		
		if (inputs.at(2) == 0)
		{
			vrn1_CTr_left *= -1;
			vrn1_CTr_right *= -1;
		}

	}
	else
	{
		vrn1_CTr_left = getVRN1output();
		vrn1_CTr_right = (-getVRN1output());
	}
	
	last_psn4_output_left = getPSN4output().at(0);	
	last_psn4_output_right = getPSN4output().at(1);
	last_vrn1_output_left = getVRN1output();
	last_vrn1_output_right = (-getVRN1output());
	
}

void Modular_neural_control_cylinder::step(const sensor* sensors, int sensornumber, motor* motors, int motornumber){
	
	assert(number_sensors == sensornumber);
	assert(number_motors == motornumber);	
	
	if (startCounter < 10) startCounter++;
	else if (startCounter == 10)
	{

		start = true;
		startCounter++;
	}

	UpdateCPG();
	
	UpdatePSN();
	
	UpdateVRN();
	
	Clipping();
	
	MovingAverage(sensors);
	
	
	if (legstart && !finished && start)
	{
		
		FL2_motor = inputs.at(3) + 0.7;
		FR2_motor = inputs.at(3) + 0.7;
		 
		// backbone joint
        motors[BJ_m] = M_PI / 180 * 10.0;

		//left front
        motors[TL0_m] = jointAngleScaling(-0.8, 0.8, -0.3, 0.3, getPSN3output().at(0));
        motors[CL0_m] = jointAngleScaling(-0.3, 1, -0.3, 0.3, psn4_CTr_left);
        motors[FL0_m] = 0;

        //right front
        motors[TR0_m] = jointAngleScaling(-0.8, 0.8, -0.3, 0.3, getPSN3output().at(1));
        motors[CR0_m] = jointAngleScaling(-0.3, 1, -0.3, 0.3, psn4_CTr_right);
        motors[FR0_m] = 0;

		//left middle
        motors[TL1_m] = jointAngleScaling(-0.8, 0.8, -0.3, 0.3, getPSN3output().at(0));
        motors[CL1_m] = jointAngleScaling(-0.6, 1.4, -0.3, 0.3, psn4_CTr_left);
        motors[FL1_m] = 0;

        //right middle
        motors[TR1_m] = jointAngleScaling(-0.8, 0.8, -0.3, 0.3, getPSN3output().at(1));
        motors[CR1_m] = jointAngleScaling(-0.6, 1.4, -0.3, 0.3, psn4_CTr_right);
        motors[FR1_m] = 0;
		 
		if (push)
		{
			// Check if robot has hit obstacle, else push harder
			if (!push_over_step && bump)
			{
				if ( (sum <= -0.07 && stationary_push)
					 || (sum <= -0.085 && boxing)
					 || (sum <= -0.09 && !boxing) )
				{
					timestepcount++;
					if (timestepcount >= 3)
					{
						push_over_step = true;
					}
				}
				else
				{
					timestepcount = 0;
				}
			
			}
			
				
			if (push_over_step)
			{
				
				inputs.at(1) = 1;
				inputs.at(2) = 0;
				inputs.at(3) = 0.0;
				inputs.at(4) = -0.3;
				
				// left back hard push
				motors[TL2_m] = jointAngleScaling(-0.4, 1.2, -0.2, 0.2, getVRN2output());
				motors[CL2_m] = jointAngleScaling(-0.5, 1.3, -0.5, 0.5, vrn1_CTr_left);
				motors[FL2_m] = FL2_motor;

				// right back hard push
				motors[TR2_m] = jointAngleScaling(-0.4, 1.2, -0.2, 0.2, -getVRN2output());
				motors[CR2_m] = jointAngleScaling(-0.5, 1.3, -0.5, 0.5, vrn1_CTr_right);
				motors[FR2_m] = FR2_motor;
				
			}
			else if (stationary_push)
			{
				// left back stationary push
				motors[TL2_m] = setJointDegrees(40);
				motors[CL2_m] = setJointDegrees(-20);
				motors[FL2_m] = setJointDegrees(25);
			
				// right back stationary push
				motors[TR2_m] = setJointDegrees(40);
				motors[CR2_m] = setJointDegrees(-20);
				motors[FR2_m] = setJointDegrees(25);
			}
			else
			{
				if(boxing)
				{
					inputs.at(1) = -1;
					inputs.at(2) = -1;
					inputs.at(3) = 1;
					inputs.at(4) = 0.0;
					
					// left back boxing
					motors[TL2_m] = jointAngleScaling(-0.1, 0.7, -0.1, 0.1, getVRN2output());
					motors[CL2_m] = jointAngleScaling(0.0, 0.9, -0.25, 0.25, vrn1_CTr_left);
					motors[FL2_m] = FL2_motor;

					// right back boxing
					motors[TR2_m] = jointAngleScaling(-0.1, 0.7, -0.1, 0.1, -getVRN2output());
					motors[CR2_m] = jointAngleScaling(0.0, 0.9, -0.25, 0.25, vrn1_CTr_right);
					motors[FR2_m] = FR2_motor;
				}
				else
				{
					inputs.at(1) = 1;
					inputs.at(2) = -1;
					inputs.at(3) = 1;
					inputs.at(4) = 1;
					
					// left back soft push
					motors[TL2_m] = jointAngleScaling(-1.05, -0.25, -0.1, 0.1, getVRN2output());
					motors[CL2_m] = jointAngleScaling(0.4, 1.3, -0.25, 0.25, vrn1_CTr_left);
					motors[FL2_m] = FL2_motor;

					// right back soft push
					motors[TR2_m] = jointAngleScaling(-1.05, -0.25, -0.1, 0.1, -getVRN2output());
					motors[CR2_m] = jointAngleScaling(0.4, 1.3, -0.25, 0.25, vrn1_CTr_right);
					motors[FR2_m] = FR2_motor;
				}
				
			}
			
		}
		else
		{
			// Check if robot has climbed object	
			if (sensors[BZ_ori] > 0.25 && (boxing || stationary_push))
			{
				timestepcount2++;
				
				if (timestepcount2 >= 2)
				{
					legstart = false; 
					climbed = true;
				} 
			}
			else if (sensors[BZ_ori] > 0.3 && !boxing)
			{
				legstart = false; 
				climbed = true; 
			}
			

				// left backwards walk
				motors[TL2_m] = jointAngleScaling(-1.2, 1.2, -0.3, 0.3, getVRN2output());
				motors[CL2_m] = jointAngleScaling(-0.4, 1.4, -0.5, 0.5, vrn1_CTr_left);
				motors[FL2_m] = FL2_motor;

				// right backwards walk
				motors[TR2_m] = jointAngleScaling(-1.2, 1.2, -0.3, 0.3, -getVRN2output());
				motors[CR2_m] = jointAngleScaling(-0.4, 1.4, -0.5, 0.5, vrn1_CTr_right);
				motors[FR2_m] = FR2_motor;
				
		}						
		
		lastoutputH2 = outputH1;
    	lastoutputH2reverse = outputH2reverse;
      
	}
	else
	{
		setStandConfiguration(motors);
		
		// Wait 5 time steps when on top of object before pushing
		if (climbed)
		{
			counter++;
			if (counter >= 5)
			{
				legstart = true;
				push = true;

				if(boxing)
				{
					inputs.at(1) = -1;
					inputs.at(2) = -1;
					inputs.at(3) = 1;
					inputs.at(4) = 0.0;
				}
				else
				{
					inputs.at(1) = 1;
					inputs.at(2) = -1;
					inputs.at(3) = 1;
					inputs.at(4) = 1;
				}
				
				counter = 0;
			}
		}	
	}
	
	/***************************************************************************************************************/
	// Front left						Front left								Front left
	// motor 3 = TC motor joint			torque sensor 40 = TC motor joint		Foot contact sensor 22
	// motor 9 = CTr motor joint		torque sensor 46 = CTr motor joint
	// motor 15 = FTi motor joint		torque sensor 52 = FTi motor joint

	// Front right						Front right								Front right
	// motor 0 = TC motor joint			torque sensor 37 = TC motor joint		Foot contact sensor 19
	// motor 6 = CTr motor joint		torque sensor 43 = CTr motor joint
	// motor 12 = FTi motor joint		torque sensor 49 = FTi motor joint

	// Middle left						Middle left								Middle left
	// motor 4 = TC motor joint			torque sensor 41 = TC motor joint		Foot contact sensor 23
	// motor 10 = CTr motor joint		torque sensor 47 = CTr motor joint
	// motor 16 = FTi motor joint		torque sensor 53 = FTi motor joint

	// Middle right						Middle right							Middle right
	// motor 1 = TC motor joint			torque sensor 38 = TC motor joint		Foot contact sensor 20
	// motor 7 = CTr motor joint		torque sensor 44 = CTr motor joint
	// motor 13 = FTi motor joint		torque sensor 50 = FTi motor joint

	// Back left						Back left								Back left
	// motor 5 = TC motor joint			torque sensor 42 = TC motor joint		Foot contact sensor 24
	// motor 11 = CTr motor joint		torque sensor 48 = CTr motor joint
	// motor 17 = FTi motor joint		torque sensor 54 = FTi motor joint

	// Back right						Back right								Back right
	// motor 2 = TC motor joint			torque sensor 39 = TC motor joint		Foot contact sensor 21
	// motor 8 = CTr motor joint		torque sensor 45 = CTr motor joint
	// motor 14 = FTi motor joint		torque sensor 51 = FTi motor joint
	/****************************************************************************************************************/
}

bool Modular_neural_control_cylinder::store(FILE* f) const{

	Configurable::print(f,"");
    return true;
}

bool Modular_neural_control_cylinder::restore(FILE* f){
	Configurable::parse(f);
    return true;
}
