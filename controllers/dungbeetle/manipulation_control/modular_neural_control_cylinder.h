#ifndef __MODULAR_NEURAL_CONTROL_CYLINDER_H
#define __MODULAR_NEURAL_CONTROL_CYLINDER_H

#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>
#include <math.h>
#include <algorithm>
#include <fstream>
#include <stdio.h>
#include <deque>

//#include "dungbeetle.h"
#include "utils/sim_robots/dungbeetle/dungbeetle_manipulationv1.h"
class Modular_neural_control_cylinder : public AbstractController {
public:

		//Changeable joints
		double BJ;
		double TL0;
		double TR0;
		double TL1;
		double TR1;
		double TL2;
		double TR2;
		double CL0;
		double CR0;
		double CL1;
		double CR1;
		double CL2;
		double CR2;
		double FL0;
		double FR0;
		double FL1;
		double FR1;
		double FL2;
		double FR2;

		//CPG variables for the front and middle legs to walk backwards
		double outputH1;
		double outputH2;
		double outputH2reverse;
		double lastoutputH2;
		double lastoutputH2reverse;
		double activityH1;
		double activityH2;
		double biasH1;
		double biasH2;
		double weightH1_H1;
		double weightH2_H2;
		double weightH1_H2;
		double weightH2_H1;

		//Booleans
		bool start;
		bool legstart;
	 	bool push;
	 	bool finished;
		bool bump;
	 	bool climbed;
	 	bool push_over_step;
	 	bool stationary_push;
	 	bool customise;
		bool boxing;
	 	
		//Functions
  		Modular_neural_control_cylinder();
  		virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);
  		virtual int getSensorNumber() const {
      		return number_sensors;}
      	virtual int getMotorNumber() const {
      		return number_motors;}
      	double jointAngleScaling(double new_min, double new_max, double old_min, double old_max, double old_value);
      	double setJointDegrees(double degree);
      	void setStandConfiguration(motor* motors);
		void MovingAverage(const sensor* sensors);
		void UpdateCPG();
		void UpdateVRN();
		double getVRN1output();
		double getVRN2output();
		void UpdatePSN();
		std::vector<double> getPSN1output();
		std::vector<double> getPSN2output();
		std::vector<double> getPSN3output();
		std::vector<double> getPSN4output();
		void Clipping();
      	virtual void step(const sensor* sensors, int sensornumber, motor* motors, int motornumber);
      	virtual void stepNoLearning(const sensor* , int number_sensors, motor* , int number_motors){};
      	virtual bool store(FILE* f) const;
      	virtual bool restore(FILE* f);
      	
	protected:

		int number_sensors;
		int number_motors;
		
		//Inputs for modular neural network
		std::vector<double> inputs;
		
		//VRN
		std::vector<double> vrn1_activity; 				
		std::vector<double> vrn1_output;						
		std::vector< std::vector<double> > vrn1_w; 			
		std::vector<double> vrn1_bias;
		double vrn1_input0_w;
		double vrn1_input1_0_w;
		double vrn1_input1_1_w;
		double* vrn1_input0;
		double* vrn1_input1_0;
		double* vrn1_input1_1;
		
		std::vector<double> vrn2_activity; 				
		std::vector<double> vrn2_output;						
		std::vector< std::vector<double> > vrn2_w; 			
		std::vector<double> vrn2_bias;
		double vrn2_input0_w;
		double vrn2_input1_w;
		double* vrn2_input0;
		double* vrn2_input1;
		
		//PSN
		std::vector<double> psn1_activity; 					
		std::vector<double> psn1_output;					
		std::vector< std::vector<double> > psn1_w;			
		std::vector<double> psn1_bias;
		std::vector<double> psn1_control_w;
		std::vector< std::vector<double> > psn1_input_w;
		std::vector<double*> psn1_inputs;
		
		std::vector<double> psn2_activity; 					
		std::vector<double> psn2_output;					
		std::vector< std::vector<double> > psn2_w;			
		std::vector<double> psn2_bias;
		std::vector<double> psn2_control_w;
		std::vector< std::vector<double> > psn2_input_w;
		std::vector<double*> psn2_inputs;
		
		std::vector<double> psn3_activity; 					
		std::vector<double> psn3_output;					
		std::vector< std::vector<double> > psn3_w;			
		std::vector<double> psn3_bias;
		std::vector<double> psn3_control_w;
		std::vector< std::vector<double> > psn3_input_w;
		std::vector<double*> psn3_inputs;
		
		std::vector<double> psn4_activity; 					
		std::vector<double> psn4_output;					
		std::vector< std::vector<double> > psn4_w;			
		std::vector<double> psn4_bias;
		std::vector<double> psn4_control_w;
		std::vector< std::vector<double> > psn4_input_w;
		std::vector<double*> psn4_inputs;
		
		//Clipping
		double psn4_low;
		double vrn1_low;
		double psn4_CTr_left;
		double psn4_CTr_right;
		double vrn1_CTr_left;
		double vrn1_CTr_right;
		double last_psn4_output_left;
		double last_psn4_output_right;
		double last_vrn1_output_left;
		double last_vrn1_output_right;
		
		//F-joint signals
		double FL2_motor;
		double FR2_motor;
		
	    //Moving Average variables
		std::deque<float> movingAverageL2;
		std::deque<float> movingAverageR2;
		float sum;
		
		///Counters
		double startCounter;
		double counter;
		double timestepcount;
		double timestepcount2;
  		
};

#endif
