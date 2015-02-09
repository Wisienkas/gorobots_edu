/*
 * coincidence.h
 *
 *  Created on: Jun 6, 2013
 *      Author: ilyas
 */

#ifndef COINCIDENCE_H_
#define COINCIDENCE_H_

#include "../Filter/sigmoid.h"

class CoincidenceDetector{
public:
	CoincidenceDetector(){
		y=0.0;
		v=0.0;
		leak=0.6;
		sigmoid_threshold=1.6;
		sigmoid_time_factor=10.0;
	};

	void update_with_input(double u,double w){
		y=y+w*u;
	};

	void update_leaking(){
		y=(1.0-leak)*y;
	};

	void update(){
		v=sigmoid_double(sigmoid_time_factor*(y-sigmoid_threshold));
	};

	double get_v(){
			return v;
		};

	void set_new_leak(double new_leak){
			leak=new_leak;
		};

	void set_new_sigmoid_threshold(double new_sigmoid_threshold){
		sigmoid_threshold=new_sigmoid_threshold;
	};

	void set_new_sigmoid_time_factor(double new_sigmoid_time_factor){
		sigmoid_time_factor=new_sigmoid_time_factor;
	};

	double get_leak(){
		return leak;
	};

	double get_sigmoid_threshold(){
		return sigmoid_threshold;
	};

	double get_sigmoid_time_factor(){
		return sigmoid_time_factor;
	};

private:
	double y; // membrane potential
	double v; // output
	double leak; // leaking factor
	double sigmoid_threshold; //threshold for sigmoid function that calculates output
	double sigmoid_time_factor; // time factor for sigmoid function that calculates output

};


#endif /* COINCIDENCE_H_ */
