/*
 * gcs_signal.h
 *
 *  Created on: May 28, 2013
 *      Author: ilyas
 */

#ifndef GCS_SIGNAL_H_
#define GCS_SIGNAL_H_

#include "low_pass.h"


#include "sigmoid.h"

/*
double sigmoid(int value){
	double s;
	s=1./(1.+exp(-value));
	return s;
};

double sigmoid_inverted(int value){
	return sigmoid((-1.)*value);
};
*/


class NewGCSFilter{
public:
	NewGCSFilter(): low_pass_l(LowPass(0.3)),low_pass_r(LowPass(0.3)){
		//low_pass_l=new LowPass(0.3);
		//low_pass_r=new LowPass(0.3);
		//low_pass_l(0.3);
		//low_pass_r(0.3);
		low_pass_l.set_current_value(2800);
		low_pass_r.set_current_value(2800);
		sigmoid_threshold_l=2600;
		sigmoid_threshold_r=2600;
		time_const=1.;
	};

	NewGCSFilter(double recurrent_weight_l,double recurrent_weight_r){
		low_pass_l=LowPass(recurrent_weight_l);
		low_pass_r=LowPass(recurrent_weight_r);
		low_pass_l.set_current_value(2800);
		low_pass_r.set_current_value(2800);
		sigmoid_threshold_l=2600;
		sigmoid_threshold_r=2600;
		time_const=1.;
	};


	NewGCSFilter(double recurrent_weight_l,double recurrent_weight_r,int _sigmoid_threshold_l,int _sigmoid_threshold_r){
		low_pass_l=LowPass(recurrent_weight_l);
		low_pass_r=LowPass(recurrent_weight_r);
		low_pass_l.set_current_value(2800);
		low_pass_r.set_current_value(2800);
		sigmoid_threshold_l=_sigmoid_threshold_l;
		sigmoid_threshold_r=_sigmoid_threshold_r;
		time_const=1.;
	};

/*                                        // Not a good solution to directly calculate u_gl/gr! Hard to add new connections to the neuron this way.
	double update_gl(int piezoleft){
		gl=sigmoid_inverted(time_const*(low_pass_l.update(piezoleft)-sigmoid_threshold_l));
		return gl;
	};

	double update_gr(int piezoright){
		gr=sigmoid_inverted(low_pass_r.update(piezoright)-sigmoid_threshold_r);
		return gr;
	};
*/

	double update_y_gl(int piezoleft){
		y_gl=low_pass_l.update(piezoleft);
		return y_gl;
	};

	double update_y_gr(int piezoright){
		y_gr=low_pass_r.update(piezoright);
		return y_gr;
	};


	void set_time_const(double value){
		time_const=value;
	};

private:
	LowPass low_pass_l;//(double 0.0);
	LowPass low_pass_r;//(double 0.0);
	//double gl;
	//double gr;
	double y_gl;
	double y_gr;
	int sigmoid_threshold_l;
	int sigmoid_threshold_r;
	double time_const;

};


#endif /* GCS_SIGNAL_H_ */
