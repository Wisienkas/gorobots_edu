/*
 * gc_neuron.h
 *
 *  Created on: Jun 9, 2013
 *      Author: ilyas
 */

#ifndef GC_NEURON_H_
#define GC_NEURON_H_

#include "../Filter/sigmoid.h"

class GCNeuron{
public:
	GCNeuron(): y_g(double(0.0)),u_g(double(0.0)){
		sigmoid_threshold=2600;
		sigmoid_time_const=1.0;
	};

	void update_y_g(double u,double w){
		y_g=y_g+w*u;
	};

	void set_y_g(double value){
		y_g=value;
	};

	double update(){
		u_g=sigmoid_inverted(sigmoid_time_const*(y_g-sigmoid_threshold));
		return u_g;
	};

	double get_u_g(){
		return u_g;
	};

	void set_time_const(double value){
		sigmoid_time_const=value;
	};

	void set_sigmoid_threshold(double value){
		sigmoid_threshold=value;
	};

private:
	double y_g;
	double u_g;
	double sigmoid_time_const;
	double sigmoid_threshold;
};




#endif /* GC_NEURON_H_ */
