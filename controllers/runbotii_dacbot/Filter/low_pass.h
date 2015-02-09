/*
 * low_pass.h
 *
 *  Created on: May 28, 2013
 *      Author: ilyas
 */

#ifndef LOW_PASS_H_
#define LOW_PASS_H_



class LowPass{
public:
	LowPass(){w=0.3;current_value=0;};
	
	LowPass(double recurrent_weight){w=recurrent_weight;current_value=0;};

	double update(int value){current_value=w*value+(1-w)*current_value;
        return current_value;};

	void set_current_value(int value){
	  current_value=value;
	};
	
private:
	double w;
	int current_value;

};


#endif /* LOW_PASS_H_ */
