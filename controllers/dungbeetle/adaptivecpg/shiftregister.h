/*
 * shiftregister.h
 *
 *  Created on: Feb 25, 2015
 *      Author: giuliano
 */

#ifndef SHIFTREGISTER_H_
#define SHIFTREGISTER_H_

#include <vector>

class shift_register {
public:
	shift_register(int delay);
	virtual ~shift_register();
	double update(double value);
	int getRegSize();
	std::vector<double> getReg();

private:
	std::vector<double> shiftRegister;
	int delay_;
};

#endif /* CONTROLLERS_RUNBOTII_DACBOT_SHIFTREGISTER_H_ */
