/*
 * DynamicVector.h
 *
 *  Created on: Apr 14, 2015
 *      Author: giuliano
 */

#ifndef CONTROLLERS_RUNBOTII_DACBOT_DYNAMICVECTOR_H_
#define CONTROLLERS_RUNBOTII_DACBOT_DYNAMICVECTOR_H_

#include <vector>

class DynamicVector {
public:

	DynamicVector(int size);
	void changeSize(unsigned long int newSize);
	virtual ~DynamicVector();
	double update(double value);
	int getSize();
private:
	std::vector<double> vec;
};

#endif /* CONTROLLERS_RUNBOTII_DACBOT_DYNAMICVECTOR_H_ */
