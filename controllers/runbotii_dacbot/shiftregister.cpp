/*
 * shiftregister.cpp
 *
 *  Created on: Feb 25, 2015
 *      Author: giuliano
 */

#include <controllers/runbotii_dacbot/shiftregister.h>

shift_register::shift_register(int delay) {
	delay_=delay;

		for(int i=0;i<delay_;i++)
			shiftRegister.push_back(0);
	// TODO Auto-generated constructor stub

}

shift_register::~shift_register() {
	// TODO Auto-generated destructor stub
}


double shift_register::update(double value)
{
    if(delay_ == 0)
    	return value;
    else if(delay_ == 1)
	{
    	double temp=shiftRegister.at(0);
    	shiftRegister[0]=value;
    	return temp;

	}

    else
    {
	double temp=shiftRegister.at(0);
    for(int i=0;i<=shiftRegister.size()-2;i++)
		shiftRegister[i]=shiftRegister.at(i+1);
	shiftRegister[shiftRegister.size()-1] = value;

    return temp;
    }
}
