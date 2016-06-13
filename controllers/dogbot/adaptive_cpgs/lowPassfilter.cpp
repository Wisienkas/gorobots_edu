/*
 * lowPassfilter.cpp
 *
 *  Created on: Feb 11, 2015
 *      Author: giuliano
 */

#include "lowPassfilter.h"

lowPass_filter::lowPass_filter() {
	w=0.3;
	current_value=0;
	// TODO Auto-generated constructor stub

}

lowPass_filter::lowPass_filter(double recurrent_weight)
{
	w=recurrent_weight;
	current_value=0;
}

lowPass_filter::~lowPass_filter() {
	// TODO Auto-generated destructor stub
}

double lowPass_filter::update(double value)
{
	current_value=w*value+(1-w)*current_value;
	return current_value;
}

void lowPass_filter::set_current_value(double value)
{
	current_value=value;
}


