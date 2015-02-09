/*
 * test_sigmoid.cpp
 *
 *  Created on: May 29, 2013
 *      Author: ilyas
 *
 *      Test that damn sigmoid!
 */

#include "sigmoid.h"
#include <stdio.h>
#include <math.h>


int main(int argc, char *argv[]){
	double value=sigmoid(0);
	printf("value=%f\n",value);
	return 0;
}


