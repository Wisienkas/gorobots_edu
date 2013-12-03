
///////Serial Communication////////////////
#include "Serial.h"
#include <string.h>
#include <stdio.h>
#define BPM_ITER	200000 //1000000 epochs 

///////////////////////////////////////////

Serial* robot;


int main (int argc, char **argv)
{
 
	

robot = new Serial();	
int t;

/////Initial Neural network///////////////////////
/*

// prepare XOR training data/////
double data[][4]={// I XOR I XOR I = O
//--------------------------------
					0, 0, 0, 0,
					0, 0, 1, 1,
					0, 1, 0, 1,
					0, 1, 1, 0,
					1, 0, 0, 1,
					1, 0, 1, 0,
					1, 1, 0, 0,
					1, 1, 1, 1 };

/////////////////////////////////


*/


for (int i=0;i<BPM_ITER;i++) {


//XNOR sigmoid function training//
robot->FeedforwardNetwork(0,0,1);	
robot->FeedforwardNetwork(0,1,0);
robot->FeedforwardNetwork(1,0,0);
robot->FeedforwardNetwork(1,1,1);

/*
//XNOR tanh function training//
robot->FeedforwardNetwork(-1,-1,1);	
robot->FeedforwardNetwork(-1,1,-1);
robot->FeedforwardNetwork(1,-1,-1);
robot->FeedforwardNetwork(1,1,1);
*/

/*
//XOR tanh
robot->FeedforwardNetwork(-1,-1,-1);
robot->FeedforwardNetwork(-1,1,1);
robot->FeedforwardNetwork(1,-1,1);
robot->FeedforwardNetwork(1,1,-1);
*/

/*
//XOR sigmoid
robot->FeedforwardNetwork(0,0,0);
robot->FeedforwardNetwork(0,1,1);
robot->FeedforwardNetwork(1,0,1);
robot->FeedforwardNetwork(1,1,0);
*/





//XNOR function testing set //

/*
robot->Run(-1,-1);
robot->Run(-1,1);
robot->Run(1,-1);
robot->Run(1,1);
*/

/*
robot->Run(0,0);
robot->Run(0,1);
robot->Run(1,0);
robot->Run(1,1);
*/
	}


  return 0;
}
