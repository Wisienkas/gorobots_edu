/*
 * neuralcontrol.cpp
 *
 *  Created on: Sep 5, 2015
 *      Author: Poramate Manoonpong
 */
#include <selforg/controller_misc.h>
#include <controllers/cdbot/neural_obstacle_avoidance_control/neuralcontrol.h>
#include <math.h>

using namespace matrix;
using namespace std;

bool sound_control = true;//false; // set this for using pure sound control
bool mrc_control = false;//false; // set this for using pure neural MRC for motor control
bool braitenberg_control = false;//false; // set this for using Braitenberg control
bool avg_input = true; // true = use three IR signals on each side, false = use only at front


neuralcontrol::neuralcontrol():
			AbstractController("neuralcontrol", "$Id: neuralcontrol.cpp,v 0.1 $"){

	//---ADD YOUR initialization here---//

	t=0;  // step counter

	//---ADD YOUR initialization here---//



};

neuralcontrol::~neuralcontrol() {
	// TODO Auto-generated destructor stub
}


void neuralcontrol::init(int sensornumber, int motornumber, RandGen* randGen) {

	numbersensors = sensornumber;
	numbermotors = motornumber;
	x.resize(sensornumber);
	y.resize(CDBOT_MOTOR_MAX);


	//MRC initialization -begin//
	mrc_input.resize(2);
	mrc_activity.resize(2);
	mrc_output.resize(2);
	mrc_input_w.resize(2);
	mrc_avg_input.resize(2);

	mrc_w.resize(2);
	for(unsigned int i=0; i<mrc_w.size(); i++)
	{
		mrc_w.at(i).resize(2);
	}

	//MRC initialization -end//

	//sound initialization -begin//
	sound_output.resize(2);


	//sound initialization -end//



	addInspectableValue("mrc_input0_L", &mrc_input.at(0),"mrc_input0");
	addInspectableValue("mrc_input1_R", &mrc_input.at(1),"mrc_input1");

	addInspectableValue("MRC0",&mrc_output.at(0),"MRC0");
	addInspectableValue("MRC1",&mrc_output.at(1),"MRC1");
	//std::cout << "hi";

}


void neuralcontrol::step(const sensor* x_, int number_sensors,motor* y_, int number_motors)
{


	assert(number_sensors == numbersensors);
	assert(number_motors == numbermotors);

	for(unsigned int i=0; i<(numbersensors);i++)
	{
		x.at(i) = x_[i];//READ SENSOR S VALUE FROM HERE

	}

	//x.at(0) = IR_RIGHTOUT
	//x.at(1) = IR_RIGHTIN
	//x.at(2) = IR_LEFTOUT
	//x.at(3) = IR_LEFTIN

	//x.at(4) = LIGHT_RIGHT
	//x.at(5) = LIGHT_LEFT

	//x.at(6) = SOUND_RIGHT
	//x.at(7) = SOUND_LEFT




	//----------------------Neural Obstacle control



	mrc_avg_input.at(0) = (x.at(1)+x.at(2))/2; // left
	mrc_avg_input.at(1) = (x.at(0)+x.at(3))/2; // right

	mrc_input.at(0) = x.at(1);//left
	mrc_input.at(1) = x.at(3);//right


	mrc_input_w.at(0) =  7.0;
	mrc_input_w.at(1) =  7.0;
	mrc_w.at(0).at(0) =  5.0;//5.6;
	mrc_w.at(1).at(1) =  5.0;//5.6;
	mrc_w.at(0).at(1) =  -3.5;
	mrc_w.at(1).at(0) =  -3.5;
	mrc_bias = 0.0;

	//average sensor signals
	if(avg_input)
	{
		mrc_activity.at(0) = mrc_w.at(0).at(0) * mrc_output.at(0) + mrc_w.at(0).at(1) * mrc_output.at(1) + mrc_bias+mrc_avg_input.at(0)*mrc_input_w.at(0);//left
		mrc_activity.at(1) = mrc_w.at(1).at(1) * mrc_output.at(1) + mrc_w.at(1).at(0) * mrc_output.at(0) + mrc_bias+mrc_avg_input.at(1)*mrc_input_w.at(1);//right
		mrc_output.at(0) = tanh(mrc_activity.at(0));
		mrc_output.at(1) = tanh(mrc_activity.at(1));
		printf("Avg\n");
	}

	if(!avg_input)
	{
		mrc_activity.at(0) = mrc_w.at(0).at(0) * mrc_output.at(0) + mrc_w.at(0).at(1) * mrc_output.at(1) + mrc_bias+mrc_input.at(0)*mrc_input_w.at(0);//left
		mrc_activity.at(1) = mrc_w.at(1).at(1) * mrc_output.at(1) + mrc_w.at(1).at(0) * mrc_output.at(0) + mrc_bias+mrc_input.at(1)*mrc_input_w.at(1);//right
		mrc_output.at(0) = tanh(mrc_activity.at(0));
		mrc_output.at(1) = tanh(mrc_activity.at(1));

		printf("Not avg\n");

	}


	//----------------------Phototropism control Add here!

	//sound_output.at(0)
	//sound_output.at(1)

	//----------------------Phototropism control


	if(braitenberg_control)
	{
		y.at(0) = -1*x.at(3);
		y.at(1) = -1*x.at(1);
		printf("B_control\n");
	}

	if(mrc_control)
	{
		y.at(0) = -1*mrc_output.at(1);
		y.at(1) = -1*mrc_output.at(0);
		printf("MRC_control\n");
	}

	if(sound_control)
	{
		y.at(0) = -1*sound_output.at(0);
		y.at(1) = -1*sound_output.at(1);
		printf("sound_control\n");
	}


	for(unsigned int i=0; i<2;i++)
	{
		y_[i] = 1*y.at(i);
	}

	// update step counter
	t++;
};


/** stores the controller values to a given file. */
bool neuralcontrol::store(FILE* f) const {

	return true;
}

/** loads the controller values from a given file. */
bool neuralcontrol::restore(FILE* f) {
	//	Configurable::parse(f);
	return true;
}




