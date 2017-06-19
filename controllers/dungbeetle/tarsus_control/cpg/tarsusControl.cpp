/*
 * hindLegControl.cpp
 *
 *  Created on: Nov 3, 2014
 *      Author: giuliano
 */
#include <controllers/dungbeetle/tarsus_control/cpg/tarsusControl.h>
#include <selforg/controller_misc.h>
#include <math.h>


using namespace std;

hindLegControl::hindLegControl():
					AbstractController("frontLegControl", "$Id: frontLegControl.cpp,v 0.1 $"){
	initialize();
	osc= new neuroOscillator(0.1,0.1,1.01,1*2*3.14);
	//Change to your own path!!
	plot.open("/home/poma/Documents/plots/dungBeetle.dat");
	// TODO Auto-generated constructor stub

}

hindLegControl::~hindLegControl() {
	// TODO Auto-generated destructor stub
}


void hindLegControl::init(int sensornumber, int motornumber, RandGen* randGen) {

	numbersensors = sensornumber;
	numbermotors = motornumber;
	x.resize(sensornumber);
	y.resize(DUNGBEETLE_FRONTLEG_MOTOR_MAX);
	//y.at(0)=0;
	//y.at(1)=0;
	//y.at(2)=0;

	std::cout << "hi";

	activityH1 = 0;
	activityH2 = 0;

	outputH1 = 0.01;
	outputH2 = 0.01;
}


void hindLegControl::initialize()//this function will be more complex when adding function to the robot
{
	t=0;

}

void hindLegControl::step(const sensor* x_, int number_sensors,motor* y_, int number_motors)
{

	assert(number_sensors == numbersensors);
	assert(number_motors == numbermotors);

	for(unsigned int i=0; i<(numbersensors);i++)
	{
		x.at(i) = x_[i];//READ SENSOR S VALUE FROM HERE

	}

	cout << "sensor 0 value: " << x.at(0) << endl;
	cout << "sensor 1 value: " << x.at(1) << endl;
	cout << "sensor 2 value: " << x.at(2) << endl;

	//x.at(0),..., x.at(7) = mean poti 0,...,7

	/*Implement your control method*/

	//x,y,z





	//q1_r =
	//q2_r = ?
	//q3_r =
	//q4_r = ?

	//q1_l =
	//q2_l = ?
	//q3_l =
	//q4_l = ?

	////////////////////////////////


	//Neural oscillator//

	alph = 1.5;//1.5;
	phi = 0.25;

	WeightH1_H1  =  alph*cos(phi);
	WeightH2_H2  =  alph*cos(phi);
	WeightH1_H2  =  alph*sin(phi);
	WeightH2_H1  = -alph*sin(phi);


	BiasH1      = 0.0;
	BiasH2      = 0.0;

	activityH1 = WeightH1_H1*outputH1+WeightH1_H2*outputH2+BiasH1;
	activityH2 = WeightH2_H2*outputH2+WeightH2_H1*outputH1+BiasH2;

	outputH1 = tanh(activityH1);
	outputH2 = tanh(activityH2);


	//////////////////////

	//COXA1_RIGHT-J1
	y_[0]= outputH1;//28
	//COXA2_RIGHT-J2
	y_[1]= outputH2;//31
	//FEMUR_RIGHT-J3
	y_[2]=  outputH1;
	//TIBIA_RIGHT-J4
	y_[3]= outputH2;//28

	//COXA1_LEFT-J1
	y_[4]= 1;//31
	//COXA2_LEFT-J2
	y_[5]= 1;//0
	//FEMUR_LEFT-J3
	y_[6]= 1;//28
	//TIBIA_LEFT-J4
	y_[7]= 1;//31

	//Body
	y_[8]= 1;//31

	std::cout<<"oscillator output: "<<osc->getOutPut0()<< "\n";


	//plot << t << " " << signal << " " << ct << " " << x.at(1) <<" " << x.at(2) << endl;
	/*
	 *
	for(int i=0;i<DUNGBEETLE_MOTOR_MAX;i++)
	{


		//y.at(i)=1;
		y_[i]=y.at(i);//SET CPG VALUE HERE
	}
	 */
	osc->update();
	t++;


}

bool hindLegControl::store(FILE* f) const {
	return true;
}

/** loads the controller values from a given file. */
bool hindLegControl::restore(FILE* f) {
	//	Configurable::parse(f);
	return true;
}

