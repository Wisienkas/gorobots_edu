/*
 * hindLegControl.cpp
 *
 *  Created on: Nov 3, 2014
 *      Author: giuliano
 */
#include <controllers/dungbeetle/cpg/hindLegControl.h>
#include <selforg/controller_misc.h>
#include <math.h>


using namespace std;

hindLegControl::hindLegControl():
	AbstractController("hindLegControl", "$Id: hindLegControl.cpp,v 0.1 $"){
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
  y.resize(DUNGBEETLE_MOTOR_MAX);
  //y.at(0)=0;
  //y.at(1)=0;
  //y.at(2)=0;

 std::cout << "hi";

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

	double signal= osc->getOutPut1();
	double temp=vec[0];
	double ct;
	vec[0]=signal;
	vec[1]=temp;

	if(signal>temp)
	{
	ct=signal;
	}

	else{
		ct=-0.18;

	}

	y_[0]=signal;//28
	y_[1]=ct;//31
	y_[2]=ct;//0

	plot << t << " " << signal << " " << ct << " " << x.at(1) <<" " << x.at(2) << endl;
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

