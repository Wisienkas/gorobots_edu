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
/*
	cout << "sensor 0 value: " << x.at(0) << endl;
	cout << "sensor 1 value: " << x.at(1) << endl;
	cout << "sensor 2 value: " << x.at(2) << endl;

	double temp=vec[0];
	double ct;

	y_[0]=ct;
	y_[1]=ct;//31
	y_[2]=ct;//0

	//plot << t << " " << signal << " " << ct << " " << x.at(1) <<" " << x.at(2) << endl;

	t++;
*/

	  for(unsigned int i=0; i<2;i++)
	  {
	    y_[i] = 1*y.at(i);
	  }

	  // update step counter
	  t++;
};


/** stores the controller values to a given file. */
bool neuralcontrol::store(FILE* f) const {
  //	std::cout << "hello \n";
  //	double bla = 10.0;
  //   fprintf(f, "%f", bla);

  //fprintf(f, "%f %f\n", preprocessing_reflex.preprosensor.at(L2_fs), preprocessing_reflex.preprosensor.at(L1_fs));
  //	Configurable::print(f, "");
  return true;
}

/** loads the controller values from a given file. */
bool neuralcontrol::restore(FILE* f) {
  //	Configurable::parse(f);
  return true;
}




