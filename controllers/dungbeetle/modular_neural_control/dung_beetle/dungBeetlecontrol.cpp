/*
 * dungBeetlecontrol.cpp
 *
 *  Created on: Oct 24, 2015
 *      Author: giuliano
 */




//#include <selforg/controller_misc.h>
#include <math.h>
#include "dungBeetlecontrol.h"

using namespace matrix;
using namespace std;

dungBeetlecontrol::dungBeetlecontrol() :
  AbstractController("dungBeetlecontrol", "$Id: dungBeetlecontrol.cpp,v 0.1 $") {
	initialize(2,false,false);
}
dungBeetlecontrol::dungBeetlecontrol(int dungBeetletype,bool mMCPGs,bool mMuscleModelisEnabled) :
  AbstractController("dungBeetlecontrol", "$Id: dungBeetlecontrol.cpp,v 0.1 $") {
	initialize(dungBeetletype, mMCPGs, mMuscleModelisEnabled);
}
void dungBeetlecontrol::initialize(int aAMOSversion,bool mMCPGs,bool mMuscleModelisEnabled)
{

  t = 0; // step counter
  cpg=new plastic(0.2,0.2,0.2,0.04*2*3.14,1.01,0.01);//CPG
  MCPGs=mMCPGs;
  inputDerivative.push_back(0);
  inputDerivative.push_back(0);

 inputDerivative2.push_back(0);
 inputDerivative2.push_back(0);
 plot.open("dungBeetle6legs.dat");//writing data to file.. change this to your directory
 };

dungBeetlecontrol::~dungBeetlecontrol() {

}


void dungBeetlecontrol::init(int sensornumber, int motornumber, RandGen* randGen) {

  numbersensors = sensornumber;
  numbermotors = motornumber;
  x.resize(sensornumber);
  //y.resize(AMOSII_MOTOR_MAX);

}

void stepNoLearning(const sensor*, int number_sensors, motor*, int number_motors) {
     // empty
   }

/// performs one step (includes learning). Calulates motor commands from sensor inputs.
void dungBeetlecontrol::step(const sensor* x_, int number_sensors, motor* y_, int number_motors) {

  assert(number_sensors == numbersensors);
  assert(number_motors == numbermotors);

  //0) Sensor inputs/scaling  ----------------


  for (unsigned int i = 0; i < DUNGBEETLE_SENSOR_MAX; i++) {
    x.at(i) = x_[i];
  }



  //2) Neural locomotion control------

  if(!MCPGs) //single CPG
    {

	  std::cout<< "timer"<< t << std::endl;
	  cpg->update(0);
	   	  	  //y_[17]=1;//back left 3j 1 extended

	  double val=((cpg->getOut1()+0.2)/(0.4))*2-1;

	  std::cout << val;
	  inputDerivative[0]=inputDerivative[1];
	  inputDerivative[1]=val;

	  double der=inputDerivative.at(1)-inputDerivative.at(0);

	  double input_secJ;

	  if(der > 0 && val > 0)
	  		input_secJ=val;
	  	else
	  		input_secJ=-1.4;

      double pattern1TC=val;
	  double pattern1CT=input_secJ;



	  double pattern2TC=-pattern1TC;

	  inputDerivative2[0]=inputDerivative2[1];
	  inputDerivative2[1]=pattern2TC;
	  double der2=inputDerivative2.at(1)-inputDerivative2.at(0);


	  double pattern2CT;

	  	  if(der2 > 0 && pattern2TC > 0)
	  	  		pattern2CT=pattern2TC;
	  	  	else
	  	  		pattern2CT=-1.4;





	  //left back
	  y_[5]=-1;
	  y_[11]=-val;
	  y_[17]=-val;

	  //right back
	  y_[2]=-1;
	  y_[8]=val;
	  y_[14]=val;

	  //left middle
	  y_[4]=0;
	  y_[10]=val;
	  y_[16]=val;

	  //right middle
	  y_[1]=0;
	  y_[7]=-val;
	  y_[13]=-val;


	  //top left
	  y_[3]=0;
	  y_[9]=0;
	  y_[15]=0;

	  //top right
	  y_[0]=0;
	  y_[6]=0;
	  y_[12]=0;


	  plot << t << " " << pattern1TC << " " << pattern1CT << " "<< pattern2TC << " " << pattern2CT << endl;



  	  /***Don't touch****Set Motor outputs begin *******************/
  	 for(unsigned int j=0; j<y_MCPGs.size();j++)
  	 for(unsigned k=0; k< 3; k++)//index of angel joints
  	 y_[j+6*k] = y_MCPGs.at(j).at(j+6*k);
   }


  // update step counter
  t++;
}
;

/** stores the controller values to a given file. */
bool dungBeetlecontrol::store(FILE* f) const {
  return true;
}

/** loads the controller values from a given file. */
bool dungBeetlecontrol::restore(FILE* f) {
  //	Configurable::parse(f);
  return true;
}

