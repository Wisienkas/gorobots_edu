/***************************************************************************
 *   Copyright (C) 2010 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *                                                                         *
 *   LICENSE:                                                              *
 *   This work is licensed under the Creative Commons                      *
 *   Attribution-NonCommercial-ShareAlike 2.5 License. To view a copy of   *
 *   this license, visit http://creativecommons.org/licenses/by-nc-sa/2.5/ *
 *   or send a letter to Creative Commons, 543 Howard Street, 5th Floor,   *
 *   San Francisco, California, 94105, USA.                                *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  *
 *                                                                         *
 *   $Log: randomcontroller.cpp,v $                                         *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/

#include <selforg/controller_misc.h>
#include "randomcontroller.h"
using namespace matrix;
using namespace std;

RandomController::RandomController( const RandomControllerConf _conf)
  : AbstractController("RandomController", "$Id: randomcontroller.cpp,v 0.1 $"), conf(_conf){
  t=0;  
  
  // prepare name;
  //Configurable::insertCVSInfo(name, "$RCSfile: randomcontroller.cpp,v $", "$Revision: 0.1 $");
  randomgen = 0;
  temp_y_ = 0;
};

RandomController::~RandomController(){
	 if (randomgen) {
//		 randomgen->~WhiteUniformNoise();
		 delete randomgen;
	 }
	 if (temp_y_) delete temp_y_;

}

void RandomController::init(int sensornumber, int motornumber, RandGen* randGen){
	number_sen = sensornumber;
	number_mot = motornumber;

	randomgen = new WhiteUniformNoise();
	randomgen->init(number_mot);

	temp_y_ = new double[number_sen];
	for (int i=0; i < number_sen; i++){
		temp_y_[i] = 0.0;
	}
//	//initialize motor output randomly (maybe fluctuation around 0 is better for walking
//	randomgen->add(temp_y_,1.0);

}


/// performs one step (includes learning). Calulates motor commands from sensor inputs.
void RandomController::step(const sensor* x_, int number_sensors, 
				    motor* y_, int number_motors){
  stepNoLearning(x_, number_sensors, y_, number_motors);
};

/// performs one step without learning. Calulates motor commands from sensor inputs.
void RandomController::stepNoLearning(const sensor* x_, int number_sensors, 
					      motor* y_, int number_motors){

	/*this adds a random number within [-1 * conf.maxdeviation, 1 *conf.maxdeviation]
	 * to each value in y_
	 * note that y_ must have dimension of number_mot
	 */
	randomgen->add(temp_y_, conf.maxdeviation);
	for (int i = 0; i < number_sen; i++) {
		//reflect random steps at output borders [-1,1]
		if (temp_y_[i] < -1.0) {
			temp_y_[i] = -2.0 - temp_y_[i];
		} else if (temp_y_[i] > 1.0) {
			temp_y_[i] = 2.0 - temp_y_[i];
		}
		//write motor output
		y_[i] = temp_y_[i];
	}
  // update step counter
  t++;
};
  
      
  

/** stores the controller values to a given file. */
bool RandomController::store(FILE* f) const{  
  return true;
}


/** loads the controller values from a given file. */
bool RandomController::restore(FILE* f){
  return true;
}



  
