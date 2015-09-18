/*
 * neuralcontrol.h
 *
 *  Created on: Sep 5, 2015
 *      Author: Poramate Manoonpong
 */

#ifndef CONTROLLERS_CDBOT_H_
#define CONTROLLERS_CDBOT_H_


#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>
#include <selforg/configurable.h>
#include <selforg/types.h>

#include <assert.h>
#include <cmath>
#include <stdlib.h>
#include <string.h>
#include <vector>

#include <stdio.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <algorithm>

#include <selforg/matrix.h>
#include <utils/real_robots/cdbot/cdbotSensMotDef.h>

class neuralcontrol : public AbstractController{
public:
	neuralcontrol();
	virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);
	virtual ~neuralcontrol();
	virtual paramkey getName() const { return name;}
	virtual int getSensorNumber() const { return numbersensors;}
	virtual int getMotorNumber() const { return numbermotors;}
	virtual void step(const sensor*, int number_sensors,motor*, int number_motors);
	virtual void stepNoLearning(const sensor*, int number_sensors,motor*, int number_motors){};


  /***** STOREABLE ****/
  /** stores the controller values to a given file. */
  virtual bool store(FILE* f) const;
  /** loads the controller values from a given file. */
  virtual bool restore(FILE* f);



protected:
  unsigned short numbersensors, numbermotors;

	int t;
	paramkey name;

public:

	std::vector<sensor> x;
	std::vector<sensor> y;

	//MRC parameters -begin//
	std::vector<double> mrc_input;    //MRC inputs
	std::vector<double> mrc_avg_input;    //MRC average inputs
	std::vector<double> mrc_activity; //MRC neural activities
	std::vector<double> mrc_output;   //MRC neural outputs
	std::vector<double> mrc_input_w;   //MRC neural weight from inputs to MRC neurons
	std::vector< std::vector<double> > mrc_w;       //MRC neural weights
	double mrc_bias;                  //MRC bias


	//sound
	std::vector<double> sound_output;   //MRC neural outputs

};
#endif /* CONTROLLERS_CDBOT_H_ */
