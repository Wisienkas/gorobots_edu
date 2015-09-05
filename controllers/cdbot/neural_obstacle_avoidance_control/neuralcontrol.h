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


	//Begin ADD YOUR VARIABLE HERE//

	//0) Sensor inputs/scaling  ----------------

public:
	//Angle sensors
	std::vector<sensor> x;
	std::vector<sensor> y;

};
#endif /* CONTROLLERS_CDBOT_H_ */
