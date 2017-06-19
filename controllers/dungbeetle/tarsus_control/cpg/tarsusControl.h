/*
 * hindLegControl.h
 *
 *  Created on: Nov 3, 2014
 *      Author: giuliano
 */

#ifndef CONTROLLERS_TARSUS_CONTROL_H_
#define CONTROLLERS_TARSUS_CONTROL_H_


#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>
#include <selforg/configurable.h>
#include <selforg/types.h>

#include <assert.h>
#include <cmath>
#include <stdlib.h>
#include <string.h>
#include <vector>

#include <selforg/matrix.h>
#include <utils/real_robots/dungbeetle/dungBeetle_frontlegSensMotDef.h>
#include <controllers/dungbeetle/tarsus_control/cpg/neuroOscillator.h>


class hindLegControl : public AbstractController{
public:
	hindLegControl();
	virtual ~hindLegControl();
	void initialize();
	virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);
	virtual void step(const sensor*, int number_sensors,motor*, int number_motors);
	virtual void stepNoLearning(const sensor*, int number_sensors,motor*, int number_motors){};
	virtual int getMotorNumber() const {
	      return numbermotors;
	    }
	virtual int getSensorNumber() const {
		return numbersensors;}
	virtual paramkey getName() const { return name;}
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
	neuroOscillator* osc;
	double vec[2]={0,0};
	ofstream plot;





	  /****Neural oscillator****/

	  double alph;
	  double phi;
	  double WeightH1_H1;
	  double WeightH2_H2;
	  double WeightH1_H2;
	  double WeightH2_H1;

	  double BiasH1;
	  double BiasH2;

	  double activityH1;
	  double activityH2;

	  double outputH1;
	  double outputH2;

};
#endif /* CONTROLLERS_DUNGBEETLE_CPG_HINDLEGCONTROL_H_ */
