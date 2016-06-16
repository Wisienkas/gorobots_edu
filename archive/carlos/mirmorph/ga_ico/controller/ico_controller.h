#ifndef __ICOCONTROLLER_H
#define __ICOCONTROLLER_H


#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>
#include <utils/ico-framework/ico.h>
//Save data
#include <iostream>
#include <fstream>
#include <string.h>
using namespace std;
//Save data
/**
 * Empty robot controller.
 * The controller gets a number of input sensor values each timestep
 * and has to generate a number of output motor values.
 *
 * Go to the step() function and enter the control commands with respect to your task!
 *
 */
class IcoController : public AbstractController {
public:
	double mc[4];
	//Define global parameters-begin//
	std::vector<double> parameter;
	//Save data
    ofstream outFileicolearning;

	double distance;
	double alpha_tmp;
	double alpha;
	double deri_alpha;

	double input_distance_s;

	double xt_reflex_angle;

	std::vector<double> input_angle_s; //input angle sensors

	//orientation detection to the Green object in a long distance
	double predictive_signal_green;//predictive signal
	//orientation detection in a short distance
	double reflexive_signal_green;//reflex signal

	std::vector<double> predictive_signals_IR_R; //Predictive signals for the right IR sensors
	std::vector<double> predictive_signals_IR_L; //Predictive signals for the left IR sensors

    std::vector<double> fitnessIR; //Signals from the 8 IR sensors used for fitness function.

	double AL, AR, ALs, ARs;

	double motor1, motor2, motor3, motor4; //Motor signals

	double xb, yb, zb;

    std::vector<double> orientation;

	//----Exploration noise---------------//

	double exp_output;
	double exploration_g;
	double exploration_lowpass_g;
	double exploration_lowpass_old_g;

	//ICO learning
	double u_ico_out;
	vector<ICO*> ico_controller;
    std::vector<double> w0, w1, w2;

	bool manual_control;

	bool stop;
    bool learn;

	//Define global parameters-end//

	/// contructor (hint: use $ID$ for revision)
    IcoController(const std::string& name, const std::string& revision);

	/** initialization of the controller with the given sensor/ motornumber
      Must be called before use. The random generator is optional.
	 */
    virtual void init(int sensornumber, int motornumber, RandGen* randGen);

	/** @return Number of sensors the controller
      was initialised with or 0 if not initialised */
    virtual int getSensorNumber() const;

	/** @return Number of motors the controller
      was initialised with or 0 if not initialised */
    virtual int getMotorNumber() const;

	/** performs one step (includes learning).
      Calculates motor commands from sensor inputs.
      @param sensors sensors inputs scaled to [-1,1]
      @param sensornumber length of the sensor array
      @param motors motors outputs. MUST have enough space for motor values!
      @param motornumber length of the provided motor array
	 */
	virtual void step(const sensor* sensors, int sensornumber,
            motor* motors, int motornumber);

	/** performs one step without learning.
      @see step
	 */
	virtual void stepNoLearning(const sensor* , int number_sensors,
            motor* , int number_motors);


	/********* STORABLE INTERFACE ******/
	/// @see Storable
    virtual bool store(FILE* f) const;

	/// @see Storable
    virtual bool restore(FILE* f);

    virtual void setMC(double left, double right);


	/*************************************************************
	 *  Gaussian random variable: just a sum of uniform distribution
	 *  Average = 0, Variance = 1, (1.2)
	 *************************************************************/
    virtual double gauss();

protected:

	int number_sensors;
	int number_motors;

};

#endif
