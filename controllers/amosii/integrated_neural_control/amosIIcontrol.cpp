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
 *   $Log: tripodgate18dof.cpp,v $                                         *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/

/*The new controller (AmosIIControl(int aAMOStype,bool mMCPGs,bool mMuscleModelisEnabled)) has numerous privileges:
 *1) selection between AMOSv1 (aAMOStype=1) and AMOSv2 (aAMOStype=2). [DEFAULT = 2]
 *2) selection between single CPG-based controller (mMCPGs=false) and Multiple CPGs-based control (mMCPGs=true). [DEFAULT = false]
 *3) possibility to utilize muscle model (mMuscleModelisEnabled=true). [DEFAULT = false]
 *4) selection between integrated navigation controller (mNNC=true) and pure locomotion controller (mNNC=false). [DEFAULT = false]
 * */

#include <selforg/controller_misc.h>
#include <math.h>
#include "amosIIcontrol.h"
//#include <ode_robots/amosII.h>
using namespace matrix;
using namespace std;

AmosIIControl::AmosIIControl(int aAMOSversion,bool mMCPGs,bool mMuscleModelisEnabled) : AbstractController("AmosIIControl", "$Id: amosIIcontrol.cpp,v 0.1 $") {

	//---ADD YOUR initialization here---//
	t = 0; // step counter
	MCPGs=mMCPGs;
	if(MCPGs)
		num_cpgs = 6;
	else
		num_cpgs = 1;

	locomotion_control = new NeuralLocomotionControl(aAMOSversion, mMCPGs, mMuscleModelisEnabled/*, mNNC*/);

	if(MCPGs==true)
	{
		// press Ctrl M to display matrix
		for(unsigned int i_cpg = 0; i_cpg < num_cpgs; i_cpg++){
/*			string numstr; // enough to hold all numbers up to 64-bits
			sprintf(numstr, " [%u]", i_cpg);
			string numstr_lr; // enough to hold all numbers up to 64-bits
			sprintf(numstr_lr, "%u", i_cpg%3);*/
			string lr = (i_cpg<3) ? "R" : "L";
			addInspectableValue("CPG" + to_string(i_cpg),&locomotion_control->cpg_output.at(i_cpg).at(0), lr + to_string(i_cpg%3) + "_0");	//to_string() is C++11
			addInspectableValue("CPG" + to_string(i_cpg),&locomotion_control->cpg_output.at(i_cpg).at(1), lr + to_string(i_cpg%3) + "_1");
		}
	}
	for(unsigned int i_leg = 0; i_leg < 6; i_leg++){
		string lr = (i_leg<3) ? "R" : "L";
		addInspectableValue("FCprepro" + to_string(i_leg),&preprocessing_learning.preprosensor.at(R0_fs+i_leg).at(0), lr + to_string(i_leg%3));
	}

	//Add edit parameter on terminal
	Configurable::addParameter("cin", &locomotion_control->Control_input,  /*minBound*/ -10,  /*maxBound*/ 10, "test description" );
	Configurable::addParameter("input3", &locomotion_control->input.at(3),  /*minBound*/ -1,  /*maxBound*/ 1, "test description" );
	Configurable::addParameter("input4", &locomotion_control->input.at(4),  /*minBound*/ -1,  /*maxBound*/ 1, "test description" );
}

AmosIIControl::~AmosIIControl() {

}
void AmosIIControl::enableContactForceMech() //enable sensory feedback mechanism
{
	for (unsigned int i=0 && MCPGs; i<num_cpgs;i++){
		locomotion_control->nlc.at(i)->enableContactForce(MCPGs);
		std::cout << "[" << i << "] " << "contactForce is enabled -> " << locomotion_control->nlc.at(i)->contactForceIsEnabled<< "\n";
	}
}

void AmosIIControl::disableContactForceMech() //disable sensory feedback mechanism
{
	for (unsigned int i=0 && MCPGs; i<num_cpgs;i++){
		locomotion_control->nlc.at(i)->disableContactForce();
		std::cout << "[" << i << "] " << "contactForce is disabled -> " << locomotion_control->nlc.at(i)->contactForceIsEnabled<< "\n";
	}
}

// Frequency increases by increasing modulatory input.
void AmosIIControl::increaseFrequency()
{
	for (unsigned int i=0 && MCPGs; i<num_cpgs;i++)
		locomotion_control->nlc.at(i)->changeControlInput(locomotion_control->nlc.at(i)->Control_input+0.01);
	std::cout << "Frequency increases"<<endl;
	std::cout << "Modulatory input:" << locomotion_control->nlc.at(0)->Control_input<<endl;
}
// Frequency decreases by decreasing modulatory input.
void AmosIIControl::decreaseFrequency()
{
	for (unsigned int i=0 && MCPGs; i<num_cpgs;i++)
		locomotion_control->nlc.at(i)->changeControlInput(locomotion_control->nlc.at(i)->Control_input-0.01);
	std::cout << "Frequency decreases"<<endl;
	std::cout << "Modulatory input:" << locomotion_control->nlc.at(0)->Control_input<<endl;
}
void AmosIIControl::enableOscillatorCoupling() //enable oscillator coupling (fully connected network)
{
	for (unsigned int i=0 && MCPGs; i<num_cpgs;i++)
		locomotion_control->nlc.at(i)->enableoscillatorsCoupling(MCPGs);
	std::cout << "Oscillator Coupling is enabled \n";
}
void AmosIIControl::disableOscillatorCoupling() //disable oscillator coupling (fully connected network)
{
	for (unsigned int i=0 && MCPGs; i<num_cpgs;i++)
		locomotion_control->nlc.at(i)->disableoscillatorsCoupling();
	std::cout << "Oscillator Coupling is disabled \n";
}

void AmosIIControl::enableTripodGait()
{
	for (unsigned int i=0 && MCPGs; i<num_cpgs;i++)
	{
		int gaitPattern=0;//Tripod
		locomotion_control->nlc.at(i)->changeGaitpattern(gaitPattern);
	}
	std::cout << "Tripod \n";
}
void AmosIIControl::enableTetrapodGait()
{
	for (unsigned int i=0 && MCPGs; i<num_cpgs;i++)
	{
		int gaitPattern=1;  //Tetrapod
		locomotion_control->nlc.at(i)->changeGaitpattern(gaitPattern);
	}
	std::cout << "Tetrapod \n";
}

void AmosIIControl::enableWaveGait()
{
	for (unsigned int i=0 && MCPGs; i<num_cpgs;i++)
	{
		int gaitPattern=2;//wave
		locomotion_control->nlc.at(i)->changeGaitpattern(gaitPattern);
	}
	std::cout << "wave \n";
}
void AmosIIControl::enableIrregularGait()
{
	for (unsigned int i=0 && MCPGs; i<num_cpgs;i++)
	{
		int gaitPattern=3;//irregular gait.
		locomotion_control->nlc.at(i)->changeGaitpattern(gaitPattern);
	}
	std::cout << "irregularEnable \n";
}

void AmosIIControl::flipBJCbool(){
	locomotion_control->switchon_backbonejoint = !(locomotion_control->switchon_backbonejoint);
	printf("BJC: %s\n", locomotion_control->switchon_backbonejoint ? "ON" : "OFF");
	if(locomotion_control->switchon_backbonejoint && locomotion_control->switchon_obstacle){
		printf("Obstacle avoidance cannot run simultaneously.\n");
		flipOAbool();
	}
}

void AmosIIControl::flipOAbool(){
	locomotion_control->switchon_obstacle = !(locomotion_control->switchon_obstacle);
	printf("Obstacle avoidance: %s\n", locomotion_control->switchon_obstacle ? "ON" : "OFF");
	if(locomotion_control->switchon_backbonejoint && locomotion_control->switchon_obstacle){
		printf("BJC cannot run simultaneously.\n");
		flipBJCbool();
	}
}

void AmosIIControl::flipReflexesbool(){
	locomotion_control->switchon_reflexes = !(locomotion_control->switchon_reflexes);
	printf("Reflexes: %s\n", locomotion_control->switchon_reflexes ? "ON" : "OFF");
}

void AmosIIControl::init(int sensornumber, int motornumber, RandGen* randGen) {
	numbersensors = sensornumber;
	numbermotors = motornumber;
	x.resize(sensornumber);
	y.resize(AMOSII_MOTOR_MAX);
}

/// performs one step (includes learning). Calulates motor commands from sensor inputs.
void AmosIIControl::step(const sensor* x_, int number_sensors, motor* y_, int number_motors) {

	assert(number_sensors == numbersensors);
	assert(number_motors == numbermotors);


	/************   0) Sensor inputs   ************/

	for (unsigned int i = 0; i < AMOSII_SENSOR_MAX; i++) {
		x.at(i) = x_[i];
	}


	/******   1) Neural preprocessing and learning   ******/

	std::vector <vector<double> > x_prep = preprocessing_learning.step_npp(x);


	/**********   3) Neural locomotion control   **********/

	y = locomotion_control->step_nlc(x, x_prep,false);


	 /****************   4) Motor outputs   ****************/

	for (unsigned int i = 0; i < AMOSII_MOTOR_MAX; i++) {
		y_[i] = y.at(i);
	}



	// update step counter
	t++;
}
;

/** stores the controller values to a given file. */
bool AmosIIControl::store(FILE* f) const {
	return true;
}

/** loads the controller values from a given file. */
bool AmosIIControl::restore(FILE* f) {
	//	Configurable::parse(f);
	return true;
}

