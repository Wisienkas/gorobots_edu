/*
 * NeuralLocomotionControlAdaptiveClimbing.h
 *
 *  Created on: May 2, 2011
 *      Author: poramate
 */

#ifndef NEURALLOCOMOTIONCONTROLADAPTIVECLIMBING_H_
#define NEURALLOCOMOTIONCONTROLADAPTIVECLIMBING_H_

#include <vector>
#include <cmath>

#include <ode_robots/amosiisensormotordefinition.h>

//Save files /read file
#include <iostream>
#include <fstream>
#include <string.h>

//atof function
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "KineController.h"

using namespace std;
//Save files

//3) Class for Neural locomotion control------

class NeuralLocomotionControlAdaptiveClimbing{

public:

	//---Start Define functions---//
	NeuralLocomotionControlAdaptiveClimbing(int amosVersion);
	~NeuralLocomotionControlAdaptiveClimbing();

	double sigmoid(double num)
	{
	return 1./(1.+exp(-num));
	}

	std::vector<double> step_nlc(const std::vector<double> in0 /*from neural preprocessing*/, const std::vector<double> in1 /*from neural learning*/);//, bool Footinhibition=false);
	// if several sensor values (like preprocessed or output of memory and learning are used, extend parameters)
	// std::vector<double> step_nlc(const std::vector<double> in0, const std::vector<double> in1, const std::vector<double> in2);

	//---End  Define functions---//



	//---Start Save files---//
	ofstream outFilenlc1;
	//---End Save files---//


	//---Start Define vector----//

	//---Input neurons
	std::vector<double> input;

	double Control_input;

	std::vector<double> m;								//motor outputs as neural activation (19 motors)


    //Reading motor signals from Text
    vector<double> m_r0_t;
    vector<double> m_r1_t;
    vector<double> m_r2_t;
    vector<double> m_l0_t;
    vector<double> m_l1_t;
    vector<double> m_l2_t;

    double m_r0_t_old;
    double m_r1_t_old;
    double m_r2_t_old;
    double m_l0_t_old;
    double m_l1_t_old;
    double m_l2_t_old;

    double m_r0_text;
    double m_r1_text;
    double m_r2_text;
    double m_l0_text;
    double m_l1_text;
    double m_l2_text;



    int i_text_loop;
    int ii;
    bool initialized;

    int amos_version;

    KineController* tripod;

    //---End Define vector----//



private:

	int global_count;

	bool reading_text_testing;
};

#endif /* NEURALLOCOMOTIONCONTROLADAPTIVECLIMBING_H_ */
