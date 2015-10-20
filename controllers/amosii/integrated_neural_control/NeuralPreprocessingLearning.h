/*
 * NeuralPreprocessingLearning.h
 *
 *  Created on: May 2, 2011
 *      Author: poramate
 *
 *      Edited by Dennis Goldschmidt
 *      Apr 27, 2012
 *
 */

#ifndef NEURALPREPROCESSINGLEARNING_H_
#define NEURALPREPROCESSINGLEARNING_H_

#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <ode_robots/amosiisensormotordefinition.h>


#include "preprocessing_modules/IRM.h"
#include "preprocessing_modules/FSM.h"
#include "preprocessing_modules/US_OA.h"
#include "preprocessing_modules/US_ON.h"
#include "utils/delayline.h"
#include "utils/ann-framework/ann.h"

using namespace std;

/// Class for Neural preprocessing and learning------------


class NeuralPreprocessingLearning{

public:
    //Save files
    ofstream outFilenpp1;

    //---Start Define functions---//
    NeuralPreprocessingLearning();
    ~NeuralPreprocessingLearning();
    vector< vector<double> > step_npp(const vector<double> in0);

    double sigmoid(double num)
    {
        return 1.0/(1.0+exp(-num));
    }

    //---End Define functions---//

    //---Start Define vector----//
    // add public available variables
    vector<double> mappingsensor;
    vector<double> sensor_activity;
    vector<double> sensor_output;
    vector< vector<double> > preprosensor;
    //---End Define vector----//


    US_Obstacleavoidance * OA;
    vector< US_Obstaclenegotiation* > ON;
    vector< ANN* > sensor_map_net;



    bool FRONT_IR;
    double rate;
    bool switchon_IRlearning;

    //Save files

private:
    // add private  variables
    double lowpass;
    const double threshold = 0.85;
    double ir_learnrate;
    int delay;

    const int num_behaviors = 2;
};




#endif /* NEURALPREPROCESSINGLEARNING_H_ */

