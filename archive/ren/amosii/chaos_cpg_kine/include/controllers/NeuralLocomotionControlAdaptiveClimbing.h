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
//#include "sensor_motor_definition.h"

//Save files /read file
#include <iostream>
#include <fstream>
#include <string.h>
#include "KineController.h"

//atof function
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

using namespace std;
//Save files

#define R0 0
#define R1 1
#define R2 2
#define L0 3
#define L1 4
#define L2 5


#define SWI_EL 0
#define SWI_DP 1
#define SUP 2

#define F_L 0.3   //Force Threshold low
#define F_H 0.6   //Force Threshold high

//3) Class for Neural locomotion control------

class NeuralLocomotionControlAdaptiveClimbing {
    
  public:
    
    //---Start Define functions---//
    NeuralLocomotionControlAdaptiveClimbing();
    ~NeuralLocomotionControlAdaptiveClimbing();

    double sigmoid(double num) {
      return 1. / (1. + exp(-num));
    }
    
    std::vector<double> step_nlc(const std::vector<double> in0 /*from neural preprocessing*/,
        const std::vector<double> in1 /*from neural learning*/); //, bool Footinhibition=false);
    // if several sensor values (like preprocessed or output of memory and learning are used, extend parameters)
    // std::vector<double> step_nlc(const std::vector<double> in0, const std::vector<double> in1, const std::vector<double> in2);
    //---End  Define functions---//
    
    //---Start Save files---//
    ofstream outFilenlc1;
    //---End Save files---//
    
    struct point {
        double x;
        double y;
        double z;
    };

    //---Start Define vector----//
    
    //---Input neurons
    std::vector<double> input;
    std::vector<double> fs;

    //---CPG
    std::vector<double> cpg_activity; //CPG neural activities
    std::vector<double> cpg_output; //CPG neural outputs
    std::vector<std::vector<double> > cpg_w; //CPG neural weights
    double cpg_bias; //CPG bias
    double Control_input;

    //---Delay line
    std::vector<double> buffer_cpg; //buffer for store CPG
    std::vector<double> cpg_leg; //CPG signal to each leg
    std::vector<double> cpg_pre; //for remembering cpg value in last time
    
    //---Trajectory generator
    std::vector<point> Tj; //Trajectory of foot end
    
    //---Force Feedback
    std::vector<point> Tj_ForceFB; //Trajectory after force feedback
    std::vector<point> Tj_pre;
    unsigned int leg_state[6];
    
    //---Inverse Kinematics
    KineController robot;

    //---motor neuron
    std::vector<double> m; //motor outputs as neural activation (19 motors)
    
    //---End Define vector----//
    
  private:
    
    int tau; //delay time
    int tau_l; //left legs delay time
    int global_count;
    
};

#endif /* NEURALLOCOMOTIONCONTROLADAPTIVECLIMBING_H_ */
