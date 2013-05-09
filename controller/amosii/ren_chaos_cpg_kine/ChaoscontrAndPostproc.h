/*
 * ChaoscontrAndPostProc.h
 *
 *  Created on: Aug 15, 2011
 *      Author: Ren Guanjiao
 */

#ifndef CHAOSCONTRANDPOSTPROC_H_
#define CHAOSCONTRANDPOSTPROC_H_

#include <vector>
#include <cmath>

#include <ode_robots/amosiisensormotordefinition.h>
//#include "sensor_motor_definition.h"

#include <assert.h>
#include <stdlib.h>

//Save files
#include <iostream>
#include <fstream>
#include <string.h>
using namespace std;
//Save files

#define PERIODMAX 10
#define FAC 8
#define	EPSILON 0.5

// Class for ChaosControl and PostProcessing
class ChaoscontrAndPostproc {
  public:
    ChaoscontrAndPostproc(int p1);
    ~ChaoscontrAndPostproc();
    void ChaosControl();

    //---Start Save files---//
    ofstream chaosoutput;
    //---End Save files---//

    void reset();
    void setPeriod(int period) {
      period_ = period;
      reset();
    }
    ;

    double getOutput1() {
      return output1;
    }
    ;
    double getOutput2() {
      return output2;
    }
    ;

  protected:
    
    double lr_;
    double o1_, o2_;
    int period_;
    int n_;
    double diff_n1_, diff_n2_, diffi_;
    double cl_;
    double input1_, input2_;
    double w11_, w12_, w21_, w22_, theta1_, theta2_;
    bool learning;

    int normalize_;
    double thetahys_[2];
    int percount;
    double input_hys1, input_hys2;
    int Freq;

    double ahys1;
    double whys;
    double ahys_old1;
    double ahys2;
    double ahys_old2;

    double inverse_;
    double slopeUP_;
    double slopeDOWN_;
    double output1;
    double output2;

    typedef struct {
        double o1, o2;
    } data_t;
    data_t* data_;

    double sigmoid_(double x) {
      return (1. / (1. + exp(-x)));
    }
    ;

    double tanh_(double x) {
      return (2. / (1. + exp(-10. * x)) - 1);
    }
    ;

    double sigmoid_steep(double x) {
      return 1. / (1. + exp(-1000 * (10000 - x)));
    }
    ;

    double deri_sigmoid_steep(double x) {
      double deribeta_steep = 10;
      double deritheta_steep = 10000;
      //deri sig = f(x)(1-f(x))
      return (1. / (1. + exp(-(deribeta_steep) * (deritheta_steep - x))))
          * (1 - (1. / (1. + exp(-(deribeta_steep) * (deritheta_steep - x)))));
    }
    ;

    void oscistep();
    void step();
    void PostProcessing();
    
};

#endif //CHAOSCONTRANDPOSTPROC_H_
