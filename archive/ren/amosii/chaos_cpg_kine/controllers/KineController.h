/*
 * KineController.h
 *
 *  Created on: Oct 24, 2011
 *      Author: Ren Guanjiao
 */

#ifndef KINECONTROLLER_H_
#define KINECONTROLLER_H_

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

#define STRIDE 50
#define LEGH 40

// Class for ChaosControl and PostProcessing
class KineController {
  public:
    KineController();
    ~KineController();

    std::vector<double> out;

    void Kine(double tmpRFx, double tmpRFy, double tmpRFz, double tmpRMx, double tmpRMy, double tmpRMz, double tmpRHx,
        double tmpRHy, double tmpRHz, double tmpLFx, double tmpLFy, double tmpLFz, double tmpLMx, double tmpLMy,
        double tmpLMz, double tmpLHx, double tmpLHy, double tmpLHz);

  protected:
    double l1, l2, l3;
    double H; //height of body
    int step;
    bool swingflag;

    typedef struct {
        double X, Y, Z;
    } Point;
    Point legend_RF, legend_RM, legend_RH, legend_LF, legend_LM, legend_LH;
    Point desire;

    typedef struct {
        double TC, CTr, FTi;
    } legangle;
    legangle RF, RM, RH, LF, LM, LH;
    legangle RF_ini, RM_ini, RH_ini, LF_ini, LM_ini, LH_ini; //initial angle of different legs
        
    void SingleLegInverKine_RF(double x, double y, double z);
    void SingleLegInverKine_RM(double x, double y, double z);
    void SingleLegInverKine_RH(double x, double y, double z);
    void SingleLegInverKine_LF(double x, double y, double z);
    void SingleLegInverKine_LM(double x, double y, double z);
    void SingleLegInverKine_LH(double x, double y, double z);
};

#endif //KINECONTROLLER_H_
