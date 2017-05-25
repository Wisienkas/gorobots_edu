#ifndef _LOCAL_LEG_CONTROLLER
#define _LOCAL_LEG_CONTROLLER

#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>    // std::rotate


class localLegController{
public:

    localLegController(std::vector<double> w_);
    localLegController(std::vector<double> w_, std::vector<double> learnParams_);
    ~localLegController(){};


    //Control steps and CPG reset
    std::vector<double> control_step(double touchSensorData);
    std::vector<double> control_step(double touchSensorData, double CPGinput1, double CPGinput2);

    void reset();
    double liftamplitude, wideamplitude;
    double psn1 = 0, psn2 = 0;


    // CPG activity
    void CPG_step();
    void CPG_step(double input1, double input2);

    std::vector<double> w;
    double B1 = 0.01, B2 = 0.01;
    double a1 = 0, a2 = 0;
    double C1 = 0, C2 = 0;

    //Sensory feedback plasticity
    double sFeedFilter(double dataInput);
    void sFeedbackLearning_step(double touchSensorData);

    bool learning_SF = true;
    bool local_SFweight = true;
    double SFweight = 0;
    double dslow = 0.0, dfast = 0;
    double dW = 0, Af = 0, As = 0, Bf = 0, Bs = 0;
    double efferenceCopy_error = 0;
    double prevC1 = 0, prevC2 = 0;

    std::vector<double> lowpass;// filtererror;
    double lowpass_n, filtererror_n;
    double fwdMod;

    bool legPos_fixation = false;


    //inter-limb plasticity

    std::vector<double> input_synapses, gait_error, delays;
    std::vector<double> interconnectionFilter;


};













#endif
