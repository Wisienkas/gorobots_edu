#ifndef _LOCAL_LEG_CONTROLLER
#define _LOCAL_LEG_CONTROLLER

#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>    // std::rotate

/// Class including the intra limb control for a millipede robot.
/// The control system uses local sensory feedback to modulate CPG outputs and
/// a learning system able to adjust automatically the sensory feedback strenght

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
    std::vector<double> w;
    double B1 = 0.01, B2 = 0.01;
    double a1 = 0, a2 = 0;
    double C1 = 0, C2 = 0;
    double prevC1 = 0, prevC2 = 0;

    // active learning
    bool learning_SF = true;
    bool local_SFweight = true;
    double SFweight = 0;
    double efferenceCopy_error = 0;


    // Variables for neural inter limb coordination (not used)
    std::vector<double> input_synapses, gait_error, delays;
    std::vector<double> interconnectionFilter;


private:
    // CPG activity
    void CPG_step();
    void CPG_step(double input1, double input2);

    //Sensory feedback (SF) computation
    double sFeedFilter(double dataInput);

    //Learning algorithm for SF adjustment
    void sFeedbackLearning_step(double touchSensorData);

    double dslow = 0.0, dfast = 0;
    double dW = 0, Af = 0, As = 0, Bf = 0, Bs = 0;

    std::vector<double> lowpass;// filtererror;
    double lowpass_n, filtererror_n;
    double fwdMod;

//    bool legPos_fixation = false;



};













#endif
