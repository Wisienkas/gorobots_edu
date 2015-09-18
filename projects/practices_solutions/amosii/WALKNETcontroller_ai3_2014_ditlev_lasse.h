#ifndef __EMPTYCONTROLLER_H
#define __EMPTYCONTROLLER_H

#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>
#include <selforg/configurable.h>

#include <assert.h>
#include <cmath>
#include <stdlib.h>
#include <string.h>

#include <fstream>
#include <iostream>


#include <selforg/matrix.h>

#define PLOT_GAIT 0

//Leg defines
#define LEG_TYPE_FRONT 0
#define LEG_TYPE_MIDDLE 1
#define LEG_TYPE_REAR 2

//Backjoint defines
#define FRONT_SENSOR_DIST_THRES_UP 0.5
#define FRONT_SENSOR_DIST_THRES_DOWN 0.1
#define BACK_JOINT_ANGLE 1;

//WALKNET DEFINES----------------------------
#define TURNING_SPEED_GAIN 1.5


//Scales parameters compared to the walknet paper
#define SCALE 0.03
#define SCALE_LEG_MOVE 1

#define SCALE_LEG_REAR_C 1
#define LEG_OFFSET_REAR_C 0

//weights for leg force reccurent net
#define LEG_FORCE_REC_W 0.95
#define LEG_FORCE_INP_W 0.05

//When leg is active
#define LEG_ACTIVE_FORCE_THRES 0.05


//Scaling for C joint movement
#define SCALE_LEG_MOVE_C_FRONT 1
#define SCALE_LEG_MOVE_C_MIDDLE 1.2
#define SCALE_LEG_MOVE_C_REAR 1.5

//PEP extremas, use from walknet paper * scale
#define PEP_EXTREME -10*SCALE
#define PEP_EXTREME_REAR -20*SCALE
#define PEP_THRESHOLD_LOW -3.3*SCALE
#define PEP_THRESHOLD_HIGH -5*SCALE

//Coactivation load thresholds
#define CO_LOAD_THRES_FRONT 0.5
#define CO_LOAD_THRES_MIDDLE 0.9
#define CO_LOAD_THRES_REAR 0.55

//threshold to determine if lef is in stance, and a coeffecient ofr the stance filter
#define STANCE_FORCE_THRESHOLD 0.1
#define STANCE_FILTER_COEFF 0.3

#define PEP_POSITIVE_VALUE 20*SCALE
//Weights, see walknet paper for w1 - w5, w 6 specifies PEP movmenet of the legs as by rule 6
#define W1 -0.4
#define W2_FRONT -0.4
#define W2_CONTRA -0.2
#define W3_BACK -0.5
#define W3_CONTRA -0.3
#define W5 0.5
#define W6_F 0.55
#define W6_M 0.2
#define W6_R -0.3

//Threshold used in HEAVISIDE function
#define HEAVISIDE_THRES 0

//Weights for backbone joint NN
//Load references for the legs
#define JOINT_FF_REF 0.15
#define JOINT_FM_REF 0.5
#define JOINT_FR_REF 0.35

//weights used for the foot contacts
#define JOINT_FF_REC_W 0.9
#define JOINT_FF_INP_W 0.1
#define JOINT_FF_OUT_W -4
#define JOINT_FF_SIG_SCALE 15

#define JOINT_FM_REC_W 0.9
#define JOINT_FM_INP_W 0.1
#define JOINT_FM_OUT_W -12
#define JOINT_FM_SIG_SCALE 15

#define JOINT_FR_REC_W 0.95
#define JOINT_FR_INP_W 0.05
#define JOINT_FR_OUT_W -10


//weights fot the joint reccurent net and US sensor
#define JOINT_JOINT_REC_W 0.85
#define JOINT_JOINT_INP_W 0.15
#define JOINT_US_OUT_W 50

#define JOINT_SIG_OFFSET 5
#define JOINT_SIG_GAIN  4
//US distance threshold
#define  JOINT_US_DIST_THRES 0.1

//FTP-CONTROLLER settings, the FTP controller is not used in current version
#define LEG_FL_SUPPORT 0.01
#define LEG_FH_SUPPORT 0.4
#define LEG_DESIRED -0.5

#define LEG_MAX_SPEED 0.008
#define LEG_MIN_SPEED 0.0001

#define LEG_FAST_DEP_GAIN_FRONT 0.5
#define LEG_SLOW_DEP_GAIN_FRONT 0.3
#define LEG_SLOW_ELE_GAIN_FRONT 0.07

#define LEG_FAST_DEP_GAIN_MIDDLE 0.6
#define LEG_SLOW_DEP_GAIN_MIDDLE 0.45
#define LEG_SLOW_ELE_GAIN_MIDDLE 0.004

#define LEG_FAST_DEP_GAIN_REAR 0.5
#define LEG_SLOW_DEP_GAIN_REAR 0.4
#define LEG_SLOW_ELE_GAIN_REAR 0.07

//Weights used in the pattern generators when legs go inactive
#define CP_WEIGHT_F_1_1 1.5
#define CP_WEIGHT_F_2_2 1.5
#define CP_WEIGHT_F_1_2 0.4
#define CP_WEIGHT_F_2_1 -0.4
#define CP_WEIGHT_OFFSET 0.01

//Bias and gains used for legs when CP is used
#define LEG_BIAS_F 0.5
#define LEG_BIAS_M 0
#define LEG_BIAS_R -0.4

#define LEG_GAIN_F 0.6
#define LEG_GAIN_M 0.3
#define LEG_GAIN_R 0.6


typedef struct EmptyControllerConf {
    double WeightH1_H1;
    double WeightH2_H2;
    double WeightH1_H2;
    double WeightH2_H1;
    double fact;
    double direction;
    double bias;




} EmptyControllerConf;


/**
 * Empty robot controller.
 * The controller gets a number of input sensor values each timestep
 * and has to generate a number of output motor values.
 *
 * Go to the step() function and enter the control commands with respect to your task!
 *
 */


class EmptyController : public AbstractController {


  public:
      EmptyController()
          : AbstractController("EmptyController", "$Id: tripodgait18dof.cpp,v 0.1 $"){
      t = 0;


      outputH1 = 0.001;
      outputH2 = 0.001;

          output_CP_R0_1 = 0.001;
          output_CP_R1_1 = 0.001;
          output_CP_R2_1 = 0.001;
          output_CP_L0_1 = 0.001;
          output_CP_L1_1 = 0.001;
          output_CP_L2_1 = 0.001;

          output_CP_R0_2 = 0.001;
          output_CP_R1_2 = 0.001;
          output_CP_R2_2 = 0.001;
          output_CP_L0_2 = 0.001;
          output_CP_L1_2 = 0.001;
          output_CP_L2_2 = 0.001;

          outputUS1 = 0;
          outputUS2 = 0;

          gaitOFile.open("gait.txt");

          stance_filter_R0 = 0;
          stance_filter_R1 = 0;
          stance_filter_R2 = 0;
          stance_filter_L0 = 0;
          stance_filter_L1 = 0;
          stance_filter_L2 = 0;

          stance_rise_R0 = 0;
          stance_rise_R1 = 0;
          stance_rise_R2 = 0;
          stance_rise_L0 = 0;
          stance_rise_L1 = 0;
          stance_rise_L2 = 0;

          //Swing values for legs
          swing_R0 = 0;
          swing_R1 = 0;
          swing_R2 = 0;
          swing_L0 = 0;
          swing_L1 = 0;
          swing_L2 = 0;

          debug_swing_W1_R1 = 0;
          debug_swing_W1_R2_R1_LOAD = 0;
          debug_W2_FRONT_stance_rise_R1 = 0;
          debug_W2_CONTRA_stance_rise_L0 = 0;
          debug_W3_CONTRA_pep_thres_L0 = 0;

        //Stance values for legs
          stance_R0 = 0;
          stance_R1 = 0;
          stance_R2 = 0;
          stance_L0 = 0;
          stance_L1 = 0;
          stance_L2 = 0;

          reccurent_FF = 0.2;
          reccurent_FM = 0.2;
          reccurent_FR = 0.3;
          recurrent_Joint = 0;

          output_FF = 0;
          output_FM = 0;
          output_FR = 0;
          output_Joint = 0;

          reccurent_FC_L0 = 1;
          reccurent_FC_R0 = 1;
          reccurent_FC_R1 = 1;
          reccurent_FC_L1 = 1;
          reccurent_FC_R2 = 1;
          reccurent_FC_L2 = 1;

          leg_state_L0 = 1;
          leg_state_R0 = 1;
          leg_state_R1 = 1;
          leg_state_L1 = 1;
          leg_state_R2 = 1;
          leg_state_L2 = 1;

          turn_scale_right = 0;
          turn_scale_left = 0;
          goal_error = 0;

          curr_pep_R0 = PEP_EXTREME;
          curr_pep_R1 = PEP_EXTREME;
          curr_pep_R2 = PEP_EXTREME;
          curr_pep_L0 = PEP_EXTREME;
          curr_pep_L1 = PEP_EXTREME;
          curr_pep_L2 = PEP_EXTREME;

          pep_thres_R0 = 0;


          foot_gait_R0 = 0;
          foot_gait_R1 = 0;
          foot_gait_R2 = 0;
          foot_gait_L0 = 0;
          foot_gait_L1 = 0;
          foot_gait_L2 = 0;


      conf.WeightH1_H1 = 1.5;
      conf.WeightH2_H2 = 1.5;
      conf.WeightH1_H2 = 0.4;
      conf.WeightH2_H1 = -0.4;
      conf.direction = -1;


          lastR0 = 0;
          lastR1 = 0;
          lastR2 = 0;
          lastL0 = 0;
          lastL1 = 0;
          lastL2 = 0;


          addInspectableValue("FootDep_R0", &foot_dep_R0,"FootDep_R0");
          addInspectableValue("FootDep_R1", &foot_dep_R1,"FootDep_R1");
          addInspectableValue("FootDep_R2", &foot_dep_R2,"FootDep_R2");
          addInspectableValue("FootDep_L0", &foot_dep_L0,"FootDep_L0");
          addInspectableValue("FootDep_L1", &foot_dep_L1,"FootDep_L1");
          addInspectableValue("FootDep_L2", &foot_dep_L2,"FootDep_L2");
          addInspectableValue("FootContactSum", &FootContactSum,"FootContactSum");
          addInspectableValue("US_preprocessed", &outputUS2,"US_preprocessed");
          addInspectableValue("stance_filter_R0", &stance_filter_R0,"stance_filter_R0");
          addInspectableValue("stance_touchdown_R0", &stance_rise_R0,"stance_touchdown_R0");
          addInspectableValue("PEP_THRES_R0", &pep_thres_R0,"PEP_THRES_R0");
          addInspectableValue("PEP_R0", &curr_pep_R0,"PEP_R0");
          addInspectableValue("PEP_R1", &curr_pep_R1,"PEP_R1");
          addInspectableValue("PEP_R2", &curr_pep_R2,"PEP_R2");
          addInspectableValue("PEP_L0", &curr_pep_L0,"PEP_L0");
          addInspectableValue("PEP_L1", &curr_pep_L1,"PEP_L1");
          addInspectableValue("PEP_L2", &curr_pep_L2,"PEP_L2");

          addInspectableValue("swing_R0", &swing_R0,"swing_R0");
          addInspectableValue("swing_R1", &swing_R1,"swing_R1");
          addInspectableValue("swing_R2", &swing_R2,"swing_R2");
          addInspectableValue("swing_L0", &swing_L0,"swing_L0");
          addInspectableValue("swing_L1", &swing_L1,"swing_L1");
          addInspectableValue("swing_L2", &swing_L2,"swing_L2");


          addInspectableValue("stance_R0", &stance_R0,"stance_R0");
          addInspectableValue("stance_R1", &stance_R1,"stance_R1");
          addInspectableValue("stance_R2", &stance_R2,"stance_R2");
          addInspectableValue("stance_L0", &stance_L0,"stance_L0");
          addInspectableValue("stance_L1", &stance_L1,"stance_L1");
          addInspectableValue("stance_L2", &stance_L2,"stance_L2");


          addInspectableValue("reccurent_FF", &reccurent_FF,"reccurent_FF");
          addInspectableValue("reccurent_FM", &reccurent_FM,"reccurent_FM");
          addInspectableValue("reccurent_FR", &reccurent_FR,"reccurent_FR");
          addInspectableValue("recurrent_Joint", &recurrent_Joint,"recurrent_Joint");
          addInspectableValue("output_FF", &output_FF,"output_FF");
          addInspectableValue("output_FM", &output_FM,"output_FM");
          addInspectableValue("output_FR", &output_FR,"output_FR");
          addInspectableValue("output_Joint", &output_Joint,"output_Joint");


          addInspectableValue("debug_swing_W1_R1", &debug_swing_W1_R1,"debug_swing_W1_R1");
          addInspectableValue("debug_swing_W1_R2_R1_LOAD", &debug_swing_W1_R2_R1_LOAD,"debug_swing_W1_R2_R1_LOAD");
          addInspectableValue("debug_W2_FRONT_stance_rise_R1", &debug_W2_FRONT_stance_rise_R1,"debug_W2_FRONT_stance_rise_R1");
          addInspectableValue("debug_W2_CONTRA_stance_rise_L0", &debug_W2_CONTRA_stance_rise_L0,"debug_W2_CONTRA_stance_rise_L0");
          addInspectableValue("debug_W3_CONTRA_pep_thres_L0", &debug_W3_CONTRA_pep_thres_L0,"debug_W3_CONTRA_pep_thres_L0");


          addInspectableValue("reccurent_FC_R0", &reccurent_FC_R0,"reccurent_FC_R0");
          addInspectableValue("reccurent_FC_R1", &reccurent_FC_R1,"reccurent_FC_R1");
          addInspectableValue("reccurent_FC_R2", &reccurent_FC_R2,"reccurent_FC_R2");
          addInspectableValue("reccurent_FC_L0", &reccurent_FC_L0,"reccurent_FC_L0");
          addInspectableValue("reccurent_FC_L1", &reccurent_FC_L1,"reccurent_FC_L1");
          addInspectableValue("reccurent_FC_L2", &reccurent_FC_L2,"reccurent_FC_L2");

          addInspectableValue("leg_state_R0", &leg_state_R0,"leg_state_R0");
          addInspectableValue("leg_state_R1", &leg_state_R1,"leg_state_R1");
          addInspectableValue("leg_state_R2", &leg_state_R2,"leg_state_R2");
          addInspectableValue("leg_state_L0", &leg_state_L0,"leg_state_L0");
          addInspectableValue("leg_state_L1", &leg_state_L1,"leg_state_L1");
          addInspectableValue("leg_state_L2", &leg_state_L2,"leg_state_L2");

          addInspectableValue("turn_scale_right", &turn_scale_right,"turn_scale_right");
          addInspectableValue("turn_scale_left", &turn_scale_left,"turn_scale_left");
          addInspectableValue("goal_error", &goal_error,"goal_error");


          addInspectableValue("foot_gait_R0", &foot_gait_R0,"foot_gait_R0");
          addInspectableValue("foot_gait_R1", &foot_gait_R1,"foot_gait_R1");
          addInspectableValue("foot_gait_R2", &foot_gait_R2,"foot_gait_R2");
          addInspectableValue("foot_gait_L0", &foot_gait_L0,"foot_gait_L0");
          addInspectableValue("foot_gait_L1", &foot_gait_L1,"foot_gait_L1");
          addInspectableValue("foot_gait_L2", &foot_gait_L2,"foot_gait_L2");


          foot_gait_R0 = 0;
          foot_gait_R1 = 0;
          foot_gait_R2 = 0;
          foot_gait_L0 = 0;
          foot_gait_L1 = 0;
          foot_gait_L2 = 0;

    };

    //Stance and swing functions-----------
    virtual double stance_net_C(double c_ang, double f_ang)
    {
        return -0.1*c_ang-0.05*f_ang;
    }

    virtual double swing_net_T(double t_ang)
    {
        return 0.1572-0.25*t_ang;
    }

    virtual double swing_net_C(double t_ang, double c_ang)
    {
        return 0.1258-0.51*t_ang-0.2*c_ang;
    }

    virtual double swing_net_F(double f_ang)
    {
        return -0.28*f_ang;
    }
    //--------------------------------------------------
    virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0){
      //Tripodgait for 18 DOF Hexapod
      assert(motornumber>=18);
    };

    virtual ~EmptyController(){};

    /// returns the name of the object (with version number)
    virtual paramkey getName() const {
      return name;
    }
    /// returns the number of sensors the controller was initialised with or 0
    /// if not initialised
    virtual int getSensorNumber() const {
      return number_channels;
    }
    /// returns the mumber of motors the controller was initialised with or 0 if
    // not initialised
    virtual int getMotorNumber() const {
      return number_channels;
    }

    /// performs one step (includes learning).
    /// Calulates motor commands from sensor inputs.
    virtual void step(const sensor* x_, int number_sensors,
        motor* y_, int number_motors){
      stepNoLearning(x_, number_sensors, y_, number_motors);
    };

    //Distance to PEP, returns constant value when positive
    virtual double getPepDist(double pep, double pos)
    {
        double tempDist = pep-pos;
        if (tempDist < 0)
            return tempDist;
        else
            return PEP_POSITIVE_VALUE;
    }

    //Tries to calculate leg depresseion for FTP, not working correctly
    virtual double getLegDepress(double angle1, double angle2)
    {
        double y_first = sin(angle1)*3.5;
        double y_sec = sin(angle2-angle1)*6;
        double y_comb = -y_first-y_sec;
        //std::cout << "y_first:\t"<< y_first << "\ty_sec:\t" << y_sec << "\ty_comb:\t" << y_comb << "\n";

        return y_comb;
    }

    //A sigmoid function that thresholds to 1 or 0 if close to limits
    virtual double sigmoid(double inp, double base)
    {
        double tempRes = 1/(1+pow(base,-inp));

        if (tempRes < 0.05)
            return 0;
        else if (tempRes > 0.95)
            return 1;
        else
            return tempRes;
    }

    //Calculates leg speed for use in FTP controller
    virtual double getLegOutputSpeedGain(double angle1, double angle2, double foot_force, int leg_type)
    {

        double fast_dep = 0;
        double slow_dep = 0;
        double slow_ele = 0;
        double weak_support_force = 0;
        double full_support_force = 0;
        double leg_speed = 0;

        if (leg_type == LEG_TYPE_FRONT)
        {
            fast_dep = LEG_FAST_DEP_GAIN_FRONT;
            slow_dep = LEG_SLOW_DEP_GAIN_FRONT;
            slow_ele = LEG_SLOW_ELE_GAIN_FRONT;

            weak_support_force = LEG_FL_SUPPORT;
            full_support_force = LEG_FH_SUPPORT;
        }
        else if (leg_type == LEG_TYPE_MIDDLE)
        {
            fast_dep = LEG_FAST_DEP_GAIN_MIDDLE;
            slow_dep = LEG_SLOW_DEP_GAIN_MIDDLE;
            slow_ele = LEG_SLOW_ELE_GAIN_MIDDLE;

            weak_support_force = LEG_FL_SUPPORT*2;
            full_support_force = LEG_FH_SUPPORT*2;
        }
        else if (leg_type == LEG_TYPE_REAR)
        {
            fast_dep = LEG_FAST_DEP_GAIN_REAR;
            slow_dep = LEG_SLOW_DEP_GAIN_REAR;
            slow_ele = LEG_SLOW_ELE_GAIN_REAR;

            weak_support_force = LEG_FL_SUPPORT;
            full_support_force = LEG_FH_SUPPORT;
        }
        else
        {
            fast_dep = LEG_FAST_DEP_GAIN_FRONT;
            slow_dep = LEG_SLOW_DEP_GAIN_FRONT;
            slow_ele = LEG_SLOW_ELE_GAIN_FRONT;

            weak_support_force = LEG_FL_SUPPORT;
            full_support_force = LEG_FH_SUPPORT;
        }


        double leg_depress = getLegDepress(angle1, angle2);

        //positive when leg is too far down
        double dep_error = LEG_DESIRED-leg_depress;

        if (foot_force <= weak_support_force)
            leg_speed = fast_dep*dep_error;
        else if ((dep_error > 0) && (foot_force < full_support_force))
            return -100;
        else if (dep_error < 0)
            leg_speed = slow_dep*dep_error;
        else
            leg_speed = slow_ele*dep_error;


        if (abs(leg_speed) > LEG_MAX_SPEED)
            leg_speed = LEG_MAX_SPEED*getSign(leg_speed);

        return angle1 + leg_speed + LEG_MIN_SPEED*getSign(leg_speed);

    }

    //PEP threshold function
    virtual double thresholdFunc(double pos)
    {
        if (PEP_THRESHOLD_LOW < pos)
            return 0;
        else if ((PEP_THRESHOLD_LOW > pos) && (PEP_THRESHOLD_HIGH < pos))
            return PEP_THRESHOLD_LOW-pos;
        else
            return PEP_THRESHOLD_LOW- PEP_THRESHOLD_HIGH;
    }

    //Return 0 if negative, else return inp
    virtual double onlyPos(double inp)
    {
        if (0 < inp)
            return inp;
        else
            return 0;
    }

    //Get sign of variable
    virtual double getSign(double x)
    {
        if (x >= 0)
            return 1;
        else
            return -1;
    }

    //Calculate coactrivation
    virtual double getCoactivation(double load, double thres)
    {
        if (load < thres)
            return 0;
        else
            return load;
    }


    virtual double getStanceRise(double lastStance, double currStance)
    {
        if (lastStance+0.005 < currStance)
            return 1;
        else
            return 0;
    }

    //Heaviside function
    virtual double heaviside(double inp)
    {
        if (inp < HEAVISIDE_THRES)
            return 0;
        else
            return 1;
    }


    /// performs one step without learning. Calulates motor commands from sensor
    /// inputs.
    virtual void stepNoLearning(const sensor* x_, int number_sensors,
        motor* y_, int number_motors) {
        //Tripodgait for 18 DOF Hexapod

        assert(number_sensors >= 18);
        assert(number_motors >= 18);

        //Leftover, not used
        double activityH1 = conf.WeightH1_H1 * outputH1 + conf.WeightH1_H2 * outputH2  + 0.01;
        double activityH2 = conf.WeightH2_H2 * outputH2 + conf.WeightH2_H1 * outputH1  + 0.01;
        outputH1 = tanh(activityH1);
        outputH2 = tanh(activityH2);


        //US filter
        outputUS1 = outputUS1 * 0.95 + x_[FR_us] * 0.05;
        outputUS2 = outputUS2 * 0.9 + outputUS1 * 0.1;


        //Gait pattern generation, saves the leg number when in stance, can be saved in -txt file and shown in spreadsheet
        foot_gait_R0 = onlyPos(getSign(stance_R0-swing_R0))*7*leg_state_R0-1*(1-leg_state_R0);
        foot_gait_R1 = onlyPos(getSign(stance_R1-swing_R1))*6*leg_state_R1-2*(1-leg_state_R1);
        foot_gait_R2 = onlyPos(getSign(stance_R2-swing_R2))*5*leg_state_R2-3*(1-leg_state_R2);
        foot_gait_L0 = onlyPos(getSign(stance_L0-swing_L0))*4*leg_state_L0-4*(1-leg_state_L0);
        foot_gait_L1 = onlyPos(getSign(stance_L1-swing_L1))*3*leg_state_L1-5*(1-leg_state_L1);
        foot_gait_L2 = onlyPos(getSign(stance_L2-swing_L2))*2*leg_state_L2-6*(1-leg_state_L2);

        //Saves the gait if PLOT_GAIT is set.
        if (PLOT_GAIT == 1)
        {
            gaitOFile.open("gait.txt");
            gaitOFile << foot_gait_R0 << "\t" << foot_gait_R1 << "\t" << foot_gait_R2 << "\t"
                    << foot_gait_L0 << "\t" << foot_gait_L1 << "\t" << foot_gait_L2 << "\n";
            gaitOFile.close();
        }


//Backjoint

        //Get thresholded binary output of US signal

        double US_out = onlyPos(getSign(outputUS2 - JOINT_US_DIST_THRES));


        //reccurent value for foot sensors
        reccurent_FF = reccurent_FF * JOINT_FF_REC_W + (x_[L0_fs] + x_[R0_fs]) * JOINT_FF_INP_W;
        reccurent_FM = reccurent_FM * JOINT_FM_REC_W + (x_[L1_fs] + x_[R1_fs]) * JOINT_FM_INP_W;
        reccurent_FR = reccurent_FR * JOINT_FR_REC_W + (x_[L2_fs] + x_[R2_fs]) * JOINT_FR_INP_W;

        //outputs from foot parts of backbone joint NN
        output_FR = onlyPos(getSign(JOINT_FR_REF - reccurent_FR));
        output_FF = sigmoid((JOINT_FF_REF - reccurent_FF - 10 * output_FR) * JOINT_FF_SIG_SCALE, JOINT_SIG_GAIN);
        output_FM = sigmoid((JOINT_FM_REF - reccurent_FM - 10 * output_FR) * JOINT_FF_SIG_SCALE, JOINT_SIG_GAIN);

        //Activation of BJ reccurent neuron
        recurrent_Joint = recurrent_Joint * JOINT_JOINT_REC_W + (JOINT_FF_OUT_W * output_FF
                + JOINT_FM_OUT_W * output_FM + JOINT_US_OUT_W * US_out) * JOINT_JOINT_INP_W;

        //Output to the BJ servo
        output_Joint = sigmoid(recurrent_Joint + JOINT_SIG_OFFSET, JOINT_SIG_GAIN)
                + sigmoid(recurrent_Joint - JOINT_SIG_OFFSET, JOINT_SIG_GAIN) - 1;


        y_[BJ_m] = output_Joint;


//---------------------------WALKNET--------------------------------------------------
// update stance filters

        double stance;
        if (x_[R0_fs] > STANCE_FORCE_THRESHOLD)
            stance = 1;
        else
            stance = 0;
        stance_last_filter_R0 = stance_filter_R0;
        stance_filter_R0 = stance_filter_R0 * STANCE_FILTER_COEFF + (1 - STANCE_FILTER_COEFF) * stance;

        if (x_[R1_fs] > STANCE_FORCE_THRESHOLD)
            stance = 1;
        else
            stance = 0;
        stance_last_filter_R1 = stance_filter_R1;
        stance_filter_R1 = stance_filter_R1 * STANCE_FILTER_COEFF + (1 - STANCE_FILTER_COEFF) * stance;

        if (x_[R2_fs] > STANCE_FORCE_THRESHOLD)
            stance = 1;
        else
            stance = 0;
        stance_last_filter_R2 = stance_filter_R2;
        stance_filter_R2 = stance_filter_R2 * STANCE_FILTER_COEFF + (1 - STANCE_FILTER_COEFF) * stance;

        if (x_[L0_fs] > STANCE_FORCE_THRESHOLD)
            stance = 1;
        else
            stance = 0;
        stance_last_filter_L0 = stance_filter_L0;
        stance_filter_L0 = stance_filter_L0 * STANCE_FILTER_COEFF + (1 - STANCE_FILTER_COEFF) * stance;

        if (x_[L1_fs] > STANCE_FORCE_THRESHOLD)
            stance = 1;
        else
            stance = 0;
        stance_last_filter_L1 = stance_filter_L1;
        stance_filter_L1 = stance_filter_L1 * STANCE_FILTER_COEFF + (1 - STANCE_FILTER_COEFF) * stance;

        if (x_[L2_fs] > STANCE_FORCE_THRESHOLD)
            stance = 1;
        else
            stance = 0;
        stance_last_filter_L2 = stance_filter_L2;
        stance_filter_L2 = stance_filter_L2 * STANCE_FILTER_COEFF + (1 - STANCE_FILTER_COEFF) * stance;


//stance rise
        stance_rise_R0 = getStanceRise(stance_last_filter_R0, stance_filter_R0);
        stance_rise_R1 = getStanceRise(stance_last_filter_R1, stance_filter_R1);
        stance_rise_R2 = getStanceRise(stance_last_filter_R2, stance_filter_R2);
        stance_rise_L0 = getStanceRise(stance_last_filter_L0, stance_filter_R0);
        stance_rise_L1 = getStanceRise(stance_last_filter_L1, stance_filter_L1);
        stance_rise_L2 = getStanceRise(stance_last_filter_L2, stance_filter_L2);

//------------ PEP-Threshold------------------------------------------------------

        pep_thres_R0 = thresholdFunc(x_[TR0_as]);
        pep_thres_R1 = thresholdFunc(x_[TR1_as]);
        pep_thres_R2 = thresholdFunc(x_[TR2_as]);
        pep_thres_L0 = thresholdFunc(x_[TL0_as]);
        pep_thres_L1 = thresholdFunc(x_[TL1_as]);
        pep_thres_L2 = thresholdFunc(x_[TL2_as]);

//Calculate leg states
        reccurent_FC_L0 = reccurent_FC_L0 * LEG_FORCE_REC_W + x_[L0_fs] * LEG_FORCE_INP_W;
        reccurent_FC_R0 = reccurent_FC_R0 * LEG_FORCE_REC_W + x_[R0_fs] * LEG_FORCE_INP_W;
        reccurent_FC_L1 = reccurent_FC_L1 * LEG_FORCE_REC_W + x_[L1_fs] * LEG_FORCE_INP_W;
        reccurent_FC_R1 = reccurent_FC_R1 * LEG_FORCE_REC_W + x_[R1_fs] * LEG_FORCE_INP_W;
        reccurent_FC_L2 = reccurent_FC_L2 * LEG_FORCE_REC_W + x_[L2_fs] * LEG_FORCE_INP_W;
        reccurent_FC_R2 = reccurent_FC_R2 * LEG_FORCE_REC_W + x_[R2_fs] * LEG_FORCE_INP_W;

        leg_state_R0 = onlyPos(getSign(reccurent_FC_R0 - LEG_ACTIVE_FORCE_THRES));
        leg_state_R1 = onlyPos(getSign(reccurent_FC_R1 - LEG_ACTIVE_FORCE_THRES));
        leg_state_R2 = onlyPos(getSign(reccurent_FC_R2 - LEG_ACTIVE_FORCE_THRES));
        leg_state_L0 = onlyPos(getSign(reccurent_FC_L0 - LEG_ACTIVE_FORCE_THRES));
        leg_state_L1 = onlyPos(getSign(reccurent_FC_L1 - LEG_ACTIVE_FORCE_THRES));
        leg_state_L2 = onlyPos(getSign(reccurent_FC_L2 - LEG_ACTIVE_FORCE_THRES));

//Calculate PEP -------------------------------------------------------------------

        //Front legs
        curr_pep_L0 = PEP_EXTREME
                + W1 * swing_L1 * leg_state_L1
                + W1 * swing_L2 * (1 - leg_state_L1) * leg_state_L2
                - W2_FRONT * stance_rise_L1 * leg_state_L1
                - W2_FRONT * stance_rise_L2 * (1 - leg_state_L1) * leg_state_L2
                - W2_CONTRA * stance_rise_R0 * leg_state_R0
                - W3_CONTRA * pep_thres_R0 * leg_state_R0
                + W6_F * output_Joint*getSign(output_Joint);

        curr_pep_R0 = PEP_EXTREME
                + W1 * swing_R1 * leg_state_R1
                + W1 * swing_R2 * (1 - leg_state_L1) * leg_state_R2
                - W2_FRONT * stance_rise_R1 * leg_state_L1
                - W2_FRONT * stance_rise_R2 * (1 - leg_state_L1) * leg_state_R2
                - W2_CONTRA * stance_rise_L0 * leg_state_L0
                - W3_CONTRA * pep_thres_L0 * leg_state_L0
                + W6_F * output_Joint*getSign(output_Joint);

        debug_swing_W1_R1 = W1 * swing_R1;
        debug_swing_W1_R2_R1_LOAD = W1 * swing_R2 * x_[R1_fs];
        debug_W2_FRONT_stance_rise_R1 = -W2_FRONT * stance_rise_R1;
        debug_W2_CONTRA_stance_rise_L0 = -W2_CONTRA * stance_rise_L0;
        debug_W3_CONTRA_pep_thres_L0 = -W3_CONTRA * pep_thres_L0;


        //Middle legs
        curr_pep_L1 = PEP_EXTREME
                + W1 * swing_L2 * leg_state_L2
                - W2_FRONT * stance_rise_L2 * leg_state_L2
                - W2_CONTRA * stance_rise_R1 * leg_state_R1
                - W3_BACK * pep_thres_L0 * leg_state_L0
                - W3_CONTRA * pep_thres_R1 * leg_state_R1
                + W6_M * output_Joint*getSign(output_Joint);

        curr_pep_R1 = PEP_EXTREME
                + W1 * swing_R2 * leg_state_R2
                - W2_FRONT * stance_rise_R2 * leg_state_R2
                - W2_CONTRA * stance_rise_L1 * leg_state_L1
                - W3_BACK * pep_thres_R0 * leg_state_R0
                - W3_CONTRA * pep_thres_L1 * leg_state_L1
                + W6_M * output_Joint*getSign(output_Joint);

        //Rear legs
        curr_pep_L2 = PEP_EXTREME_REAR
                - W2_CONTRA * stance_rise_R2 * leg_state_R2
                - W3_BACK * pep_thres_L1 * leg_state_L1
                - W3_BACK * pep_thres_L0 * (1 - leg_state_L1) * leg_state_L0
                - W3_CONTRA * pep_thres_R2 * leg_state_R2
                + W6_R * output_Joint* getSign(output_Joint);

        curr_pep_R2 = PEP_EXTREME_REAR
                - W2_CONTRA * stance_rise_L2 * leg_state_L2
                - W3_BACK * pep_thres_R1 * leg_state_R1
                - W3_BACK * pep_thres_R0 * (1 - leg_state_R1) * leg_state_R0
                - W3_CONTRA * pep_thres_L2 * leg_state_L2
                + W6_R * output_Joint* getSign(output_Joint);

//----------------------Calculate activations--------------------

        //Front legs
        double tempStance = stance_R0;

        stance_R0 = x_[R0_fs]
                + W5 * getCoactivation(x_[L0_fs], CO_LOAD_THRES_FRONT)
                + W5 * getCoactivation(x_[R1_fs], CO_LOAD_THRES_MIDDLE)
                + heaviside(stance_R0) / 8
                - getPepDist(curr_pep_R0, x_[TR0_as])
                - swing_R0 / 15;
        swing_R0 = -x_[R0_fs]
                + heaviside(swing_R0)
                + getPepDist(curr_pep_R0, x_[TR0_as])
                - tempStance / 3;

        tempStance = stance_L0;

        stance_L0 = x_[L0_fs]
                + W5 * getCoactivation(x_[R0_fs], CO_LOAD_THRES_FRONT)
                + W5 * getCoactivation(x_[L1_fs], CO_LOAD_THRES_MIDDLE)
                + heaviside(stance_L0) / 8
                - getPepDist(curr_pep_L0, x_[TL0_as])
                - swing_L0 / 15;
        swing_L0 = -x_[L0_fs]
                + heaviside(swing_L0)
                + getPepDist(curr_pep_L0, x_[TL0_as])
                - tempStance / 3;

        //Middle legs
        tempStance = stance_R1;

        stance_R1 = x_[R1_fs]
                + W5 * getCoactivation(x_[L1_fs], CO_LOAD_THRES_MIDDLE)
                + W5 * getCoactivation(x_[R0_fs], CO_LOAD_THRES_FRONT)
                + W5 * getCoactivation(x_[R2_fs], CO_LOAD_THRES_REAR)
                + heaviside(stance_R1) / 8
                - getPepDist(curr_pep_R1, x_[TR1_as])
                - swing_R1 / 15;
        swing_R1 = -x_[R1_fs]
                + heaviside(swing_R1)
                + getPepDist(curr_pep_R1, x_[TR1_as])
                - tempStance / 3;

        tempStance = stance_L1;

        stance_L1 = x_[L1_fs]
                + W5 * getCoactivation(x_[R1_fs], CO_LOAD_THRES_MIDDLE)
                + W5 * getCoactivation(x_[L0_fs], CO_LOAD_THRES_FRONT)
                + W5 * getCoactivation(x_[L2_fs], CO_LOAD_THRES_REAR)
                + heaviside(stance_L1) / 8
                - getPepDist(curr_pep_L1, x_[TL1_as])
                - swing_L1 / 15;
        swing_L1 = -x_[L1_fs]
                + heaviside(swing_L1)
                + getPepDist(curr_pep_L1, x_[TL1_as])
                - tempStance / 3;

        //Rear legs
        tempStance = stance_R2;

        stance_R2 = x_[R2_fs]
                + W5 * getCoactivation(x_[L2_fs], CO_LOAD_THRES_REAR)
                + W5 * getCoactivation(x_[R1_fs], CO_LOAD_THRES_MIDDLE)
                + heaviside(stance_R2) / 8
                - getPepDist(curr_pep_R2, x_[TR2_as])
                - swing_R2 / 15;
        swing_R2 = -x_[R2_fs]
                + heaviside(swing_R2)
                + getPepDist(curr_pep_R2, x_[TR2_as])
                - tempStance / 3;

        tempStance = stance_L2;

        stance_L2 = x_[L2_fs]
                + W5 * getCoactivation(x_[R2_fs], CO_LOAD_THRES_REAR)
                + W5 * getCoactivation(x_[L1_fs], CO_LOAD_THRES_MIDDLE)
                + heaviside(stance_L2) / 8
                - getPepDist(curr_pep_L2, x_[TL2_as])
                - swing_L2 / 15;
        swing_L2 = -x_[L2_fs]
                + heaviside(swing_L2)
                + getPepDist(curr_pep_L2, x_[TL2_as])
                - tempStance / 3;

        //Only use positive values
        stance_R0 = onlyPos(stance_R0);
        stance_R1 = onlyPos(stance_R1);
        stance_R2 = onlyPos(stance_R2);
        stance_L0 = onlyPos(stance_L0);
        stance_L1 = onlyPos(stance_L1);
        stance_L2 = onlyPos(stance_L2);

        swing_R0 = onlyPos(swing_R0);
        swing_R1 = onlyPos(swing_R1);
        swing_R2 = onlyPos(swing_R2);
        swing_L0 = onlyPos(swing_L0);
        swing_L1 = onlyPos(swing_L1);
        swing_L2 = onlyPos(swing_L2);

//-----------------------------------------------------------------------------------
        foot_dep_R0 = getLegDepress(x_[CR0_as], x_[FR0_as]);
        foot_dep_R1 = getLegDepress(x_[CR1_as], x_[FR1_as]);
        foot_dep_R2 = getLegDepress(x_[CR2_as], x_[FR2_as]);
        foot_dep_L0 = getLegDepress(x_[CL0_as], x_[FL0_as]);
        foot_dep_L1 = getLegDepress(x_[CL1_as], x_[FL1_as]);
        foot_dep_L2 = getLegDepress(x_[CL2_as], x_[FL2_as]);

        double temp_C_speed;
        double tempOut = 0;
        // generate motor commands

//Correct walking to goal

        goal_error = (x_[G0angleyaw_s]/180)*TURNING_SPEED_GAIN;

        if (goal_error > 0)
        {
            turn_scale_right = 1;
            turn_scale_left = 1-goal_error;
        }
        else
        {
            turn_scale_right = 1+goal_error;
            turn_scale_left = 1;
        }

        //right front coxa (knee) forward-backward joint
        if (leg_state_R0 == 1) { //Leg is active
            y_[TR0_m] = y_[TR0_m] + ((stance_R0 * (-0.12) + swing_R0 * swing_net_T(x_[TR0_as])) * SCALE_LEG_MOVE)*turn_scale_right;

            temp_C_speed = (stance_R0 * stance_net_C(y_[CR0_m], y_[FR0_m]) +
                    swing_R0 * swing_net_C(y_[TR0_m], y_[CR0_m])) * SCALE_LEG_MOVE * SCALE_LEG_MOVE_C_FRONT;

            y_[CR0_m] = y_[CR0_m] + temp_C_speed;
        }
        else { //Leg is inactive

            activityH1 = CP_WEIGHT_F_1_1 * output_CP_R0_1 + CP_WEIGHT_F_1_2 * output_CP_R0_2  + 0.01;
            activityH2 = CP_WEIGHT_F_2_2 * output_CP_R0_2 + CP_WEIGHT_F_2_1 * output_CP_R0_1  + 0.01;
            output_CP_R0_1 = tanh(activityH1);
            output_CP_R0_2 = tanh(activityH2);

            y_[TR0_m] = output_CP_R0_2 * LEG_GAIN_F + LEG_BIAS_F;
            y_[CR0_m] = -output_CP_R0_1 * LEG_GAIN_F;
        }
        //left front coxa (knee) forward-backward joint
        if (leg_state_L0 == 1) {
            y_[TL0_m] = y_[TL0_m] + ((stance_L0 * (-0.12) + swing_L0 * swing_net_T(x_[TL0_as])) * SCALE_LEG_MOVE)*turn_scale_left;

            temp_C_speed = (stance_L0 * stance_net_C(y_[CL0_m], y_[FL0_m]) +
                    swing_L0 * swing_net_C(y_[TL0_m], y_[CL0_m])) * SCALE_LEG_MOVE * SCALE_LEG_MOVE_C_FRONT;

            y_[CL0_m] = y_[CL0_m] + temp_C_speed;
        }
        else
        {
            activityH1 = CP_WEIGHT_F_1_1 * output_CP_L0_1 + CP_WEIGHT_F_1_2 * output_CP_L0_2  + 0.01;
            activityH2 = CP_WEIGHT_F_2_2 * output_CP_L0_2 + CP_WEIGHT_F_2_1 * output_CP_L0_1  + 0.01;
            output_CP_L0_1 = tanh(activityH1);
            output_CP_L0_2 = tanh(activityH2);

            y_[TL0_m] = output_CP_L0_2 * LEG_GAIN_F + LEG_BIAS_F;
            y_[CL0_m] = -output_CP_L0_1 * LEG_GAIN_F;
        }




        //right middle coxa (knee) forward-backward joint
        if (leg_state_R1 == 1) {
        y_[TR1_m] = y_[TR1_m] + ((stance_R1 * (-0.12) + swing_R1 * swing_net_T(x_[TR1_as])) * SCALE_LEG_MOVE)*turn_scale_right;

        temp_C_speed = (stance_R1 * stance_net_C(y_[CR1_m], y_[FR1_m]) +
                swing_R1 * swing_net_C(y_[TR1_m], y_[CR1_m])) * SCALE_LEG_MOVE * SCALE_LEG_MOVE_C_MIDDLE;

        y_[CR1_m] = y_[CR1_m] + temp_C_speed;
        }
        else
        {
            activityH1 = CP_WEIGHT_F_1_1 * output_CP_R1_1 + CP_WEIGHT_F_1_2 * output_CP_R1_2  + 0.01;
            activityH2 = CP_WEIGHT_F_2_2 * output_CP_R1_2 + CP_WEIGHT_F_2_1 * output_CP_R1_1  + 0.01;
            output_CP_R1_1 = tanh(activityH1);
            output_CP_R1_2 = tanh(activityH2);

            y_[TR1_m] = output_CP_R1_2 * LEG_GAIN_M + LEG_BIAS_M;
            y_[CR1_m] = -output_CP_R1_1 * LEG_GAIN_M;
         }
        //left middle coxa (knee) forward-backward joint
        if (leg_state_L1 == 1) {
        y_[TL1_m] = y_[TL1_m] + ((stance_L1 * (-0.12) + swing_L1 * swing_net_T(x_[TL1_as])) * SCALE_LEG_MOVE)*turn_scale_left;

        temp_C_speed = (stance_L1 * stance_net_C(y_[CL1_m], y_[FL1_m]) +
                swing_L1 * swing_net_C(y_[TL1_m], y_[CL1_m])) * SCALE_LEG_MOVE * SCALE_LEG_MOVE_C_MIDDLE;

        y_[CL1_m] = y_[CL1_m] + temp_C_speed;
        }
        else
        {
            activityH1 = CP_WEIGHT_F_1_1 * output_CP_L1_1 + CP_WEIGHT_F_1_2 * output_CP_L1_2  + 0.01;
            activityH2 = CP_WEIGHT_F_2_2 * output_CP_L1_2 + CP_WEIGHT_F_2_1 * output_CP_L1_1  + 0.01;
            output_CP_L1_1 = tanh(activityH1);
            output_CP_L1_2 = tanh(activityH2);

            y_[TL1_m] = output_CP_L1_2 * LEG_GAIN_M + LEG_BIAS_M;
            y_[CL1_m] = -output_CP_L1_1 * LEG_GAIN_M;
        }

        // right rear coxa (knee) forward-backward joint (back is positive)
        if (leg_state_R2 == 1) {
            y_[TR2_m] = y_[TR2_m] + ((stance_R2 * (-0.13) + swing_R2 * swing_net_T(x_[TR2_as])) * SCALE_LEG_MOVE)*turn_scale_right;

            temp_C_speed = (stance_R2 * stance_net_C(y_[CR2_m], y_[FR2_m])*2 +
                    swing_R2 * swing_net_C(y_[TR2_m], y_[CR2_m])) * SCALE_LEG_MOVE * SCALE_LEG_MOVE_C_REAR;
            y_[CR2_m] = LEG_OFFSET_REAR_C + y_[CR2_m] + temp_C_speed*SCALE_LEG_REAR_C;
        }
        else
        {
            activityH1 = CP_WEIGHT_F_1_1 * output_CP_R2_1 + CP_WEIGHT_F_1_2 * output_CP_R2_2  + 0.01;
            activityH2 = CP_WEIGHT_F_2_2 * output_CP_R2_2 + CP_WEIGHT_F_2_1 * output_CP_R2_1  + 0.01;
            output_CP_R2_1 = tanh(activityH1);
            output_CP_R2_2 = tanh(activityH2);

            y_[TR2_m] = output_CP_R2_2 * LEG_GAIN_R + LEG_BIAS_R;
            y_[CR2_m] = -output_CP_R2_1 * LEG_GAIN_R;
        }
      //left rear coxa (knee) forward-backward joint
        if (leg_state_L2 == 1) {
            y_[TL2_m] = y_[TL2_m] + ((stance_L2 * (-0.13) + swing_L2 * swing_net_T(x_[TL2_as])) * SCALE_LEG_MOVE)*turn_scale_left;

            temp_C_speed = (stance_L2 * stance_net_C(y_[CL2_m], y_[FL2_m])*2 +
                    swing_L2 * swing_net_C(y_[TL2_m], y_[CL2_m])) * SCALE_LEG_MOVE * SCALE_LEG_MOVE_C_REAR;

            y_[CL2_m] = LEG_OFFSET_REAR_C + y_[CL2_m] + temp_C_speed*SCALE_LEG_REAR_C;
        }
        else
        {
            activityH1 = CP_WEIGHT_F_1_1 * output_CP_L2_1 + CP_WEIGHT_F_1_2 * output_CP_L2_2  + 0.01;
            activityH2 = CP_WEIGHT_F_2_2 * output_CP_L2_2 + CP_WEIGHT_F_2_1 * output_CP_L2_1  + 0.01;
            output_CP_L2_1 = tanh(activityH1);
            output_CP_L2_2 = tanh(activityH2);

            y_[TL2_m] = output_CP_L2_2 * LEG_GAIN_R + LEG_BIAS_R;
            y_[CL2_m] = -output_CP_L2_1 * LEG_GAIN_R;
        }




        FootContactSum = x_[L0_fs] + x_[L1_fs]+ x_[L2_fs] + x_[R0_fs] + x_[R1_fs]+ x_[R2_fs];

      // backbone joint
        /*
        y_[FR2_m] = -y_[1];
        y_[FL2_m] = -y_[4];
        y_[FR1_m] = -y_[7];
        y_[FL1_m] = -y_[10];
        y_[FR0_m] = -y_[13];
        */

        y_[FL0_m] = -y_[16];

        double f_joint = 0;
        y_[FR0_m] = f_joint;
        y_[FR1_m] = f_joint;
        y_[FR2_m] = f_joint;

        y_[FL0_m] = f_joint;
        y_[FL1_m] = f_joint;
        y_[FL2_m] = f_joint;



      // update step counter
      t++;
    };

    /***** STOREABLE ****/
    /** stores the controller values to a given file. */
    virtual bool store(FILE* f) const {
      return true;
    };
    /** loads the controller values from a given file. */
    virtual bool restore(FILE* f){
      return true;
    };


  protected:
    unsigned short number_channels;

    int t;
    paramkey name;
    double outputH1;
    double outputH2;

    double output_CP_R0_1;
    double output_CP_R1_1;
    double output_CP_R2_1;
    double output_CP_L0_1;
    double output_CP_L1_1;
    double output_CP_L2_1;

    double output_CP_R0_2;
    double output_CP_R1_2;
    double output_CP_R2_2;
    double output_CP_L0_2;
    double output_CP_L1_2;
    double output_CP_L2_2;



    std::ofstream gaitOFile;

    double outputUS1;
    double outputUS2;

    double reccurent_FF;
    double reccurent_FM;
    double reccurent_FR;
    double recurrent_Joint;

    double output_FF;
    double output_FM;
    double output_FR;
    double output_Joint;


//Goal orientation

    double turn_scale_right;
    double turn_scale_left;
    double goal_error;

//Bandpass filters for stance
    double stance_filter_R0;
    double stance_filter_R1;
    double stance_filter_R2;
    double stance_filter_L0;
    double stance_filter_L1;
    double stance_filter_L2;

//Bandpass last
    double stance_last_filter_R0;
    double stance_last_filter_R1;
    double stance_last_filter_R2;
    double stance_last_filter_L0;
    double stance_last_filter_L1;
    double stance_last_filter_L2;

//stance rise
    double stance_rise_R0;
    double stance_rise_R1;
    double stance_rise_R2;
    double stance_rise_L0;
    double stance_rise_L1;
    double stance_rise_L2;

//Swing values for legs
    double swing_R0;
    double swing_R1;
    double swing_R2;
    double swing_L0;
    double swing_L1;
    double swing_L2;

//Stance values for legs
    double stance_R0;
    double stance_R1;
    double stance_R2;
    double stance_L0;
    double stance_L1;
    double stance_L2;

    double lastR0;
    double lastR1;
    double lastR2;
    double lastL0;
    double lastL1;
    double lastL2;

    double foot_dep_R0;
    double foot_dep_R1;
    double foot_dep_R2;
    double foot_dep_L0;
    double foot_dep_L1;
    double foot_dep_L2;

//Foot contact for gait inspection
    double foot_gait_R0;
    double foot_gait_R1;
    double foot_gait_R2;
    double foot_gait_L0;
    double foot_gait_L1;
    double foot_gait_L2;


//PEP-threshold value
    double pep_thres_R0;
    double pep_thres_R1;
    double pep_thres_R2;
    double pep_thres_L0;
    double pep_thres_L1;
    double pep_thres_L2;

//PEP-values
    double curr_pep_R0;
    double curr_pep_R1;
    double curr_pep_R2;
    double curr_pep_L0;
    double curr_pep_L1;
    double curr_pep_L2;

    double debug_swing_W1_R1;
    double debug_swing_W1_R2_R1_LOAD;
    double debug_W2_FRONT_stance_rise_R1;
    double debug_W2_CONTRA_stance_rise_L0;
    double debug_W3_CONTRA_pep_thres_L0;

//Stufff for seeing if leg is active

    double reccurent_FC_L0;
    double reccurent_FC_R0;
    double reccurent_FC_R1;
    double reccurent_FC_L1;
    double reccurent_FC_R2;
    double reccurent_FC_L2;

    double leg_state_L0;
    double leg_state_R0;
    double leg_state_R1;
    double leg_state_L1;
    double leg_state_R2;
    double leg_state_L2;




//PEP-thres

    double FootContactSum;

    EmptyControllerConf conf;

  public:
    virtual paramval getParam(const paramkey& key) const {
      if (key == "WeightH1_H1")
        return conf.WeightH1_H1;
      else if (key == "WeightH2_H2")
        return conf.WeightH2_H2;
      else if (key == "WeightH1_H2")
        return conf.WeightH1_H2;
      else if (key == "WeightH2_H1")
        return conf.WeightH2_H1;
      else if (key == "fact")
        return conf.fact;
      else if (key == "direction")
        return conf.direction;
      else if (key == "bias")
        return conf.bias;
      else
        return AbstractController::getParam(key);
    }

    virtual bool setParam(const paramkey& key, paramval val) {
      if (key == "WeightH1_H1")
        conf.WeightH1_H1 = val;
      else if (key == "WeightH2_H2")
        conf.WeightH2_H2 = val;
      else if (key == "WeightH1_H2")
        conf.WeightH1_H2 = val;
      else if (key == "WeightH2_H1")
        conf.WeightH2_H1 = val;
      else if (key == "fact")
        conf.fact = val;
      else if (key == "direction")
        conf.direction = val;
      else if (key == "bias")
        conf.bias = val;
      else
        return false;
      return true;
    }

    virtual paramlist getParamList() const {
      paramlist list;
      list.push_back(
          std::pair<paramkey, paramval>("WeightH1_H1", conf.WeightH1_H1));
      list.push_back(
          std::pair<paramkey, paramval>("WeightH2_H2", conf.WeightH2_H2));
      list.push_back(
          std::pair<paramkey, paramval>("WeightH1_H2", conf.WeightH1_H2));
      list.push_back(
          std::pair<paramkey, paramval>("WeightH2_H1", conf.WeightH2_H1));
      list.push_back(
          std::pair<paramkey, paramval>("fact", conf.fact));
      list.push_back(
          std::pair<paramkey, paramval>("direction", conf.direction));
      list.push_back(
          std::pair<paramkey, paramval>("bias", conf.bias));
      return list;
    }
};

#endif


/*
 * List of available Sensors and their numbers/enum's to be used for example as x_[TR0_as] in the step function
 *
// Angle sensors (for actoric-sensor board (new board))
       TR0_as=0, //Thoracic joint of right front leg
       TR1_as=1, //Thoracic joint of right middle leg
       TR2_as=2, //Thoracic joint of right hind leg

       TL0_as=3, //Thoracic joint of left front leg
       TL1_as=4, //Thoracic joint of left middle leg
       TL2_as=5, //Thoracic joint of left hind leg

       CR0_as=6, //Coxa joint of right front leg
       CR1_as=7, //Coxa joint of right middle leg
       CR2_as=8, //Coxa joint of right hind leg

       CL0_as=9,  //Coxa joint of left hind leg
       CL1_as=10, //Coxa joint of left hind leg
       CL2_as=11, //Coxa joint of left hind leg

       FR0_as=12, //Fibula joint of right front leg
       FR1_as=13, //Fibula joint of right middle leg
       FR2_as=14, //Fibula joint of right hind leg

       FL0_as=15, //Fibula joint of left front leg
       FL1_as=16, //Fibula joint of left middle leg
       FL2_as=17, //Fibula joint of left hind leg

       BJ_as= 18, //Backbone joint angle

       //Foot contact sensors (AMOSII v1 and v2)
       R0_fs= 19, //Right front foot
       R1_fs= 20, //Right middle foot
       R2_fs= 21, //Right hind foot
       L0_fs= 22, //Left front foot
       L1_fs= 23, //Left middle foot
       L2_fs= 24, //Left hind foot

       // US sensors (AMOSII v1 and v2)
       FR_us=25, //Front Ultrasonic sensor (right)
       FL_us=26, //Front Ultrasonic sensor (left)

       // IR reflex sensors at legs (AMOSIIv2)
       R0_irs=31,
       R1_irs=29,
       R2_irs=27,
       L0_irs=32,
       L1_irs=30,
       L2_irs=28,

       //Body speed sensors (only simulation)
        BX_spd= 66,
        BY_spd= 67,
        BZ_spd= 68,

        // goal orientation sensors (relative angle to reference object 1, e.g. camera)
        G0angleroll_s=69,
        G0anglepitch_s=70,
        G0angleyaw_s=71,

        // goal orientation sensors (relative position to reference object 2, e.g. camera)
        G1x_s=72,
        G1y_s=73,
        G1z_s=74,

        // goal orientation sensors (relative angle to reference object 2, e.g. camera)
        G1angleroll_s=75,
        G1anglepitch_s=76,
        G1angleyaw_s=77,

        // goal orientation sensors (relative position to reference object 3, e.g. camera)
        G2x_s=78,
        G2y_s=79,
        G2z_s=80,

        // goal orientation sensors (relative angle to reference object 3, e.g. camera)
        G2angleroll_s=81,
        G2anglepitch_s=82,
        G2angleyaw_s=83,


*/
