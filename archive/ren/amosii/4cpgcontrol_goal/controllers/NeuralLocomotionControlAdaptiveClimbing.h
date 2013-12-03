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
//#include "sensor_motor_definition.h"
#include <ode_robots/amosiisensormotordefinition.h>

//Save files /read file
#include <iostream>
#include <fstream>
#include <string.h>

//atof function
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

using namespace std;
//Save files

#define LEG_RF 0
#define LEG_RM 1
#define LEG_RH 2
#define LEG_LF 3
#define LEG_LM 4
#define LEG_LH 5

#define P4 0
#define P5 1
#define P6 2
#define P8 3
#define P9 4

//define how many steps the robot should wait for stabling
#define WAITSTEP 600

//define how many steps for one trial
#define TRIALSTEP 1000

//define the deviation threshold (degree), if the deviation angle is less than this value, stop learning
#define DEVI_THRESH 8

//define a factor in Mento-Carlo random value
#define BETA 0.3

#define ABS(x) (x)<0?(-(x)):(x)

/*double sign(double v)
 {
 if(v > 0.0)
 return 1.0;
 else if(v < 0.0)
 return -1.0;
 else
 return 0.0;
 }*/

//3) Class for Neural locomotion control------
class NeuralLocomotionControlAdaptiveClimbing {

  public:

    //---Start Define functions---//
    NeuralLocomotionControlAdaptiveClimbing();
    ~NeuralLocomotionControlAdaptiveClimbing();

    double sigmoid(double num) {
      return 1. / (1. + exp(-num));
    }
    double sign(double v) {
      if (v > 0.0)
        return 1.0;
      else if (v < 0.0)
        return -1.0;
      else
        return 0.0;
    }

//	double get_deviation_angle()
//	{
//		return alpha;
//	}

    bool getlearning() {
      return learning;
    }

    std::vector<double> step_nlc(const std::vector<double> in0 /*from neural preprocessing*/,
        const std::vector<double> in1 /*from neural learning*/); //, bool Footinhibition=false);
    // if several sensor values (like preprocessed or output of memory and learning are used, extend parameters)
    // std::vector<double> step_nlc(const std::vector<double> in0, const std::vector<double> in1, const std::vector<double> in2);

    //---End  Define functions---//

    //---Start Save files---//
    //ofstream outFilenlc1;
    //ofstream outFilenlcr;
    //ofstream outFilenlft;
    ofstream recordperiod;
    //---End Save files---//

    //---Start Define vector----//

    //---Input neurons
    std::vector<double> input;

    //---CPG
    std::vector<double> cpg_activity; //CPG neural activities
    std::vector<double> cpg_output; //CPG neural outputs
    std::vector<double> cpg_output1; //CPG neural outputs1 --guanjiao
    std::vector<double> cpg_output2; //CPG neural outputs2 --guanjiao
    std::vector<double> cpg_output3; //CPG neural outputs3 --guanjiao
    std::vector<double> cpg_output4; //CPG neural outputs4 --guanjiao
    std::vector<double> cpg_output5; //CPG neural outputs5 --guanjiao

    std::vector<std::vector<double> > cpg_w; //CPG neural weights
    double cpg_bias; //CPG bias
    double Control_input;

    //---pCPG
    /*std::vector<double> pcpg_output;					//pCPG neural outputs
     std::vector<double> pcpg_step;						//step function
     std::vector<double> set;			   		        //step function
     std::vector<double> setold;			   		    	//step function
     std::vector<double> diffset;			   		    //step function

     std::vector<double> countup; 						//counter
     std::vector<double> countupold;						//counter
     std::vector<double> countdown;						//counter
     std::vector<double> countdownold; 					//counter

     std::vector<double> deltaxup;						//delta
     std::vector<double> deltaxdown;						//delta

     std::vector<double> xup;							//delta
     std::vector<double> xdown;							//delta

     std::vector<double> yup;							//delta
     std::vector<double> ydown;							//delta

     //---pCPG1 --guanjiao
     std::vector<double> pcpg_output1;					//pCPG neural outputs
     std::vector<double> pcpg_step1;						//step function
     std::vector<double> set1;			   		        //step function
     std::vector<double> setold1;			   		    //step function
     std::vector<double> diffset1;			   		    //step function

     std::vector<double> countup1; 						//counter
     std::vector<double> countupold1;					//counter
     std::vector<double> countdown1;						//counter
     std::vector<double> countdownold1; 					//counter

     std::vector<double> deltaxup1;						//delta
     std::vector<double> deltaxdown1;					//delta

     std::vector<double> xup1;							//delta
     std::vector<double> xdown1;							//delta

     std::vector<double> yup1;							//delta
     std::vector<double> ydown1;							//delta

     //---pCPG2
     std::vector<double> pcpg_output2;					//pCPG neural outputs
     std::vector<double> pcpg_step2;						//step function
     std::vector<double> set2;			   		        //step function
     std::vector<double> setold2;			   		    //step function
     std::vector<double> diffset2;			   		    //step function

     std::vector<double> countup2; 						//counter
     std::vector<double> countupold2;					//counter
     std::vector<double> countdown2;						//counter
     std::vector<double> countdownold2; 					//counter

     std::vector<double> deltaxup2;						//delta
     std::vector<double> deltaxdown2;					//delta

     std::vector<double> xup2;							//delta
     std::vector<double> xdown2;							//delta

     std::vector<double> yup2;							//delta
     std::vector<double> ydown2;							//delta
     */

    //---PSN
    std::vector<double> psn_activity; //PSN neural activities
    std::vector<double> psn_output; //PSN neural outputs
    std::vector<std::vector<double> > psn_w; //PSN neural weights
    std::vector<double> psn_bias; //PSN bias

    //---PSN1 --guanjiao
    std::vector<double> psn_activity1; //PSN neural activities
    std::vector<double> psn_output1; //PSN neural outputs
    std::vector<std::vector<double> > psn_w1; //PSN neural weights
    std::vector<double> psn_bias1; //PSN bias

    //---PSN2 --guanjiao
    std::vector<double> psn_activity2; //PSN neural activities
    std::vector<double> psn_output2; //PSN neural outputs
    std::vector<std::vector<double> > psn_w2; //PSN neural weights
    std::vector<double> psn_bias2; //PSN bias

    //---PSN3 --guanjiao
    std::vector<double> psn_activity3; //PSN neural activities
    std::vector<double> psn_output3; //PSN neural outputs
    std::vector<std::vector<double> > psn_w3; //PSN neural weights
    std::vector<double> psn_bias3; //PSN bias

    //---PSN4 --guanjiao
    std::vector<double> psn_activity4; //PSN neural activities
    std::vector<double> psn_output4; //PSN neural outputs
    std::vector<std::vector<double> > psn_w4; //PSN neural weights
    std::vector<double> psn_bias4; //PSN bias

    //---PSN5 --guanjiao
    std::vector<double> psn_activity5; //PSN neural activities
    std::vector<double> psn_output5; //PSN neural outputs
    std::vector<std::vector<double> > psn_w5; //PSN neural weights
    std::vector<double> psn_bias5; //PSN bias

    //---VRN
    std::vector<double> vrn_activity; //VRN neural activities
    std::vector<double> vrn_output; //VRN neural outputs
    std::vector<std::vector<double> > vrn_w; //VRN neural weights
    double vrn_bias;

    //---VRN1
    std::vector<double> vrn_activity1; //VRN neural activities
    std::vector<double> vrn_output1; //VRN neural outputs
    std::vector<std::vector<double> > vrn_w1; //VRN neural weights
    double vrn_bias1;

    //---VRN2
    std::vector<double> vrn_activity2; //VRN neural activities
    std::vector<double> vrn_output2; //VRN neural outputs
    std::vector<std::vector<double> > vrn_w2; //VRN neural weights
    double vrn_bias2;

    //---Interconnections
    std::vector<std::vector<double> > psn_pcpg_w; //PSN neural weights
    std::vector<std::vector<double> > vrn_psn_w; //VRN neural weights

    //---Interconnections
    std::vector<std::vector<double> > psn_pcpg_w1; //PSN neural weights
    std::vector<std::vector<double> > vrn_psn_w1; //VRN neural weights

    //---Interconnections
    std::vector<std::vector<double> > psn_pcpg_w2; //PSN neural weights
    std::vector<std::vector<double> > vrn_psn_w2; //VRN neural weights

    //---Interconnections
    std::vector<std::vector<double> > psn_pcpg_w3; //PSN neural weights
    std::vector<std::vector<double> > psn_pcpg_w4; //PSN neural weights
    std::vector<std::vector<double> > psn_pcpg_w5; //PSN neural weights

    //---Interconnections--input2 to PSN
    std::vector<std::vector<double> > psn_input2_w; //PSN neural weights
    double vrn_input3_w;
    double vrn_input4_w;

    //---Interconnections--input2 to PSN  --guanjiao
    std::vector<std::vector<double> > psn_input2_w1; //PSN neural weights
    double vrn_input3_w1;
    double vrn_input4_w1;

    //---Interconnections--input2 to PSN  --guanjiao
    std::vector<std::vector<double> > psn_input2_w2; //PSN neural weights
    double vrn_input3_w2;
    double vrn_input4_w2;

    //---Interconnections--input2 to PSN  --guanjiao
    std::vector<std::vector<double> > psn_input2_w3; //PSN neural weights
    std::vector<std::vector<double> > psn_input2_w4; //PSN neural weights
    std::vector<std::vector<double> > psn_input2_w5; //PSN neural weights

    //---Motor neurons
    std::vector<double> tr_activity; //motor neural activities
    std::vector<double> tl_activity; //motor neural activities
    std::vector<double> cr_activity; //motor neural activities
    std::vector<double> cl_activity; //motor neural activities
    std::vector<double> fr_activity; //motor neural activities
    std::vector<double> fl_activity; //motor neural activities
    std::vector<double> bj_activity; //motor neural activities

    std::vector<double> tr_output; //motor neural outputs
    std::vector<double> tl_output; //motor neural outputs
    std::vector<double> cr_output; //motor neural outputs
    std::vector<double> cl_output; //motor neural outputs
    std::vector<double> fr_output; //motor neural outputs
    std::vector<double> fl_output; //motor neural outputs
    std::vector<double> bj_output; //motor neural outputs

    std::vector<double> cr_outputold; //motor neural outputs_old
    std::vector<double> cl_outputold; //motor neural outputs_old

    std::vector<double> diffcr_output; //diff motor neural outputs
    std::vector<double> diffcl_output; //diff motor neural outputs

    std::vector<double> postcr; //postCL motor neural outputs
    std::vector<double> postcl; //postCR motor neural outputs
    std::vector<double> postcrold; //postCL motor neural outputs
    std::vector<double> postclold; //postCR motor neural outputs
    double threshold_c;

    std::vector<double> buffer_t; //delay buffer vector thorical
    std::vector<double> buffer_c; //delay buffer vector coxa
    std::vector<double> buffer_f; //delay buffer vector fibia

    //added by guanjiao
    std::vector<double> buffer_t1;
    std::vector<double> buffer_c1;
    std::vector<double> buffer_f1;
    std::vector<double> buffer_t2;
    std::vector<double> buffer_c2;
    std::vector<double> buffer_f2;

    std::vector<double> buffer_tl;
    std::vector<double> buffer_cl;
    std::vector<double> buffer_fl;
    std::vector<double> buffer_tl1;
    std::vector<double> buffer_cl1;
    std::vector<double> buffer_fl1;
    std::vector<double> buffer_tl2;
    std::vector<double> buffer_cl2;
    std::vector<double> buffer_fl2;

    std::vector<double> m_pre; //pre motor outputs (19 motors)
    std::vector<double> m_reflex; //reflex motor outputs  (19 motors)
    std::vector<double> m; //motor outputs as neural activation (19 motors)
    std::vector<double> m_deg; //motor outputs in deg (19 motors)

    //---Reflex motor neurons
    std::vector<double> fmodel_cmr_activity; //coxa motor right neural activities
    std::vector<double> fmodel_cmr_output; //coxa motor right neural outputs
    std::vector<double> fmodel_cmr_error; //error coxa motor right and foot right
    std::vector<double> fmodel_cmr_outputfinal; //coxa motor right neural outputs

    std::vector<double> fmodel_cml_activity; //coxa motor left neural activities
    std::vector<double> fmodel_cml_output; //coxa motor left neural outputs
    std::vector<double> fmodel_cml_error; //error coxa motor left and foot left
    std::vector<double> fmodel_cml_outputfinal; //coxa motor right neural outputs

    //---Reflex foot sensors
    std::vector<double> reflex_R_fs;
    std::vector<double> reflex_L_fs;

    //Learning forward models to expected foot sensors
    std::vector<double> lr_fmodel_cr; //learning rate
    std::vector<double> fmodel_cmr_w; //forward model weights
    std::vector<double> fmodel_fmodel_cmr_w; //forward model recurrent weights
    std::vector<double> fmodel_post_cmr_w; //forward model postprocessing weights
    std::vector<double> fmodel_cmr_bias; //forward model biases
    std::vector<double> acc_cmr_error; //forward model biases
    std::vector<double> acc_cmr_error_old; //forward model biases
    std::vector<double> deri_acc_cmr_error; //forward model biases
    std::vector<double> acc_cmr_error_elev; //error for elevator reflex
    std::vector<double> error_cmr_elev; //error for elevator reflex

    std::vector<double> lr_fmodel_cl; //learning rate
    std::vector<double> fmodel_cml_w; //forward model weights
    std::vector<double> fmodel_fmodel_cml_w; //forward model recurrent weights
    std::vector<double> fmodel_post_cml_w; //forward model postprocessing weights
    std::vector<double> fmodel_cml_bias; //forward model biases
    std::vector<double> acc_cml_error; //forward model biases
    std::vector<double> acc_cml_error_old; //forward model biases
    std::vector<double> deri_acc_cml_error; //forward model biases
    std::vector<double> acc_cml_error_elev; //error for elevator reflex
    std::vector<double> error_cml_elev; //error for elevator reflex

    std::vector<double> lowpass_cmr_error_activity; //lowpass neuron activities
    std::vector<double> lowpass_cmr__error_output; //lowpass neuron outputs
    std::vector<double> lowpass_cmr_w; //lowpass weights
    std::vector<double> lowpass_lowpass_cmr_w; //lowpass recurrent weights
    std::vector<double> lowpass_cmr_bias; //lowpass biases

    std::vector<double> lowpass_cml_error_activity; //lowpass neuron activities
    std::vector<double> lowpass_cml__error_output; //lowpass neuron outputs
    std::vector<double> lowpass_cml_w; //lowpass weights
    std::vector<double> lowpass_lowpass_cml_w; //lowpass recurrent weights
    std::vector<double> lowpass_cml_bias; //lowpass biases

    std::vector<double> a1_r; //Emd_2D, learning parameters
    std::vector<double> a2_r; //Emd_2D, learning parameters
    std::vector<double> a3_r; //Emd_2D, learning parameters
    std::vector<double> fcn_r; //Emd_2D
    std::vector<double> fac_r; //Emd_2D
    std::vector<double> normxsq_r; //Emd_2D
    std::vector<double> pred_r;
    std::vector<double> a1_l; //Emd_2D, learning parameters
    std::vector<double> a2_l; //Emd_2D, learning parameters
    std::vector<double> a3_l; //Emd_2D, learning parameters
    std::vector<double> fcn_l; //Emd_2D
    std::vector<double> fac_l; //Emd_2D
    std::vector<double> normxsq_l; //Emd_2D
    std::vector<double> pred_l;
    std::vector<double> delay_CR0; //Delay signal
    std::vector<double> delay_CR1; //Delay signal
    std::vector<double> delay_CR2; //Delay signal
    std::vector<double> delay_CL0; //Delay signal
    std::vector<double> delay_CL1; //Delay signal
    std::vector<double> delay_CL2; //Delay signal
    std::vector<double> m_pre_delay;

    //Motor mapping

    double min_tc; // network output range
    double max_tc; // network output range
    //Adjust
    double min_tc_f_nwalking_deg; //deg **
    double max_tc_f_nwalking_deg; //deg **
    double min_tc_f_nwalking;
    double max_tc_f_nwalking;

    //TC_middle
    //Adjust
    double min_tc_m_nwalking_deg; //deg **
    double max_tc_m_nwalking_deg; //deg **
    double min_tc_m_nwalking;
    double max_tc_m_nwalking;

    //TC_rear
    //Adjust
    double min_tc_r_nwalking_deg; //deg **
    double max_tc_r_nwalking_deg; //deg **
    double min_tc_r_nwalking;
    double max_tc_r_nwalking;

    //CTR joints
    double min_ctr; // network output range
    double max_ctr; // network output range

    std::vector<double> min_ctr_nwalking_deg; //deg
    std::vector<double> max_ctr_nwalking_deg; //deg
    std::vector<double> min_ctr_nwalking;
    std::vector<double> max_ctr_nwalking;
    std::vector<double> offset_ctr;

    std::vector<double> min_ctl_nwalking_deg; //deg
    std::vector<double> max_ctl_nwalking_deg; //deg
    std::vector<double> min_ctl_nwalking;
    std::vector<double> max_ctl_nwalking;
    std::vector<double> offset_ctl;

    //FTI joints
    double min_fti; // network output range
    double max_fti; // network output range

    std::vector<double> min_ftir_nwalking_deg; //deg
    std::vector<double> max_ftir_nwalking_deg; //deg
    std::vector<double> min_ftir_nwalking;
    std::vector<double> max_ftir_nwalking;
    std::vector<double> offset_ftir;

    std::vector<double> min_ftil_nwalking_deg; //deg
    std::vector<double> max_ftil_nwalking_deg; //deg
    std::vector<double> min_ftil_nwalking;
    std::vector<double> max_ftil_nwalking;
    std::vector<double> offset_ftil;

    //BJ joint
    double min_bj; // network output range
    double max_bj; // network output range
    double min_bj_fwalking_deg; //deg
    double max_bj_fwalking_deg; //deg
    double min_bj_fwalking;
    double max_bj_fwalking;

    double max_c; // max range
    double max_f; // max range
    double max_c_offset; // Adjust from 45,...185=> 185 (cin= 0.02, wave), 45 (cin= 0.18, tripod)
    double max_f_offset; // Adjust from 45,...185=> 185 (cin= 0.02, wave), 45 (cin= 0.18, tripod)

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

    //---End Define vector----//

    //-------deviation parameters by Ren -----
    double deviation_angle;
    double ref_angle;
    double deviation_angle_used;

    //---define leg damage flag by Ren----
    bool LegDamage_flag[6];
    bool LegDamage_flag_tmp[6];

    //--------period of leg and relevant values ---------
    unsigned int legperiod[6];
    unsigned int legperiod_tmp[6];
    unsigned int period[5];
    unsigned int legnum_random;
    unsigned int periodnum_random;
    vector<unsigned int> period_combination;
    //unsigned int vectornum;
    bool ChooseAnotherConbination;

    // ----------for control the loop -----------------
    unsigned int step_tmp;
    bool new_loop_flag;
    unsigned int trials;

    //---------decide learning or not--------------------
    double Evaluation;
    bool learning;
    bool keepnewperiod;

  private:

    double h;
    double count1;
    double count2;

    int T1;
    int T2;
    int T1old;
    int T2old;
    int period1;
    int period2;
    double y1;
    double y2;
    std::vector<double> triH1; //triangle output vector
    std::vector<double> triH2;

    int tau;
    int tau_l;
    //int time;

    int option_wiring;
    std::vector<double> counter_cr;
    std::vector<double> counter_cl;

    int global_count;
    int allfoot_off_ground;

    int option_fmodel;
    bool switchon_ED;
    bool switchon_footinhibition;
    bool switchon_reflexes;
    bool switchon_purefootsignal;
    bool switchon_learnweights;
    bool softlanding;
    bool reading_text_testing;
};

#endif /* NEURALLOCOMOTIONCONTROLADAPTIVECLIMBING_H_ */
