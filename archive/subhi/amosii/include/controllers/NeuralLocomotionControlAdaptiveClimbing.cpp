/*
 * NeuralLocomotionControlAdaptiveClimbing.cpp
 *
 *  Created on: May 2, 2011
 *      Author: poramate
 *
 *      Edited by Dennis Goldschmidt
 *      Apr 27, 2012
 *
 *      Edited by Eduard Grinke
 *      May 10, 2012
 *
 *      Edited by Subhi Shaker Barikhan
 *      April 01, 2014
 */

#include "NeuralLocomotionControlAdaptiveClimbing.h"
#include <ode_robots/amosiisensormotordefinition.h>

//3) Step function of Neural locomotion control------

NeuralLocomotionControlAdaptiveClimbing::NeuralLocomotionControlAdaptiveClimbing(int aamosVersion) {

	//Save files
	outFilenlc1.open("Neurallocomotion.txt");
amosVersion=aamosVersion;
	//---Set vector size----//
	//Input vector size
	input.resize(5);
	isMuscle=false;


	//----------------muscle initializations---------------

	for (int i =0;i<6;i++)
	{
	K1[i] = 3;//3;
	K2[i] = 6;//6;

	D1[i] = 1;
	D2[i] = 1;


	}


	K2[1] = K2[1] -1;
	K2[4] = K2[4] -1;

  K2[2] = K2[2] -2;
  K2[5] = K2[5] -2;


dSWFTiOffset = FTIUP;//0.4;//0.4;//0.5;//More less dSWFTiOffset is , more high legs swing.
dSWFTiFac = -0.1;

Raw_R_fs_old.resize(3);
Raw_L_fs_old.resize(3);
Raw_R_fs.resize(3);
Raw_L_fs.resize(3);
m_reflex_muscle.resize(19);
m_Prereflex.resize(19);
JointAng.resize(19);
PreJointAng.resize(19);
JointVel.resize(19);
PreJointVel.resize(19);

m_preN.resize(19);//pre motor outputs (19 motors)
m_preold.resize(19);//pre motor outputs old (19 motors)

	//----------------muscle initializations---------------




	//TESTING
	test_cpg_output.resize(2);
	test_pcpg_output.resize(2);
	test_psn_output.resize(12);
	test_vrn_output.resize(14);
	test_motor_activity.resize(AMOSII_MOTOR_MAX);
	postcrold.resize(AMOSII_MOTOR_MAX);
	postcr.resize(AMOSII_MOTOR_MAX);
	postclold.resize(AMOSII_MOTOR_MAX);
	postcl.resize(AMOSII_MOTOR_MAX);
	postcldelay.resize(AMOSII_MOTOR_MAX);
  postcrdelay.resize(AMOSII_MOTOR_MAX);
	//NLC output vector sizes
	Neighbourcpg_output.resize(2);
	cpg_output.resize(2);
	cpg_Activity.resize(2);
	pcpg_output.resize(2);
	psn_output.resize(12);
	vrn_output.resize(14);
	tr_output.resize(3);
	tr_outputOld.resize(3);
	tl_outputOld.resize(3);
	tl_output.resize(3);
	cr_output.resize(3);
	cl_output.resize(3);
	fr_output.resize(3);
	fl_output.resize(3);

	//Motor vector sizes
	m_pre.resize(AMOSII_MOTOR_MAX);
	m_reflex.resize(AMOSII_MOTOR_MAX);
	m.resize(AMOSII_MOTOR_MAX);
	m_deg.resize(AMOSII_MOTOR_MAX);
	delta_m_pre.resize(AMOSII_MOTOR_MAX);

	//---Reflex foot sensors
	reflex_fs.resize(AMOSII_SENSOR_MAX);

	//---Reflex infrared sensors
	reflex_irs.resize(AMOSII_SENSOR_MAX);

	//---Initial parameters----//

	//BJ
	bjc_offset = 0.0;

	//Other parameters
	global_count = 0;
	allfoot_off_ground = 0;
	counter = 0;
	ext = 0.0;

	//Neural Locomotion Control constructor
	option_cpg = AdaptiveCpg; // = 1, kohs CPG SO2cpg 2= AdaptiveSO2cpg  3=AdaptiveCpg
	nlc = new ModularNeuralControl(option_cpg);
	//---MODULE 1 parameters
	Control_input = nlc->Control_input; //wave for climbing
	turning = false;
	counter_turn = 0;
	//Inputs
	input.at(0) = 0;
	input.at(1) = 0;
	input.at(2) = 1; // 0, or 1
	input.at(3) = -1;
	input.at(4) = -1;

	//Backbone Joint Control constructor
	bjc = new BackboneJointControl();
	//BJC output vector sizes
	bj_activity.resize(bjc->getNeuronNumber());
	bj_output.resize(bjc->getNeuronNumber());
	bias_bjc = -0.05;

	//Delayline constructor
	//motor time delaydelay

	time = 0;

	//Forward model constructor
	fmodel.resize(AMOSII_MOTOR_MAX);
	acc_error.resize(AMOSII_MOTOR_MAX);
	fmodel_fmodel_w.resize(AMOSII_MOTOR_MAX);
	fmodel_w.resize(AMOSII_MOTOR_MAX);
	fmodel_b.resize(AMOSII_MOTOR_MAX);
	fmodel_counter.resize(AMOSII_MOTOR_MAX);
	fmodel_activity.resize(AMOSII_MOTOR_MAX);
	fmodel_output.resize(AMOSII_MOTOR_MAX);
	fmodel_outputfinal.resize(AMOSII_MOTOR_MAX);
	fmodel_error.resize(AMOSII_MOTOR_MAX);
	fmodel_lerror.resize(AMOSII_MOTOR_MAX);
	converge.resize(AMOSII_MOTOR_MAX);

	//Used learn weights
	switchon_learnweights = true;// 1== used learn weight, 0 == initial weights = 0.0 and let learning develops weights

	if (switchon_learnweights) {
		//cin = 0.05;
		fmodel_fmodel_w.at(CR0_m) = 0.906;//1.65053;//fmodel_cmr_bias= 1.3 fmodel_cmr_w.at= 1.77;counter726cin=0.05
		fmodel_fmodel_w.at(CR1_m) = 0.84;//1.57495;//fmodel_cmr_bias= 1.3 fmodel_cmr_w.at= 1.77;counter2513cin=0.05
		fmodel_fmodel_w.at(CR2_m) = 0.915;//1.29372;//fmodel_cmr_bias= 1.3 fmodel_cmr_w.at= 1.77;counter1699cin=0.05
		fmodel_fmodel_w.at(CL0_m) = 0.906;//1.46362;//fmodel_cml_bias= 1.3 fmodel_cml_w.at= 1.77;counter2282cin=0.05
		fmodel_fmodel_w.at(CL1_m) = 0.84;//1.57309;//fmodel_cml_bias= 1.3 fmodel_cml_w.at= 1.77;counter2532cin=0.05
		fmodel_fmodel_w.at(CL2_m) = 0.944;//1.46362;//fmodel_cml_bias= 1.3 fmodel_cml_w.at= 1.77;counter2250cin=0.05
		fmodel_w.at(CR0_m) = 0.14;
		fmodel_w.at(CR1_m) = 1.828;
		fmodel_w.at(CR2_m) = 0.14;
		fmodel_w.at(CL0_m) = 0.14;
		fmodel_w.at(CL1_m) = 1.828;
		fmodel_w.at(CL2_m) = 0.09;
		fmodel_b.at(CR0_m) = 0.03;
		fmodel_b.at(CR1_m) = 1.707;
		fmodel_b.at(CR2_m) = 0.03;
		fmodel_b.at(CL0_m) = 0.03;
		fmodel_b.at(CL1_m) = 1.707;
		fmodel_b.at(CL2_m) = 0.02;
	} else {
		for (unsigned int i = CR0_m; i < (CL2_m + 1); i++) {
			// MUST! fmodel_cmr_w.at(i)>fmodel_cmr_bias.at(i)
			fmodel_w.at(i) = 1.0;//1.77;                   //2.0;//1.77;//2.0;//1.77;
			fmodel_fmodel_w.at(i) = 1.0;
			fmodel_b.at(i) = 1.0; //1.3;//1.5;//1.3;//1.5;
		}
	}

	for (unsigned int i = TR0_m; i < (FL2_m + 1); i++) {
		fmodel.at(i) = new Forwardmodel(fmodel_fmodel_w.at(i), fmodel_w.at(i), fmodel_b.at(i), switchon_learnweights);
	}

	//Motor mapping constructor
	motormap.resize(AMOSII_MOTOR_MAX);

	for (unsigned int i = TR0_m; i < (FL2_m + 1); i++) {
		motormap.at(i) = new Mapping(amosVersion);
	}

	/*******************************************************************************
	 *  CONTROL OPTION!!!!
	 *******************************************************************************/
	//Switch on or off all reflexes
	switchon_allreflexactions=false;

	//Switch on or off backbone joint control
	switchon_backbonejoint = false;//true;

	//Switch on or off reflexes
	switchon_reflexes = false;//true;//1;// true==on, false == off

	//Switch on pure foot signal
	switchon_purefootsignal = true;//true;//false;//true; // 1==on using only foot signal, 0 == using forward model & foot signal

	//Switch on or off IR reflexes
	switchon_irreflexes = false;//true;

	//Switch on foot inhibition
	switchon_footinhibition = false; //true = hind foot inhibit front foot, false;

	//Switch on soft landing  = reset to normal walking as  soon as acc error = 0
	softlanding = false;

	switchon_obstacle = false;
	lift_body_up=true;
	coxawithContactSensorSignalIsEnabled =false; // add signal from contact sensor to Coxa(Motor Pre) signal for fast convergence(phase reset)
if (lift_body_up==true)
    lifting_value=20;
}
;

NeuralLocomotionControlAdaptiveClimbing::~NeuralLocomotionControlAdaptiveClimbing() {

	//Save files
	outFilenlc1.close();

}
;

void NeuralLocomotionControlAdaptiveClimbing::calculate(const std::vector<double> x,AmosIISensorNames sensorname,NeuralLocomotionControlAdaptiveClimbing * NeighbourNLC,int id,NeuralLocomotionControlAdaptiveClimbing * NLCAC[6],double aNeighbourCpg_activity_0,double aNeighbourCpg_activity_1,double* y)
{
   x_s=x;
   nlc->calculate(x,sensorname,y);
     NeighbourNLCGlobal=NeighbourNLC;
     CPGID=id;
     NeighbourCpg_activity_0=aNeighbourCpg_activity_0;
     NeighbourCpg_activity_1=aNeighbourCpg_activity_1;
     for(int i=0;i<6;i++)
     {
     GlobalNeighbourNLCAC[i]=NLCAC[i];
     }

}
;
std::vector<double> NeuralLocomotionControlAdaptiveClimbing::step_nlc(const std::vector<double> inreflex,
		const std::vector< vector<double> > in0, bool Footinhibition) {


  Raw_R_fs_old.at(0) = Raw_R_fs.at(0); //R0_fs = 19
  Raw_R_fs_old.at(1) = Raw_R_fs.at(1); //R1_fs = 20
  Raw_R_fs_old.at(2) = Raw_R_fs.at(2); //R2_fs = 21
  Raw_L_fs_old.at(0) = Raw_L_fs.at(0); //L0_fs = 22
  Raw_L_fs_old.at(1) = Raw_L_fs.at(1); //L1_fs = 23
  Raw_L_fs_old.at(2) = Raw_L_fs.at(2); //L2_fs = 24


Raw_R_fs.at(0) = inreflex[R0_fs]/FSSCAL; //R0_fs = 19
Raw_R_fs.at(1)= inreflex[R1_fs]/FSSCAL; //R1_fs = 20
Raw_R_fs.at(2)= inreflex[R2_fs]/FSSCAL; //R2_fs = 21
Raw_L_fs.at(0)= inreflex[L0_fs]/FSSCAL; //L0_fs = 22
Raw_L_fs.at(1)= inreflex[L1_fs]/FSSCAL; //L1_fs = 23
Raw_L_fs.at(2)= inreflex[L2_fs]/FSSCAL; //L2_fs = 24





	/*******************************************************************************
	 *  MODULE 1 NEURAL LOCOMOTION CONTROL (ANN framework)
	 *******************************************************************************/

	for (unsigned int i = 0; i < 5; i++) {
		//without obstacle avoidance, deactivate the lines below

		input.at(0) = 0;
		input.at(1) = 0;
		input.at(2) = 1; // 0, or 1
		input.at(3) = -1;
		input.at(4) = -1;
		nlc->setInputNeuronInput(i, input.at(i));
	}

	if( switchon_obstacle){
		//for obstacle avoidance
		input.at(3) = in0[FL_us][1];
		input.at(4) = in0[FR_us][1];
		nlc->setInputNeuronInput(3,input.at(3));//in0[FL_us][1]);
		nlc->setInputNeuronInput(4,input.at(4));// in0[FR_us][1]);

	}
  //nlc->calcualte
	Neighbourcpg_output.at(0) = NeighbourNLCGlobal->nlc->getCpgOutput(0);
	Neighbourcpg_output.at(1) = NeighbourNLCGlobal->nlc->getCpgOutput(1);

	nlc->step(Neighbourcpg_output,CPGID,GlobalNeighbourNLCAC,NeighbourCpg_activity_0,NeighbourCpg_activity_1);
	//nlc->step();
	cpg_output.at(0) = nlc->getCpgOutput(0);
	cpg_output.at(1) = nlc->getCpgOutput(1);
	cpg_Activity.at(0)=nlc->getCpgActivity(0);
	cpg_Activity.at(1)=nlc->getCpgActivity(1);
	cpg_act_0 = nlc->getCpgActivity(0);
	cpg_act_1 = nlc->getCpgActivity(1);
	cpg_w_0 = nlc->getCpgWeight(1, 0);
	cpg_w_1 = nlc->getCpgWeight(0, 1);
	cpg_rec_0 = nlc->getCpgWeight(0, 0);
	cpg_rec_1 = nlc->getCpgWeight(1, 1);
	cpg_bias_0 = nlc->getCpgBias(0);
	cpg_bias_1 = nlc->getCpgBias(1);

	pcpg_output.at(0) = nlc->getpcpgOutput(0);
	pcpg_output.at(1) = nlc->getpcpgOutput(1);

	psn_output.at(10) = nlc->getPsnOutput(10);
	psn_output.at(11) = nlc->getPsnOutput(11);

	vrn_output.at(12) = nlc->getVrnLeftOutput(6);
	vrn_output.at(13) = nlc->getVrnRightOutput(6);


	/// test//
	vrn_outputM=vrn_output.at(13)*(-2.5);
  vrn_outputTanh=tanh(vrn_outputM);
	/////

	for (unsigned int i = 0; i < tr_output.size(); i++) {
		tr_output.at(i) = nlc->getMotorNeuronOutput(AmosIIMotorNames(i + TR0_m));
	}

	for (unsigned int i = 0; i < tl_output.size(); i++) {
		tl_output.at(i) = nlc->getMotorNeuronOutput(AmosIIMotorNames(i + TL0_m));
	}

	for (unsigned int i = 0; i < cr_output.size(); i++) {
		cr_output.at(i) = nlc->getMotorNeuronOutput(AmosIIMotorNames(i + CR0_m));
	}

	for (unsigned int i = 0; i < cl_output.size(); i++) {
		cl_output.at(i) = nlc->getMotorNeuronOutput(AmosIIMotorNames(i + CL0_m));
	}

	for (unsigned int i = 0; i < fr_output.size(); i++) {
		fr_output.at(i) = nlc->getMotorNeuronOutput(AmosIIMotorNames(i + FR0_m));
	}

	for (unsigned int i = 0; i < fl_output.size(); i++) {
		fl_output.at(i) = nlc->getMotorNeuronOutput(AmosIIMotorNames(i + FL0_m));
	}

	/*******************************************************************************
	 *  MODULE 2 BACKBONE JOINT CONTROL
	 *******************************************************************************/

	if (switchon_backbonejoint && global_count > 50) {

		bjc->setInput(0, 0.5 * (in0.at(FR_us).at(0) + in0.at(FL_us).at(0)));
		bjc->setInput(1, 0.5 * (in0.at(R0_fs).at(0) + in0.at(L0_fs).at(0)));
		bjc->setInput(2, neg(in0.at(BJ_as).at(0)));
		bjc->setInput(3, pos(in0.at(BJ_as).at(0)));
		bjc->step();

		for (unsigned int i = 0; i < bjc->getNeuronNumber(); i++) {
			bj_output.at(i) = bjc->getOutput(i);//can be removed when BJC works
			bj_activity.at(i) = bjc->getActivity(i);//can be removed when BJC works
		}
		if (bj_output.at(0) < 0.0) {
			bjc->setOutput(0, 0.0);
		}

		m_pre.at(BJ_m) = bjc->getOutput(5);
		bjc_offset = /*0.5*/0.1 * bjc->getOutput(0);//0.75 for > 0.10

	}
	if (!switchon_backbonejoint) {
		m_pre.at(BJ_m) = 0.0;
	}
	if(!switchon_backbonejoint && !switchon_learnweights) {
		m_pre.at(BJ_m) = bias_bjc;
	}

	/*******************************************************************************
	 *  MODULE 3 WIRING
	 *******************************************************************************/

	//TC joints
	for (unsigned int i = TR0_m; i < (TL2_m + 1); i++)
	{
      if (i < TL0_m)
      {
        tr_outputOld.at(i%tr_outputOld.size())=m_pre.at(i);
          m_pre.at(i)=tr_output.at(2);
      }
      else
      {
         tl_outputOld.at(i%tl_outputOld.size())=m_pre.at(i);
         m_pre.at(i)=tr_output.at(2);
      }
	}
//
  for (unsigned int i = CR0_m; i < (CR2_m + 1); i++) {

      postcr.at(i) = cr_output.at(2);//read(delay)
    }
  for (unsigned int i = CL0_m; i < (CL2_m + 1); i++) {

      postcl.at(i) = cr_output.at(2);
    }

	//CTr joints
  for (unsigned int i = CR0_m; i < (CR2_m + 1); i++) {
    if (coxawithContactSensorSignalIsEnabled==true )// combine contact force feedback with the afferent control signals from PSN to drive coxa joints (CTr)
    {
      m_pre.at(i) = postcr.at(i)+x_s.at((i-CR0_m)+R0_fs);
    }
    else
    {
      m_pre.at(i) = postcr.at(i);
    }

      if (tr_outputOld.at(i%tr_outputOld.size()) >= tr_output.at(i%tr_output.size())) {//(tr_outputOld.at(i%tr_outputOld.size()) >= tr_output.at(i%tr_output.size()))  (postcrdelay.at(i) >= postcrold.at(i))
        m_pre.at(i) = -1; //postprocessing
      }
    }
  for (unsigned int i = CL0_m; i < (CL2_m + 1); i++) {
    if (coxawithContactSensorSignalIsEnabled==true )// combine contact force feedback with the afferent control signals from PSN to drive coxa joints (CTr)
       {
      m_pre.at(i) = postcl.at(i)+x_s.at((i-CR0_m)+R0_fs);//postcl.at(i);//ctr_delayline->Read((CL2_m - i) * tau + tau_l + (CL2_m - i));
       }
    else
    {
      m_pre.at(i) = postcl.at(i);
    }
       if (tl_outputOld.at(i%tl_outputOld.size()) >= tl_output.at(i%tl_output.size()))
      {

        m_pre.at(i) = -1; //postprocessing
      }
  }


	//FTi joints
	for (unsigned int i = FR0_m; i < (FR2_m + 1); i++) {
		m_pre.at(i) = fr_output.at(2);
	}
	for (unsigned int i = FL0_m; i < (FL2_m + 1); i++) {
		m_pre.at(i) = fr_output.at(2);
	}


	//postprocessing

  m_pre.at(FR1_m) = -1.2 *fr_output.at(1);
  m_pre.at(FL1_m) = -1.2 *fr_output.at(1);

	m_pre.at(FR0_m) *= -1.5 * -input.at(4);
	m_pre.at(FL0_m) *= -1.5 * -input.at(3);
	m_pre.at(FR2_m) *= 1.5 * -input.at(4);
	m_pre.at(FL2_m) *= 1.5 * -input.at(3);



	/*******************************************************************************
	 *  MODULE 4 FORWARD MODELS OF PRE MOTOR NEURONS FOR STATE ESTIMATION
	 *******************************************************************************/
	//  cout << "MODULE 4 " << endl;

	for (unsigned int i = R0_fs; i < (L2_fs + 1); i++) {
		reflex_fs.at(i) = in0.at(i).at(0);
	}
	for (unsigned int i = R2_irs; i < (L0_irs + 1); i++) {
		reflex_irs.at(i) = in0.at(i).at(0);
	}

	if (reflex_fs.at(R0_fs) > 0 && reflex_fs.at(R1_fs) > 0 && reflex_fs.at(R2_fs) > 0 && reflex_fs.at(L0_fs) > 0
			&& reflex_fs.at(L1_fs) > 0 && reflex_fs.at(L2_fs) > 0) {
		allfoot_off_ground++; // check if all legs of ground
	}

	if (global_count > 500) {
		//Forward Model Class Test (using learning algorithm from option 4)
		for (unsigned int i = CR0_m; i < (FL2_m + 1); i++) {
			if (i < FR0_m) {
				acc_error.at(i) = fmodel.at(i)->step(m_pre.at(i), reflex_fs.at(i + 13), switchon_learnweights,
						switchon_purefootsignal);
				if (fmodel.at(i)->finish) {
					converge.at(i) = "CONVERGED";
				}
			}
			if (i > CL2_m) {
				acc_error.at(i) = acc_error.at(i - 6);
			}
			//      if (!switchon_reflexes) {
			//        acc_error.at(i) = 0.0;
			//      }
			test_sensorinput = fmodel.at(CR0_m)->input;
			fmodel_activity.at(i) = fmodel.at(i)->activity;
			fmodel_output.at(i) = fmodel.at(i)->output;
			fmodel_outputfinal.at(i) = fmodel.at(i)->outputfinal;
			fmodel_fmodel_w.at(i) = fmodel.at(i)->rec_w;
			fmodel_w.at(i) = fmodel.at(i)->input_w;
			fmodel_b.at(i) = fmodel.at(i)->bias;
			fmodel_counter.at(i) = fmodel.at(i)->counter;
			fmodel_error.at(i) = fmodel.at(i)->error;
			fmodel_lerror.at(i) = fmodel.at(i)->learning_error;
		}
	}

	// To stop all searching reflex
	if(!switchon_reflexes) {
		for (unsigned int i = CR0_m; i < (FL2_m + 1); i++) {
			acc_error.at(i) = 0.0;
		}
		bjc_offset = 0.0;
	}




	/**************** Applying LIFT *********************/
	   if(lift_body_up==true)
	    {
		   lifting_value=20;


	    }
	    else
	    {
	      //Lifting body up
	       lifting_value=0;


	      }

	for (unsigned int i = TR0_m; i < (CR0_m); i++) {
		if (i == CR0_m || i == CL0_m || i == FR0_m || i == FL0_m) {
			m_reflex.at(i) = motormap.at(i)->getReflex(i, m_pre.at(i), 0.5 * acc_error.at(i),0,amosVersion);
		} else
			m_reflex.at(i) = motormap.at(i)->getReflex(i, m_pre.at(i), acc_error.at(i),0,amosVersion);
		m_deg.at(i) = motormap.at(i)->getDegrees(i, m_reflex.at(i),amosVersion);
	}

  for (unsigned int i = CR0_m; i < (FR0_m); i++) {
    if (i == CR0_m || i == CL0_m || i == FR0_m || i == FL0_m) {
      m_reflex.at(i) = motormap.at(i)->getReflex(i, m_pre.at(i), 0.5 * acc_error.at(i),lifting_value,amosVersion);
    } else
      m_reflex.at(i) = motormap.at(i)->getReflex(i, m_pre.at(i), acc_error.at(i),lifting_value,amosVersion);
    m_deg.at(i) = motormap.at(i)->getDegrees(i, m_reflex.at(i),amosVersion);
  }

  for (unsigned int i = FR0_m; i < (BJ_m); i++) {
    if (i == CR0_m || i == CL0_m || i == FR0_m || i == FL0_m) {
      m_reflex.at(i) = motormap.at(i)->getReflex(i, m_pre.at(i), 0.5 * acc_error.at(i),lifting_value,amosVersion);
    } else
      m_reflex.at(i) = motormap.at(i)->getReflex(i, m_pre.at(i), acc_error.at(i),lifting_value,amosVersion);
    m_deg.at(i) = motormap.at(i)->getDegrees(i, m_reflex.at(i),amosVersion);
  }

	//******BJC offset reflexes*****

	for (unsigned int i = CR0_m; i < (BJ_m); i++) {
		if (i == CR1_m || i == CL1_m) {
			m_reflex.at(i) -= bjc_offset;
		}
		if (i == FR1_m || i == FL1_m) {
			m_reflex.at(i) += bjc_offset;
		}
		m_deg.at(i) = motormap.at(i)->getDegrees(i, m_reflex.at(i),amosVersion);
	}
	//******Elevator reflexes*****

	for (unsigned int i = TR0_m; i < (BJ_m); i++) {
		//    std::cout << i << " -> " << motormap.at(i)->getIRsensor(i) << endl;
		if (fmodel.at(i)->error_neg > 6.5 || reflex_irs.at(motormap.at(i)->getIRsensor(i)) > 0.9) {
			if (delta_m_pre.at(i % 6) > 0.0 && switchon_irreflexes) {
				if (motormap.at(i)->TC(i) && in0.at(i).at(0) > 0.4) {
					m_reflex.at(i) -= motormap.at(i)->getElevator(i);
				}
				if (!motormap.at(i)->TC(i)) {
					m_reflex.at(i) = motormap.at(i)->getElevator(i);
				}
				m_deg.at(i) = motormap.at(i)->getDegrees(i, m_reflex.at(i),amosVersion);
			}
		}
	}




	/*******************************************************************************
		                   *  Msucle Model
	 *******************************************************************************/


  //save current neural activations
   for (unsigned int i = TR0_m; i < BJ_m; i++) {

    m_preold.at(i) = m_preN.at(i);
    }

if ((isMuscle ==true) ) //&& (amosVersion ==1)
{
  for (unsigned int i = CR0_m; i < FR0_m; i++)
    {

    m_reflex.at(i) = m_reflex.at(i) + 0.31;
    }

  for (unsigned int i = FR0_m; i < BJ_m; i++)
    {

    m_reflex.at(i) = ((m_reflex.at(i) + 0.76)/0.49) * 0.30 - 0.98;
    }

}
 // get current neural activations
	for (unsigned int i = TR0_m; i < BJ_m; i++)
	{

	m_preN.at(i) = m_reflex.at(i);
	}
//-----------------muscle implementations----------------------

	// for joints in the right front leg
	if (m_preN.at(TR0_m) < m_preold.at(TR0_m))// TR joint
	// stance phases
	{
	// for a FTi joint

	  MusclePara FR0Out,CR0Out;

	     MusclePara ParaR11 = RungeKutta41(PreJointAng.at(FR0_m), PreJointVel.at(FR0_m),
	         SampleTime , Raw_R_fs_old.at(0),Raw_R_fs.at(0), m_preold.at(FR0_m), m_preN.at(FR0_m),
	         0, K1[0], D1[0], HipLength, 0, FMusclActFac);

	  FR0Out.Pos = ParaR11.Pos;// here Pos is the angle of the hip joint.
	  PreJointAng.at(FR0_m) = ParaR11.Pos;// should save the angle (i.e., Pos)

	m_reflex_muscle.at(FR0_m) = Scaling(FR0Out.Pos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
	     FR0Out.Vel = ParaR11.Vel;
	     PreJointVel.at(FR0_m) =  ParaR11.Vel;
	// should save the velocity (i.e., Vel)

	     double DisVe = (HipLength + Radius) * sin(ParaR11.Pos);


	     // for a knee joint
	     MusclePara ParaR12 = RungeKutta42(PreJointAng.at(CR0_m), PreJointVel.at(CR0_m)
	         , SampleTime , Raw_R_fs_old.at(0), Raw_R_fs.at(0), m_preold.at(CR0_m), m_preN.at(CR0_m),
	         DisVe, K2[0], D2[0], KneeLength, 0, CMusclActFac);

	     CR0Out.Pos = ParaR12.Pos;
	     PreJointAng.at(CR0_m) = ParaR12.Pos;
	     m_reflex_muscle.at(CR0_m) = Scaling(CR0Out.Pos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
	     CR0Out.Vel = ParaR12.Vel;
	     PreJointVel.at(CR0_m) = ParaR12.Vel;

	if (m_reflex_muscle.at(CR0_m) > MINCTROUT)
	{
	m_reflex_muscle.at(CR0_m) = MINCTROUT;

	}
	}

	else
	//swing phases
	{
	  m_reflex_muscle.at(FR0_m) = dSWFTiFac * m_reflex.at(FR0_m)-dSWFTiOffset;
	  PreJointAng.at(FR0_m) = Scaling(m_reflex.at(FR0_m), FRMAXOUT, FRMINOUT, FRMAXANG, FRMINANG);
	  PreJointVel.at(FR0_m) = 0;
	  m_reflex_muscle.at(CR0_m) = m_reflex.at(CR0_m);
	  PreJointAng.at(CR0_m) = Scaling(m_reflex.at(CR0_m), CRMAXOUT, CRMINOUT, CRMAXANG, CRMINANG);
	  PreJointVel.at(CR0_m) = 0;

	}


	// for joints in the right middle leg
	  if (m_preN.at(TR1_m) < m_preold.at(TR1_m))// TR joint
	  // stance phases
	  {
	  // for a FTi joint

	    MusclePara FR1Out,CR1Out;

	       MusclePara ParaR11 = RungeKutta41(PreJointAng.at(FR1_m), PreJointVel.at(FR1_m), SampleTime , Raw_R_fs_old.at(1),
	           Raw_R_fs.at(1), m_preold.at(FR1_m), m_preN.at(FR1_m), 0, K1[1], D1[1], HipLength, 0, FMusclActFac);

	    FR1Out.Pos = ParaR11.Pos;// here Pos is the angle of the hip joint.
	    PreJointAng.at(FR1_m) = ParaR11.Pos;// should save the angle (i.e., Pos)

	  m_reflex_muscle.at(FR1_m) = Scaling(FR1Out.Pos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
	       FR1Out.Vel = ParaR11.Vel;
	       PreJointVel.at(FR1_m) =  ParaR11.Vel;
	  // should save the velocity (i.e., Vel)

	       double DisVe = (HipLength + Radius) * sin(ParaR11.Pos);


	       // for a knee joint
	       MusclePara ParaR12 = RungeKutta42(PreJointAng.at(CR1_m), PreJointVel.at(CR1_m), SampleTime
	           , Raw_R_fs_old.at(1), Raw_R_fs.at(1), m_preold.at(CR1_m), m_preN.at(CR1_m),
	           DisVe, K2[1], D2[1], KneeLength, 0, CMusclActFac);

	       CR1Out.Pos = ParaR12.Pos;
	       PreJointAng.at(CR1_m) = ParaR12.Pos;
	       m_reflex_muscle.at(CR1_m) = Scaling(CR1Out.Pos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
	       CR1Out.Vel = ParaR12.Vel;
	       PreJointVel.at(CR1_m) = ParaR12.Vel;

	  if (m_reflex_muscle.at(CR1_m) > MINCTROUT)
	  {
	  m_reflex_muscle.at(CR1_m) = MINCTROUT;

	  }
	  }

	  else
	  //swing phases
	  {
	    m_reflex_muscle.at(FR1_m) = dSWFTiFac * m_reflex.at(FR1_m)-dSWFTiOffset;
	    PreJointAng.at(FR1_m) = Scaling(m_reflex.at(FR1_m), FRMAXOUT, FRMINOUT, FRMAXANG, FRMINANG);
	    PreJointVel.at(FR1_m) = 0;
	    m_reflex_muscle.at(CR1_m) = m_reflex.at(CR1_m);
	    PreJointAng.at(CR1_m) = Scaling(m_reflex.at(CR1_m), CRMAXOUT, CRMINOUT, CRMAXANG, CRMINANG);
	    PreJointVel.at(CR1_m) = 0;

	  }


	  // for joints in the right hind leg
	      if (m_preN.at(TR2_m) < m_preold.at(TR2_m))// TR joint
	      // stance phases
	      {
	      // for a FTi joint

	        MusclePara FR2Out,CR2Out;

	           MusclePara ParaR11 = RungeKutta41(PreJointAng.at(FR2_m), PreJointVel.at(FR2_m), SampleTime , Raw_R_fs_old.at(2),
	               Raw_R_fs.at(2), m_preold.at(FR2_m), m_preN.at(FR2_m), 0, K1[2], D1[2], HipLength, 0, FMusclActFac);

	        FR2Out.Pos = ParaR11.Pos;// here Pos is the angle of the hip joint.
	        PreJointAng.at(FR2_m) = ParaR11.Pos;// should save the angle (i.e., Pos)

	      m_reflex_muscle.at(FR2_m) = Scaling(FR2Out.Pos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
	           FR2Out.Vel = ParaR11.Vel;
	           PreJointVel.at(FR2_m) =  ParaR11.Vel;
	      // should save the velocity (i.e., Vel)

	           double DisVe = (HipLength + Radius) * sin(ParaR11.Pos);


	           // for a knee joint
	           MusclePara ParaR12 = RungeKutta42(PreJointAng.at(CR2_m), PreJointVel.at(CR2_m), SampleTime
	               , Raw_R_fs_old.at(2), Raw_R_fs.at(2), m_preold.at(CR2_m), m_preN.at(CR2_m),
	               DisVe, K2[2], D2[2], KneeLength, 0, CMusclActFac);

	           CR2Out.Pos = ParaR12.Pos;
	           PreJointAng.at(CR2_m) = ParaR12.Pos;
	           m_reflex_muscle.at(CR2_m) = Scaling(CR2Out.Pos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
	           CR2Out.Vel = ParaR12.Vel;
	           PreJointVel.at(CR2_m) = ParaR12.Vel;

	      if (m_reflex_muscle.at(CR2_m) > MINCTROUT)
	      {
	      m_reflex_muscle.at(CR2_m) = MINCTROUT;

	      }
	      }

	      else
	      //swing phases
	      {
	        m_reflex_muscle.at(FR2_m) = dSWFTiFac * m_reflex.at(FR2_m)-dSWFTiOffset;
	        PreJointAng.at(FR2_m) = Scaling(m_reflex.at(FR2_m), FRMAXOUT, FRMINOUT, FRMAXANG, FRMINANG);
	        PreJointVel.at(FR2_m) = 0;
	        m_reflex_muscle.at(CR2_m) = m_reflex.at(CR2_m);
	        PreJointAng.at(CR2_m) = Scaling(m_reflex.at(CR2_m), CRMAXOUT, CRMINOUT, CRMAXANG, CRMINANG);
	        PreJointVel.at(CR2_m) = 0;

	      }






	      // for joints in the left front leg
	        if (m_preN.at(TL0_m) < m_preold.at(TL0_m))// TR joint
	        // stance phases
	        {
	        // for a FTi joint

	          MusclePara FL0Out,CL0Out;

	             MusclePara ParaR11 = RungeKutta41(PreJointAng.at(FL0_m), PreJointVel.at(FL0_m),
	                 SampleTime , Raw_L_fs_old.at(0),Raw_L_fs.at(0), m_preold.at(FL0_m), m_preN.at(FL0_m),
	                 0, K1[3], D1[3], HipLength, 0, FMusclActFac);

	          FL0Out.Pos = ParaR11.Pos;// here Pos is the angle of the hip joint.
	          PreJointAng.at(FL0_m) = ParaR11.Pos;// should save the angle (i.e., Pos)

	        m_reflex_muscle.at(FL0_m) = Scaling(FL0Out.Pos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
	             FL0Out.Vel = ParaR11.Vel;
	             PreJointVel.at(FL0_m) =  ParaR11.Vel;
	        // should save the velocity (i.e., Vel)

	             double DisVe = (HipLength + Radius) * sin(ParaR11.Pos);


	             // for a knee joint
	             MusclePara ParaR12 = RungeKutta42(PreJointAng.at(CL0_m), PreJointVel.at(CL0_m)
	                 , SampleTime , Raw_L_fs_old.at(0), Raw_L_fs.at(0), m_preold.at(CL0_m), m_preN.at(CL0_m),
	                 DisVe, K2[3], D2[3], KneeLength, 0, CMusclActFac);

	             CL0Out.Pos = ParaR12.Pos;
	             PreJointAng.at(CL0_m) = ParaR12.Pos;
	             m_reflex_muscle.at(CL0_m) = Scaling(CL0Out.Pos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
	             CL0Out.Vel = ParaR12.Vel;
	             PreJointVel.at(CL0_m) = ParaR12.Vel;

	        if (m_reflex_muscle.at(CL0_m) > MINCTROUT)
	        {
	        m_reflex_muscle.at(CL0_m) = MINCTROUT;

	        }
	        }

	        else
	        //swing phases
	        {
	          m_reflex_muscle.at(FL0_m) = dSWFTiFac * m_reflex.at(FL0_m)-dSWFTiOffset;
	          PreJointAng.at(FL0_m) = Scaling(m_reflex.at(FL0_m), FRMAXOUT, FRMINOUT, FRMAXANG, FRMINANG);
	          PreJointVel.at(FL0_m) = 0;
	          m_reflex_muscle.at(CL0_m) = m_reflex.at(CL0_m);
	          PreJointAng.at(CL0_m) = Scaling(m_reflex.at(CL0_m), CRMAXOUT, CRMINOUT, CRMAXANG, CRMINANG);
	          PreJointVel.at(CL0_m) = 0;

	        }

	        // for joints in the left middle leg
	                  if (m_preN.at(TL1_m) < m_preold.at(TL1_m))// TR joint
	                  // stance phases
	                  {
	                  // for a FTi joint

	                    MusclePara FL1Out,CL1Out;

	                       MusclePara ParaR11 = RungeKutta41(PreJointAng.at(FL1_m), PreJointVel.at(FL1_m),
	                           SampleTime , Raw_L_fs_old.at(1),Raw_L_fs.at(1), m_preold.at(FL1_m), m_preN.at(FL1_m),
	                           0, K1[4], D1[4], HipLength, 0, FMusclActFac);

	                    FL1Out.Pos = ParaR11.Pos;// here Pos is the angle of the hip joint.
	                    PreJointAng.at(FL1_m) = ParaR11.Pos;// should save the angle (i.e., Pos)

	                  m_reflex_muscle.at(FL1_m) = Scaling(FL1Out.Pos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
	                       FL1Out.Vel = ParaR11.Vel;
	                       PreJointVel.at(FL1_m) =  ParaR11.Vel;
	                  // should save the velocity (i.e., Vel)

	                       double DisVe = (HipLength + Radius) * sin(ParaR11.Pos);


	                       // for a knee joint
	                       MusclePara ParaR12 = RungeKutta42(PreJointAng.at(CL1_m), PreJointVel.at(CL1_m)
	                           , SampleTime , Raw_L_fs_old.at(1), Raw_L_fs.at(1), m_preold.at(CL1_m), m_preN.at(CL1_m),
	                           DisVe, K2[4], D2[4], KneeLength, 0, CMusclActFac);

	                       CL1Out.Pos = ParaR12.Pos;
	                       PreJointAng.at(CL1_m) = ParaR12.Pos;
	                       m_reflex_muscle.at(CL1_m) = Scaling(CL1Out.Pos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
	                       CL1Out.Vel = ParaR12.Vel;
	                       PreJointVel.at(CL1_m) = ParaR12.Vel;

	                  if (m_reflex_muscle.at(CL1_m) > MINCTROUT)
	                  {
	                  m_reflex_muscle.at(CL1_m) = MINCTROUT;

	                  }
	                  }

	                  else
	                  //swing phases
	                  {
	                    m_reflex_muscle.at(FL1_m) = dSWFTiFac * m_reflex.at(FL1_m)-dSWFTiOffset;
	                    PreJointAng.at(FL1_m) = Scaling(m_reflex.at(FL1_m), FRMAXOUT, FRMINOUT, FRMAXANG, FRMINANG);
	                    PreJointVel.at(FL1_m) = 0;
	                    m_reflex_muscle.at(CL1_m) = m_reflex.at(CL1_m);
	                    PreJointAng.at(CL1_m) = Scaling(m_reflex.at(CL1_m), CRMAXOUT, CRMINOUT, CRMAXANG, CRMINANG);
	                    PreJointVel.at(CL1_m) = 0;

	                  }
	                  // for joints in the left hind leg
	                                if (m_preN.at(TL2_m) < m_preold.at(TL2_m))// TR joint
	                                // stance phases
	                                {
	                                // for a FTi joint

	                                  MusclePara FL2Out,CL2Out;

	                                     MusclePara ParaR11 = RungeKutta41(PreJointAng.at(FL2_m), PreJointVel.at(FL2_m),
	                                         SampleTime , Raw_L_fs_old.at(2),Raw_L_fs.at(2), m_preold.at(FL2_m), m_preN.at(FL2_m),
	                                         0, K1[5], D1[5], HipLength, 0, FMusclActFac);

	                                  FL2Out.Pos = ParaR11.Pos;// here Pos is the angle of the hip joint.
	                                  PreJointAng.at(FL2_m) = ParaR11.Pos;// should save the angle (i.e., Pos)

	                                m_reflex_muscle.at(FL2_m) = Scaling(FL2Out.Pos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
	                                     FL2Out.Vel = ParaR11.Vel;
	                                     PreJointVel.at(FL2_m) =  ParaR11.Vel;
	                                // should save the velocity (i.e., Vel)

	                                     double DisVe = (HipLength + Radius) * sin(ParaR11.Pos);


	                                     // for a knee joint
	                                     MusclePara ParaR12 = RungeKutta42(PreJointAng.at(CL2_m), PreJointVel.at(CL2_m)
	                                         , SampleTime , Raw_L_fs_old.at(2), Raw_L_fs.at(2), m_preold.at(CL2_m), m_preN.at(CL2_m),
	                                         DisVe, K2[5], D2[5], KneeLength, 0, CMusclActFac);

	                                     CL2Out.Pos = ParaR12.Pos;
	                                     PreJointAng.at(CL2_m) = ParaR12.Pos;
	                                     m_reflex_muscle.at(CL2_m) = Scaling(CL2Out.Pos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
	                                     CL2Out.Vel = ParaR12.Vel;
	                                     PreJointVel.at(CL2_m) = ParaR12.Vel;

	                                if (m_reflex_muscle.at(CL2_m) > MINCTROUT)
	                                {
	                                m_reflex_muscle.at(CL2_m) = MINCTROUT;

	                                }
	                                }

	                                else
	                                //swing phases
	                                {
	                                  m_reflex_muscle.at(FL2_m) = dSWFTiFac * m_reflex.at(FL2_m)-dSWFTiOffset;
	                                  PreJointAng.at(FL2_m) = Scaling(m_reflex.at(FL2_m), FRMAXOUT, FRMINOUT, FRMAXANG, FRMINANG);
	                                  PreJointVel.at(FL2_m) = 0;
	                                  m_reflex_muscle.at(CL2_m) = m_reflex.at(CL2_m);
	                                  PreJointAng.at(CL2_m) = Scaling(m_reflex.at(CL2_m), CRMAXOUT, CRMINOUT, CRMAXANG, CRMINANG);
	                                  PreJointVel.at(CL2_m) = 0;

	                                }



//-----------------muscle implementations----------------------



	//------------muscle outputs-----------------------

/*****************************************/
/*******************************************************************************
	                   *  FINAL MOTOR OUTPUTS TO MOTOR NEURONS
 *******************************************************************************/
	 if (isMuscle == true)
	 {
	   for (unsigned int i = TR0_m; i < CR0_m; i++) {

	       m.at(i) = m_reflex.at(i);  //m_reflex.at(i);  -1
	     }
	  for (unsigned int i = CR0_m; i < BJ_m; i++) {

	    m.at(i) = m_reflex_muscle.at(i);
	  }


	  m.at(BJ_m) = m_pre.at(BJ_m);
	 }
	 else
	 {

	   for (unsigned int i = TR0_m; i < FR0_m; i++) {

	      m.at(i) = m_reflex.at(i);
	    }
	    for (unsigned int i = FR0_m; i < (BJ_m + 1); i++) {

	      m.at(i) = m_reflex.at(i);  //m_reflex.at(i);  -1
	    }
	    m.at(BJ_m) = m_pre.at(BJ_m);

	 }

  global_count++;
	return m;

}
;
