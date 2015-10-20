/*
 * NeuralLocomotionControl.cpp
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
 *      May 25, 2014
 */

#include "NeuralLocomotionControl.h"
#include <ode_robots/amosiisensormotordefinition.h>

//3) Step function of Neural locomotion control------
NeuralLocomotionControl::NeuralLocomotionControl() {
	init(2,false,false);
};

NeuralLocomotionControl::NeuralLocomotionControl(int aamosVersion,bool mMCPGs,bool mMuscleModel)
{
	init(aamosVersion,mMCPGs,mMuscleModel);
}

/**
 * init(int aamosVersion,bool mMCPGs,bool mMuscleModelIsEnabled)
 * param: aamosVersion -indicates to AMOS version
 * param: mMCPGs		  -indicates whether single CPG-based control(false) or Multiple CPGs-based control (true)
 * param: mMuscleModelIsEnabled -  muscle model is deployed when mMuscleModelIsEnabled =true
 * author: subhi shaker barikhan
 * date:27.05.2014
 */
void NeuralLocomotionControl::init(int aamosVersion,bool mMCPGs,bool mMuscleModelIsEnabled)
{
	printf("Initialize: Neural Locomotion Control\nAMOS version=%u\nMultiple CPGs=%s\nMuscle model=%s\n", aamosVersion,  mMCPGs ? "ON" : "OFF", mMuscleModelIsEnabled ? "ON" : "OFF");
	//Save files
	outFilenlc1.open("Neurallocomotion.txt");
	amosVersion=aamosVersion;
	MCPGs=mMCPGs;
	muscleModelIsEnabled=mMuscleModelIsEnabled;
	if(MCPGs)
		num_cpgs = 6;
	else
		num_cpgs = 1;

	//---Set vector size----//

	//Input vector size
	input.resize(5);

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

	//NLC output vector sizes
	tr_output.resize(3);
	tl_output.resize(3);
	cr_output.resize(3);
	cl_output.resize(3);
	fr_output.resize(3);
	fl_output.resize(3);
	tr_outputOld.resize(3);
	tl_outputOld.resize(3);

	//Neural Locomotion Control constructor
	option_cpg = SO2cpg; // = 1, kohs CPG
	nlc.resize(num_cpgs);
	cpg_output.resize(num_cpgs);
	pcpg_output.resize(num_cpgs);
	psn_output.resize(num_cpgs);
	vrn_output.resize(num_cpgs);
	for(unsigned int i = 0; i < num_cpgs; i++){
		nlc.at(i) = new ModularNeuralNetwork(option_cpg);
		if(MCPGs){
			if(i<3)
				nlc.at(i)->setCpgOutput(0, -1.);
			else
				nlc.at(i)->setCpgOutput(0, 1.);
		}
		cpg_output.at(i).resize(2);
		pcpg_output.at(i).resize(2);
		psn_output.at(i).resize(12);
		vrn_output.at(i).resize(14);
	}
	Control_input = nlc.at(0)->Control_input;

	//---MODULE 1 parameters
	turning = true;
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
	//motor time delay
	tau = 16;
	tau_l = 48;
	time = 0;
	tr_delayline = new Delayline(2 * tau + 1/*size of delayline buffer*/);
	tl_delayline = new Delayline(tau_l + 2 * tau + 1);
	ctr_delayline = new Delayline(5 * tau + 3);
	fti_delayline = new Delayline(5 * tau + 1);
	ftim_delayline = new Delayline(5 * tau + 1);
	ctr_oldvalue.resize(18);

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
	//amosVersion=2;
	if (amosVersion==1 || amosVersion==2)
	{

		for (unsigned int i = TR0_m; i < (FL2_m + 1); i++) {
			motormap.at(i) = new Mapping(amosVersion); //amosVersion
		}
		if (muscleModelIsEnabled)
		{
			muscleModel=new MuscleModel(amosVersion);
		}
	}
	/*******************************************************************************
	 *  CONTROL OPTION!!!!
	 *******************************************************************************/
	//Switch on or off all reflexes
	switchon_allreflexactions=true;

	//Switch on or off backbone joint control
	switchon_backbonejoint = true;//true;

	//Switch on or off reflexes
	switchon_reflexes = true;//true;//true;//1;// true==on, false == off

	//Switch on pure foot signal
	switchon_purefootsignal = true;//true;//false;//true; // 1==on using only foot signal, 0 == using forward model & foot signal

	//Switch on or off IR reflexes
	switchon_irreflexes = true;//true;

	//Switch on foot inhibition
	switchon_footinhibition = false; //true = hind foot inhibit front foot, false;

	//Switch on soft landing  = reset to normal walking as  soon as acc error = 0
	softlanding = false;

	//Switch on or off obstacle avoidance
	switchon_obstacle = false;

	// lift_value determines how much the lift of AMOS is.
	// For Multiple CPGs set to 20
	// For climbing and adaptive obstacle avoidance set to 0
	// The larger the value, the higher the body is lift.
	//lifting_value = 0; not lifting body up   // Walking on flat
	//lifting_value = 10; small lifting body up // Walking on gravel
	//lifting_value = 20; intermediate lifting body up // Walking on grass
	//lifting_value = 50; high lifting body up // Walking on rocky terrain

	//default is set to 0.0
	lift_value= 0.0;

	if(MCPGs == true)
		lift_value= 10.0; // for MCPGs with foot contact sensory feedback (Subhi controller)
	else
		lift_value= 0.0;

}


NeuralLocomotionControl::~NeuralLocomotionControl() {

	//Save files
	outFilenlc1.close();

}


std::vector<double> NeuralLocomotionControl::step_nlc(const std::vector<double> inreflex,
		const std::vector< vector<double> > in0, double in_navi, bool Footinhibition) {

	/*******************************************************************************
	 *  MODULE 1 NEURAL LOCOMOTION CONTROL (ANN framework)
	 *******************************************************************************/

	if(switchon_obstacle){                 //for obstacle avoidance
		input.at(3) = in0[FL_us][1];
		input.at(4) = in0[FR_us][1];
	}

	if(turning && switchon_backbonejoint){ 						   // keep it straight for obstacle negotiation
		input.at(3)=-1.0 + 3.0*(inreflex.at(119));
		input.at(4)=-1.0 - 3.0*(inreflex.at(119));
		if(input.at(3)> 1.0)
			input.at(3)=1.0;
		if(input.at(3)< -1.0)
			input.at(3)=-1.0;
		if(input.at(4)> 1.0)
			input.at(4)=1.0;
		if(input.at(4)< -1.0)
			input.at(4)=-1.0;
	}

	for(unsigned int i_cpg = 0; i_cpg < num_cpgs; i_cpg++){
		for(unsigned int i_in = 0; i_in < input.size(); i_in++)
			nlc.at(i_cpg)->setInputNeuronInput(i_in, input.at(i_in));

		if(!MCPGs) // Single CPG is utilized
			nlc.at(i_cpg)->step();
		else // multiple CPGs are utilized
			nlc.at(i_cpg)->step(i_cpg, cpg_output, inreflex);


		for (int i_legs = 0; i_legs < num_legpairs; i_legs++) {
			if(i_cpg<3){
				if(i_legs == i_cpg && MCPGs){
					tr_output.at(i_legs) = nlc.at(i_cpg)->getMotorNeuronOutput(AmosIIMotorNames(i_legs + TR0_m));
					if(i_legs%2)
						cr_output.at(i_legs) = nlc.at(i_cpg)->getMotorNeuronOutput(AmosIIMotorNames(((i_legs + 3)%6) + CR0_m));		// TODO: Hack for uneven leg CTR signals to take the even counterpart
					else
						cr_output.at(i_legs) = nlc.at(i_cpg)->getMotorNeuronOutput(AmosIIMotorNames(i_legs + CR0_m));
					if(i_legs==1)
						fr_output.at(i_legs) = nlc.at(i_cpg)->getMotorNeuronOutput(AmosIIMotorNames(FR1_m));
					else
						fr_output.at(i_legs) = nlc.at(i_cpg)->getMotorNeuronOutput(AmosIIMotorNames(FR2_m));
				}
				if(!MCPGs){
					tr_output.at(i_legs) = nlc.at(i_cpg)->getMotorNeuronOutput(AmosIIMotorNames(i_legs + TR0_m));
					cr_output.at(i_legs) = nlc.at(i_cpg)->getMotorNeuronOutput(AmosIIMotorNames(i_legs + CR0_m));
					fr_output.at(i_legs) = nlc.at(i_cpg)->getMotorNeuronOutput(AmosIIMotorNames(i_legs + FR0_m));
					tl_output.at(i_legs) = nlc.at(i_cpg)->getMotorNeuronOutput(AmosIIMotorNames(i_legs + TL0_m));
					cl_output.at(i_legs) = nlc.at(i_cpg)->getMotorNeuronOutput(AmosIIMotorNames(i_legs + CL0_m));
					fl_output.at(i_legs) = nlc.at(i_cpg)->getMotorNeuronOutput(AmosIIMotorNames(i_legs + FL0_m));
				}
			}
			else{
				if(i_legs == i_cpg%3 && MCPGs){
					tl_output.at(i_legs) = nlc.at(i_cpg)->getMotorNeuronOutput(AmosIIMotorNames(i_legs + TL0_m));
					if(i_cpg%2)
						cl_output.at(i_legs) = nlc.at(i_cpg)->getMotorNeuronOutput(AmosIIMotorNames(((i_cpg + 3)%6) + CR0_m));	// TODO: Hack for uneven leg CTR signals to take the even counterpart
					else
						cl_output.at(i_legs) = nlc.at(i_cpg)->getMotorNeuronOutput(AmosIIMotorNames((i_legs + CL0_m)));
					if(i_legs==1)
						fl_output.at(i_legs) = nlc.at(i_cpg)->getMotorNeuronOutput(AmosIIMotorNames(FR1_m));
					else
						fl_output.at(i_legs) = nlc.at(i_cpg)->getMotorNeuronOutput(AmosIIMotorNames(FR2_m));
				}
			}
		}
	}

	for(unsigned int i_cpg = 0; i_cpg < num_cpgs; i_cpg++){
		cpg_output.at(i_cpg).at(0) = nlc.at(i_cpg)->getCpgOutput(0);
		cpg_output.at(i_cpg).at(1) = nlc.at(i_cpg)->getCpgOutput(1);

		pcpg_output.at(i_cpg).at(0) = nlc.at(i_cpg)->getpcpgOutput(0);
		pcpg_output.at(i_cpg).at(1) = nlc.at(i_cpg)->getpcpgOutput(1);

		psn_output.at(i_cpg).at(10) = nlc.at(i_cpg)->getPsnOutput(10);
		psn_output.at(i_cpg).at(11) = nlc.at(i_cpg)->getPsnOutput(11);

		vrn_output.at(i_cpg).at(12) = nlc.at(i_cpg)->getVrnLeftOutput(6);
		vrn_output.at(i_cpg).at(13) = nlc.at(i_cpg)->getVrnRightOutput(6);
	}

//	for(unsigned int i_cpg = 0; i_cpg < num_cpgs; i_cpg++){
//		for(unsigned int i_in = 0; i_in < input.size(); i_in++)
//			nlc.at(i_cpg)->setInputNeuronInput(i_in, input.at(i_in));
//
//		if(!MCPGs) // Single CPG is utilized
//			nlc.at(i_cpg)->step();
//		else // multiple CPGs are utilized
//			nlc.at(i_cpg)->step(i_cpg, cpg_output, inreflex);
//
//		cpg_output.at(i_cpg).at(0) = nlc.at(i_cpg)->getCpgOutput(0);
//		cpg_output.at(i_cpg).at(1) = nlc.at(i_cpg)->getCpgOutput(1);
//
//		pcpg_output.at(i_cpg).at(0) = nlc.at(i_cpg)->getpcpgOutput(0);
//		pcpg_output.at(i_cpg).at(1) = nlc.at(i_cpg)->getpcpgOutput(1);
//
//		psn_output.at(i_cpg).at(10) = nlc.at(i_cpg)->getPsnOutput(10);
//		psn_output.at(i_cpg).at(11) = nlc.at(i_cpg)->getPsnOutput(11);
//
//		vrn_output.at(i_cpg).at(12) = nlc.at(i_cpg)->getVrnLeftOutput(6);
//		vrn_output.at(i_cpg).at(13) = nlc.at(i_cpg)->getVrnRightOutput(6);
//
//		for (unsigned int i_legs = 0; i_legs < tr_output.size(); i_legs++) {
//			tr_output.at(i_legs) = nlc.at(i_cpg)->getMotorNeuronOutput(AmosIIMotorNames(i_legs + TR0_m));
//			tl_output.at(i_legs) = nlc.at(i_cpg)->getMotorNeuronOutput(AmosIIMotorNames(i_legs + TL0_m));
//			cr_output.at(i_legs) = nlc.at(i_cpg)->getMotorNeuronOutput(AmosIIMotorNames(i_legs + CR0_m));
//			cl_output.at(i_legs) = nlc.at(i_cpg)->getMotorNeuronOutput(AmosIIMotorNames(i_legs + CL0_m));
//			fr_output.at(i_legs) = nlc.at(i_cpg)->getMotorNeuronOutput(AmosIIMotorNames(i_legs + FR0_m));
//			fl_output.at(i_legs) = nlc.at(i_cpg)->getMotorNeuronOutput(AmosIIMotorNames(i_legs + FL0_m));
//		}
//	}

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
	 *
	 *  author: Subhi Shaker Barikhan
	 *******************************************************************************/

	if (MCPGs==false) // Single CPG-based control
	{
		//efficient delay line wiring (using delay line function)

		//writing values into delayline buffer
		// Single CPG
		tr_delayline->Write(tr_output.at(2));
		tl_delayline->Write(tl_output.at(2));
		ctr_delayline->Write(cr_output.at(2));
		ftim_delayline->Write(fr_output.at(1));
		fti_delayline->Write(fr_output.at(2));

		for (unsigned int i = CR0_m; i < (CR2_m + 1); i++) {
			postcrold.at(i) = postcr.at(i);
			postcr.at(i) = ctr_delayline->Read((CR2_m - i) * tau + (CR2_m - i));
		}
		for (unsigned int i = CL0_m; i < (CL2_m + 1); i++) {
			postclold.at(i) = postcl.at(i);
			postcl.at(i) = ctr_delayline->Read((CL2_m - i) * tau + tau_l + (CL2_m - i));
		}

		//read out delayed values

		//TC joints
		for (int i = TR0_m; i < (TL2_m + 1); i++) {
			if (i < TL0_m) {
				delta_m_pre.at(i) = tr_delayline->Read((TR2_m - i) * tau) - m_pre.at(i);
				m_pre.at(i) = tr_delayline->Read((TR2_m - i) * tau); //Right side
			} else {
				delta_m_pre.at(i) = tl_delayline->Read((TL2_m - i) * tau + tau_l) - m_pre.at(i);
				m_pre.at(i) = tl_delayline->Read((TL2_m - i) * tau + tau_l); //Left side
			}
		}

		//CTr joints
		for (unsigned int i = CR0_m; i < (CR2_m + 1); i++) {
			m_pre.at(i) = ctr_delayline->Read((CR2_m - i) * tau + (CR2_m - i));
			if (postcrold.at(i) > postcr.at(i)) {
				m_pre.at(i) = -1; //postprocessing
			}
		}
		for (unsigned int i = CL0_m; i < (CL2_m + 1); i++) {
			m_pre.at(i) = ctr_delayline->Read((CL2_m - i) * tau + tau_l + (CL2_m - i));
			if (postclold.at(i) > postcl.at(i)) {
				m_pre.at(i) = -1; //postprocessing
			}
		}

		//FTi joints
		for (unsigned int i = FR0_m; i < (FR2_m + 1); i++) {
			m_pre.at(i) = fti_delayline->Read((FR2_m - i) * tau);
		}
		for (unsigned int i = FL0_m; i < (FL2_m + 1); i++) {
			m_pre.at(i) = fti_delayline->Read((FL2_m - i) * tau + tau_l);
		}


		//postprocessing
		m_pre.at(FR1_m) = -1.2 * ftim_delayline->Read(tau - 1);
		m_pre.at(FL1_m) = -1.2 * ftim_delayline->Read(tau + tau_l - 1);
		m_pre.at(FR0_m) *= -1.5 * -input.at(4);
		m_pre.at(FL0_m) *= -1.5 * -input.at(3);
		m_pre.at(FR2_m) *= 1.5 * -input.at(4);
		m_pre.at(FL2_m) *= 1.5 * -input.at(3);

		//take one step
		tr_delayline->Step();
		tl_delayline->Step();
		ctr_delayline->Step();
		fti_delayline->Step();
		ftim_delayline->Step();
	}
	else //  MCPGs-based control
	{  //MCPGs
		//TC joints
		for (unsigned int i = TR0_m; i < (TL2_m + 1); i++)
		{
			if (i < TL0_m)
			{
				tr_outputOld.at(i%tr_outputOld.size())=m_pre.at(i);
				m_pre.at(i)=tr_output.at(i%tr_output.size());
			}
			else
			{
				tl_outputOld.at(i%tl_outputOld.size())=m_pre.at(i);
				m_pre.at(i)=tl_output.at(i%tl_output.size());
			}
		}
		//
		for (unsigned int i = CR0_m; i < (CR2_m + 1); i++) {

			postcr.at(i) = cr_output.at(i%cr_output.size());//read(delay)
		}
		for (unsigned int i = CL0_m; i < (CL2_m + 1); i++) {
			postcl.at(i) = cl_output.at(i%cl_output.size());
		}

		//CTr joints
		for (unsigned int i = CR0_m; i < (CR2_m + 1); i++) {


			m_pre.at(i) = postcr.at(i);


			if (tr_outputOld.at(i%tr_outputOld.size()) >= tr_output.at(i%tr_output.size())) {//(tr_outputOld.at(i%tr_outputOld.size()) >= tr_output.at(i%tr_output.size()))  (postcrdelay.at(i) >= postcrold.at(i))
				m_pre.at(i) = -1; //postprocessing
			}
		}
		for (unsigned int i = CL0_m; i < (CL2_m + 1); i++) {

			m_pre.at(i) = postcl.at(i);
			if (tl_outputOld.at(i%tl_outputOld.size()) >= tl_output.at(i%tl_output.size()))
			{
				m_pre.at(i) = -1; //postprocessing
			}
		}


		//FTi joints
		for (unsigned int i = FR0_m; i < (FR2_m + 1); i++) {
			m_pre.at(i) = fr_output.at(i%fr_output.size());
		}
		for (unsigned int i = FL0_m; i < (FL2_m + 1); i++) {
			m_pre.at(i) = fl_output.at(i%fl_output.size());
		}


		//postprocessing

		m_pre.at(FR1_m) = -1.2 *fr_output.at(1);
		m_pre.at(FL1_m) = -1.2 *fl_output.at(1);

		m_pre.at(FR0_m) *= -1.5 * -input.at(4);
		m_pre.at(FL0_m) *= -1.5 * -input.at(3);
		m_pre.at(FR2_m) *= 1.5 * -input.at(4);
		m_pre.at(FL2_m) *= 1.5 * -input.at(3);
	}

	/*********************End*************************/





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
	// >> i/o operations here <<
	outFilenlc1 << m_pre.at(CL0_m) << ' ' << reflex_fs.at(L0_fs) << ' ' << m_pre.at(CL1_m) << ' ' << reflex_fs.at(L1_fs)
    								  << ' ' << m_pre.at(CL2_m) << ' ' << reflex_fs.at(L2_fs) << endl;

	/*******************************************************************************
	 *  MODULE 5 REFLEX MECHANISMS
	 *******************************************************************************/
	//******Motor mapping and searching reflexes*****
	if (amosVersion==1 || amosVersion==2 )
	{
		for (unsigned int i = TR0_m; i < (BJ_m); i++) {
			if (i == CR0_m || i == CL0_m || i == FR0_m || i == FL0_m) {
				m_reflex.at(i) = motormap.at(i)->getReflex(i, m_pre.at(i), 0.5 * acc_error.at(i),amosVersion,lift_value);
			} else
				m_reflex.at(i) = motormap.at(i)->getReflex(i, m_pre.at(i), acc_error.at(i),amosVersion,lift_value);
			m_deg.at(i) = motormap.at(i)->getDegrees(i, m_reflex.at(i),amosVersion);
		}
	}
	else
	{
		for (unsigned int i = TR0_m; i < (BJ_m); i++) {
			if (i == CR0_m || i == CL0_m || i == FR0_m || i == FL0_m) {
				m_reflex.at(i) = motormap.at(i)->getReflex(i, m_pre.at(i), 0.5 * acc_error.at(i));
			} else
				m_reflex.at(i) = motormap.at(i)->getReflex(i, m_pre.at(i), acc_error.at(i));
			m_deg.at(i) = motormap.at(i)->getDegrees(i, m_reflex.at(i));
		}
	}


	//******BJC offset reflexes*****

	for (unsigned int i = CR0_m; i < (BJ_m); i++) {
		if (i == CR1_m || i == CL1_m) {
			m_reflex.at(i) -= bjc_offset;
		}
		if (i == FR1_m || i == FL1_m) {
			m_reflex.at(i) += bjc_offset;
		}
		m_deg.at(i) = motormap.at(i)->getDegrees(i, m_reflex.at(i));
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
				m_deg.at(i) = motormap.at(i)->getDegrees(i, m_reflex.at(i));
			}
		}
	}

	/*******************************************************************************
	 *  FINAL MOTOR OUTPUTS TO MOTOR NEURONS
	 *******************************************************************************/
	if (muscleModelIsEnabled) // Muscle model is utilized
	{
		std::vector<double> m_reflex_muscle=muscleModel->getMuscleReflex(m_reflex,inreflex);

		for (unsigned int i = TR0_m; i < (BJ_m + 1); i++) {
			m.at(i) =m_reflex_muscle.at(i);
		}

		m.at(BJ_m) = m_pre.at(BJ_m);

	}
	else
	{
		for (unsigned int i = TR0_m; i < (BJ_m + 1); i++) {
			m.at(i) = m_reflex.at(i);
		}
		m.at(BJ_m) = m_pre.at(BJ_m);


	}
	global_count++;
	return m;

}
;
