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
 *      Edited By Eduard Grinke and Patrick Kesper
 *      July 23, 2012 - Climbing - Avoiding Obstacles and Gap- Using Laserscanner
 *
 *
 */

#include "NeuralLocomotionControlAdaptiveClimbing.h"
#include <ode_robots/amosiisensormotordefinition.h>

//3) Step function of Neural locomotion control------

NeuralLocomotionControlAdaptiveClimbing::NeuralLocomotionControlAdaptiveClimbing() {
	//Save files
	outFilenlc1.open("Neurallocomotion.txt");

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
	test.resize(2);

	//NLC output vector sizes
	cpg_output.resize(2);
	pcpg_output.resize(2);
	psn_output.resize(12);
	vrn_output.resize(14);
	tr_output.resize(3);
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
	option_cpg = SO2cpg; // = 1, kohs CPG
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

	for (unsigned int i = TR0_m; i < (FL2_m + 1); i++) {
		motormap.at(i) = new Mapping();
	}

	/*******************************************************************************
	 *  CONTROL OPTION!!!!
	 *******************************************************************************/
	//Switch on or off all reflexes
	switchon_allreflexactions=false;

	//Switch on or off backbone joint control
	switchon_backbonejoint = false;//true;

	//Switch on or off reflexes
	switchon_reflexes = false;//true;//true;//1;// true==on, false == off

	//Switch on pure foot signal
	switchon_purefootsignal = false;//true;//false;//true; // 1==on using only foot signal, 0 == using forward model & foot signal

	//Switch on or off IR reflexes
	switchon_irreflexes = false;//true;

	//Switch on foot inhibition
	switchon_footinhibition = false; //true = hind foot inhibit front foot, false;

	//Switch on soft landing  = reset to normal walking as  soon as acc error = 0
	softlanding = false;

	switchon_obstacle = false;

	stop=false;

	//----------------Add laser scanner by Patrick-----------------
#ifdef LASERSCANNER
//	this->wasAbove = false;
//	this->wasBelow = false;
	log = true;

	oldWalkingPattern = nlc->Control_input;

	countClimbing = 0;
	countAvoidance = 0;
	countRough = 0;

	heightThreshold = 140;
	gapThreshold = -160;
	roughThreshold = 7.;
	rough2Threshold = 8.2;
	groundThreshold = 30;
	distThreshold = 0.96;
	walkingPatternRough = 0.18;

	climbingThreshold = 700;
	avoidanceThreshold = 20;
	roughStepThreshold = 300;

	if(log){
		out.open("laserData.dat");
		out<<"#Step\tEdges\tAvg.Height\tRough\tMaxHeight\tMinHeight\tBJ\tUS_Right\tUS_left"<<endl;
	}
#endif
	//---------------------------------------------------------

}
;

NeuralLocomotionControlAdaptiveClimbing::~NeuralLocomotionControlAdaptiveClimbing() {

	//Save files
	outFilenlc1.close();
#ifdef LASERSCANNER
	if(log)
		out.close();
#endif

}


void NeuralLocomotionControlAdaptiveClimbing::startClimbing(){
#ifdef LASERSCANNER
	countClimbing++;
#endif
	switchon_backbonejoint=true;
	switchon_obstacle=false;
	switchon_allreflexactions = true;
	switchon_reflexes=true;
	switchon_irreflexes= true;
	switchon_purefootsignal=true;
}

void NeuralLocomotionControlAdaptiveClimbing::startObstacleAvoidance(){
#ifdef LASERSCANNER
	countAvoidance++;
#endif
	switchon_obstacle=true;
	switchon_backbonejoint=false;
	switchon_allreflexactions = false;
	switchon_reflexes=false;
	switchon_irreflexes= false;
	switchon_purefootsignal=false;
//	switchon_backbonejoint=false;
//	switchon_obstacle=false;
//	switchon_allreflexactions = true;
//	switchon_reflexes=true;
//	switchon_irreflexes= true;
//	switchon_purefootsignal=true;
//	input.at(3)=-1;
//	input.at(4)=-1;
}

void NeuralLocomotionControlAdaptiveClimbing::startWalkingForward(){
	switchon_backbonejoint=false;
	switchon_obstacle=false;
	switchon_allreflexactions = false;
	switchon_reflexes=false;
	switchon_irreflexes= false;
	switchon_purefootsignal=false;
	input.at(3)=-1;
	input.at(4)=-1;
}

void NeuralLocomotionControlAdaptiveClimbing::roughWalkingForward(){
	switchon_backbonejoint=false;
	switchon_obstacle=false;
	switchon_allreflexactions = true;
	switchon_reflexes=true;
	switchon_irreflexes= true;
	switchon_purefootsignal=true;
	input.at(3)=-1;
	input.at(4)=-1;
}

void NeuralLocomotionControlAdaptiveClimbing::startWalkingBackward(){
	switchon_backbonejoint=false;
	switchon_obstacle=false;
	switchon_allreflexactions = false;
	switchon_reflexes=false;
	switchon_irreflexes= false;
	switchon_purefootsignal=false;
	input.at(3)=1;
	input.at(4)=1;
}

void NeuralLocomotionControlAdaptiveClimbing::turnLeft(){
	switchon_backbonejoint=false;
	switchon_obstacle=false;
	switchon_allreflexactions = false;
	switchon_reflexes=false;
	switchon_irreflexes= false;
	switchon_purefootsignal=false;
	input.at(3)=1;
	input.at(4)=-1;
}

void NeuralLocomotionControlAdaptiveClimbing::turnRight(){
	switchon_backbonejoint=false;
	switchon_obstacle=false;
	switchon_allreflexactions = false;
	switchon_reflexes=false;
	switchon_irreflexes= false;
	switchon_purefootsignal=false;
	input.at(3) = -1;
	input.at(4) = 1;
}

std::vector<double> NeuralLocomotionControlAdaptiveClimbing::step_nlc(const std::vector<double> inreflex,
		const std::vector< vector<double> > in0, bool Footinhibition) {

	/*******************************************************************************
	 *  MODULE 1 NEURAL LOCOMOTION CONTROL (ANN framework)
	 *******************************************************************************/

	//-------------Laser scanner Patrick----------------------------
#ifdef LASERSCANNER
	//Logging data
	if(log){
		out << global_count; //step count
		//Laser sensor values
		for(int i = LaserNmbEdge_s ; i<=LaserMinHeight_s;i++)
			out << "\t"<<inreflex[i];
		//Backbone joint
		out << "\t"<<bj_output.at(5);
		out << "\t"<<inreflex[BJ_as];
		//Ultrasonic sensors
		out << "\t"<<inreflex.at(FR_us)<<"\t"<<inreflex.at(FL_us);
		out << "\t"<<in0[FR_us][1]<<"\t"<<in0[FL_us][1];
		//All foot sensor and joints
		for(int i = TR0_as; i<=L2_fs;i++)
			out << "\t"<<in0[i][0];
		for(int i = R1_irs; i <= L0_irs; i++)
			out << "\t"<<in0[i][0];
		out <<endl;

	}

	//First try to test when robot has finished climbing. First head goes up. After reaching the top of the obstacle
	//the head will go down to be able pull the main body after him. At some point the head will return to horizontal
	//position. This is not in all cases at the time, when the robot has finished climbing
//	if(bj_output.at(5) > 0.1 && !this->wasAbove && !this->wasBelow){
//		this->wasAbove = true;
//		cout << "was above"<<endl;
//	}
//
//	else if(this->wasAbove){
//		cout <<"is above"<<endl;
//		if(bj_output.at(5)< -0.1){
//			this->wasBelow = true;
//			this->wasAbove = false;
//		}
//	}
//
//	else if(this->wasBelow){
//		cout << "is below"<<endl;
//		if(bj_output.at(5) > -0.05){
//			cout <<"----------------------------------------------------------"<<endl;
//			this->wasBelow = false;
//			this->wasAbove = false;
//		}
//	}

	//Check if climbing
	if(countClimbing >0 && countClimbing <=climbingThreshold){
		countClimbing++;
		cout << countClimbing<<"/"<<climbingThreshold<<endl;
	}
	else if(countClimbing > climbingThreshold)
		countClimbing = 0;

	//Check if avoiding
	else if(countAvoidance >0 && countAvoidance <=avoidanceThreshold){
		countAvoidance++;
		cout << countAvoidance<<"/"<<avoidanceThreshold<<endl;
	}
	else if(countAvoidance > avoidanceThreshold)
		countAvoidance = 0;

	//Check if walking towards rough terrain
	else if(countRough >0 && countRough <=roughStepThreshold){
		countRough++;
		cout << countRough<<"/"<<roughStepThreshold<<endl;
	}
	else if(countRough > roughStepThreshold){
		if(oldWalkingPattern == nlc->Control_input)
			nlc->setControlInput(walkingPatternRough);
		else
			nlc->setControlInput(oldWalkingPattern);
		countRough = 0;
	}

	//Check laser scanner
	else if(inreflex[LaserNmbEdge_s] != -1){ //-1 is error value ,no valid data

		if(inreflex[LaserMinHeight_s] < gapThreshold){ //non crossable gap
			cout << "Gap too low"<<endl;
			turnRight();
		}

		else if(inreflex[LaserMaxHeight_s] > heightThreshold){ //Object is not crossable (too high)
			//Walking with obstacle avoidance
			cout << "Object too high"<<endl;
			startObstacleAvoidance();

		}

//		//About 3 for flat ground, up to 25 for very rough surfaces
//		else if(inreflex[LaserRough_s] < roughThreshold && oldWalkingPattern != nlc->Control_input ){ //Reset walking pattern, when ground is not rough anymore
//			//Change walking pattern
//			countRough ++;
//			startWalkingForward();
//		}
//
//		else if(inreflex[LaserRough_s] > roughThreshold){ //Rough surface found
//			cout <<"Rough surface"<<endl;
//			//Change walking pattern, if current one is not suitable
//			walkingPatternRough = 0.01;
//			if(oldWalkingPattern == nlc->Control_input)
//				countRough ++;
//			//Else continue walking. In this way, climbing is not started, when walking on rough terrain, as
//			//rough terrain often has average heights, which are significantly different from zero
//			roughWalkingForward();
//		}
//
//		else if(inreflex[LaserRough_s] > rough2Threshold){ //Very Rough surface found
//			//Change walking pattern, if current one is not suitable
//			cout <<"Very rough surface"<<endl;
//			walkingPatternRough = 0.03;
//			if(oldWalkingPattern == nlc->Control_input)
//				countRough ++;
//			//Else continue walking. In this way, climbing is not started, when walking on rough terrain, as
//			//rough terrain often has average heights, which are significantly different from zero
//			roughWalkingForward();
//		}

		else if(inreflex[LaserHeight_s] > groundThreshold){ //Crossable object/ found
			if((inreflex.at(FL_us)>distThreshold ||inreflex.at(FR_us)>distThreshold )){
				//Start climbing
				cout <<"Start climbing"<<endl;
				startClimbing();
			}
			else{
				//continue walking without avoidance
				cout << "Object too far away"<<endl;
				startWalkingForward();
			}
		}

		else if(inreflex[LaserHeight_s] < -groundThreshold){ //Crossable gap
			cout << "Crossable gap"<<endl;
			startClimbing();
		}

		else{ //No object in fov
			cout << "No object in fov"<<endl;
			//continue walking without avoidance
			startWalkingForward();
		}

	}

	else{ //Error values from laser
		cout << "error value"<<endl;
	}

#endif

	if(switchon_backbonejoint){
		input.at(0) = 0;
		input.at(1) = 0;
		input.at(2) = 1; // 0, or 1
		input.at(3) = -1;
		input.at(4) = -1;
		for (unsigned int i = 0; i < 5; i++) {
			//without obstacle avoidance, deactivate the lines below
			nlc->setInputNeuronInput(i, input.at(i));
			switchon_allreflexactions = true;
		}
	}

	if( switchon_obstacle){
		//for obstacle avoidance
		input.at(3) = in0[FL_us][1];
		input.at(4) = in0[FR_us][1];
		nlc->setInputNeuronInput(3,input.at(3));//in0[FL_us][1]);
		nlc->setInputNeuronInput(4,input.at(4));// in0[FR_us][1]);

	}


	for (unsigned int i = 0; i < 5; i++) {
		nlc->setInputNeuronInput(i, input.at(i));
	}



	nlc->setInputNeuronInput(3,input.at(3));//in0[FL_us][1]);
	nlc->setInputNeuronInput(4,input.at(4));// in0[FR_us][1]);


	if(!stop){
		nlc->step();
	}



	cpg_output.at(0) = nlc->getCpgOutput(0);
	cpg_output.at(1) = nlc->getCpgOutput(1);
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
	//efficient delay line wiring (using delay line function)

	//writing values into delayline buffer
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
//	outFilenlc1 << m_pre.at(CL0_m) << ' ' << reflex_fs.at(L0_fs) << ' ' << m_pre.at(CL1_m) << ' ' << reflex_fs.at(L1_fs)
//    										  << ' ' << m_pre.at(CL2_m) << ' ' << reflex_fs.at(L2_fs) << endl;


	/*******************************************************************************
	 *  MODULE 5 REFLEX MECHANISMS
	 *******************************************************************************/
	//  cout << "MODULE 5 " << endl;


	//******Motor mapping and searching reflexes*****

	for (unsigned int i = TR0_m; i < (BJ_m); i++) {
		if (i == CR0_m || i == CL0_m || i == FR0_m || i == FL0_m) {
			m_reflex.at(i) = motormap.at(i)->getReflex(i, m_pre.at(i), 0.5 * acc_error.at(i));
		} else
			m_reflex.at(i) = motormap.at(i)->getReflex(i, m_pre.at(i), acc_error.at(i));
		m_deg.at(i) = motormap.at(i)->getDegrees(i, m_reflex.at(i));
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

	for (unsigned int i = TR0_m; i < (BJ_m + 1); i++) {
		m.at(i) = m_reflex.at(i);
	}
	m.at(BJ_m) = m_pre.at(BJ_m);

	global_count++;

	test[0]=inreflex.at(FL_us);
	test[1]=inreflex.at(FR_us);
	return m;

}
;
