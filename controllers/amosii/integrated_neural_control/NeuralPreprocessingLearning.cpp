/*
 * NeuralPreprocessingLearning.cpp
 *
 *  Created on: May 2, 2011
 *      Author: poramate
 *
 *      Edited by Dennis Goldschmidt
 *      Apr 27, 2012
 *
 *      Edited by Eduard Grinke
 *      May 10, 2012
 */

#include "NeuralPreprocessingLearning.h"

/// Class for Neural preprocessing and learning------------


NeuralPreprocessingLearning::NeuralPreprocessingLearning() {

	// iostream -- write data to file
	outFilenpp1.open("Neuralpreprocessing.txt");

	//---Set vector size----//
	sensor_map_net.resize(AMOSII_SENSOR_MAX);

	///    Foot Contact Sensor Mapping ANNs
	for (unsigned int i = R0_fs; i < (L2_fs + 1); i++) {
		sensor_map_net.at(i) = new FSM();
	}
	///    Leg Infrared Reflex Sensor Mapping ANNs
	for (unsigned int i = R2_irs; i < (L0_irs + 1); i++) {
		sensor_map_net.at(i) = new IRM();
	}

	mappingsensor.resize(AMOSII_SENSOR_MAX);
	sensor_activity.resize(AMOSII_SENSOR_MAX);
	sensor_output.resize(AMOSII_SENSOR_MAX);

	preprosensor.resize(AMOSII_SENSOR_MAX);
	for(int i=0;i<AMOSII_SENSOR_MAX;i++){preprosensor[i].resize(2);}

	//---Initialize your values

	lowpass = 0.1;
	ir_learnrate = 1.0;//0.35;//1.0//1.5 if > 0.10
	switchon_IRlearning = false;
	delay = 250;

	/// Obstacle avoidance constructor
	OA = new US_Obstacleavoidance();

	/// Obstacle negotiation constructors
	ON.resize(2);
	ON.at(0) = new US_Obstaclenegotiation();
	ON.at(1) = new US_Obstaclenegotiation();

	//Include forn IR and set mix rate for IR and US signals
	FRONT_IR=false;
	rate=1;

}

NeuralPreprocessingLearning::~NeuralPreprocessingLearning() {
	//Save files
	outFilenpp1.close();
}

///  Step function of Neural preprocessing------------
std::vector< vector<double> > NeuralPreprocessingLearning::step_npp(const std::vector<double> in0) {


	/// (1) Prepro Foot sensors for searching reflexes

	for (unsigned int i = R0_fs; i < (L2_fs + 1); i++) {
		sensor_map_net.at(i)->setInput(0, in0.at(i));
		sensor_map_net.at(i)->step();
		mappingsensor.at(i) = sensor_map_net.at(i)->getInput(0);       //TODO: can be removed
		preprosensor.at(i).at(0) = sensor_map_net.at(i)->getOutput(1);
	}


	/// (2) Prepro IR leg sensors for elevator reflexes

	for (unsigned int i = R2_irs; i < (L0_irs + 1); i++) {
		sensor_map_net.at(i)->setInput(0, in0.at(i));
		sensor_map_net.at(i)->step();
		mappingsensor.at(i) = sensor_map_net.at(i)->getOutput(0);
		preprosensor.at(i).at(0) = sensor_map_net.at(i)->getOutput(1);
	}

	/// (3) Prepro Speed Sensors for landing reflexes

	for (unsigned int i = BX_spd; i < (BZ_spd + 1); i++) {
		preprosensor.at(i).at(0) = in0.at(i);
	}

	/// (4) Prepro BJ angle sensors for BJC

	sensor_activity.at(BJ_as) = in0.at(BJ_as);
	sensor_output.at(BJ_as) = lowpass * sensor_activity.at(BJ_as) + (1.0 - lowpass) * sensor_output.at(BJ_as);
	preprosensor.at(BJ_as).at(0) = sensor_output.at(BJ_as);

	/// (5) Learning US Sensors for BJC
	// Inputs to obstacle negotiation network
	for (unsigned int i = FR_us; i < (FL_us + 1); i++) {
		ON.at(i - FR_us)->setInput(ON.at(i - FR_us)->getSubnet(predictive_on)->getNeuron(0), in0.at(i));
		ON.at(i - FR_us)->setInput(ON.at(i - FR_us)->getSubnet(reflex_on)->getNeuron(0), (1./(1.-threshold))*in0.at(i));

		//Return output of OA to preprosensor
		preprosensor.at(i).at(0)=(ON.at(i - FR_us)->getOutput(0));

		//Run ANN step and update all neurons of ON network
		ON.at(i - FR_us)->step();
	}

	for (unsigned int i = TR0_as; i < BJ_as; i++) {
		preprosensor.at(i).at(0) = in0.at(i);
	}

	/// (6) Obstacle avoidance, both sensors have the same network so I just use the FR_us
	if(FRONT_IR){
		sensor_output.at(FR_us)=(rate*in0.at(FR_us)+(1-rate)*in0.at(R0_irs));
		sensor_output.at(FL_us)=(rate*in0.at(FL_us)+(1-rate)*in0.at(L0_irs));
	}
	else{
		sensor_output.at(FL_us)=(in0.at(FL_us));
		sensor_output.at(FR_us)=(in0.at(FR_us));
	}
	//non crossed US config
	OA->setInput(0,sensor_output.at(FL_us)*2-1);
	OA->setInput(1,sensor_output.at(FR_us)*2-1);

	//Return output of OA to preprosensor
	preprosensor.at(FL_us).at(1)=(OA->ANN::getOutput(5));
	preprosensor.at(FR_us).at(1)=(OA->ANN::getOutput(4));

	//Run ANN step and update all neurons of OA network
	OA->step();

	return preprosensor;
}
