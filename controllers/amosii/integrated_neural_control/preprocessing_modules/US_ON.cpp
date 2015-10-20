/*
 * US_ON.cpp
 *
 *  Created on: Mar 25, 2015
 *      Author: degoldschmidt
 */

#include "US_ON.h"

US_Obstaclenegotiation::US_Obstaclenegotiation(){
	learning_on = false;
	setNeuronNumber(1);
	setAllTransferFunctions(identityFunction(), true);
	for(int i = 0; i < num_signals; i++){
		ANN * sub = new ANN(4);
		sub->setAllTransferFunctions(identityFunction(), true);
		sub->setWeight(1, 0, lowpass);
		sub->setWeight(1, 1, 1. - lowpass);
		sub->setWeight(2, 1, lowpass);
		sub->setWeight(2, 2, 1. - lowpass);
		sub->setWeight(3, 2, lowpass);
		sub->setWeight(3, 3, 1. - lowpass);
		addSubnet(sub);
		if(i == predictive_on){
			if(learning_on)
				w(getNeuron(0), sub->getNeuron(3), 0.);
			else
				w(getNeuron(0), sub->getNeuron(3), 3.);
		}
		if(i == reflex_on)
			w(getNeuron(0), sub->getNeuron(3), 0.1);
	}
	learning = new ICO(learn_rate);
}

US_Obstaclenegotiation::~US_Obstaclenegotiation(){

}

void US_Obstaclenegotiation::step(){
	updateActivities();
	updateOutputs();
	reflex_input = getOutput(getSubnet(reflex_on)->getNeuron(3));
	predictive_input = getOutput(getSubnet(predictive_on)->getNeuron(3));

	learning->setReflexiveNeuronInput(reflex_input);
	learning->setPredictiveNeuronInput(0, predictive_input);
	learning->step();
}
