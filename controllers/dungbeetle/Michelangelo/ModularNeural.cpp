#include "ModularNeural.h"


ModularNeural::ModularNeural(){


	//*******INPUT*******//
		for(int i=0;i<3;i++)
		{
			inputNeurons.push_back(addNeuron());
			inputNeurons.at(i)->setTransferFunction(identityFunction());
		}

	//*******CPG********//
		cpg = new CPG(0.2,0.2,0.2);
		
		addSubnet(cpg);
	//*******POST PROCESSING********
		pcpg = new PCPG();

		w(pcpg->getNeuron(0),cpg->getNeuron(0),5);
		w(pcpg->getNeuron(1),cpg->getNeuron(1),5);

		addSubnet(pcpg);


	//*******PSN********//
		psn = new PSN();

	//connection of input I3 with PSN
		w(psn->getNeuron(0),inputNeurons[2],-1);
		w(psn->getNeuron(1),inputNeurons[2],1);

	//cpg to PSN
		w(psn->getNeuron(2), pcpg->getNeuron(0), 0.5);
		w(psn->getNeuron(3), pcpg->getNeuron(1), 0.5);
		w(psn->getNeuron(4), pcpg->getNeuron(1), 0.5);
		w(psn->getNeuron(5), pcpg->getNeuron(0), 0.5);

		addSubnet(psn);

	//*******VRN********//
		vrn_right = new VRN();
		vrn_left = new VRN();
	
	//connection of PSN output neuron H14 with VRN input X
		w(vrn_left->getNeuron(0), psn->getNeuron(11), 1.75);
		w(vrn_right->getNeuron(0), psn->getNeuron(11), 1.75);

	//connection of I_l,I_r,I3 input neurons with VRN input Y and PSN
		setInputNeuronInput(0.0,0.0,0.0);

		w(vrn_left ->getNeuron(1), inputNeurons[0], 5);
		w(vrn_right->getNeuron(1), inputNeurons[1], 5);

		addSubnet(vrn_left);
		addSubnet(vrn_right);


}

double ModularNeural::getVrnLeftOut(){
	return vrn_left->getOutput(6);
}
double ModularNeural::getVrnRightOut(){
	return vrn_right->getOutput(6);
}

void ModularNeural::update(double p){
	updateActivities();
	cpg->inputPerturbation(p);
	psn->updateWeights();
	pcpg->updateWeights();
	vrn_right->updateWeights();
	vrn_left->updateWeights();
	updateOutputs();
}

void ModularNeural::setInputNeuronInput(double l,double r,double i){

	setInput(inputNeurons[0],l);
	setInput(inputNeurons[1],r);
	setInput(inputNeurons[2],i);
}

double ModularNeural::getCpgOut0(){
	return cpg->getOutput(0);
}

double ModularNeural::getCpgOut1(){
	return cpg->getOutput(1);
}

double ModularNeural::getCpgOut2(){
	return cpg->getOut2();
}

double ModularNeural::getCpgFrequency(){
	return cpg->getFrequency();
}

double ModularNeural::getPsnOutput(int neuron_index){
	return psn->getOutput(neuron_index);
}
double ModularNeural::getP(){
	return cpg->getP();
}
