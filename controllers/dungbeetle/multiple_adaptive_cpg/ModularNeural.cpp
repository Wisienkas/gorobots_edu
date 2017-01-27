#include "ModularNeural.h"


ModularNeural::ModularNeural(bool multiple,int nCPGs){

	if(!multiple){
	//*******INPUT*******//
		inputNeurons.resize(3);
		for(int i=0;i<3;i++)
		{
			inputNeurons.at(i)=addNeuron();
			inputNeurons.at(i)->setTransferFunction(identityFunction());
		}

	//*******CPG********//
		cpg = new CPG(0.2,0.2,0.2);
		
		addSubnet(cpg);
	//*******POST PROCESSING********
		pcpg = new PCPG();

		w(pcpg->getNeuron(0),cpg->getNeuron(0),6);
		w(pcpg->getNeuron(1),cpg->getNeuron(1),6);
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
	
	//connection of PSN output neurok_en H14 with VRN input X
		w(vrn_left->getNeuron(0), psn->getNeuron(11), 1.75);
		w(vrn_right->getNeuron(0), psn->getNeuron(11), 1.75);

	//connection of I_l,I_r,I3 input neurons with VRN input Y and PSN
		//setInputNeuronInput(0.0,0.0,0.0);

		w(vrn_left ->getNeuron(1), inputNeurons[0], 5);
		w(vrn_right->getNeuron(1), inputNeurons[1], 5);

		addSubnet(vrn_left);
		addSubnet(vrn_right);

	}else if(multiple){
		n_CPGs=nCPGs;
		delta = new double*[nCPGs];
		k = new double*[nCPGs];
		for(int i = 0;i<nCPGs;i++){
			delta[i]= new double[nCPGs];
			k[i]= new double[nCPGs];
		}
		a_t.resize(2);
		a_t1.resize(2);
		mCPGs.resize(nCPGs);
		mPSNs.resize(nCPGs);
		for(int i = 0;i < nCPGs; i ++){
			mCPGs.at(i) = new CPG(0.2,0.2,0.2);
			addSubnet(mCPGs.at(i));

			mPSNs.at(i) = new PSN();
	//connection of input I3 with PSN
		setInput(mPSNs.at(i)->getNeuron(0),0);
		setInput(mPSNs.at(i)->getNeuron(1),0);

	//cpg to PSN
		w(mPSNs.at(i)->getNeuron(2), mCPGs.at(i)->getNeuron(0), 0.5);
		w(mPSNs.at(i)->getNeuron(3), mCPGs.at(i)->getNeuron(1), 0.5);
		w(mPSNs.at(i)->getNeuron(4), mCPGs.at(i)->getNeuron(1), 0.5);
		w(mPSNs.at(i)->getNeuron(5), mCPGs.at(i)->getNeuron(0), 0.5);
			addSubnet(mPSNs.at(i));

		}
		///BEGIN COUPLING MATRIX DEFINITION
			for(int i=0;i<nCPGs;i++)
		{
			for(int j=0;j<nCPGs;j++)
			{	
				delta[i][j]=0.0;
				k[i][j]=0.0;
			}
		}
		double k_ij=0.008;
		//4 and 6 legs case
		switch(nCPGs)
		{	//IMPLEMENT MULTIPLE GAIT CHOICE
			//TRIPOD-GAIT
			case 4:
			delta[1][0]=M_PI;
			delta[2][0]=M_PI;delta[2][1]=0;
			delta[3][0]=0;delta[3][1]=M_PI;delta[3][2]=M_PI;

			k[1][0]=k_ij;
			k[2][0]=k_ij;k[2][1]=k_ij;
			k[3][0]=k_ij;k[3][1]=k_ij;k[3][2]=k_ij;

			break;

			case 6:
			delta[1][0]=M_PI;
			delta[2][0]=0;delta[2][1]=M_PI;
			delta[3][0]=M_PI;delta[3][1]=0;delta[3][2]=M_PI;
			delta[4][0]=0;delta[4][1]=M_PI;delta[4][2]=0;delta[4][3]= M_PI;
			delta[5][0]=M_PI; delta[5][1]=0; delta[5][2]=M_PI;delta[5][3]=0; delta[5][4]=M_PI;

			k[1][0]=k_ij;
			k[2][0]=k_ij;k[2][1]=k_ij;
			k[3][0]=k_ij;k[3][1]=k_ij;k[3][2]=k_ij;
			k[4][0]=k_ij;k[4][1]=k_ij;k[4][2]=k_ij;k[4][3]= k_ij;
			k[5][0]=k_ij;k[5][1]=k_ij; k[5][2]=k_ij;k[5][3]=k_ij; k[5][4]=k_ij;
			
		}
			

			for(int i=0;i<nCPGs;i++)
		{
			for(int j=i+1;j<nCPGs;j++)
			{
				delta[i][j]=-delta[j][i];
				k[i][j]=k[j][i];
			}
		}
}///END COUPLING MATRIX DEFINITION
}

double ModularNeural::getVrnLeftOut(){
	return vrn_left->getOutput(6);
}
double ModularNeural::getVrnRightOut(){
	return vrn_right->getOutput(6);
}

void ModularNeural::update(){

	updateActivities();
	cpg->inputPerturbation(0);
	psn->updateWeights();
	pcpg->updateWeights();
	vrn_right->updateWeights();
	vrn_left->updateWeights();
	updateOutputs();
}
void ModularNeural::update(double p,int ID){
	

	a_t.at(0)=mCPGs.at(ID)->getActivity(mCPGs.at(ID)->getNeuron(0));
	a_t.at(1)=mCPGs.at(ID)->getActivity(mCPGs.at(ID)->getNeuron(1));
	mCPGs.at(ID)->inputPerturbation(p);		
	mCPGs.at(ID)->updateActivities();
	mPSNs.at(ID)->updateActivities();
	
	a_t1.at(0)=mCPGs.at(ID)->getActivity(mCPGs.at(ID)->getNeuron(0));
	a_t1.at(1)=mCPGs.at(ID)->getActivity(mCPGs.at(ID)->getNeuron(1));
	
	for(int i=0;i<n_CPGs;i++) 
		{ if(ID!=i){
			osc_couple0=0.1*(1-cos(a_t.at(0) - mCPGs.at(i)->getActivity(mCPGs.at(i)->getNeuron(0))) - delta[ID][i]) + sin(a_t.at(0) - mCPGs.at(i)->getActivity(mCPGs.at(i)->getNeuron(0))- delta[ID][i]);
			osc_couple1=0.1*(1-cos(a_t.at(1) - mCPGs.at(i)->getActivity(mCPGs.at(i)->getNeuron(1))) - delta[ID][i]) + sin(a_t.at(1) - mCPGs.at(i)->getActivity(mCPGs.at(i)->getNeuron(1))- delta[ID][i]);
			
			a_t1.at(0) -= k[ID][i]*osc_couple0;
			a_t1.at(1) -= k[ID][i]*osc_couple1;
			
		}

		
		mCPGs.at(ID)->setActivity(0,a_t1.at(0));
		mCPGs.at(ID)->setActivity(1,a_t1.at(1));
		
	}
			

		
	mCPGs.at(ID)->updateWeights();
	mCPGs.at(ID)->updateOutputs();
	mPSNs.at(ID)->updateWeights();
	mPSNs.at(ID)->updateOutputs();
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

double ModularNeural::getCpgOut0(int ID){
	return mCPGs.at(ID)->getOutput(0);
}
double ModularNeural::getCpgOut1(){
	return cpg->getOutput(1);
}

double ModularNeural::getCpgOut1(int ID){
	return mCPGs.at(ID)->getOutput(1);
}

double ModularNeural::getCpgOut2(){
	return cpg->getOut2();
}

double ModularNeural::getCpgOut2(int ID){
	return mCPGs.at(ID)->getOutput(2);
}

double ModularNeural::getCpgFrequency(){
	return cpg->getFrequency();
}

double ModularNeural::getCpgFrequency(int ID){
	return mCPGs.at(ID)->getFrequency();
}
double ModularNeural::getPsnOutput(int neuron_index){
	return psn->getOutput(neuron_index);
}
double ModularNeural::getP(){
	return cpg->getP();
}
double ModularNeural::getP(int ID){
	return mCPGs.at(ID)->getP();
}

double ModularNeural::getPsnOutput(int ID,int neuron_index){
	
	return mPSNs.at(ID)->getOutput(neuron_index);
}

