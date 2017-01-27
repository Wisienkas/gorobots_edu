#include "cpg.h"

using namespace std;




CPG::CPG(double o0,double o1,double o2){ 

	//three neuron network with 2 plastic synapses
	setDefaultTransferFunction(tanhFunction());
	setNeuronNumber(3);
	
	//perturbation neuron
	P = addNeuron();
	P->setTransferFunction(identityFunction());

	phi = PHI_INIT;
	//weights initialization
	w(n(2),P,EPSILON_INIT);
	w(0,2,GAMMA_INIT);
	w(2,0,BETA_INIT);
	w(0,0,ALPHA*cos(phi));
	w(0,1,ALPHA*sin(phi));
	w(1,0,-ALPHA*sin(phi));
	w(1,1,ALPHA*cos(phi));

	setOutput(2,o2);
	setOutput(0,o0);
	setOutput(1,o1);
	
	//parameters initialization
	A_e = _A;
	A_b = _A;
	A_g = _A;
	B_e = _B;
	B_b = _B;
	B_g = _B;


}

void CPG::updateWeights(){

	double aphi = getPhi();
	double out0 = getOut0();
	double out1 = getOut1();
	double out2 = getOut2();
	double epsilon = getEpsilon();
	double beta = getBeta();
	double gamma = getGamma();
	double W01 = getWeight(0,1);
	double perturbation = getP();
	//update phi and weights
	setPhi(aphi + m * gamma * out2 * W01 * out1);
	setEpsilon(epsilon + A_e * (perturbation * out2) - B_e * ( epsilon - EPSILON_INIT));
	setBeta(beta - A_b * (out0 * out2) - B_b * (beta - BETA_INIT));
	setGamma(gamma - A_g * ( out2 * out0) - B_g * (gamma - GAMMA_INIT));
	
	w(0,0,ALPHA*cos(phi));
	w(0,1,ALPHA*sin(phi));
	w(1,0,ALPHA*(-sin(phi)));
	w(1,1,ALPHA*cos(phi));

}

void CPG::inputPerturbation(double p){

	P->setInput(p);
}

double CPG::getOut0(){
	return getOutput(0);
}
double CPG::getOut1(){
	return getOutput(1);
}
double CPG::getOut2(){
	return getOutput(2);
}

parameter CPG::getFrequency(){

	if(ALPHA != 1.01){
		cout << "Chose alpha = 1 + epsilon,to have a proportinal relationship between phi and frequency"<< endl;
		return -1;
	}
	return phi/(2*M_PI);
}

double CPG::getPhi(){
	return phi;
}

double CPG::getEpsilon(){
	return getSynapse(n(2),P)->getWeight();
}

double CPG::getBeta(){
	return getSynapse(2,0)->getWeight();
}

double CPG::getGamma(){
	return getSynapse(0,2)->getWeight();
}

double CPG::getP(){
	return getOutput(P);
}

void CPG::setPhi(double aphi){
	phi = aphi;
}

void CPG::setEpsilon(double e){
	setWeight(n(2),P,e);
}

void CPG::setBeta(double b){
	setWeight(2,0,b);
}

void CPG::setGamma(double g){
	setWeight(0,2,g);
}

