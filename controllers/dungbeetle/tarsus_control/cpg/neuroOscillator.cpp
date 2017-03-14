/*
 * neuronOscillator.cpp
 *
 *  Created on: Sep 27, 2014
 *      Author: giuliano
 */

#include <controllers/dungbeetle/tarsus_control/cpg/neuroOscillator.h>



neuroOscillator::neuroOscillator(double out0_init,double out1_init,double alpha,double phi)
{
	w00=alpha*cos(phi);
	w01=alpha*sin(phi);
	w10=alpha*(-sin(phi));
	w11=alpha*cos(phi);
	out0_t=out0_init;
	out1_t=out1_init;
	Phi=phi;
	Alpha=alpha;
	//cout << out0_t;
}

neuroOscillator::~neuroOscillator() {
	// TODO Auto-generated destructor stub
}

void neuroOscillator::computeOut0()
{
	out0_t1=tanh((w00*out0_t+w01*out1_t));

}

double neuroOscillator::getOutPut0()
{
	return out0_t;
}

double neuroOscillator::getOutPut1()
{
	return out1_t;
}



void neuroOscillator::computeOut1()
{

	out1_t1=tanh((w10*out0_t+w11*out1_t));

}
double neuroOscillator::getFrequency()
{
	if (Alpha != 1.01)
	{
		cout << "change alpha to compute frequency";
		return -1;
	}
	else
	return Phi/(2*3.14);

}


void neuroOscillator::update()
{

		computeOut0();
		computeOut1();
	    out0_t=out0_t1;
	    out1_t=out1_t1;


}

void neuroOscillator::printWeights()
{
	cout << "w00: " << w00 << "  w01: " <<w10 << "  w01: " << w01 << "  w11: " << w11 <<  endl;


}




