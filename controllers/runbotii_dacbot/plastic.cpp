/*
 * plastic.cpp
 *
 *  Created on: Feb 5, 2015
 *      Author: giuliano
 */

#include "plastic.h"


plastic::plastic(double o0,double o1,double o2, double initial_phi,double _alpha, double _bi)
{
	out0_t=o0;
	out1_t=o1;
	out2_t=o2;
	phi=initial_phi;
	alpha=_alpha;
	w20_t=0;//0
	w02_t=1;//1
	w2p_t=0.01;//0.01
	learning=1.3;//1.3
	A02=1;//1
	A20=1;//1
	A2p=1;//1
	B02=_bi;
	B20=_bi;
	B2p=_bi;

	w00=alpha*cos(phi);
	w01=alpha*sin(phi);
	w10=alpha*(-sin(phi));
	w11=alpha*cos(phi);
}
plastic::plastic(double o0,double o1,double o2, double initial_phi,double _alpha) {
	out0_t=o0;
	out1_t=o1;
	out2_t=o2;
	phi=initial_phi;
	alpha=_alpha;
	w20_t=0;//
	w02_t=1;//1
	w2p_t=0.03;//0.03
	learning=1.3;//1.3
	A02=1;//1
	A20=1;//1
	A2p=1;//1
	B02=0.01;
	B20=0.01;
	B2p=0.01;

	w00=alpha*cos(phi);
	w01=alpha*sin(phi);
	w10=alpha*(-sin(phi));
	w11=alpha*cos(phi);

	// TODO Auto-generated constructor stub

}

plastic::~plastic() {
	// TODO Auto-generated destructor stub
}

void plastic::updateWeights()
{
	double e=1;

	phi=phi+learning*w02_t*out2_t*w01*out1_t*e; //update phi and weights
	w00=alpha*cos(phi);
	w01=alpha*sin(phi);
	w10=alpha*(-sin(phi));
	w11=alpha*cos(phi);
}

double plastic::getOut0()
{
	return out0_t;
}

double plastic::getOut1()
{
	return out1_t;
}

double plastic::getOut2()
{
	return out2_t;
}


void plastic::setPhi(double newPhi)
{
phi=newPhi;

}


double plastic::getFrequency()
{

	if (alpha != 1.01)
	{
		std::cout << "change alpha to compute frequency";
		return -1;
	}
	else
	return phi/(2*3.14);



}

double plastic::getW20()
{
	return w20_t;
}

double plastic::getW02()
{
	return w02_t;
}

double plastic::getW2p()
{
	return w2p_t;
}

double plastic::getw01()
{
	return w01;
}


void plastic::update(double perturbation)
{
	double w20_init=0;
	double w02_init=1;//1
	double w2p_init=0.03;//0.05//must check on this!!!!!

	out0_t1=tanh(w00*out0_t+w01*out1_t+w02_t*out2_t);
	out1_t1=tanh(w10*out0_t+w11*out1_t);
	out2_t1=tanh(w20_t*out0_t+w2p_t*perturbation);

	updateWeights();

	w20_t1=w20_t-A20*out2_t*out0_t-B20*(w20_t-w20_init);
	w02_t1=w02_t-A02*out0_t*out2_t-B02*(w02_t-w02_init);
	w2p_t1=w2p_t+A2p*out2_t*perturbation-B2p*(w2p_t-w2p_init);//delete w20


	out0_t=out0_t1;
	out1_t=out1_t1;
	out2_t=out2_t1;

	w20_t=w20_t1;
	w02_t=w02_t1;
	w2p_t=w2p_t1;


}



