#include "dnf.h"
#include <math.h>
#include <iostream>
#include <iomanip>

std::vector<double> conv(std::vector<double> const &f, std::vector<double> const &g) {
	int const nf = f.size();
	int const ng = g.size();
	int const n  = nf + ng - 1;
	std::vector<double> out(n);
	for(int i=0; i < n; ++i) {
		int const jmn = (i >= ng - 1)? i - (ng - 1) : 0;
		int const jmx = (i <  nf - 1)? i            : nf - 1;
		for(int j=jmn; j <= jmx; ++j) {
			out[i] += (f[j] * g[i - j]);
		}
	}
	std::vector<double> returnVec;
	for (int i=0;i<nf;i++){
		returnVec.push_back(out[nf-1+i]);
	}
	return returnVec; 
}

DNF::DNF(){
	stimSize = 0;
}

DNF::~DNF(){
}

void DNF::setup(int _size, double _tau, double _h, double _beta){
	size = _size;
	tau = _tau;
	h = _h;
	beta = _beta;
	for (int i=0;i<size;i++){
		activation.push_back( (0.0 + h) );
		output.push_back(sigmoid(activation[i],beta));
	}
}

double DNF::sigmoid(double x, double beta){
	return (1.0 / (1.0 + exp(-beta * x)));
}


double DNF::gauss(int pos, int mu, double sigma){
	if (sigma==0) {
		if (pos == mu) return 1.0;
		else return 0.0;
	} else {
		return exp(-0.5 * pow((pos-mu),2.0) / pow(sigma,2.0));
	}
}

void DNF::addStim(int pos, double sigma){
	GaussStimulus tmp;
	tmp.pos = pos;
	tmp.sigma = sigma;
	stimuli.push_back(tmp);
	stimSize++;
}

void DNF::setAmplitudes(std::vector<double> amplitudes){
	for (unsigned int i=0; i < amplitudes.size(); i++){
		stimuli[i].amplitude = amplitudes[i];
	}
}

std::vector<double> DNF::getSumStims(){
	std::vector<double> stimVec;
	int stimID = 0;
	for (int stimID=0;stimID<stimSize;stimID++){
		for (int i=0; i<size; i++){
			double value = stimuli[stimID].amplitude * gauss(i,stimuli[stimID].pos, stimuli[stimID].sigma);
			if (stimID==0) stimVec.push_back(value);
			else stimVec[i] += value;
		}
	}
	return stimVec;
}

void DNF::step(){
	std::vector<double> stimVec = getSumStims();
//for (unsigned int i=0; i<stimVec.size();i++) std::cout<<std::setprecision(4)<<std::fixed<<i<<": "<<stimVec[i]<<std::endl; std::cout<<std::endl<<std::endl;
	
	// do DNF step()	
	for (int i=0; i<size; i++){
		activation[i] = activation[i] + (1.0 / tau * (- activation[i] + h + (stimVec[i]+lateralInteraction.output[i])));
		output[i] = sigmoid(activation[i],beta);
	} 	
//for (unsigned int i=0; i<activation.size();i++) std::cout<<std::setprecision(4)<<std::fixed<<i<<": "<<activation[i]<<std::endl; std::cout<<std::endl<<std::endl;
//for (unsigned int i=0; i<output.size();i++) std::cout<<std::setprecision(4)<<std::fixed<<i<<": "<<output[i]<<std::endl; std::cout<<std::endl<<std::endl;
	
	// lateral step	
	lateralInteraction.fullSum = 0.0;
	for(std::vector<double>::iterator it = output.begin(); it != output.end(); ++it) lateralInteraction.fullSum += *it;

	// convolution
	std::vector<double> convIn;
	for ( int i = size - lateralInteraction.kernelRangeRight; i < size; i++ ) convIn.push_back(output[i]);
	for ( int i = 0; i < size; i++ ) convIn.push_back(output[i]);
	for ( int i = 0; i < lateralInteraction.kernelRangeLeft; i++ ) convIn.push_back(output[i]);	

	std::vector<double> convOut = conv(lateralInteraction.kernel, convIn);
	
	for ( int i = 0; i < size; i++ ) {
		lateralInteraction.output[i] = convOut[i] + lateralInteraction.amplitudeGlobal * lateralInteraction.fullSum;
	}
//for (unsigned int i=0; i<lateralInteraction.kernel.size();i++) std::cout<<std::setprecision(4)<<std::fixed<<i<<": "<<lateralInteraction.kernel[i]<<std::endl; std::cout<<std::endl<<std::endl;	
//for (unsigned int i=0; i<convOut.size();i++) std::cout<<std::setprecision(4)<<std::fixed<<i<<": "<<convOut[i]<<std::endl; std::cout<<std::endl<<std::endl;

}

void DNF::setupLateral(double _sigmaExc, double _amplitudeExc, double _sigmaInh, double _amplitudeInh, double _amplitudeGlobal, double _cutoffFactor){
	lateralInteraction.sigmaExc 		= _sigmaExc;
	lateralInteraction.amplitudeExc 	= _amplitudeExc;
	lateralInteraction.sigmaInh 		= _sigmaInh;
	lateralInteraction.amplitudeInh 	= _amplitudeInh;			
	lateralInteraction.amplitudeGlobal	= _amplitudeGlobal;
	lateralInteraction.cutoffFactor 	= _cutoffFactor;

	for (int i=0; i<size; i++) lateralInteraction.output.push_back(0.0);

	
	bool useExc;
	if (lateralInteraction.amplitudeExc != 0) useExc = true;
	else useExc = false;
	
	bool useInh;
	if (lateralInteraction.amplitudeInh != 0) useInh = true;
	else useExc = false;

	double kernelRange;
	if (useExc && useInh) {
		if (lateralInteraction.sigmaExc > lateralInteraction.sigmaInh) kernelRange = lateralInteraction.sigmaExc;
		else kernelRange = lateralInteraction.sigmaInh;
	} else if (useExc) kernelRange = lateralInteraction.sigmaExc;
	else if (useInh) kernelRange = lateralInteraction.sigmaInh;
	else kernelRange = 0.0;
		
	kernelRange *= lateralInteraction.cutoffFactor;
	

	if ( ceil(kernelRange) < floor(((double)size-1.0)/2.0) ) lateralInteraction.kernelRangeLeft = ceil(kernelRange);
	else lateralInteraction.kernelRangeLeft = floor(((double)size-1.0)/2.0);

	if ( ceil(kernelRange) < ceil(((double)size-1.0)/2.0) ) lateralInteraction.kernelRangeRight = ceil(kernelRange);
	else lateralInteraction.kernelRangeRight = ceil(((double)size-1.0)/2.0);



	lateralInteraction.fullSum = 0.0;

	std::vector<double> gaussNormExc;
	std::vector<double> gaussNormInh;		
	for (int i=-lateralInteraction.kernelRangeLeft;i<=lateralInteraction.kernelRangeRight;i++){
		gaussNormExc.push_back(gauss(i,0.0,lateralInteraction.sigmaExc));
		gaussNormInh.push_back(gauss(i,0.0,lateralInteraction.sigmaInh));		
	}
	//normalize
	double sumGaussExc;
	double sumGaussInh;
	
	for(std::vector<double>::iterator it = gaussNormExc.begin(); it != gaussNormExc.end(); ++it) sumGaussExc += *it;
	for(std::vector<double>::iterator it = gaussNormInh.begin(); it != gaussNormInh.end(); ++it) sumGaussInh += *it;

	for(std::vector<double>::iterator it = gaussNormExc.begin(); it != gaussNormExc.end(); ++it) *it /= sumGaussExc;
	for(std::vector<double>::iterator it = gaussNormInh.begin(); it != gaussNormInh.end(); ++it) *it /= sumGaussInh;	
	
	for (unsigned int i=0;i<gaussNormExc.size();i++){
		lateralInteraction.kernel.push_back(lateralInteraction.amplitudeExc * gaussNormExc[i] - lateralInteraction.amplitudeInh * gaussNormInh[i]);
	}

}

