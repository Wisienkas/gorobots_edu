#include "SoftMaxLayer.h"
#include <iostream>
#include <math.h> // exp

float SoftMaxLayer::sumVecWeight(std::vector<float> inputVec, std::vector<float> weights){
	float sum = 0.0;
	if (inputVec.size() != weights.size()) {
		std::cout<<"Input Vector and Weights Vector sizes dont match"<<std::endl;
		return -1;
	}
	for(std::vector<float>::size_type i = 0; i != inputVec.size(); i++) {
    	sum += inputVec[i]*weights[i];
    }
	return sum;
}

SoftMaxLayer::SoftMaxLayer(){

}

SoftMaxLayer::~SoftMaxLayer(){
}

void SoftMaxLayer::setNeurons(int _numNeurons, int _numInputs, float _bias){
	numNeurons = _numNeurons;
	numInputs = _numInputs;
	bias = _bias;
	for (int i=0;i<numInputs;i++) inputs.push_back(0.0);
	if (neurons.size() > numNeurons) {
		while (neurons.size() != numNeurons) neurons.pop_back();
	} else if (neurons.size() < numNeurons) {
		while (neurons.size() != numNeurons) {
			Neuron newNeuron;
			for (int i=0;i<numInputs;i++) newNeuron.weights.push_back(0.0);
			neurons.push_back(newNeuron);
		}
	}
}


void SoftMaxLayer::setInputs(std::vector<float> values){
	for (int i=0; i<numInputs;i++) inputs[i] = values[i];
}

void SoftMaxLayer::setWeights(std::vector<float> values){
/*TODO: check if this weight allocation fits*/
	for (int i=0; i<numNeurons;i++){
		for (int j=0; j<numInputs;j++){
			neurons[i].weights[j] = values[(i*numInputs)+j];
		}
	}
}

void SoftMaxLayer::step(){
	for (int i=0; i<numNeurons;i++){
		neurons[i].activation = sumVecWeight(inputs,neurons[i].weights);
		neurons[i].activation += bias*neurons[i].biasWeight;
	}
	float sum = 0.0;
	for (int i=0; i<numNeurons;i++) sum += exp(neurons[i].activation);
	for (int i=0; i<numNeurons;i++) neurons[i].output = exp(neurons[i].activation) / sum;
	int maxNeuron = 0;
	float maxValue = -2;
	for (int i=0; i<numNeurons;i++) {
		if (neurons[i].output >= maxValue) {
			maxValue = neurons[i].output;
			maxNeuron = i;
		}
	}
	output = maxNeuron;
}

std::vector<float> SoftMaxLayer::getOutputVec(){
	std::vector<float> out;
	for (int i=0; i<numNeurons;i++) out.push_back(neurons[i].output);
	return out;
}

