#include "FeedForwardLayer.h"
#include <iostream>
#include "../activation_functions/activation_functions.cpp"

float FeedForwardLayer::sumVecWeight(std::vector<float> inputVec, std::vector<float> weights){
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

FeedForwardLayer::FeedForwardLayer(){

}

FeedForwardLayer::~FeedForwardLayer(){
}

void FeedForwardLayer::setNeurons(int _numNeurons, int _numInputs, float _bias){
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
			newNeuron.biasWeight = 0.0;
			neurons.push_back(newNeuron);
		}
	}
}


void FeedForwardLayer::setInputs(std::vector<float> values){
	for (int i=0; i<numInputs;i++) inputs[i] = values[i];
}

void FeedForwardLayer::setWeights(std::vector<float> values){
/*TODO: check if this weight allocation fits*/
	for (int i=0; i<numNeurons;i++){
		for (int j=0; j<numInputs;j++){
			neurons[i].weights[j] = values[(i*numInputs)+j];
		}
	}
}

void FeedForwardLayer::step(){
	for (int i=0; i<numNeurons;i++){
		neurons[i].activation = sumVecWeight(inputs,neurons[i].weights);
		neurons[i].activation += bias*neurons[i].biasWeight;
	}

/* TODO: this so far is only for 1 output*/	
	output = activation_functions::Logistic::fn(neurons[0].activation);
}


