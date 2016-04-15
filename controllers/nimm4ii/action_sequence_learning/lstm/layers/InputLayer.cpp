#include "InputLayer.h"
#include <iostream>

InputLayer::InputLayer(){
}

InputLayer::~InputLayer(){
}

void InputLayer::setNeurons(int _numNeurons){
	numNeurons = _numNeurons;
	if (neurons.size() > numNeurons) {
		while (neurons.size() != numNeurons) neurons.pop_back();
	} else if (neurons.size() < numNeurons) {
		while (neurons.size() != numNeurons) neurons.push_back(0.0);
	}
}
void InputLayer::setInput(int id, float value){
	neurons[id] = value;
}

float InputLayer::getInput(int id){
	return neurons[id];
}

std::vector<float> InputLayer::getInputVec(){
	std::vector<float> vec;
	for (int i=0; i<numNeurons;i++) vec.push_back(neurons[i]);
	return vec;
}

void InputLayer::setInputs(std::vector<float> values){
	for (int i=0; i<numNeurons;i++) neurons[i] = values[i];
}

