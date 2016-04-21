#include "LstmBlock.h"
#include <iostream>
#include "../activation_functions/activation_functions.cpp"

    typedef activation_functions::Logistic gate_act_fn_t;
    typedef activation_functions::Tanh     cell_input_act_fn_t;
    typedef activation_functions::Tanh     cell_output_act_fn_t;

float LstmBlock::sumVecWeight(std::vector<float> inputVec, std::vector<float> weights){
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

LstmBlock::LstmBlock(){
	c=0.0;

}

LstmBlock::~LstmBlock(){
}

void LstmBlock::reset(){
	c=0.0;
}

void LstmBlock::setup(int _numInputs, int _numBlocksInLayer, float _bias){

	bias = _bias;
	numInputs = _numInputs;
	numBlocksInLayer = _numBlocksInLayer;
	for (int i=0;i<numInputs;i++){
		inputsPreceding.push_back(0.0);
		precedingToNet.push_back(0.0);
		precedingToInput.push_back(0.0);
		precedingToForget.push_back(0.0);
		precedingToOutput.push_back(0.0);		
	}

	for (int i=0;i<numBlocksInLayer;i++){
		inputsInternal.push_back(0.0);
		internalToNet.push_back(0.0);
		internalToInput.push_back(0.0);
		internalToForget.push_back(0.0);
		internalToOutput.push_back(0.0);
	}
	
	biasToNet = 0.0;
	biasToInput = 0.0;
	biasToForget = 0.0;
	biasToOutput = 0.0;

	peepCellToInput = 0.0;
	peepCellToForget = 0.0;
	peepCellToOutput = 0.0;
}

void LstmBlock::setInputs(std::vector<float> inputs){
	for (int i=0;i<numInputs;i++) inputsPreceding[i] = inputs[i];
}
void LstmBlock::setInternal(std::vector<float> internal){
	for (int i=0;i<numBlocksInLayer;i++) inputsInternal[i] = internal[i];
}

void LstmBlock::step(){
	// calculate activation sums
	float aIg = sumVecWeight(inputsPreceding,precedingToInput) 
				+ sumVecWeight(inputsInternal,internalToInput) 
				+ c*peepCellToInput
				+ bias*biasToInput;
	float aFg = sumVecWeight(inputsPreceding,precedingToForget) 
				+ sumVecWeight(inputsInternal,internalToForget) 
				+ c*peepCellToForget
				+ bias*biasToForget;

	float aNi = sumVecWeight(inputsPreceding,precedingToNet) 
				+ sumVecWeight(inputsInternal,internalToNet) 
				+ bias*biasToNet;
	// apply activation function
	z = cell_input_act_fn_t::fn(aNi);
	i = gate_act_fn_t      ::fn(aIg);
	f = gate_act_fn_t      ::fn(aFg);	

	// calculate new cell state
	c = (i*z) + (f*c);
	// calculate output Gate Activation
	float aOg = sumVecWeight(inputsPreceding,precedingToOutput) 
			+ sumVecWeight(inputsInternal,internalToOutput) 
			+ c*peepCellToOutput
			+ bias*biasToOutput;

	o = gate_act_fn_t::fn(aOg);
	
	// calculate block output
	y = cell_output_act_fn_t::fn(c) * o;
	//std::cout<<y<<std::endl;

}

float LstmBlock::getOutput(){
	return y;
}
