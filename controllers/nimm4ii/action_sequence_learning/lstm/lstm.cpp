#include "layers/InputLayer.h"
#include "lstm.h"

#include <iostream>
#include <fstream>
#include <cstdlib>

LSTM::LSTM(){
}

LSTM::~LSTM(){
}

void LSTM::setup(int _inputs, int _lstms, int _outputs, float _bias){
	inputs = _inputs;
	lstms = _lstms;
	outputs = _outputs;
	inputLayer.setNeurons(_inputs);
	lstmLayer.setup(_inputs,_lstms, _bias);
	softMaxLayer.setNeurons(_outputs,_lstms,_bias);
	feedForwardLayer.setNeurons(1, _lstms, _bias);
}

void LSTM::reset(){
	lstmLayer.reset();
}

void LSTM::setInput(std::vector<float> inVec){
	inputLayer.setInputs(inVec);
}

void LSTM::step(){
	lstmLayer.setInputs(inputLayer.getInputVec());
	lstmLayer.step();
	if (outputs ==1){
		feedForwardLayer.setInputs(lstmLayer.getOutputs());
		feedForwardLayer.step();
	} else {
		softMaxLayer.setInputs(lstmLayer.getOutputs());
		softMaxLayer.step();
	}
}

std::vector<float> LSTM::getOutput(){
	return softMaxLayer.getOutputVec();
}

int LSTM::getState(){
	return softMaxLayer.getOutput();
}

float LSTM::getBinary(){
	return feedForwardLayer.getOutput();
}

void LSTM::loadWeights(std::string filename){
	// load file and store all weights(lines with . in it) to float vector
	std::ifstream infile(filename.c_str());
	std::string line;
	std::vector<float> weights;
	while (std::getline(infile, line))
	{
		//std::cout <<line.c_str() << '\n';
		if (line.find(".") != std::string::npos) {
			//std::cout <<std::atof(line.c_str()) << '\n';
			weights.push_back(std::atof(line.c_str()));
		}
	}

	/******************************************************************************************//**
	/*
	/* assign weights to LSTM Layer according to: 
	/*
	/******************************************************************************************//**
	 * Represents a fully connected layer which uses LSTM cells with forget gates, peephole
	 * connections and one cell per block
	 *
	 * weights; with P = precedingLayer().size() and L = size():
	 *    ~ weights from preceding layer:
	 *        - [0 .. PL-1]:    net input
	 *        - [PL .. 2PL-1]:  input gate
	 *        - [2PL .. 3PL-1]: forget gate
	 *        - [3PL .. 4PL-1]: output gate
	 *    ~ bias weights:
	 *        - [4PL + 0  .. 4PL + L-1]:  net input
	 *        - [4PL + L  .. 4PL + 2L-1]: input gate
	 *        - [4PL + 2L .. 4PL + 3L-1]: forget gate
	 *        - [4PL + 3L .. 4PL + 4L-1]: output gate
	 *    ~ internal weights (from other cells in the same layer):
	 *        - [4(P+1)L + 0   .. 4(P+1)L + LL-1]:  net input
	 *        - [4(P+1)L + LL  .. 4(P+1)L + 2LL-1]: input gate
	 *        - [4(P+1)L + 2LL .. 4(P+1)L + 3LL-1]: forget gate
	 *        - [4(P+1)L + 3LL .. 4(P+1)L + 4LL-1]: output gate
	 *    ~ peephole weights (from cell state to all gates in the same cell):
	 *        - [4(P+1+L)L + 0   .. 4(P+1+L)L + L-1]:  input gate
	 *        - [4(P+1+L)L + LL  .. 4(P+1+L)L + 2L-1]: forget gate
	 *        - [4(P+1+L)L + 2LL .. 4(P+1+L)L + 3L-1]: output gate
	 *
	 * @param TDevice The computation device (Cpu or Gpu)
	 *********************************************************************************************/
	int weightID = 0;
	// preceding layer
	for (int l=0;l<lstms;l++){
		for (int p=0;p<inputs;p++){
			lstmLayer.blocks[l].precedingToNet[p] = weights[weightID];
			weightID++;
		}
	}
	for (int l=0;l<lstms;l++){
		for (int p=0;p<inputs;p++){
			lstmLayer.blocks[l].precedingToInput[p] = weights[weightID];
			weightID++;
		}
	}
	for (int l=0;l<lstms;l++){
		for (int p=0;p<inputs;p++){
			lstmLayer.blocks[l].precedingToForget[p] = weights[weightID];
			weightID++;
		}
	}
	for (int l=0;l<lstms;l++){
		for (int p=0;p<inputs;p++){
			lstmLayer.blocks[l].precedingToOutput[p] = weights[weightID];
			weightID++;
		}
	}
	// biases
	for (int l=0;l<lstms;l++){
	   	lstmLayer.blocks[l].biasToNet = weights[weightID];
	   	weightID++;
	}
	for (int l=0;l<lstms;l++){
	   	lstmLayer.blocks[l].biasToInput = weights[weightID];
	   	weightID++;
	}
	for (int l=0;l<lstms;l++){
	   	lstmLayer.blocks[l].biasToForget = weights[weightID];
	   	weightID++;
	}
	for (int l=0;l<lstms;l++){
	   	lstmLayer.blocks[l].biasToOutput = weights[weightID];
	   	weightID++;
	}
	// internal
	for (int l=0;l<lstms;l++){
		for (int m=0;m<lstms;m++){
			lstmLayer.blocks[l].internalToNet[m] = weights[weightID];
			weightID++;
		}
	}
	for (int l=0;l<lstms;l++){
		for (int m=0;m<lstms;m++){
			lstmLayer.blocks[l].internalToInput[m] = weights[weightID];
			weightID++;
		}
	}
	for (int l=0;l<lstms;l++){
		for (int m=0;m<lstms;m++){
			lstmLayer.blocks[l].internalToForget[m] = weights[weightID];
			weightID++;
		}
	}
	for (int l=0;l<lstms;l++){
		for (int m=0;m<lstms;m++){
			lstmLayer.blocks[l].internalToOutput[m] = weights[weightID];
			weightID++;
		}
	}
	// peephole
	for (int l=0;l<lstms;l++){
		lstmLayer.blocks[l].peepCellToInput = weights[weightID];
		weightID++;
	}
	for (int l=0;l<lstms;l++){
		lstmLayer.blocks[l].peepCellToForget = weights[weightID];
		weightID++;
	}
	for (int l=0;l<lstms;l++){
		lstmLayer.blocks[l].peepCellToOutput = weights[weightID];
		weightID++;
	}
	if (outputs ==1){
		/******************************************************************************************//**
		/*
		/* assign weights to FeedForward Layer - 
		/* which is really only one neuron with 1 input per LSTM block and bias
		/*
		/******************************************************************************************/
		for (int l=0;l<lstms;l++){
			feedForwardLayer.neurons[0].weights[l] = weights[weightID];
			weightID++;
		}
		feedForwardLayer.neurons[0].biasWeight = weights[weightID];
		weightID++;
	} else {
		/******************************************************************************************//**
		/*
		/* assign weights to Softmax Layer
		/*
		/******************************************************************************************/
		for (int o=0;o<outputs;o++){
			for (int l=0;l<lstms;l++){
				softMaxLayer.neurons[o].weights[l] = weights[weightID];
				weightID++;
			}
		}
		for (int o=0;o<outputs;o++){
			softMaxLayer.neurons[o].biasWeight = weights[weightID];
			weightID++;
		}
	}
		
//	std::cout<<weightID<<" of "<<weights.size()<<std::endl;
//	std::cout<<"Weights loaded"<<std::endl;
}
