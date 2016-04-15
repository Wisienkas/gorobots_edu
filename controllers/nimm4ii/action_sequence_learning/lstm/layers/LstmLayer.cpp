#include "LstmLayer.h"
#include <iostream>

LstmLayer::LstmLayer(){
}

LstmLayer::~LstmLayer(){
}

void LstmLayer::reset(){
	for (unsigned int i=0;i<outputs.size();i++) {
		outputs[i]=0.0;
		blocks[i].reset();
	}
}
void LstmLayer::setup(int _numInputs, int _numBlocksInLayer, float _bias){
	numBlocks = _numBlocksInLayer;
	for (int i=0;i<_numBlocksInLayer;i++){
		LstmBlock block;
		block.setup(_numInputs,_numBlocksInLayer,_bias);
		blocks.push_back(block);
		outputs.push_back(0.0);
	}
}

void LstmLayer::setInputs(std::vector<float> _inputs){
	for (int i=0;i<numBlocks;i++) {
		blocks[i].setInputs(_inputs);
		blocks[i].setInternal(outputs);
	}
}

void LstmLayer::step(){
	for (int i=0;i<numBlocks;i++) {
		blocks[i].step();
		outputs[i] = blocks[i].getOutput();
	}
}

std::vector<float> LstmLayer::getOutputs(){
	return outputs;
}

