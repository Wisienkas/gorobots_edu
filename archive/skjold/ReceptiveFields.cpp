/*
 * ReceptiveFields.cpp
 *
 *  Created on: Feb 17, 2016
 *      Author: martin
 */

#include "ReceptiveFields.h"



ReceptiveFields::ReceptiveFields(const double lowerLimit, const double upperLimit, const int numberOfKernels,
		const double kernelWidth, const double learningRate, const int learningIterations, const int targetSize, bool use_gpu)
: lowerLimit(lowerLimit), upperLimit(upperLimit), numberOfKernels(numberOfKernels), kernelWidth(kernelWidth),
  learningRate(learningRate), learningIterations(learningIterations), targetSize(targetSize),
  gaussianKernels2d(numberOfKernels*targetSize+1), kernelCenters1d(numberOfKernels), alfa1d(targetSize),
  weights1d(numberOfKernels), targetPattern1d(targetSize), output1d(targetSize), use_gpu(use_gpu), input1d(targetSize){
	gaussianKernels2d[0] = targetSize;
	kernelCenters1d = linSpace(lowerLimit, upperLimit, numberOfKernels);
	outputsizeToCentersize = linSpace(1, targetSize, numberOfKernels);
	targetcount = 0;
	kernelCreationCounter = 0;
	if(use_gpu){
		if(targetSize < 65535){
			numberOfGpuBlocks = targetSize;
		}else{
			numberOfGpuBlocks = 65535;
		}
		para = new Parallelize();
	}
}

//Creates a value in each kernel for every step
void ReceptiveFields::createStep(double step){
	if(step >= lowerLimit && step <= upperLimit){
		if(use_gpu){
			std::vector<double> kernels = para->createKernels(kernelCenters1d, step, kernelWidth, numberOfKernels, numberOfGpuBlocks);
			for(int i = 0; i < kernels.size(); i++){
				gaussianKernels2d[kernelCreationCounter+i*gaussianKernels2d[0]+1] = kernels[i];
			}

		}else{
			for(int i = 0; i < numberOfKernels; i++){
				gaussianKernels2d[kernelCreationCounter+i*gaussianKernels2d[0]+1] = exp(-(((kernelCenters1d[i]-step)*(kernelCenters1d[i]-step))/(2*kernelWidth*kernelWidth)));
			}
		}
		input1d[kernelCreationCounter] = step;
		kernelCreationCounter++;
	}
}

ReceptiveFields::~ReceptiveFields() {
	gaussianKernels2d.clear();
	kernelCenters1d.clear();
	output1d.clear();
	outputsizeToCentersize.clear();
	alfa1d.clear();
	weights1d.clear();
	targetPattern1d.clear();
}

//Used to generate kernel centers
std::vector<double> ReceptiveFields::linSpace(double start, double stop, double space){
	double addValue = (stop-start)/(space-1);
	double tmp = start;
	std::vector<double> returnVector(space);
	for(int i = 0; i < space; i++){
		if(i == 0){
			returnVector[i] = start;
		}else{
			tmp += addValue;
			returnVector[i] = tmp;
		}
	}
	return returnVector;
}


//LEARNING
//Generate target for supervised learning
void ReceptiveFields::generateTarget(double input){
	targetPattern1d[targetcount]=input;
	targetcount++;
}

//applying simple learing rule
void ReceptiveFields::applyDeltaRule(){
	double value;

	if(use_gpu){
		output1d = para->applyDeltaRule(learningRate, outputsizeToCentersize, targetPattern1d, weights1d, gaussianKernels2d, learningIterations, numberOfGpuBlocks);
	}else{
		for(int k = 0; k < learningIterations; k++){
			for(int i = 0; i < targetSize; i++){
				value = 0;
				for(int j = 0; j < numberOfKernels; j++){
					value += gaussianKernels2d[(j*gaussianKernels2d[0]+1+i)]*weights1d[j];
				}
				output1d[i]=value;
			}
			for(int l = 0; l < numberOfKernels; l++){
				weights1d[l] += learningRate*((double)targetPattern1d[round(outputsizeToCentersize[l])]-(double)output1d[round(outputsizeToCentersize[l])]);
			}
		}
	}
}

std::vector<double> ReceptiveFields::getOutputs(){
	return output1d;
}

double ReceptiveFields::getSpecifiedOutput(double input){
	for(int i = 0; i < targetSize; i++){
		if(input == input1d[i]){
			return output1d[i];
		}
	}
	return 0.0;
}

//TODO: debugging
void ReceptiveFields::toString(std::string filename){
	for (int i = 0; i < targetSize; i++){
		std::cout << i << "  " << output1d[i] << "  " << targetPattern1d[i] << std::endl;
	}
	std::ofstream file;
	file.open(filename.c_str());
	for (int i = 0; i < targetSize; i++){
		file << i << "," << output1d[i] << "," << targetPattern1d[i] << "\n";
	}

}
