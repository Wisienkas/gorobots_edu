/*
 * parallelize.cpp
 *
 *  Created on: Apr 24, 2016
 *      Author: martin
 */

#include "Parallelize.h"

Parallelize::Parallelize() {
	cudaDeviceReset();
}

Parallelize::~Parallelize() {
	// TODO Auto-generated destructor stub
}

//GENEREL KERNEL CREATION
//kernel function runs on gpu
__global__ void d_createKernel(double *center, double step, double width, double *kernel, int size){

	int i = threadIdx.x + blockIdx.x * blockDim.x;
		//printf("kernel, threadI %d, blockid %d, blockdim %d\n", threadIdx.x, blockIdx.x, blockDim.x);
	if(i < size)
		kernel[i] = exp(-(((center[i]-step)*(center[i]-step))/(2*width*width)));
}

//prepare function(runs on CPU prepares GPU)
void Parallelize::d_createKernels(double *centers, double step, double width, double *kernelsArr, int centerSize, int kernelSize, int numberOfBlocks){
	//std::cout << "step" << step << std::endl;
	//std::cout << "width" << width << std::endl;
	double *d_kernelCenter,*d_kernel;


	  cudaMalloc((void**)&d_kernelCenter, centerSize*sizeof(double));
	  cudaMalloc((void**)&d_kernel, centerSize*sizeof(double));

	  for(int i = 0; i < centerSize; i++){
		  kernelsArr[i] = 0;
	  }

	  cudaMemcpy(d_kernelCenter, centers, centerSize*sizeof(double), cudaMemcpyHostToDevice);
	  cudaMemcpy(d_kernel, kernelsArr, centerSize*sizeof(double), cudaMemcpyHostToDevice);

	  //function call
	  d_createKernel<<<centerSize, 1>>>(d_kernelCenter, step, width, d_kernel, centerSize);
	  //d_createKernel<<<1, 20>>>(d_kernelCenter, step, width, d_kernel, centerSize);
	  cudaDeviceSynchronize();

	  cudaMemcpy(kernelsArr, d_kernel, centerSize*sizeof(double), cudaMemcpyDeviceToHost);

	  cudaFree(d_kernelCenter);
	  cudaFree(d_kernel);

}

//Public method to be called from outside
std::vector<double> Parallelize::createKernels(std::vector<double> centers, double step, double width, int size, int numberOfBlocks){
	double centersArr[centers.size()];
	for(int i = 0; i < centers.size(); i++){
			centersArr[i] = centers[i];
		}
	//std::copy(centers.begin(), centers.end(), centersArr);
	double kernelsArr[size];
	d_createKernels(centersArr, step, width, kernelsArr, centers.size(), size, numberOfBlocks);
	std::vector<double> returnVector;
	for(int i = 0; i < size; i++){
		returnVector.push_back(kernelsArr[i]);
	}

	return returnVector;
}

//SIMPLE LEARNING

//kernel functions

__global__ void d_calcOutput(double *weights, double *kernels, double *output, int centerSize, int targetSize){
	int i = threadIdx.x + blockIdx.x * blockDim.x;
			//printf("output, threadI %d, blockid %d, blockdim %d\n", threadIdx.x, blockIdx.x, blockDim.x);
	if(i < targetSize){
		double value = 0;
		double tmpvalue = 0;
		for(int j = 0; j < centerSize; j++){
			tmpvalue = value+(kernels[(j*targetSize+1+i)]*weights[j]);
			value = tmpvalue;
		}
		output[i]=value;
	}
}

__global__ void d_updateWeights(double learningRate, double *centers, double *target, double *output, double *weights, int size){
	int i = threadIdx.x + blockIdx.x * blockDim.x;
		//printf("weight, threadI %d, blockid %d, blockdim %d, size %d \n", threadIdx.x, blockIdx.x, blockDim.x, size);
	double d = 0;
	if(i < size){
		d = weights[i]+(learningRate*(target[(int)(centers[i]+0.5f)]-output[(int)(centers[i]+0.5f)]));
		weights[i] = d;
	}
}


//prepare function
void d_deltarule(double learningRate, double *centers, double *target, double *output, double *weights, double *kernels, int centerSize, int targetSize, int kernelSize, int iterations, int numberOfBlocks){
	double *d_centers, *d_target, *d_output, *d_weights, *d_kernels;

	//std::cout << learningRate << "     " << targetSize << "     " << centerSize << "     " << kernelSize<< "     " << iterations << std::endl;

	cudaMalloc((void**)&d_centers, centerSize*sizeof(double));
	cudaMalloc((void**)&d_target, targetSize*sizeof(double));
	cudaMalloc((void**)&d_output, targetSize*sizeof(double));
	cudaMalloc((void**)&d_weights, centerSize*sizeof(double));
	cudaMalloc((void**)&d_kernels, kernelSize*sizeof(double));

	cudaMemcpy(d_centers, centers, centerSize*sizeof(double), cudaMemcpyHostToDevice);
	cudaMemcpy(d_target, target, targetSize*sizeof(double), cudaMemcpyHostToDevice);
	cudaMemcpy(d_output, output, targetSize*sizeof(double), cudaMemcpyHostToDevice);
	cudaMemcpy(d_weights, weights, centerSize*sizeof(double), cudaMemcpyHostToDevice);
	cudaMemcpy(d_kernels, kernels, kernelSize*sizeof(double), cudaMemcpyHostToDevice);
	//std::cout << "before iterations:" << iterations << std::endl;
	for(int i = 0; i < iterations; i++){
		//std::cout << "in iteration: " << i << std::endl;
		d_calcOutput<<<numberOfBlocks, 1>>>(d_weights, d_kernels, d_output, centerSize, targetSize);
		//d_calcOutput<<<1, 85>>>(d_weights, d_kernels, d_output, centerSize, targetSize);
		cudaDeviceSynchronize();

		if(i != iterations-1){

			d_updateWeights<<<centerSize, 1>>>(learningRate, d_centers, d_target, d_output, d_weights, centerSize);
			//d_updateWeights<<<1, 20>>>(learningRate, d_centers, d_target, d_output, d_weights, centerSize);
			cudaDeviceSynchronize();

		}
	}
	cudaMemcpy(output, d_output, targetSize*sizeof(double), cudaMemcpyDeviceToHost);

	cudaFree(d_centers);
	cudaFree(d_target);
	cudaFree(d_output);
	cudaFree(d_weights);
	cudaFree(d_kernels);

}

//public function to apply learning
std::vector<double> Parallelize::applyDeltaRule(double learningRate, std::vector<double> centers, std::vector<double> target, std::vector<double> weights, std::vector<double> kernels, int learningIterations, int numberOfBlocks){
	double centersArr[centers.size()];
	for(int i = 0; i < centers.size(); i++){
		centersArr[i] = centers[i];
	}

	double targetArr[target.size()];
	for(int i = 0; i < target.size(); i++){
			targetArr[i] = target[i];
		}

	double weightsArr[weights.size()];
	for(int i = 0; i < weights.size(); i++){
			weightsArr[i] = weights[i];
		}

	double kernelsArr[kernels.size()];
	for(int i = 0; i < kernels.size(); i++){
			kernelsArr[i] = kernels[i];
		}

	double outputArr[target.size()];

	for(int i = 0; i < target.size(); i++){
		outputArr[i] = 0;
	}
	d_deltarule(learningRate, centersArr, targetArr, outputArr, weightsArr, kernelsArr, centers.size(), target.size(), kernels.size(), learningIterations, numberOfBlocks);
	std::vector<double> finalOutput(outputArr, outputArr+target.size());

	return finalOutput;
}
