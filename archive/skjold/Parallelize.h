/*
 * Parallelize.h
 *
 *  Created on: Apr 24, 2016
 *      Author: martin
 */

#ifndef PARALLELIZE_H_
#define PARALLELIZE_H_

#include <cuda_runtime_api.h>
#include <cuda.h>
#include <math.h>
#include <device_launch_parameters.h>
#include <vector>
#include <iostream>
#include <stdio.h>

class Parallelize {
	public:
		Parallelize();
		~Parallelize();
		//GENERAL
		//Call function to create "step" in kernel
		std::vector<double> createKernels(std::vector<double> centers, double step, double width, int size, int numberOfBlocks);
		//LEARNING
		//function for simple supervised learning
		std::vector<double> applyDeltaRule(double learningRate, std::vector<double> centers, std::vector<double> target, std::vector<double> weights, std::vector<double> kernels, int learningIterations, int numberOfBlocks);
	private:
		//prepare functions
		void d_createKernels(double *centers, double step, double width, double *kernelsArr, int centerSize, int kernelSize, int numberOfBlocks);
};
#endif /* PARALLELIZE_H_ */

