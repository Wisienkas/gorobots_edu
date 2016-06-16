/*
 * TestReceptiveFields.cpp
 *
 *  Created on: Apr 25, 2016
 *      Author: sd
 */

#include "gtest/gtest.h"
#include <vector>
#include <string>

// %%%%
// This needs to be changed when implemented into Gorobots
#include <ReceptiveFields.h>
// %%%%

using namespace std;

// TODO
// Test if The amount of kernelCenters is equal to amount of kernels
// Test if the last kernelCenter is equal to TargetSize
//
// Test the learning algorithm
//

// 1st parameter will be checked if it is equal to the 2nd parameter
//EXPECT_EQ(a,b);

// %%%%
// Not needed in Gorobots
int main(int argc, char* argv[]){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
// %%%%

TEST (KernelCenterTest, CenterSizeEqualTargetSize){
	// Test data: target motor output
	vector<string> targetArr = {0.000300,0.000453,0.000508,0.000529,0.000536,0.000539,0.000540,0.000541,0.000541,0.000541,0.000541,0.000541,0.000540,0.000540,0.000540,0.000540,0.000540,0.000540,1.135422,2.101054,2.161676,2.168449,2.169561,2.169834,2.169184,0.450690,0.041479,0.009084,0.002470,0.000505,-0.000164,-0.000403,-0.000490,-0.000522,-0.000534,-0.000538,-0.000540,-0.000540,-0.000541,-0.000541,-0.000541,-0.000541,-0.000541,-0.000541,-0.000541,-0.000541,-0.000541,-0.000541,-0.000541,-0.000541,-0.000541,-0.000541,-0.000541,-1.135422,-2.103818,-2.170221,-2.181135,-2.184147,-2.185159,-2.185124,-2.181568,-2.176110,-2.172602,-2.171010,-2.169755,-0.453912,-0.041690,-0.009126,-0.002483,-0.000509,0.000163,0.000403,0.000490,0.000522,0.000534,0.000538,0.000540,0.000540,0.000541,0.000541,0.000541,0.000541,0.000541,0.000541,0.000541}

	int minLimit = 60;
	int maxLimit = 120;
	int kernels = 40;
	double kernelWidth = 0.1;
	double learningWidth = 0.4;
	int learningIterations = 50000;
	int targetSize = targetArr.size();

	// Lower Limit, Max Limit, Kernels, Kernel Width, Learning Width, Learning Iteration, Target Size
	ReceptiveFields rf(minLimit, maxLimit, kernels, kernelWidth, learningWidth, learningIterations, targetSize);

	// Generate target trajectory
	for(int i = 0; i < targetSize; i++){
		rf.generateTarget(atof(targetArr[i].c_str()));
	}

	// Generate kernels for each test data
	for(int i = 0; i < targetSize; i++){
		rf.createStep(i);
	}

	// Applies Delta Rule // Learning
	rf.applyDeltaRule();


	// KernelCenters should be in a size of the amount of kernels
	ASSERT_EQ(rf.kernelCenters1d.size(), kernels);

	// The last kernel center should be equal to targetSize (This might need to be changed to maxLimit)
	ASSERT_EQ(rf.kernelCenters1d[rf.kernelCenters1d.size()-1], targetSize);

	// None of the generated gaussian kernels should be above 1
	for (int i = 0; i < kernels; i++){
		ASSERT_LE(rf.gaussianKernels2d[(rf.gaussianKernels2d[0]*i+1+rf.kernelCenters1d[i])], 1);
	}




}

