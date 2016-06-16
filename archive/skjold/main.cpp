#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include "ReceptiveFields.h"
#include <gtest/gtest.h>
#include <time.h>

using namespace std;
/*
TEST(Re){

}
*/

ReceptiveFields createReceptiveFields(string inputName, int kernels, double width, double learningRate, int iterations, bool gpu, int &size, double addValue){
	string token;
	vector<string> splitStrings;
	ifstream filein;
	filein.open(inputName.c_str());
	while(getline(filein, token, ',')){
		splitStrings.push_back(token);
	}
	ReceptiveFields RF(1, splitStrings.size()*addValue, kernels, width, learningRate, iterations, splitStrings.size(), gpu);
	size = splitStrings.size();
	cout << size << endl;
	for(int i = 0; i < splitStrings.size(); i++){
		RF.generateTarget(atof(splitStrings[i].c_str()));
	}
	return RF;

}

int main(){
	int iterations = 5000;
	bool gpu = false;
	int size_lefthip = 0;
	int size_righthip = 0;
	int size_leftknee = 0;
	int size_rightknee = 0;
	int kernels = 500;
	int tmp_lefthip = 1;
	int tmp_righthip = 1;
	int tmp_leftknee = 1;
	int tmp_rightknee = 1;
	double addValue = 1;

	double width = 2;
	ReceptiveFields lefthip = createReceptiveFields("runbot_lefthip_cycle_interp.csv", kernels, width, 0.4, iterations, gpu, size_lefthip, addValue);
	//ReceptiveFields righthip = createReceptiveFields("runbot_righthip_cycle_interp.csv", kernels, width, 0.4, iterations, gpu, size_righthip, addValue);
	//ReceptiveFields leftknee = createReceptiveFields("runbot_leftknee_cycle_interp.csv", kernels, width, 0.4, iterations, gpu, size_leftknee, addValue);
	//ReceptiveFields rightknee = createReceptiveFields("runbot_rightknee_cycle_interp.csv", kernels, width, 0.4, iterations, gpu, size_rightknee, addValue);

	clock_t tstart = clock();

	for(int i = 0; i < size_lefthip; i++){
		lefthip.createStep(tmp_lefthip);
		tmp_lefthip+=addValue;
	}
/*
	for(int i = 0; i < size_righthip; i++){
		righthip.createStep(tmp_righthip);
		tmp_righthip+=addValue;
	}
	for(int i = 0; i < size_leftknee; i++){
		leftknee.createStep(tmp_leftknee);
		tmp_leftknee+=addValue;
	}
	for(int i = 0; i < size_rightknee; i++){
		rightknee.createStep(tmp_rightknee);
		tmp_rightknee+=addValue;
	}
*/
	lefthip.applyDeltaRule();
	//righthip.applyDeltaRule();
	//leftknee.applyDeltaRule();
	//rightknee.applyDeltaRule();

	double time = ((double)(clock()-tstart)/CLOCKS_PER_SEC);

	//lefthip.toString("interp_lefthip_10times.csv");
	//righthip.toString("interp_righthip_10times.csv");
	//leftknee.toString("interp_leftknee_10times.csv");
	//rightknee.toString("interp_rightknee_10times.csv");
	std::ofstream file;
	file.open("repeatingPattern.csv");
	vector<double> output1d = lefthip.getOutputs();
	for(int j = 0; j < 10; j++){
		for (int i = 0; i < size_lefthip; i++){
			file << i << "," << output1d[i] << "\n";
		}
	}
	std::cout << "Time: " << time << std::endl;
	return 0;
}


