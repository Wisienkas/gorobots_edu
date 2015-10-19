/*
 *  Created on: Apr 10, 2012
 *      Author: Eduard Grinke
 *
 *      Edited by Dennis Goldschmidt
 *      Apr 27, 2012
 */

#ifndef MODULARNEURALNETWORK_H_
#define MODULARNEURALNETWORK_H_

#include "utils/ann-framework/ann.h"
#include "utils/ann-library/so2cpg.h"
#include "utils/ann-library/pcpg.h"
#include "utils/ann-library/psn.h"
#include "utils/ann-library/vrn.h"
#include "utils/ann-library/pmn.h"
#include "utils/ann-framework/neuron.h"
#include "utils/ann-library/adaptiveso2cpgsynplas.h"
#include <ode_robots/amosiisensormotordefinition.h>
#include <map>
#include <fstream>
using namespace std;

// forward declarations
class SO2CPG;
class PCPG;
class PSN;
class VRN;
class PMN;
class AdaptiveSO2CPGSynPlas;
class NeuralLocomotionControlAdaptiveClimbing;
//class ModularNeuralNetwork;

enum gaits{tripod, tetrapod, wave, irregular};

class ModularNeuralNetwork: public ANN {

public:

	ModularNeuralNetwork(int cpg_option);
	double getCpgOutput(int output);
	double getCpgActivity(int output);
	double getpcpgOutput(int output);
	double getPsnOutput(int output);
	double getVrnLeftOutput(int output);
	double getVrnRightOutput(int output);

	void setInputVrnLeft(int input, double value);
	void setInputVrnRight(int input, double value);
	void setInputPsn(int input, double value);
	void setCpgOutput(int neuron,double value);
	void setInputNeuronInput(int input, double value);
	void setInputMotorNeuronInput(int input, double value);
	double getMotorNeuronActivity(AmosIIMotorNames motor);
	double getMotorNeuronOutput(AmosIIMotorNames motor);
	double Control_input;
	double cpg_bias;
	double getCpgWeight(int neuron1, int neuron2);
	double getCpgBias(int neuron);
	void enableoscillatorsCoupling(bool mMCPGs);
	void disableoscillatorsCoupling();

	void enableContactForce(bool MCPGs);
	void disableContactForce();
	/**********************************/

	bool oscillatorsCouplingIsEnabled;
	vector<double> currentActivity;
	double delta[6][6];
	double cnctCoeffMat[6][6];
	void  changeControlInput(double new_ControlInput);
	void changeGaitpattern(int gaitPattern);
	virtual void step();
	virtual void step(int CPGID, vector< vector <double> > cCPGs,const vector<double> x);
	bool contactForceIsEnabled;
	double oscillatorcouple1;
	double oscillatorcouple0;
	double ContactForceEffect1;
	double ContactForceEffect0;

	//Save files
	//ofstream outFilemlc;

	/*************************************/

private:
	SO2CPG * cpg;
	AdaptiveSO2CPGSynPlas * cpg_s;
	PCPG * pcpg;
	PSN * psn;
	VRN * vrnLeft;
	VRN * vrnRight;
	PMN* pmn;

	vector<Neuron*> inputNeurons;
	map<AmosIIMotorNames,Neuron*> outputNeurons;

};


#endif /* MODULARNEURALNETWORK_H_ */
