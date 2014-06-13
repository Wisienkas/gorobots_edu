/*
 *  Created on: Apr 10, 2012
 *      Author: Eduard Grinke
 *
 *      Edited by Dennis Goldschmidt
 *      Apr 27, 2012
 */

#ifndef MODULARNEURALCONTROL_H_
#define MODULARNEURALCONTROL_H_

#include "ann-framework/ann.h"
#include <ode_robots/amosiisensormotordefinition.h>
#include <map>
#include <queue>


// forward declarations
class NeuralLocomotionControlAdaptiveClimbing;
class SO2CPG;
class PCPG;
class PSN;
class VRN;
class PMN;
class AdaptiveSO2CPGSynPlas;
class CPGPhaseresetInhibition;

//class ModularNeuralControl;



class ModularNeuralControl: public ANN {

public:
    typedef double sensor;
    int T;
    double phase_phi;
	ModularNeuralControl(int cpg_option);

	 virtual void step(std::vector<double> cpg_output,int CPGID,NeuralLocomotionControlAdaptiveClimbing * NLCAC[6],double NeighbourCpgActivity_0,double NeighbourCpgActivity_1);
	 virtual void calculate(const std::vector<double> x_,AmosIISensorNames sensorname,double* y);
	 virtual void step();
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
    void changeGaitpattern(bool CoupledOscillators);
    void changeGaitpatternRingCoupling(bool RingCoupledOscillators);
    void changeGaitpattern(int gaitPattern);
    void changeGaitpatternRingCoupling(int gaitPattern);
   double max(double a,double b);
   int getSign(double a);
   // void checkPhaseReset();
    double Control_input;
    double cpg_bias;

    /*this variable denotes  whether phase reset and inhibition mechanisms will be used or not
     * when phaseResetInhibition is True, this means phase reset and inhibition mechanisms will be deployed
     *
     * @author Subhi Shaker Barikhan
     */
    bool phaseResetInhibitionIsEnabled;

    /*this variable indicates whether fully connected network will be used or not
     * when oscillatorsCoupling is True, this means each of six CPGs is connected to each other
     *
     * @author Subhi Shaker Barikhan
     */
    bool oscillatorsCouplingIsEnabled; //
    /*this variable indicates whether fully connected network is used or not
     * when oscillatorsCoupling is True, this means CPGs connection model is a ring
     *
     * @author Subhi Shaker Barikhan
     */
    bool oscillatorsRingCouplingIsEnabled;

    bool footSensorenable;
    double ContactForce;
    double ContactForceEffect0;
    double ContactForceEffect1;


   bool contactForceIsEnabled;
    std::vector<double> footbias;
    std::vector<double> predictActivity;
    std::vector<double> predictOutput;
    std::vector<double> currentActivity;
    std::vector<double> currentOutput;
    double getCpgWeight(int neuron1, int neuron2);
    double getCpgBias(int neuron);
    void changeControlInput(double new_ControlInput);
    bool touchF;
    double delta[6][6];
    double cnctCoeffMat[6][6];
    double oscillatorcouple1;
    double oscillatorcouple0;




private:
    SO2CPG * cpg;
    AdaptiveSO2CPGSynPlas * cpg_s;
    PCPG * pcpg;
    PSN * psn;
    VRN * vrnLeft;
    VRN * vrnRight;
    PMN* pmn;
    CPGPhaseresetInhibition* cpgPhaseResetInhibition;
    std::vector<Neuron*> inputNeurons;
    std::map<AmosIIMotorNames,Neuron*> outputNeurons;
   int t;
};


#endif /* MODULARNEURALCONTROL_H_ */
