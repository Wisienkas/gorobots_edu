/*
 *  Created on: Apr 10, 2012
 *      Author: Eduard Grinke
 *
 *      Edited by Dennis Goldschmidt
 *      Apr 27, 2012
 */

#ifndef MODULARNEURALCONTROL_H_
#define MODULARNEURALCONTROL_H_

#include "utils/ann-framework/ann.h"
#include <ode_robots/amosiisensormotordefinition.h>
#include <map>


// forward declarations
class SO2CPG;
class PCPG;
class PSN;
class VRN;
class PMN;
class AdaptiveSO2CPGSynPlas;
//class ModularNeuralControl;



class ModularNeuralControl: public ANN {

public:

	ModularNeuralControl(int cpg_option);
    double getCpgOutput(int output);
    double getCpgActivity(int output);
    double getpcpgOutput(int output);
    double getPsnOutput(int output);
    double getVrnLeftOutput(int output);
    double getVrnRightOutput(int output);

    void setInputVrnLeft(int input, double value);
    void setInputVrnRight(int input, double value);
    void setInputPsn(int input, double value);

    void setInputNeuronInput(int input, double value);
    void setInputMotorNeuronInput(int input, double value);
    double getMotorNeuronActivity(AmosIIMotorNames motor);
    double getMotorNeuronOutput(AmosIIMotorNames motor);
    double Control_input;
    double cpg_bias;
    double getCpgWeight(int neuron1, int neuron2);
    double getCpgBias(int neuron);



private:
    SO2CPG * cpg;
    AdaptiveSO2CPGSynPlas * cpg_s;
    PCPG * pcpg;
    PSN * psn;
    VRN * vrnLeft;
    VRN * vrnRight;
    PMN* pmn;

    std::vector<Neuron*> inputNeurons;
    std::map<AmosIIMotorNames,Neuron*> outputNeurons;

};


#endif /* MODULARNEURALCONTROL_H_ */
