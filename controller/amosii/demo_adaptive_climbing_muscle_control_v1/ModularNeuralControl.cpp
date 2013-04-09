#include "ModularNeuralControl.h"
#include "ann-library/so2cpg.h"
#include "ann-library/pcpg.h"
#include "ann-library/psn.h"
#include "ann-library/vrn.h"
#include "ann-library/pmn.h"
#include "ann-framework/neuron.h"
#include "ann-library/adaptiveso2cpgsynplas.h"




ModularNeuralControl::ModularNeuralControl(int cpg_option){

	 /*******************************************************************************
	 *  MODULE 0 IO'S for modularneuralcontrol
	 *******************************************************************************/

	//IO'S for modularneuralcontrol
	////create 5 input neurons for modularneuralcontrol

	for(int i=0;i<5;i++)
	{
		inputNeurons.push_back(addNeuron());
		inputNeurons.at(i)->setTransferFunction(identityFunction());
	}

	//create 19 output neurons for modularneuralcontrol
	for(int i=0;i<19;i++)
	{
		outputNeurons[AmosIIMotorNames(i)]=addNeuron();
	}
	//end
	 /*******************************************************************************
	 *  MODULE 1 CPG
	 *******************************************************************************/

	switch(cpg_option) {
    case 1: //Kohs CPG

      cpg = new SO2CPG();
      cpg_bias = 0.0;
      //From 0.02-1.5
      //Control_input = 0.02;// slow Wave
     // Control_input = 0.03;// slow Wave OK USED original one used for only climbing
      //Control_input = 0.04;
      Control_input = 0.05;//slow stable Tetrapod OK USED
      //Control_input = 0.14; //terapod OK USED
      //Control_input = 0.18; //Tripod fast OK USED
      //Control_input = 0.34; //Faster than tripod

      //destabilize cpg to oscillate
      cpg->setOutput(0, 0.1);
      cpg->setOutput(1, 0.1);
      cpg->setActivity(0, 0.1);
      cpg->setActivity(1, 0.1);

      //set cpg weights to override timos phi weight matrix
      cpg->setWeight(0, 0, 1.4);
      cpg->setWeight(0, 1, 0.18 + Control_input);
      cpg->setWeight(1, 0, -0.18 - Control_input);
      cpg->setWeight(1, 1, 1.4);

      //set bias
      cpg->setBias(0, cpg_bias);
      cpg->setBias(1, cpg_bias);

      break;

    case 2: //Timos CPG

      cpg_s = new AdaptiveSO2CPGSynPlas(addNeuron());
      cpg_s->setPhi(0.1);
      cpg_s->setEpsilon ( 0.01 );
      //cpg->setEpsilon(0.01/5.0);
      //cpg->setEpsilon(0.01/3.0);
      cpg_s->setGamma   ( 1.0 );
      cpg_s->setBeta    ( 0.0 );
      cpg_s->setMu      ( 1.0 );
      cpg_s->setBetaDynamics   ( -1.0, 0.010, 0.00);
      cpg_s->setGammaDynamics  ( -1.0, 0.010, 1.00);
      cpg_s->setEpsilonDynamics(  1.0, 0.010, 0.01);
      //cpg->setEpsilonDynamics( 1.0/25.0, 0.010, 0.01/5.0);
      cpg_s->setOutput(0,0.2);

      cpg = cpg_s;

      break;

  };

	//for updating the subnets (to do the time step)
	addSubnet(cpg);
	 /*******************************************************************************
	 *  MODULE 2 PCPG
	 *******************************************************************************/
	pcpg     = new PCPG();

	//cpg to pcpg
	w(pcpg->getNeuron(0),cpg->getNeuron(0),1);
	w(pcpg->getNeuron(1),cpg->getNeuron(1),1);

	addSubnet(pcpg);

	 /*******************************************************************************
	 *  MODULE 3 PSN
	 *******************************************************************************/
	psn		 = new PSN();

	w(psn->getNeuron(0), inputNeurons[2], -1);
	w(psn->getNeuron(1), inputNeurons[2], 1);


	// pcpg to PSN
	w(psn->getNeuron(2), pcpg->getNeuron(0), 0.5);
	w(psn->getNeuron(3), pcpg->getNeuron(1), 0.5);
	w(psn->getNeuron(4), pcpg->getNeuron(1), 0.5);
	w(psn->getNeuron(5), pcpg->getNeuron(0), 0.5);




	addSubnet(psn);

	 /*******************************************************************************
	 *  MODULE 4 VRN
	 *******************************************************************************/
	vrnLeft  = new VRN();
	vrnRight = new VRN();

	// create connections from PSN to VRN
	w(vrnLeft->getNeuron(0), psn->getNeuron(11), 1.75);
	w(vrnRight->getNeuron(0), psn->getNeuron(11), 1.75);

	w(vrnLeft ->getNeuron(1), inputNeurons[3], 5);
	w(vrnRight->getNeuron(1), inputNeurons[4], 5);

	addSubnet(vrnLeft);
	addSubnet(vrnRight);



	/*******************************************************************************
	 *  MODULE 5 Pre Motor Neurons PMN
	*******************************************************************************/

	pmn     = new PMN();


	//vrn to pmn

  //TL TR

  w(pmn->getNeuron(TL0_m), vrnLeft->getNeuron(6), -2.5);
  w(pmn->getNeuron(TL0_m), inputNeurons[0], -10);

  w(pmn->getNeuron(TL1_m), vrnLeft->getNeuron(6), -2.5);
  w(pmn->getNeuron(TL1_m), inputNeurons[0], -10);

  w(pmn->getNeuron(TL2_m), vrnLeft->getNeuron(6), -2.5);
  w(pmn->getNeuron(TL2_m), inputNeurons[0], -10);

  w(pmn->getNeuron(TR0_m), vrnRight->getNeuron(6), -2.5);
  w(pmn->getNeuron(TR0_m), inputNeurons[0], -10);

  w(pmn->getNeuron(TR1_m), vrnRight->getNeuron(6), -2.5);
  w(pmn->getNeuron(TR1_m), inputNeurons[0], -10);

  w(pmn->getNeuron(TR2_m), vrnRight->getNeuron(6), -2.5);
  w(pmn->getNeuron(TR2_m), inputNeurons[0], -10);

  //CL CR
  w(pmn->getNeuron(CL0_m), psn->getNeuron(11), -5.0);
  w(pmn->getNeuron(CL0_m), inputNeurons[0], 10);
  b(pmn->getNeuron(CL0_m), -0.5);

  w(pmn->getNeuron(CL1_m), psn->getNeuron(11), 5.0);
  w(pmn->getNeuron(CL1_m), inputNeurons[0], 10);
  b(pmn->getNeuron(CL1_m), -0.5);

  w(pmn->getNeuron(CL2_m), psn->getNeuron(11), -5.0);
  w(pmn->getNeuron(CL2_m), inputNeurons[0], 10);
  b(pmn->getNeuron(CL2_m), -0.5);

  w(pmn->getNeuron(CR0_m), psn->getNeuron(11), 5.0);
  w(pmn->getNeuron(CR0_m), inputNeurons[0], 10);
  b(pmn->getNeuron(CR0_m), -0.5);

  w(pmn->getNeuron(CR1_m), psn->getNeuron(11), -5.0);
  w(pmn->getNeuron(CR1_m), inputNeurons[0], 10);
  b(pmn->getNeuron(CR1_m), -0.5);

  w(pmn->getNeuron(CR2_m), psn->getNeuron(11), 5.0);
  w(pmn->getNeuron(CR2_m), inputNeurons[0], 10);
  b(pmn->getNeuron(CR2_m), -0.5);

  //FR FL
  w(pmn->getNeuron(FL0_m), psn->getNeuron(10), -2.2);
  w(pmn->getNeuron(FL0_m), inputNeurons[0], 10);
  b(pmn->getNeuron(FL0_m), -0.5);

  w(pmn->getNeuron(FL1_m), psn->getNeuron(10), 2.2);
  w(pmn->getNeuron(FL1_m), inputNeurons[0], 10);
  b(pmn->getNeuron(FL1_m), -0.5);

  w(pmn->getNeuron(FL2_m), psn->getNeuron(10), -2.2);
  w(pmn->getNeuron(FL2_m), inputNeurons[0], 10);
  b(pmn->getNeuron(FL2_m), -0.5);

  w(pmn->getNeuron(FR0_m), psn->getNeuron(11), -2.2);
  w(pmn->getNeuron(FR0_m), inputNeurons[0], 10);
  b(pmn->getNeuron(FR0_m), -0.5);

  w(pmn->getNeuron(FR1_m), psn->getNeuron(10), 5);
  w(pmn->getNeuron(FR1_m), inputNeurons[0], 10);
  b(pmn->getNeuron(FR1_m), -0.5);

  w(pmn->getNeuron(FR2_m), psn->getNeuron(11), -5.0);
  w(pmn->getNeuron(FR2_m), inputNeurons[0], 10);
  b(pmn->getNeuron(FR2_m), -0.5);

  addSubnet(pmn);

	/*******************************************************************************
	 *  MODULE 6 OUTPUTS
	 *******************************************************************************/

	for(int i =0; i<19;i++)
	{
		outputNeurons[AmosIIMotorNames(i)]=pmn->getNeuron(AmosIIMotorNames(i));
	}



};

void ModularNeuralControl::setInputNeuronInput(int input, double value)
{
	setInput(inputNeurons[input],value);
}

void ModularNeuralControl::setInputMotorNeuronInput(int input, double  value)
{
  pmn->setInput(input,value);
}

//with preprocessing probably depratched
double ModularNeuralControl::getMotorNeuronActivity(AmosIIMotorNames motor)
{
  return getActivity(outputNeurons[motor]);
}


//with preprocessing probably depratched
double ModularNeuralControl::getMotorNeuronOutput(AmosIIMotorNames motor)
{
	return getOutput(outputNeurons[motor]);
}


//with preprocessing probably depratched
double ModularNeuralControl::getCpgOutput(int output)
{
	return cpg->getOutput(output);
}

double ModularNeuralControl::getCpgActivity(int output)
{
  return cpg->getActivity(output);
}

double ModularNeuralControl::getCpgWeight(int neuron1, int neuron2)
{
  return cpg->getWeight(neuron1, neuron2);
}

double ModularNeuralControl::getCpgBias(int neuron)
{
  return cpg->getBias(neuron);
}

//with preprocessing probably depratched
double ModularNeuralControl::getpcpgOutput(int output)
{
	return pcpg->getOutput(output);
}

//with preprocessing probably depratched
void ModularNeuralControl::setInputPsn(int input, double value)
{
	psn->setInput(input,value);
}

//with preprocessing probably depratched
void ModularNeuralControl::setInputVrnLeft(int input, double value)
{
	vrnLeft->setInput(input,value);
}

//with preprocessing probably depratched
void ModularNeuralControl::setInputVrnRight(int input, double  value)
{
	vrnRight->setInput(input,value);
}


//with preprocessing probably depratched
double ModularNeuralControl::getPsnOutput(int output)
{
	return psn->getOutput(output);
}

//with preprocessing probably depratched
double ModularNeuralControl::getVrnLeftOutput(int output)
{
	return vrnLeft->getOutput(output);
}

//with preprocessing probably depratched
double ModularNeuralControl::getVrnRightOutput(int output)
{
	return vrnRight->getOutput(output);
}
