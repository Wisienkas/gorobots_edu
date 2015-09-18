/*
 *  Created on: Apr 10, 2012
 *      Author: eduard grinke
 *
 *  edited by degoldschmidt
 *
 *  edited by Subhi shaker Barikhan
 *  Date 26.05.2014
 */

#include "ModularNeuralControl.h"
#include "NeuralLocomotionControlAdaptiveClimbing.h"
#include "utils/ann-library/so2cpg.h"
#include "utils/ann-library/pcpg.h"
#include "utils/ann-library/psn.h"
#include "utils/ann-library/vrn.h"
#include "utils/ann-library/pmn.h"
#include "utils/ann-framework/neuron.h"
#include "utils/ann-library/adaptiveso2cpgsynplas.h"




ModularNeuralControl::ModularNeuralControl(int cpg_option){

	 /*******************************************************************************
	 *  MODULE 0 IO'S for modularneuralcontrol
	 *******************************************************************************/
	/*MCPGs*/
		  currentActivity.resize(2);
		  for(int i=0;i<6;i++)
		  {
		    for(int j=0;j<6;j++)
		    {
		         delta[i][j]=0;
		         cnctCoeffMat[i][j]=0.0;
		    }
		  }
		  /*End of MCPGs*/

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
      Control_input = 0.03;// slow Wave OK USED
      //Control_input = 0.05;//slow stable Tetrapod OK USED
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
void  ModularNeuralControl::setCpgOutput(int neuron,double value)
{
   cpg->setOutput(neuron,value);
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
void ModularNeuralControl::changeControlInput(double new_ControlInput)
{
	Control_input=new_ControlInput;
  cpg->setWeight(0, 1, 0.18 + new_ControlInput);
  cpg->setWeight(1, 0, -0.18 - new_ControlInput);
}
void ModularNeuralControl::enableoscillatorsCoupling(bool mMCPGs)
{
 if(mMCPGs)
	 oscillatorsCouplingIsEnabled=true;

}
void ModularNeuralControl::disableoscillatorsCoupling()
{
	 oscillatorsCouplingIsEnabled=false;

}
void ModularNeuralControl::enableContactForce(bool mMCPGs)
{
	 if(mMCPGs)
		 contactForceIsEnabled=true;
}
void ModularNeuralControl::disableContactForce()
{

		 contactForceIsEnabled=false;
}

void ModularNeuralControl::step()
{
	  updateActivities();
	    updateWeights();
	    updateOutputs();
	    postProcessing();
}

/**
 * param: CPGID: id of the CPG (each CPG has a unique id (0,1,...,5))
 * param: x : sensory information
 * author: subhi shaker barikhan
 * date:26.05.2014
 */
void ModularNeuralControl::step(int CPGID,std::vector<NeuralLocomotionControlAdaptiveClimbing *> NLCAC,const std::vector<double> x)
{
	  currentActivity.at(0)=cpg->getActivity(cpg->getNeuron(0));
	  currentActivity.at(1)=cpg->getActivity(cpg->getNeuron(1));
	  updateActivities();
	  double activity0=cpg->getActivity(cpg->getNeuron(0))+cpg->getBias(0);
	  double activity1=cpg->getActivity(cpg->getNeuron(1))+cpg->getBias(1);

	  int sensorname=AmosIISensorNames(CPGID+R0_fs);
	if ( oscillatorsCouplingIsEnabled==true)
	  {

		 for(int i=0;i<6;i++) // 6 is the number of CPGs
		      {
              if (CPGID!=i)
              {
                oscillatorcouple0= 0.1*(1-cos(currentActivity.at(0)-NLCAC.at(i)->nlc->getCpgActivity(0)-delta[CPGID][i]))
                                         +sin(currentActivity.at(0)-NLCAC.at(i)->nlc->getCpgActivity(0)-delta[CPGID][i]);

                oscillatorcouple1= 0.1*(1-cos(currentActivity.at(1)-NLCAC.at(i)->nlc->getCpgActivity(1)-delta[CPGID][i]))
                                         +sin(currentActivity.at(1)-NLCAC.at(i)->nlc->getCpgActivity(1)-delta[CPGID][i]);

                activity0-=cnctCoeffMat[CPGID][i]*(oscillatorcouple0);

                activity1-=cnctCoeffMat[CPGID][i]*(oscillatorcouple1);
              }

              cpg->setActivity(0, activity0);
              cpg->setActivity(1, activity1);
		      }

	  }
		 if (contactForceIsEnabled && !oscillatorsCouplingIsEnabled)
		  {

		   if (sensorname==R1_fs || sensorname==L1_fs )
		   {
		 	 ContactForceEffect1=-(0.03)*(x.at(sensorname))*sin(currentActivity.at(1));//predictActivity
		    ContactForceEffect0=-(0.03)*(x.at(sensorname))*cos(currentActivity.at(0));
		   }
		   else if(sensorname==R0_fs || sensorname==L0_fs  )
		   {
		 	  ContactForceEffect1=-(0.03)*(x.at(sensorname))*sin(currentActivity.at(1));//0.03
		 	    ContactForceEffect0=-(0.04)*(x.at(sensorname))*cos(currentActivity.at(0)); //0.04
		   }
		   else
		   {
		 	  ContactForceEffect1=-(0.03)*(x.at(sensorname))*sin(currentActivity.at(1));
		 	    ContactForceEffect0=-(0.035)*(x.at(sensorname))*cos(currentActivity.at(0));//0.035
		   }
		  }
		  /* *
		  * when contactForceEnable is true but oscillatorsCouplingIsEnabled is false,
		  *                         continuous local sensory feedback mechanism will be deployed.
		  */
		  else if(contactForceIsEnabled && oscillatorsCouplingIsEnabled)
		  {

		 	   ContactForceEffect0=-(0.035)*(x.at(sensorname))*cos(currentActivity.at(0));
		 	   ContactForceEffect1=-(0.03)*(x.at(sensorname))*sin(currentActivity.at(1));

		  }

		  else
		  {
		    ContactForceEffect0=0;
		    ContactForceEffect1=0;
		  }

		  cpg->setActivity(0, activity0+ContactForceEffect0);
		  cpg->setActivity(1, activity1+ContactForceEffect1);



   // updateActivities();
    updateWeights();
    updateOutputs();
    postProcessing();
}


/*
 *
 * Parameter gaitPattern:
 * gaitPattern=0==>Tripod
 * gaitPattern=1==>Tetrapod
 * gaitPattern=2==>Wave
 * gaitPattern=3==> irrgeular gait, where lateral legs move in phase and contralateral legs move out of phase.
 * For more information about gait generation, it is recommended to take a look at P.37 (Multiple CPGs with Local Feedback Mechanisms
																for Locomotor Adaptation of Hexapod Robots)
 * author subhi Shaker Barikhan
 * Date: 12.05.2014
 * */
void ModularNeuralControl::changeGaitpattern(int gaitPattern)
{
	if (oscillatorsCouplingIsEnabled)
	  {
		  for(int i=0;i<6;i++)
		  {
		    for(int j=0;j<6;j++)
		    {
		         delta[i][j]=0;
		         cnctCoeffMat[i][j]=0.0;
		    }
		  }
	  double k_ij=0.006;//0.01 0.0035 0.004 0.005

	  switch  (gaitPattern)
	  {
	  case 0:
	  	  ////************************ TRIPOD-fullyConnected****************************/////////
	  	//double k_ij=0.01;
	  	// it is possible to use either 0 or 2*Pi to mention phase difference between oscillators
	  	delta[1][0]=M_PI;
	  	delta[2][0]=0;delta[2][1]=M_PI;
	  	delta[3][0]=M_PI;delta[3][1]=0;delta[3][2]=M_PI;
	  	delta[4][0]=0;delta[4][1]=M_PI;delta[4][2]=0;delta[4][3]= M_PI;
	  	delta[5][0]=M_PI; delta[5][1]=0; delta[5][2]=M_PI;delta[5][3]=0; delta[5][4]=M_PI;

	  	cnctCoeffMat[1][0]=k_ij;
	  	cnctCoeffMat[2][0]=k_ij;cnctCoeffMat[2][1]=k_ij;
	  	cnctCoeffMat[3][0]=k_ij;cnctCoeffMat[3][1]=k_ij;cnctCoeffMat[3][2]=k_ij;
	  	cnctCoeffMat[4][0]=k_ij;cnctCoeffMat[4][1]=k_ij;cnctCoeffMat[4][2]=k_ij;cnctCoeffMat[4][3]= k_ij;
	  	cnctCoeffMat[5][0]=k_ij;cnctCoeffMat[5][1]=k_ij; cnctCoeffMat[5][2]=k_ij;cnctCoeffMat[5][3]=k_ij; cnctCoeffMat[5][4]=k_ij;

	  	break;
	  	case 1:
	  	  ////************************ TETRAPOD-fullyConnected****************************/////////
	  	//double k_ij=0.01;
	  	delta[1][0]=2*M_PI/3;
	  	delta[2][0]=4*M_PI/3;delta[2][1]=2*M_PI/3;
	  	delta[3][0]=2*M_PI/3;delta[3][1]=0;delta[3][2]=-2*M_PI/3;
	  	delta[4][0]=4*M_PI/3;delta[4][1]=2*M_PI/3;delta[4][2]=0;delta[4][3]= 2*M_PI/3;//-2*M_PI/3
	  	delta[5][0]=0; delta[5][1]=4*M_PI/3; delta[5][2]=2*M_PI/3;delta[5][3]=4*M_PI/3; delta[5][4]=2*M_PI/3;

	  	cnctCoeffMat[1][0]=k_ij;
	  	cnctCoeffMat[2][0]=k_ij;cnctCoeffMat[2][1]=k_ij;
	  	cnctCoeffMat[3][0]=k_ij;cnctCoeffMat[3][1]=k_ij;cnctCoeffMat[3][2]=k_ij;
	  	cnctCoeffMat[4][0]=k_ij;cnctCoeffMat[4][1]=k_ij;cnctCoeffMat[4][2]=k_ij;cnctCoeffMat[4][3]= k_ij;
	  	cnctCoeffMat[5][0]=k_ij;cnctCoeffMat[5][1]=k_ij; cnctCoeffMat[5][2]=k_ij;cnctCoeffMat[5][3]=k_ij; cnctCoeffMat[5][4]=k_ij;
	  	break;

	case 2:
	////************************ Wave-fullyConnected****************************/////////
	//	 double k_ij=0.01;
	  delta[1][0]=2*M_PI/6;
	  delta[2][0]=4*M_PI/6;delta[2][1]=2*M_PI/6;
	  delta[3][0]=6*M_PI/6;delta[3][1]=4*M_PI/6;delta[3][2]=2*M_PI/6;
	  delta[4][0]=8*M_PI/6;delta[4][1]=6*M_PI/6;delta[4][2]=4*M_PI/6;delta[4][3]= 2*M_PI/6;
	  delta[5][0]=10*M_PI/6; delta[5][1]=8*M_PI/6; delta[5][2]=6*M_PI/6;delta[5][3]=4*M_PI/6; delta[5][4]=2*M_PI/6;

	  cnctCoeffMat[1][0]=k_ij;
	  cnctCoeffMat[2][0]=k_ij;cnctCoeffMat[2][1]=k_ij;
	  cnctCoeffMat[3][0]=k_ij;cnctCoeffMat[3][1]=k_ij;cnctCoeffMat[3][2]=k_ij;
	  cnctCoeffMat[4][0]=k_ij;cnctCoeffMat[4][1]=k_ij;cnctCoeffMat[4][2]=k_ij;cnctCoeffMat[4][3]= k_ij;
	  cnctCoeffMat[5][0]=k_ij;cnctCoeffMat[5][1]=k_ij; cnctCoeffMat[5][2]=k_ij;cnctCoeffMat[5][3]=k_ij; cnctCoeffMat[5][4]=k_ij;

	  break;

	case 3:
		////************************ irregular gait-fullyConnected****************************/////////
		//	 double k_ij=0.01;
		  delta[1][0]=0;
		  delta[2][0]=0;delta[2][1]=0;
		  delta[3][0]=M_PI;delta[3][1]=M_PI;delta[3][2]=M_PI;
		  delta[4][0]=M_PI;delta[4][1]=M_PI;delta[4][2]=M_PI;delta[4][3]= 0;
		  delta[5][0]=M_PI; delta[5][1]=M_PI; delta[5][2]=M_PI;delta[5][3]=0; delta[5][4]=0;

		  cnctCoeffMat[1][0]=k_ij;
		  cnctCoeffMat[2][0]=k_ij;cnctCoeffMat[2][1]=k_ij;
		  cnctCoeffMat[3][0]=k_ij;cnctCoeffMat[3][1]=k_ij;cnctCoeffMat[3][2]=k_ij;
		  cnctCoeffMat[4][0]=k_ij;cnctCoeffMat[4][1]=k_ij;cnctCoeffMat[4][2]=k_ij;cnctCoeffMat[4][3]= k_ij;
		  cnctCoeffMat[5][0]=k_ij;cnctCoeffMat[5][1]=k_ij; cnctCoeffMat[5][2]=k_ij;cnctCoeffMat[5][3]=k_ij; cnctCoeffMat[5][4]=k_ij;

		  break;

	  }
	for(int i=0;i<6;i++)
	  {
	    for(int j=i+1;j<6;j++)
	    {
	         delta[i][j]=-delta[j][i];
	        cnctCoeffMat[i][j]=cnctCoeffMat[j][i];
	    }
	  }
	  }

}


