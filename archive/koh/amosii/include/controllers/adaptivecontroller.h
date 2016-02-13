#ifndef __EMPTYCONTROLLER_H
#define __EMPTYCONTROLLER_H

#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>
#include <selforg/configurable.h>
#include <assert.h>
#include <cmath>
#include <stdlib.h>
#include <string.h>
#include <selforg/matrix.h>
#include "ModularNeuralControl.h"
#include "NeuralLocomotionControlAdaptiveClimbing.h"
#include "NeuralPreprocessingLearning.h"
//#include "../../controller/amosii/modular_neural_control/ModularNeuralControl.h"
/**
 * Empty robot controller.
 * The controller gets a number of input sensor values each timestep
 * and has to generate a number of output motor values.
 *
 * Go to the step() function and enter the control commands with respect to your task!
 *
 */
class AdaptiveController : public AbstractController {

  public:
	AdaptiveController(int amosVersion)
    : AbstractController("EmptyController", "$Id: tripodgait18dof.cpp,v 0.1 $"){
     // t = 0;

      pushingobject=false;
      y_TR0_m=1.00; //0.85
      y_TL0_m=1.00; //0.85


      robotSpeed=0;
      Ttime=1;
      Ttime2=1;
      counter=1;
      sum=0;
      calculationofSpeed=false;
      LegAmputation=false;


      for(int i=0;i<6;i++)
      {
        NLCAC[i]= new NeuralLocomotionControlAdaptiveClimbing(amosVersion);
      }
     // CPG-option =1
     //preprocessinglearning=new NeuralPreprocessingLearning();
      outputH1 = 0.1;
      outputH2 = 0.1;

      addInspectableValue("Xrefdistance",&Xrefdistance,"Robot_speed");
      addInspectableValue("Yrefdistance",&Yrefdistance,"Robot_speed");
      addInspectableValue("CPG.R1_1_phase_phi",&NLCAC[0]->nlc->phase_phi,"CPG.R1_1_frequency");

      addInspectableValue("lifting_valueR0",&NLCAC[0]->lifting_value,"lifting_valueR0");
      addInspectableValue("tr_output.R0",&NLCAC[0]->tr_output.at(0),"tr_output.R0 without delay");
      addInspectableValue("m_pre[TR0]",&NLCAC[0]->m_pre.at(TR0_m),"_pre[TR0] after delay");
      addInspectableValue("m_reflex[TR0]",&NLCAC[0]->m_reflex.at(TR0_m),"m_reflex[TR0]");

      addInspectableValue("cr_output2.R0",&NLCAC[0]->cr_output.at(2),"cr_output.R0");
      addInspectableValue("cr_output0.R0",&NLCAC[0]->cr_output.at(0),"cr_output.R0");
      addInspectableValue("m_pre[CR0]",&NLCAC[0]->m_pre.at(CR0_m),"_pre[CR0]");
      addInspectableValue("fr_output.R0",&NLCAC[0]-> fr_output.at(0),"fr_output.R0");
      addInspectableValue("fr_delayedOutput.R0",&NLCAC[0]-> m_pre.at(FR0_m),"fr_delayedOutput.R0");

      addInspectableValue("predictOutputCPG.R0_0",&NLCAC[0]->nlc->predictOutput.at(0),"predictOutputCPG.R0_0");
      addInspectableValue("predictOutputCPG.R0_1",&NLCAC[0]->nlc->predictOutput.at(1),"predictOutputCPG.R0_1");
      addInspectableValue("CPG.R0_0",&NLCAC[0]->cpg_output.at(0),"CPG.R0_0"); // cpg0 control leg R0
      addInspectableValue("CPG.R0_1",&NLCAC[0]->cpg_output.at(1),"CPG.R0_1");
      addInspectableValue("predictActivityCPG.R0_0",&NLCAC[0]->nlc->predictActivity.at(0),"predictActivityCPG.R0_0");
      addInspectableValue("predictActivityCPG.R0_1",&NLCAC[0]->nlc->predictActivity.at(1),"predictActivityCPG.R0_1");
      addInspectableValue("ActualActivityCPG.R0_0",&NLCAC[0]->cpg_Activity.at(0),"ActualActivityCPG.R0_0");
      addInspectableValue("ActualActivityCPG.R0_1",&NLCAC[0]->cpg_Activity.at(1),"ActualActivityCPG.R0_1");
      addInspectableValue("footSensor.R0_0",&NLCAC[0]->nlc->ContactForce,"ContactForce.R0_0");
      addInspectableValue("ContactForceEffect0.R0_0",&NLCAC[0]->nlc->ContactForceEffect0,"ContactForceEffect0.R0_0");
      addInspectableValue("ContactForceEffect1.R0_1",&NLCAC[0]->nlc->ContactForceEffect1,"ContactForceEffect1.R0_0");
      addInspectableValue("footbias.R0_0",&NLCAC[0]->nlc->footbias.at(0),"footbias.R0_0");
      addInspectableValue("footbias.R0_1",&NLCAC[0]->nlc->footbias.at(1),"footbias.R0_1");
      addInspectableValue("pcpg.R0_0",&NLCAC[0]->pcpg_output.at(0),"pcpg.R0_0");
      addInspectableValue("pcpg.R0_1",&NLCAC[0]->pcpg_output.at(1),"pcpg.R0_1");
      addInspectableValue("vrn_outputM.R0",&NLCAC[0]->vrn_outputM,"vrn_outputM.R0");
      addInspectableValue("vrn_output.R0",&NLCAC[0]->vrn_output.at(13),"vrn_output.R0");
      addInspectableValue("vrn_outputTanh.R0",&NLCAC[0]->vrn_outputTanh,"vrn_outputTanh.R0");




     // addInspectableValue("predictOutputCPG.R1_0",&NLCAC[1]->.at(0),"predictOutputCPG.R1_0");
      addInspectableValue("lifting_valueR1",&NLCAC[1]->lifting_value,"lifting_valueR1");
      addInspectableValue("tr_output.R1",&NLCAC[1]->tr_output.at(1),"tr_output.R1 without delay");
      addInspectableValue("m_pre[TR1]",&NLCAC[1]->m_pre.at(TR1_m),"m_pre[TR1] after delay");
      addInspectableValue("cr_output2.R1",&NLCAC[1]->cr_output.at(2),"cr_output2.R1");
      addInspectableValue("cr_output.R1",&NLCAC[1]->cr_output.at(1),"cr_output.R1");

      addInspectableValue("fr_output.R1",&NLCAC[1]-> fr_output.at(1),"fr_output.R1");
      addInspectableValue("fr_delayedOutput.R1",&NLCAC[1]-> m_pre.at(FR1_m),"fr_delayedOutput.R1");

      addInspectableValue("predictOutputCPG.R1_0",&NLCAC[1]->nlc->predictOutput.at(0),"predictOutputCPG.R1_0");
      addInspectableValue("predictOutputCPG.R1_1",&NLCAC[1]->nlc->predictOutput.at(1),"predictOutputCPG.R1_1");
      addInspectableValue("CPG.R1_0",&NLCAC[1]->cpg_output.at(0),"CPG.R1_0"); // cpg1 control leg R1
      addInspectableValue("CPG.R1_1",&NLCAC[1]->cpg_output.at(1),"CPG.R1_1");
      addInspectableValue("predictActivityCPG.R1_0",&NLCAC[1]->nlc->predictActivity.at(0),"predictActivityCPG.R1_0");
      addInspectableValue("predictActivityCPG.R1_1",&NLCAC[1]->nlc->predictActivity.at(1),"predictActivityCPG.R1_1");
      addInspectableValue("ActualActivityCPG.R1_0",&NLCAC[1]->cpg_Activity.at(0),"ActualActivityCPG.R1_0");
      addInspectableValue("ActualActivityCPG.R1_1",&NLCAC[1]->cpg_Activity.at(1),"ActualActivityCPG.R1_1");
      addInspectableValue("footSensor.R1_0",&NLCAC[1]->nlc->ContactForce,"ContactForce.R1_0");
      addInspectableValue("ContactForceEffect0.R1_0",&NLCAC[1]->nlc->ContactForceEffect0,"ContactForceEffect0.R1_0");
      addInspectableValue("ContactForceEffect1.R1_1",&NLCAC[1]->nlc->ContactForceEffect1,"ContactForceEffect1.R0_0");
      addInspectableValue("footbias.R1_0",&NLCAC[1]->nlc->footbias.at(0),"footbias.R1_0");
      addInspectableValue("footbias.R1_1",&NLCAC[1]->nlc->footbias.at(1),"footbias.R1_1");

      addInspectableValue("lifting_valueR2",&NLCAC[2]->lifting_value,"lifting_valueR2");
      addInspectableValue("tr_output.R2",&NLCAC[2]->tr_output.at(2),"tr_output.R2 without delay");
      addInspectableValue("m_pre[TR2]",&NLCAC[2]->m_pre.at(TR2_m),"m_pre[TR2] after delay");

      addInspectableValue("cr_output.R2",&NLCAC[2]->cr_output.at(2),"cr_output.R2");

      addInspectableValue("fr_output.R2",&NLCAC[2]-> fr_output.at(2),"fr_output.R2");
       addInspectableValue("fr_delayedOutput.R2",&NLCAC[2]-> m_pre.at(FR2_m),"fr_delayedOutput.R2");

      addInspectableValue("predictOutputCPG.R2_0",&NLCAC[2]->nlc->predictOutput.at(0),"predictOutputCPG.R2_0");
      addInspectableValue("predictOutputCPG.R2_1",&NLCAC[2]->nlc->predictOutput.at(1),"predictOutputCPG.R2_1");
      addInspectableValue("CPG.R2_0",&NLCAC[2]->cpg_output.at(0),"CPG.R2_0"); // cpg2 control leg R2
      addInspectableValue("CPG.R2_1",&NLCAC[2]->cpg_output.at(1),"CPG.R2_1");
      addInspectableValue("predictActivityCPG.R2_0",&NLCAC[2]->nlc->predictActivity.at(0),"predictActivityCPG.R2_0");
      addInspectableValue("predictActivityCPG.R2_1",&NLCAC[2]->nlc->predictActivity.at(1),"predictActivityCPG.R2_1");
      addInspectableValue("ActualActivityCPG.R2_0",&NLCAC[2]->cpg_Activity.at(0),"ActualActivityCPG.R2_0");
      addInspectableValue("ActualActivityCPG.R2_1",&NLCAC[2]->cpg_Activity.at(1),"ActualActivityCPG.R2_1");
      addInspectableValue("footSensor.R2_0",&NLCAC[2]->nlc->ContactForce,"ContactForce.R0_0");
      addInspectableValue("ContactForceEffect0.R2_0",&NLCAC[2]->nlc->ContactForceEffect0,"ContactForceEffect0.R2_0");
      addInspectableValue("ContactForceEffect1.R2_1",&NLCAC[2]->nlc->ContactForceEffect1,"ContactForceEffect1.R2_0");
      addInspectableValue("footbias.R2_0",&NLCAC[2]->nlc->footbias.at(0),"footbias.R2_0");
      addInspectableValue("footbias.R2_1",&NLCAC[2]->nlc->footbias.at(1),"footbias.R2_1");
      addInspectableValue("vrn_output.R2",&NLCAC[2]->vrn_output.at(13),"vrn_output.R2");

      addInspectableValue("tl_output.L0",&NLCAC[3]->tl_output.at(0),"tr_output.L0 without delay");
      addInspectableValue("m_pre[TL0_m]",&NLCAC[3]->m_pre.at(TL0_m),"m_pre[TL0_m] after delay");
      addInspectableValue("cl_output2.L0",&NLCAC[3]->cl_output.at(2),"cr_output2.L0");
      addInspectableValue("cl_output.L0",&NLCAC[3]->cl_output.at(0),"cl_output.L0");
      addInspectableValue("fl_output.L0",&NLCAC[3]-> fl_output.at(0),"fl_output.L0");
      addInspectableValue("fr_delayedOutput.L0",&NLCAC[3]-> m_pre.at(FL0_m),"fr_delayedOutput.L0");

      addInspectableValue("lifting_valueL0",&NLCAC[3]->lifting_value,"lifting_valueL0");
      addInspectableValue("predictOutputCPG.L0_0",&NLCAC[3]->nlc->predictOutput.at(0),"predictOutputCPG.L0_0");
     addInspectableValue("predictOutputCPG.L0_1",&NLCAC[3]->nlc->predictOutput.at(1),"predictOutputCPG.L0_1");
      addInspectableValue("CPG.L0_0",&NLCAC[3]->cpg_output.at(0),"CPG.L0_0"); // cpg3 control leg L0
      addInspectableValue("CPG.L0_1",&NLCAC[3]->cpg_output.at(1),"CPG.L0_1");
      addInspectableValue("predictActivityCPG.L0_0",&NLCAC[3]->nlc->predictActivity.at(0),"predictActivityCPG.L0_0");
       addInspectableValue("predictActivityCPG.L0_1",&NLCAC[3]->nlc->predictActivity.at(1),"predictActivityCPG.L0_1");
       addInspectableValue("ActualActivityCPG.L0_0",&NLCAC[3]->cpg_Activity.at(0),"ActualActivityCPG.L0_0");
       addInspectableValue("ActualActivityCPG.L0_1",&NLCAC[3]->cpg_Activity.at(1),"ActualActivityCPG.L0_1");
       addInspectableValue("footSensor.L0_0",&NLCAC[3]->nlc->ContactForce,"ContactForce.L0_0");
       addInspectableValue("ContactForceEffect0.L0_0",&NLCAC[3]->nlc->ContactForceEffect0,"ContactForceEffect0.L1_0");
       addInspectableValue("ContactForceEffect1.L0_1",&NLCAC[3]->nlc->ContactForceEffect1,"ContactForceEffect1.L1_0");
       addInspectableValue("footbias.L0_0",&NLCAC[3]->nlc->footbias.at(0),"footbias.L0_0");
      addInspectableValue("footbias.L0_1",&NLCAC[3]->nlc->footbias.at(1),"footbias.L0_1");

      addInspectableValue("lifting_valueL1",&NLCAC[4]->lifting_value,"lifting_valueL1");
      addInspectableValue("tl_output.L1",&NLCAC[4]->tl_output.at(1),"tr_output.L1 without delay");
      addInspectableValue("m_pre[TL1_m]",&NLCAC[4]->m_pre.at(TL1_m),"m_pre[TL1_m] after delay");
      addInspectableValue("cl_output2.L2",&NLCAC[4]->cl_output.at(2),"cr_output2.L2");
      addInspectableValue("cl_output.L1",&NLCAC[4]->cl_output.at(1),"cl_output.L1");
      addInspectableValue("predictOutputCPG.L1_0",&NLCAC[4]->nlc->predictOutput.at(0),"predictOutputCPG.L1_0");
      addInspectableValue("predictOutputCPG.L1_1",&NLCAC[4]->nlc->predictOutput.at(1),"predictOutputCPG.L1_1");
      addInspectableValue("CPG.L1_0",&NLCAC[4]->cpg_output.at(0),"CPG.L1_0"); // cpg4 control leg L1
      addInspectableValue("CPG.L1_1",&NLCAC[4]->cpg_output.at(1),"CPG.L1_1");
      addInspectableValue("predictActivityCPG.L1_0",&NLCAC[4]->nlc->predictActivity.at(0),"predictActivityCPG.L1_0");
      addInspectableValue("predictActivityCPG.L1_1",&NLCAC[4]->nlc->predictActivity.at(1),"predictActivityCPG.L1_1");
      addInspectableValue("ActualActivityCPG.L1_0",&NLCAC[4]->cpg_Activity.at(0),"ActualActivityCPG.L1_0");
      addInspectableValue("ActualActivityCPG.L1_1",&NLCAC[4]->cpg_Activity.at(1),"ActualActivityCPG.L1_1");
      addInspectableValue("footSensor.L1_0",&NLCAC[4]->nlc->ContactForce,"ContactForce.L1_0");
      addInspectableValue("ContactForceEffect0.L1_0",&NLCAC[4]->nlc->ContactForceEffect0,"ContactForceEffect0.L0_0");
      addInspectableValue("ContactForceEffect1.L1_1",&NLCAC[4]->nlc->ContactForceEffect1,"ContactForceEffect1.L0_0");
      addInspectableValue("footbias.L1_0",&NLCAC[4]->nlc->footbias.at(0),"footbias.L1_0");
      addInspectableValue("footbias.L1_1",&NLCAC[4]->nlc->footbias.at(1),"footbias.L1_1");


      addInspectableValue("tl_output.L2",&NLCAC[4]->tl_output.at(2),"tr_output.L2 without delay");
      addInspectableValue("m_pre[TL2_m]",&NLCAC[4]->m_pre.at(TL2_m),"m_pre[TL2_m] after delay");
      addInspectableValue("cl_output.L2",&NLCAC[4]->cl_output.at(2),"cl_output.L2");

      addInspectableValue("lifting_valueL2",&NLCAC[5]->lifting_value,"lifting_valueL2");
      addInspectableValue("predictOutputCPG.L2_0",&NLCAC[5]->nlc->predictOutput.at(0),"predictOutputCPG.L2_0");
        addInspectableValue("predictOutputCPG.L2_1",&NLCAC[5]->nlc->predictOutput.at(1),"predictOutputCPG.L2_1");
      addInspectableValue("CPG.L2_0",&NLCAC[5]->cpg_output.at(0),"CPG.L2_0"); // cpg5 control leg L2
      addInspectableValue("CPG.L2_1",&NLCAC[5]->cpg_output.at(1),"CPG.L2_1");
      addInspectableValue("predictActivityCPG.L2_0",&NLCAC[5]->nlc->predictActivity.at(0),"predictActivityCPG.L2_0");
       addInspectableValue("predictActivityCPG.L2_1",&NLCAC[5]->nlc->predictActivity.at(1),"predictActivityCPG.L2_1");
       addInspectableValue("ActualActivityCPG.L2_0",&NLCAC[5]->cpg_Activity.at(0),"ActualActivityCPG.L2_0");
       addInspectableValue("ActualActivityCPG.L2_1",&NLCAC[5]->cpg_Activity.at(1),"ActualActivityCPG.L2_1");
       addInspectableValue("footSensor.L2_0",&NLCAC[5]->nlc->ContactForce,"ContactForce.L2_0");
       addInspectableValue("ContactForceEffect0.L2_0",&NLCAC[5]->nlc->ContactForceEffect0,"ContactForceEffect0.L2_0");
       addInspectableValue("ContactForceEffect1.L2_1",&NLCAC[5]->nlc->ContactForceEffect1,"ContactForceEffect1.L2_0");

       addInspectableValue("footbias.L2_0",&NLCAC[5]->nlc->footbias.at(0),"footbias.L2_0");
      addInspectableValue("footbias.L2_1",&NLCAC[5]->nlc->footbias.at(1),"footbias.L2_1");
    };

    void ChangeHeightofForelegs()
        {
         NLCAC[0]->lifting_value=40;
          NLCAC[3]->lifting_value=40;
          std::cout << " ChangeHeightofForelegs \n";
        }
    void ChangeHeightofHindlegs()
           {
          NLCAC[2]->lifting_value=40;
              NLCAC[5]->lifting_value=40;
             std::cout << " ChangeHeightofHindlegs \n";
           }


    void enableMultiple_stepping()
    {

      /******************************multiple stepping*********************************/

       NLCAC[0]->nlc->changeControlInput(NLCAC[3]->nlc->Control_input*1.3);
       NLCAC[3]->nlc->changeControlInput(NLCAC[3]->nlc->Control_input*1.65);

 /*********************************************************************************************/

      std::cout << " enable multiple_stepping \n";
    }
    void disableMultiple_stepping()
       {
        NLCAC[0]->nlc->changeControlInput(NLCAC[3]->nlc->Control_input);
         NLCAC[3]->nlc->changeControlInput(NLCAC[3]->nlc->Control_input);
         std::cout << " disable multiple_stepping \n";
       }
    void adjustJointanglesforpushing()
    {
      pushingobject=true;
    }

    void enableFootsensors()
    {
    	for (unsigned int i=0; i<y.size();i++)
    		NLCAC[i]->nlc->footSensorenable=true;

    	std::cout << "footsensorsEnable \n";
    }
    void disableFootsensors()
    {
      for (unsigned int i=0; i<y.size();i++)
          	  NLCAC[i]->nlc->footSensorenable=false;

      std::cout << "footsensorsDisable \n";

    }

    void saveModulatoryInput()
    {

      for (int i=0;i<6;i++)
         {
    	  	  ControlInput[i]=  NLCAC[i]->nlc->Control_input;

         }
    }

    void enableLift() // change the height of the legs
    {
      for (int i=0;i<6;i++)
      NLCAC[i]->lift_body_up=true;

      std::cout << "enable lift \n";
    }

    void disableLift()
      {
      for (int i=0;i<6;i++)
           NLCAC[i]->lift_body_up=false;
        std::cout << "disable lift \n";
      }

// Frequency increases by increasing modulatory input.
    void increaseFrequency()
    {
        for (int i=0;i<6;i++)
        NLCAC[i]->nlc->changeControlInput(NLCAC[i]->nlc->Control_input+0.01);
        std::cout << "Frequency increases"<<endl;
         std::cout << "Modulatory input:" << NLCAC[0]->nlc->Control_input <<endl;
         Ttime=1;
         Ttime2=1;
         counter=1;
         sum=0;
         Xrefdistance=x.at(115);
         Yrefdistance=x.at(116);
    }

// Frequency decreases by decreasing modulatory input.
    void decreaseFrequency()
       {
    	      for (int i=0;i<6;i++)
    	           NLCAC[i]->nlc->changeControlInput(NLCAC[i]->nlc->Control_input-0.01);
    	      std::cout << "Frequency decreases"<<endl;
    	      std::cout << "Modulatory input: "<< NLCAC[0]->nlc->Control_input << endl;

    	      counter=1;
    	      sum=0;
    	      Ttime=1;
    	      Ttime2=1;
    	      Xrefdistance=x.at(115);
    	      Yrefdistance=x.at(116);
       }

// restore the previous frequency by restoring modulatory input saved in .
    void resetFrequency()
    {
      for (int i=0;i<6;i++)
		  {
				NLCAC[i]->nlc->changeControlInput(ControlInput[i]);
		  }
      std::cout << "resetFrequency \n";
    }
    // combine contact force feedback with the afferent control signals to drive coxa joints (CTr)
    void enableCoxawithContactSensorSignal()
    {
      for (unsigned int i=0; i<y.size();i++)
              NLCAC[i]->coxawithContactSensorSignalIsEnabled=true;

      std::cout << "CoxawithContactSensorSignalEnable \n";
    }

    void disableCoxawithContactSensorSignal()
      {
      for (unsigned int i=0; i<y.size();i++)
              NLCAC[i]->coxawithContactSensorSignalIsEnabled=false;

      std::cout << "CoxawithContactSensorSignalDisable \n";
      }

    void enablePhaseResetInhibitionMech()
    {
		  for (unsigned int i=0; i<y.size();i++)
		  {
				NLCAC[i]->nlc->phaseResetInhibitionIsEnabled=true;
				NLCAC[i]->coxawithContactSensorSignalIsEnabled=true; // combine contact force feedback with the afferent control signals to drive coxa joints (CTr)

		  }
		  std::cout << "Phase reset and Inhibition mechanism are enabled \n";
    }
    void disablePhaseResetInhibitionMech()
       {
      for (unsigned int i=0; i<y.size();i++)
      {
                    NLCAC[i]->nlc->phaseResetInhibitionIsEnabled=false;
                    NLCAC[i]->coxawithContactSensorSignalIsEnabled=false;
      }
      std::cout << "Phase reset and Inhibition mechanism are disabled \n";
       }

    void enableContactForceMech()
          {
         for (unsigned int i=0; i<y.size();i++)
                       NLCAC[i]->nlc->contactForceIsEnabled=true;
         std::cout << "contactForce is enabled \n";
          }

    void disableContactForceMech()
          {
         for (unsigned int i=0; i<y.size();i++)
                       NLCAC[i]->nlc->contactForceIsEnabled=false;
         std::cout << "contact force is disabled \n";
          }
    void enableOscillatorCoupling()
    {
    	  for (unsigned int i=0; i<y.size();i++)
    	                        NLCAC[i]->nlc->oscillatorsCouplingIsEnabled=true;
    	          std::cout << "Oscillator Coupling is enabled \n";
    }
    void disableOscillatorCoupling()
       {
       	  for (unsigned int i=0; i<y.size();i++)
       	                        NLCAC[i]->nlc->oscillatorsCouplingIsEnabled=false;
       	          std::cout << "Oscillator Coupling is disabled \n";
       }
    void enableOscillatorRingCoupling()
    {
    	  for (unsigned int i=0; i<y.size();i++)
    	                        NLCAC[i]->nlc->oscillatorsRingCouplingIsEnabled=true;
    	          std::cout << "Oscillator ring coupling is enabled \n";
    }
    void disableOscillatorRingCoupling()
     {
     	  for (unsigned int i=0; i<y.size();i++)
     	                        NLCAC[i]->nlc->oscillatorsRingCouplingIsEnabled=false;
     	          std::cout << "Oscillator ring coupling is disabled \n";
     }

    void enableTripodGait()
          {
          	  for (unsigned int i=0; i<y.size();i++)
          	  {
                  int gaitPattern=0;//Tripod
                  NLCAC[i]->nlc->changeGaitpattern(gaitPattern);
          	  }
          	          std::cout << "Tripod \n";
          }
    void enableTetrapodGait()
      {
      	  for (unsigned int i=0; i<y.size();i++)
      	  {
      		  int gaitPattern=1;  //Tetrapod
      		NLCAC[i]->nlc->changeGaitpattern(gaitPattern);
      	  }
      	          std::cout << "Tetrapod \n";
      }

    void enableWaveGait()
         {
         	  for (unsigned int i=0; i<y.size();i++)
         	  {
         	        int gaitPattern=2;//wave
         	        NLCAC[i]->nlc->changeGaitpattern(gaitPattern);
         	  }
         	          std::cout << "wave \n";
         }
    void enableIrregularGait()
         {
            for (unsigned int i=0; i<y.size();i++)
            {
                  int gaitPattern=3;//irregular gait.
                  NLCAC[i]->nlc->changeGaitpattern(gaitPattern);
            }
                    std::cout << "irregularEnable \n";
         }

     void calculateSpeed()
              {
			   std::cout << "calculate Speed \n";
			   std::string fileName;
			   // speed.txt contains speeds of the robot.

			   fileName = "Speed";
			   std::string a =  ".txt";
			   fileName += a;
			   char speedFilename[30];
			   strcpy(speedFilename,fileName.c_str());
			   ofs.open(speedFilename);
			   std::cout << "make a file"<< std::endl;
			   //   ofs << "# This is the data of the speed of hexapod"<<std::endl;
			   ofs << "# Frequency Speed"<<std::endl;
			   Ttime=1;
			   Ttime2=1;
			   sum=0;
			   // Xrefdistance & Yrefdistance indicate the postion of the robot at the same moment the method calculateSpeed is called
			   Xrefdistance=x.at(115);
			   Yrefdistance=x.at(116);
			   //////////////
			   calculationofSpeed=true;

              }

     void disableCalculateSpeed()
              {
       std::cout << "calculateSpeedDisable \n";
       calculationofSpeed=false;
              }

// The following method is used when two robots are connected to each other for a collaborative task.
     void ConnectTwoRobotsVert(int arobotId)
       {
           twoconnectedRobots=true;
           robotId=arobotId;
       }

       void pushObject()
          {
         y_TR0_m+=0.03;
         y_TL0_m+=0.03;
         std::cout << "push Object y_TR0_m="<< y_TR0_m << endl;
          }

// set modulatory input to zero.
       void stopGoing ()
       {
         for (int i=0;i<6;i++)
                    NLCAC[i]->nlc->changeControlInput(0);
                   std::cout << "changeFrequencySlow "<< NLCAC[0]->nlc->Control_input << endl;

       }

       void printObjectWeight(double ObjWeight)  //(experiment about changing the center of mass)
       {
         std::cout<<"the weight of object :"<< ObjWeight<<endl;

       }

       void  enableLegAmputation()
       {
    	   LegAmputation=true;
    	   std::cout << "enable Leg Amputation \n";
       }

       void  disableLegAmputation()
             {
    	   LegAmputation=false;
          	   std::cout << "disable Leg Amputation \n";
             }

       void  enableMuscleModel()
       {
    	   for (int i=0;i<6;i++)
    	          NLCAC[i]->isMuscle=true;
    	   std::cout << "enable muscle model \n";
       }
       void disableMuscleModel()
        {
    	   for (int i=0;i<6;i++)
    		   NLCAC[i]->isMuscle=false;
    	   std::cout << "disable muscle model \n";
        }

    virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0){
      //Tripodgait for 18 DOF Hexapod
      numbersensors = sensornumber;
      numbermotors = motornumber;

      x.resize(sensornumber);
      y.resize(6);




      /******************************Initiating the robot with tetrapod *********************************/
      /*   NLCAC[5]->nlc->setCpgOutput(0,0.1);
           NLCAC[5]->nlc->setCpgOutput(1,-1);
           NLCAC[2]->nlc->setCpgOutput(0,1);
           NLCAC[2]->nlc->setCpgOutput(1,0.1);
           NLCAC[1]->nlc->setCpgOutput(0,0.1);
           NLCAC[1]->nlc->setCpgOutput(1,1);
           NLCAC[4]->nlc->setCpgOutput(0,1);
           NLCAC[4]->nlc->setCpgOutput(1,0.1);
           NLCAC[3]->nlc->setCpgOutput(0,0.1);
           NLCAC[3]->nlc->setCpgOutput(1,1);
           NLCAC[0]->nlc->setCpgOutput(0,-1);
           NLCAC[0]->nlc->setCpgOutput(1,0.1);*/
     /****************************** End (Initiating the robot with tetrapod)*********************************/


      for (unsigned int i=0; i<y.size();i++)
      {
       y.at(i).resize(motornumber);
       NLCAC[i]->nlc->footSensorenable= true;// all foot sensors are working
       NLCAC[i]->nlc->phaseResetInhibitionIsEnabled= false;// disable phase reset mechanism
       // initiate stable walking pattern  Tetrapod


       if (i==0 || i==1 || i==2 )  // for phase differences between cpgs || i==3 || i==4 || i==5 || i==3 || i==4 || i==5
       {
       NLCAC[i]->nlc->setCpgOutput(0,-1);
       NLCAC[i]->nlc->setCpgOutput(1,0.1);
      // NLCAC[i]->nlc->Control_input=0.02;
       }
       else
       {
         NLCAC[i]->nlc->setCpgOutput(0,1);
         NLCAC[i]->nlc->setCpgOutput(1,0.1);
       //  NLCAC[i]->nlc->Control_input=0.02;
       }

      }

    };

    virtual ~AdaptiveController()
    { ofs.close();};

    /// returns the name of the object (with version number)+
    virtual paramkey getName() const {
      return name;
    }
    /// returns the number of sensors the controller was initialised with or 0
    /// if not initialised
    virtual int getSensorNumber() const {
      return number_channels;
    }
    /// returns the mumber of motors the controller was initialised with or 0 if
    // not initialised
    virtual int getMotorNumber() const {
      return number_channels;
    }

    /// performs one step (includes learning).
    /// Calulates motor commands from sensor inputs.
    virtual void step(const sensor* x_, int number_sensors, motor* y_, int number_motors){
            assert(number_sensors == numbersensors);
            assert(number_motors == numbermotors);
            //0) Sensor inputs/scaling  ----------------


              for (unsigned int i = 0; i < AMOSII_SENSOR_MAX; i++) {
                x.at(i) = x_[i];
              }

              /*** calculation of speed
                At each frequency of CPG (modulatory input) the speed of the robot is calculated.
				For more accurate the speed is calculated after 250 time steps,
				 after 1000 time steps the average speed is calculated.
				Modulatory input (frequency) is increased after 1000 time steps and the robot's speed is measured as already described.
                 **/
              if (calculationofSpeed)
              {
                if (Ttime2==250)
                                  {
                                    //sum+=100*sqrt(sqr(abs(x.at(115)-Xrefdistance)/((Ttime2)*0.03725))+sqr(abs(x.at(116)-Yrefdistance)/((Ttime2)*0.03725)));
                  sum+= sqrt((x.at(115)-Xrefdistance)*(x.at(115)-Xrefdistance)
                      + (x.at(116)-Yrefdistance) * (x.at(116)-Yrefdistance))/((Ttime2)*0.03725);
                                      Xrefdistance=x.at(115);
                                     Yrefdistance=x.at(116);
                                     Ttime2=1;
                                  }

                if(Ttime==1000)
                {

                  robotSpeed=sum/4; // calculate the average speed.
                  robotSpeed*=100; // using speed unit [cm/sec]
                  std::cout << "Modulatory Input: "<< NLCAC[0]->nlc->Control_input << endl;
                  std::cout << "robotSpeed "<< robotSpeed << endl;
                  // save the values of modulatory input and speed in speed.txt
                  ofs <<  NLCAC[0]->nlc->Control_input << " " << robotSpeed <<std::endl;

                  sum=0;
                  increaseFrequency();
                  Ttime=1;
                  Ttime2=1;
                  // update the reference position (Xrefdistance,Yrefdistance) of the robot.
                  Xrefdistance=x.at(115);
                  Yrefdistance=x.at(116);
                }
                Ttime++;
                Ttime2++;
              }
              /*** End of the procedure "calculation of speed" **/


              //1) Neural preprocessing-----------
              std::vector <vector<double> > x_prep = preprocessinglearning.step_npp(x);


              //2) Neural locomotion control------

              for (unsigned int i=0;i<6;i++)
              {
              //  NeighbourCpg_phase_phi[i]=NLCAC[i]->cpg_Activity;
               NLCAC[i]->calculate(x,AmosIISensorNames(i+R0_fs),NLCAC[(i+3)%6],i,NLCAC,NLCAC[(i+3)%6]->cpg_Activity.at(0),NLCAC[(i+3)%6]->cpg_Activity.at(1),y_);//R0_fs:Right front foot sensor
                y.at(i) = NLCAC[i]->step_nlc(x, x_prep,/*Footinhibition = false, true*/false);
              }

             /** robotId is used to distinguish between the fore robot (robotId=1)
              * and the rear robot (robotId=2).
              * twoconnectedRobots is boolean variable indicates whether there is two connected robots
              * for a collaborative task. In this case the the joints of the hind legs of the fore robot and
              * the joints of the fore legs of the rear robot are adjusted to be suitable for the  collaborative task. See the following figure.
              *
              *   \__|__ _ _ __|__/
              *   |_____|_ _|_____|
              *   /   |        |  \
             */
              if((robotId==1) && twoconnectedRobots)
                   {
                    for(unsigned int j=0; j<6;j++)//j:index of CPGs 0->R0 1->R1 2->R2 3->l0 4->l1 5->l2
                             {

								  if (j==2)
								  {
									y_[TR2_m] = -1.5;
									y_[CR2_m] =  1.57;
									y_[FR2_m] =  -1.57;

								  }
								  else if (j==5)
								  {
									y_[TL2_m] = -1.5;
									y_[CL2_m] =  1.57;
									y_[FL2_m] =  -1.57;
								  }
								  else if((j!=2) || (j!=5) )
								  {
									 for(unsigned k=0; k< 3; k++)
										 y_[j+6*k] = y.at(j).at(j+6*k);
								  }

                              }
                     }
               else if ((robotId==2) && twoconnectedRobots)
                    {
                            for(unsigned int j=0; j<6;j++)//j:index of cpgs 0->R0 1->R1 2->R2 3->l0 4->l1 5->l2
                            {
								  if (j==0)
									  {
										y_[TR0_m] = 1.8;
										y_[CR0_m] =  1.57;
										y_[FR0_m] =  -1.57;

									  }
								  else if (j==3)
									  {
										y_[TL0_m] = 1.8;
										y_[CL0_m] =  1.57;
										y_[FL0_m] =  -1.57;

									  }
								  else  if((j!=0) || (j!=3 ))
									   {
									  	for(unsigned k=0; k< 3; k++)//index of angel joints
											 y_[j+6*k] = y.at(j).at(j+6*k);
									   }
                            }


                     }
                else
                     {
                        for(unsigned int j=0; j<6;j++)
							 for(unsigned k=0; k< 3; k++)//index of angel joints
									  y_[j+6*k] = y.at(j).at(j+6*k);



                     }
              /** modifying the joint angles of the front legs to be suitable for the pushing mission */

              if (pushingobject)
                   {
                         y_[TR0_m] = y_TR0_m;
                          y_[CR0_m] =  0.2;
                          y_[FR0_m] =  1.7;
                          y_[TL0_m] = y_TL0_m;
                          y_[CL0_m] =  0.2;
                          y_[FL0_m] =  1.7;
                         y_[BJ_m]=y.at(5).at(BJ_m);

                    }

              if(LegAmputation)
              {
            	  	  y_[TR1_m] = 0.;
					  y_[CR1_m] =  1.57;
					  y_[FR1_m] =  -1.57;

					  y_[TL1_m] = 0.;
					  y_[CL1_m] =  1.57;
					  y_[FL1_m] =  -1.57;
              }

				/** disable femur joints by setting their values to -1 or -0.5 */
							/*  y_[FR2_m] = -1;
							  y_[FR1_m] = -1;
							  y_[FR0_m] = -1;
							  y_[FL2_m] = -1;
							  y_[FL1_m] = -1;
							  y_[FL0_m] = -1;*/

							/*   y_[FR0_m] =- 0.5;
								 y_[FR1_m] = -0.5;
								 y_[FR2_m] = -0.5;
								 y_[FL0_m] = -0.5;
								 y_[FL1_m] = -0.5;
							    y_[FL2_m] = -0.5;*/
    };

    /// Performs one step without learning. Calculates motor commands from sensor inputs.
    virtual void stepNoLearning(const sensor* x_, int number_sensors,
        motor* y_, int number_motors){
      //Tripodgait for 18 DOF Hexapod

      assert(number_sensors >= 18);
      assert(number_motors >= 18);


      // generate motor commands
      // right rear coxa (knee) forward-backward joint (back is positive)

      y_[TR2_m] = 0;


      y_[CR2_m] = 0;
      y_[FR2_m] = 0;
      //left rear coxa (knee) forward-backward joint
      y_[TL2_m] = 0;
      y_[CL2_m] = 0;
      y_[FL2_m] = 0;
      //right mrobotIddle coxa (knee) forward-backward joint
      y_[TR1_m] = 0;
      y_[CR1_m] = 0;
      y_[FR1_m] = 0;
      //left mrobotIddle coxa (knee) forward-backward joint
      y_[TL1_m] = 0;
      y_[CL1_m] = 0;
      y_[FL1_m] = 0;
      //right front coxa (knee) forward-backward joint
      y_[TR0_m] = 0;
      y_[CR0_m] = 0;
      y_[FR0_m] = 0;
      //left front coxa (knee) forward-backward joint
      y_[TL0_m] = 0;
      y_[CL0_m] = 0;
      y_[FL0_m] = 0;
      // backbone joint
      y_[BJ_m] = 0;

      // update step counter
  //    t++;
    };

    /***** STOREABLE ****/
    /** stores the controller values to a given file. */
    virtual bool store(FILE* f) const {
      return true;
    };
    /** loads the controller values from a given file. */
    virtual bool restore(FILE* f){
      return true;
    };


  public:
    NeuralPreprocessingLearning preprocessinglearning;

  protected:
    bool pushingobject;
    double y_TR0_m;
      double y_TL0_m;
      bool twoconnectedRobots;
      int robotId;
    std::vector<sensor> x;
    std::vector< std::vector<double> > y;
    unsigned short numbersensors, numbermotors;

    unsigned short number_channels;
    static const int CPG_option=1;
    double CPG_frequency;
    int Ttime;
    int Ttime2;
    paramkey name;
    double outputH1;
    double outputH2;
    NeuralLocomotionControlAdaptiveClimbing * NLCAC[6];
   double  ControlInput[6];
    double CPG_previous;
    double NeighbourCpg_phase_phi[6];
    double robotSpeed;
    double Xdistance;
    double Ydistance;
    double Xrefdistance;
    double Yrefdistance;
    double sum;
    double counter;
    std::ofstream ofs;
    bool calculationofSpeed;
    bool LegAmputation;
};

#endif


