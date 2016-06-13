#include <math.h>
#include "controller.h"


using namespace matrix;
using namespace std;




//utility to draw outputs of the neurons
void modularNeuroController::updateGui(){

  R0_H0 = cpg->getCpgOut0(0);
  R0_H1 = cpg->getCpgOut1(0);
  R0_H2 = cpg->getCpgOut2(0);
  R0_Per = cpg->getP(0);

  R1_H0 = cpg->getCpgOut0(1);
  R1_H1 = cpg->getCpgOut1(1);
  R1_H2 = cpg->getCpgOut2(1);
  R1_Per = cpg->getP(1);


  L0_H0 = cpg->getCpgOut0(2);
  L0_H1 =cpg->getCpgOut1(2);
  L0_H2 =cpg->getCpgOut2(2);
  L0_Per =cpg->getP(2);

  L1_H0 = cpg->getCpgOut0(3);
  L1_H1 = cpg->getCpgOut1(3);
  L1_H2 = cpg->getCpgOut2(3);
  L1_Per = cpg->getP(3);

 

  omega0=cpg->getCpgFrequency(0);
  omega1=cpg->getCpgFrequency(1);
  omega2=cpg->getCpgFrequency(2);
  omega3=cpg->getCpgFrequency(3);

  

}




modularNeuroController::modularNeuroController():AbstractController("adaptivelocomotioncontroller", "$Id: controller.cpp,v 0.1 $"){
  initialize(2,true,false);

    //low pass filter to clean the feedback signals of the different legs,cut off frequency has been set to 0.3,
  //setting it to a frequency bigger than 0.4 results in too much sensitive adaptive oscillator's response
  joint_R0 = new lowPass_filter(0.3);
  joint_R1 = new lowPass_filter(0.3);
  joint_L0 = new lowPass_filter(0.3);
  joint_L1 = new lowPass_filter(0.3);

  
  R0_H0 = cpg->getCpgOut0(0);
  R0_H1 = cpg->getCpgOut1(0);
  R0_H2 = cpg->getCpgOut2(0);
  R0_Per = cpg->getP(0);

  R1_H0 = cpg->getCpgOut0(1);
  R1_H1 = cpg->getCpgOut1(1);
  R1_H2 = cpg->getCpgOut2(1);
  R1_Per = cpg->getP(1);


  L0_H0 = cpg->getCpgOut0(2);
  L0_H1 =cpg->getCpgOut1(2);
  L0_H2 =cpg->getCpgOut2(2);
  L0_Per =cpg->getP(2);

  L1_H0 = cpg->getCpgOut0(3);
  L1_H1 = cpg->getCpgOut1(3);
  L1_H2 = cpg->getCpgOut2(3);
  L1_Per = cpg->getP(3);

  

  omega0=cpg->getCpgFrequency(0);
  omega1=cpg->getCpgFrequency(1);
  omega2=cpg->getCpgFrequency(2);
  omega3=cpg->getCpgFrequency(3);



  addInspectableValue("R0_outputH0",&R0_H0,"R0_outputH0");
  addInspectableValue("R0_outputH1",&R0_H1,"R0_outputH1");
  addInspectableValue("R0_outputH2",&R0_H2,"R0_outputH2");
  addInspectableValue("R0_perturbation",&R0_Per,"R0_perturbation");

  addInspectableValue("R1_outputH0",&R1_H0,"R1_outputH0");
  addInspectableValue("R1_outputH1",&R1_H1,"R1_ooutputH1");
  addInspectableValue("R1_outputH2",&R1_H2,"R1_outputH2");
  addInspectableValue("R1_perturbation",&R1_Per,"R1_perturbation");

  addInspectableValue("L0_outputH0",&L0_H0,"L0_outputH0");
  addInspectableValue("L0_outputH1",&L0_H1,"L0_L0_ooutputH1");
  addInspectableValue("L0_outputH2",&L0_H2,"L0_outputH2");
  addInspectableValue("L0_perturbation",&L0_Per,"L0_perturbation");

  addInspectableValue("L1_outputH0",&L1_H0,"L1_outputH0");
  addInspectableValue("L1_outputH1",&L1_H1,"L1_ooutputH1");
  addInspectableValue("L1_outputH2",&L1_H2,"L1_outputH2");
  addInspectableValue("L1_perturbation",&L1_Per,"L1_perturbation");
     
  addInspectableValue("frequency_R0",&omega0,"frequency_R0");
  addInspectableValue("frequency_R1",&omega1,"frequency_R1");
  addInspectableValue("frequency_L0",&omega2,"frequency_L0");
  addInspectableValue("frequency_L1",&omega3,"frequency_L1");
}
//Constructor of the abstract controller
modularNeuroController::modularNeuroController(int lilDogtype,bool mCPGs,bool mMuscleModelisEnabled):AbstractController("adaptivelocomotioncontroller", "$Id: controller.cpp,v 0.1 $"){
  initialize(lilDogtype, mCPGs, mMuscleModelisEnabled);

 
  //low pass filter to clean the feedback signals of the different legs,cut off frequency has been set to 0.3,
  //setting it to a frequency bigger than 0.4 results in too much sensitive adaptive oscillator's response
  joint_R0 = new lowPass_filter(0.4);
  joint_R1 = new lowPass_filter(0.4);
  joint_L0 = new lowPass_filter(0.4);
  joint_L1 = new lowPass_filter(0.4);

  
  R0_H0 = cpg->getCpgOut0(0);
  R0_H1 = cpg->getCpgOut1(0);
  R0_H2 = cpg->getCpgOut2(0);
  R0_Per = cpg->getP(0);

  R1_H0 = cpg->getCpgOut0(1);
  R1_H1 = cpg->getCpgOut1(1);
  R1_H2 = cpg->getCpgOut2(1);
  R1_Per = cpg->getP(1);


  L0_H0 = cpg->getCpgOut0(2);
  L0_H1 =cpg->getCpgOut1(2);
  L0_H2 =cpg->getCpgOut2(2);
  L0_Per =cpg->getP(2);

  L1_H0 = cpg->getCpgOut0(3);
  L1_H1 = cpg->getCpgOut1(3);
  L1_H2 = cpg->getCpgOut2(3);
  L1_Per = cpg->getP(3);

  

  omega0=cpg->getCpgFrequency(0);
  omega1=cpg->getCpgFrequency(1);
  omega2=cpg->getCpgFrequency(2);
  omega3=cpg->getCpgFrequency(3);



  addInspectableValue("R0_outputH0",&R0_H0,"R0_outputH0");
  addInspectableValue("R0_outputH1",&R0_H1,"R0_outputH1");
  addInspectableValue("R0_outputH2",&R0_H2,"R0_outputH2");
  addInspectableValue("R0_perturbation",&R0_Per,"R0_perturbation");

  addInspectableValue("R1_outputH0",&R1_H0,"R1_outputH0");
  addInspectableValue("R1_outputH1",&R1_H1,"R1_ooutputH1");
  addInspectableValue("R1_outputH2",&R1_H2,"R1_outputH2");
  addInspectableValue("R1_perturbation",&R1_Per,"R1_perturbation");

  addInspectableValue("L0_outputH0",&L0_H0,"L0_outputH0");
  addInspectableValue("L0_outputH1",&L0_H1,"L0_L0_ooutputH1");
  addInspectableValue("L0_outputH2",&L0_H2,"L0_outputH2");
  addInspectableValue("L0_perturbation",&L0_Per,"L0_perturbation");

  addInspectableValue("L1_outputH0",&L1_H0,"L1_outputH0");
  addInspectableValue("L1_outputH1",&L1_H1,"L1_ooutputH1");
  addInspectableValue("L1_outputH2",&L1_H2,"L1_outputH2");
  addInspectableValue("L1_perturbation",&L1_Per,"L1_perturbation");
     
  addInspectableValue("frequency_R0",&omega0,"frequency_R0");
  addInspectableValue("frequency_R1",&omega1,"frequency_R1");
  addInspectableValue("frequency_L0",&omega2,"frequency_L0");
  addInspectableValue("frequency_L1",&omega3,"frequency_L1");
     
  
}

void modularNeuroController::initialize(int aAMOSversion,bool mCPGs,bool mMuscleModelisEnabled)
{ 
  
  I_l = 0.0;
  I_r = 0.0;
  I3 = 0.0;
  t = 0;
  //the second variable corresponds to the number of cpgs to create,i.e. fourlegs=4
  cpg = new ModularNeural(true,4);
  
}



void modularNeuroController::init(int sensornumber, int motornumber, RandGen* randGen) {
  numbersensors = sensornumber;
    numbermotors = motornumber;
    x.resize(sensornumber);
  //y.resize(AMOSII_MOTOR_MAX);

}

//implement controller here
void modularNeuroController::step(const sensor* x_, int number_sensors, motor* y_, int number_motors){

  assert(number_sensors == numbersensors);
    assert(number_motors == numbermotors);

  //0) Sensor inputs/scaling  ----------------


    for (unsigned int i = 0; i < LILDOG_SENSOR_MAX; i++) {
      x.at(i) = x_[i];
    }


    
  //filtering of the feedback signal
    feedback0 =joint_R0->update(x.at(1));
    feedback1 =joint_R1->update(x.at(4));
    feedback2 =joint_L0->update(x.at(7));
    feedback3 =joint_L1->update(x.at(10));
;
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //input perturbation to the adaptive oscillator the number is the ID of the single CPGs
    //front right ID 0
    //middle right ID 1
    //rear right ID 2
    //front left ID 3
    //middle left ID 4
    //rear left ID 5
    //the feedback corresponds to the filtered signal from the CT angle sensor for middle/hind legs and TC angle sensor for front legs.
    //Infact for the given dungbeetle simulation,the CT joint is responsible for forward/backward movement (middle/hind legs).
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  cpg->update(feedback0,0);    
  cpg->update(feedback1,1);
  cpg->update(feedback2,2);    
  cpg->update(feedback3,3);

  
    //left back
        y_[9]=0;//cpg->getCpgOut1(3);
        y_[10]=cpg->getCpgOut1(3);
        y_[11]=0;//cpg->getCpgOut1(3);

    //right back
        y_[3]=0;//cpg->getCpgOut1(1);
        y_[4]=cpg->getCpgOut1(1);
        y_[5]=0;//cpg->getCpgOut1(1);
    //top left
        y_[6]=0;//cpg->getCpgOut1(2);
        y_[7]=cpg->getCpgOut1(2);
        y_[8]=0;//cpg->getCpgOut1(2);

    //top right
        y_[0]=0;//cpg->getCpgOut1(0);
        y_[1]=cpg->getCpgOut1(0);
        y_[2]=0;//cpg->getCpgOut1(0);


        updateGui();
   
    // /***Don't touch****Set Motor outputs begin *******************/
    // for(unsigned int j=0; j<y_MCPGs.size();j++)
    //  for(unsigned k=0; k< 3; k++)//index of angel joints
    //      y_[j+6*k] = y_MCPGs.at(j).at(j+6*k);

  
    t++;
}

/** stores the controller values to a given file. */
bool modularNeuroController::store(FILE* f) const {
  return true;
}

/** loads the controller values from a given file. */
bool modularNeuroController::restore(FILE* f) {
  //  Configurable::parse(f);
  return true;
}