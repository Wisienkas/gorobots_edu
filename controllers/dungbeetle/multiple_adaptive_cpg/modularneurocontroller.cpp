#include <math.h>
#include "modularneurocontroller.h"
#include "utils/delayline.cpp"
<<<<<<< HEAD
#include <controllers/dungbeetle/adaptivecpg/shiftregister.cpp>
=======
#include <controllers/dungbeetle/hind_leg_control/adaptivecpg/shiftregister.cpp>
>>>>>>> d7e6b105acaa0fa52730375fd20873511d4c69c5

using namespace matrix;
using namespace std;




//utility to draw outputs of the neurons
void modularNeuroController::updateGui(){
	if(!mul_cpgs){

	o0 = cpg->getCpgOut0();
	o1 = cpg->getCpgOut1();
	o2 = cpg->getCpgOut2();
	omega = cpg->getCpgFrequency();
	
	}else if(mul_cpgs){

	R0_H0 = cpg->getCpgOut0(0);
	R0_H1 = cpg->getCpgOut1(0);
	R0_H2 = cpg->getCpgOut2(0);
	R0_Per = cpg->getP(0);

	R1_H0 = cpg->getCpgOut0(1);
	R1_H1 = cpg->getCpgOut1(1);
	R1_H2 = cpg->getCpgOut2(1);
	R1_Per = cpg->getP(1);

	R2_H0 =cpg->getCpgOut0(2);
	R2_H1 = cpg->getCpgOut1(2);
	R2_H2 = cpg->getCpgOut2(2);
	R2_Per = cpg->getP(2);

	L0_H0 = cpg->getCpgOut0(3);
    L0_H1 =cpg->getCpgOut1(3);
    L0_H2 =cpg->getCpgOut2(3);
    L0_Per =cpg->getP(3);

    L1_H0 = cpg->getCpgOut0(4);
	L1_H1 = cpg->getCpgOut1(4);
	L1_H2 = cpg->getCpgOut2(4);
	L1_Per = cpg->getP(4);

	L2_H0 = cpg->getCpgOut0(5);
	L2_H1 = cpg->getCpgOut1(5);
	L2_H2 = cpg->getCpgOut2(5);
	L2_Per = cpg->getP(5);
	

	omega0=cpg->getCpgFrequency(0);
	omega1=cpg->getCpgFrequency(1);
	omega2=cpg->getCpgFrequency(2);
	omega3=cpg->getCpgFrequency(3);
	omega4=cpg->getCpgFrequency(4);
	omega5=cpg->getCpgFrequency(5);
	}

}




modularNeuroController::modularNeuroController():AbstractController("modularNeuroController", "$Id: modularneurocontroller.cpp,v 0.1 $"){
	initialize(2,false,false);
}
//Constructor of the abstract controller
modularNeuroController::modularNeuroController(int dungBeetletype,bool mCPGs,bool mMuscleModelisEnabled):AbstractController("modularNeuroController", "$Id: modularneurocontroller.cpp,v 0.1 $"){
	initialize(dungBeetletype, mCPGs, mMuscleModelisEnabled);

    ///Single CPG Modular neuaral controller
	if(!mCPGs){

	o0 = cpg->getCpgOut0();
	o1 = cpg->getCpgOut1();
	o2 = cpg->getCpgOut2();
	omega = cpg->getCpgFrequency();
	perturbation = cpg->getP();

	addInspectableValue("frequency",&omega,"frequency");
    addInspectableValue("VRN_LEFT",&pattern2TC,"VRN_LEFT");
    addInspectableValue("VRN_RIGHT",&pattern1TC,"VRN_RIGHT");
    addInspectableValue("PSN_PATTERN1",&pattern1CT,"PSN_PATTERN1");
    addInspectableValue("PSN_PATTERN2",&pattern1FT,"PSN_PATTERN2");

    addInspectableValue("outputH0",&o0,"outputH0");
	addInspectableValue("outputH1",&o1,"outputH1");
    addInspectableValue("outputH2",&o2,"outputH2");
    addInspectableValue("perturbation",&perturbation,"perturbation");
	
	//Multiple CPG Modular Neural controller
	}else if(mCPGs){
	//low pass filter to clean the feedback signals of the different legs,cut off frequency has been set to 0.3,
	//setting it to a frequency bigger than 0.4 results in too much sensitive adaptive oscillator's response
	joint_R0 = new lowPass_filter(0.3);
	joint_R1 = new lowPass_filter(0.3);
	joint_R2 = new lowPass_filter(0.3);
	joint_L0 = new lowPass_filter(0.3);
	joint_L1 = new lowPass_filter(0.3);
	joint_L2 = new lowPass_filter(0.3);
	
	R0_H0 = cpg->getCpgOut0(0);
	R0_H1 = cpg->getCpgOut1(0);
	R0_H2 = cpg->getCpgOut2(0);
	R0_Per = cpg->getP(0);

	R1_H0 = cpg->getCpgOut0(1);
	R1_H1 = cpg->getCpgOut1(1);
	R1_H2 = cpg->getCpgOut2(1);
	R1_Per = cpg->getP(1);

	R2_H0 =cpg->getCpgOut0(2);
	R2_H1 = cpg->getCpgOut1(2);
	R2_H2 = cpg->getCpgOut2(2);
	R2_Per = cpg->getP(2);

	L0_H0 = cpg->getCpgOut0(3);
    L0_H1 =cpg->getCpgOut1(3);
    L0_H2 =cpg->getCpgOut2(3);
    L0_Per =cpg->getP(3);

    L1_H0 = cpg->getCpgOut0(4);
	L1_H1 = cpg->getCpgOut1(4);
	L1_H2 = cpg->getCpgOut2(4);
	L1_Per = cpg->getP(4);

	L2_H0 = cpg->getCpgOut0(5);
	L2_H1 = cpg->getCpgOut1(5);
	L2_H2 = cpg->getCpgOut2(5);
	L2_Per = cpg->getP(5);
	

	omega0=cpg->getCpgFrequency(0);
	omega1=cpg->getCpgFrequency(1);
	omega2=cpg->getCpgFrequency(2);
	omega3=cpg->getCpgFrequency(3);
	omega4=cpg->getCpgFrequency(4);
	omega5=cpg->getCpgFrequency(5);


	addInspectableValue("R0_outputH0",&R0_H0,"R0_outputH0");
	addInspectableValue("R0_outputH1",&R0_H1,"R0_outputH1");
    addInspectableValue("R0_outputH2",&R0_H2,"R0_outputH2");
    addInspectableValue("R0_perturbation",&R0_Per,"R0_perturbation");

    addInspectableValue("R1_outputH0",&R1_H0,"R1_outputH0");
	addInspectableValue("R1_outputH1",&R1_H1,"R1_ooutputH1");
    addInspectableValue("R1_outputH2",&R1_H2,"R1_outputH2");
    addInspectableValue("R1_perturbation",&R1_Per,"R1_perturbation");

 	addInspectableValue("R2_outputH0",&R2_H0,"R2_outputH0");
	addInspectableValue("R2_outputH1",&R2_H1,"R2_ooutputH1");
    addInspectableValue("R2_outputH2",&R2_H2,"R2_outputH2");
    addInspectableValue("R2_perturbation",&R2_Per,"R2_perturbation");

	addInspectableValue("L0_outputH0",&L0_H0,"L0_outputH0");
	addInspectableValue("L0_outputH1",&L0_H1,"L0_L0_ooutputH1");
    addInspectableValue("L0_outputH2",&L0_H2,"L0_outputH2");
    addInspectableValue("L0_perturbation",&L0_Per,"L0_perturbation");

    addInspectableValue("L1_outputH0",&L1_H0,"L1_outputH0");
	addInspectableValue("L1_outputH1",&L1_H1,"L1_ooutputH1");
    addInspectableValue("L1_outputH2",&L1_H2,"L1_outputH2");
    addInspectableValue("L1_perturbation",&L1_Per,"L1_perturbation");

	addInspectableValue("L2_outputH0",&L2_H0,"L2_outputH0");
	addInspectableValue("L2_outputH1",&L2_H1,"L2_ooutputH1");
    addInspectableValue("L2_outputH2",&L2_H2,"L2_outputH2");
    addInspectableValue("L2_perturbation",&L2_Per,"L2_perturbation");
     
    addInspectableValue("frequency_R0",&omega0,"frequency_R0");
    addInspectableValue("frequency_R1",&omega1,"frequency_R1");
    addInspectableValue("frequency_R2",&omega2,"frequency_R2");
    addInspectableValue("frequency_L0",&omega3,"frequency_L0");
    addInspectableValue("frequency_L1",&omega4,"frequency_L1");
    addInspectableValue("frequency_L2",&omega5,"frequency_L2");
     
	}
}

void modularNeuroController::initialize(int aAMOSversion,bool mCPGs,bool mMuscleModelisEnabled)
{	
	
	I_l = 0.0;
	I_r = 0.0;
	I3 = 0.0;
	t = 0;
	mul_cpgs=mCPGs;
	cpg = new ModularNeural(mCPGs,6);
	
}
//function to set the inputNeurons input according to the keyboard button,allows almost omnidirectional walking using the single CPG controller (amosII controller).
void getCommand(char key){
				switch(char(key)){
					case 'w':
					std::cout << "forward"<<std::endl;
					I_l = -1.0;
					I_r = -1.0;
					//I2 = 1.0;
					break;
					case 's':
					std::cout << "backward"<<std::endl;
					I_l = 1.0;
					I_r = 1.0;
					//I2 = 1.0;
					break;
					case 'q':
					std::cout << "left"<<std::endl;
					I_l = -1.0;
					I_r = 1.0;
					//I2 = 1.0;
					break;
					case 'e':
					std::cout << "right"<<std::endl;
					I_l = 1.0;
					I_r = -1.0;
					//I2 = 1.0;
					break;
					case 'z':
					std::cout << "stop"<<std::endl;
					I_l = 0.0;
					I_r = 0.0;
					//I2 = 1.0;
					break;					
					case 'a':
					std::cout << "I3=1.0"<<std::endl;
					I3 = 1.0;
					break;
					case 'd':
					std::cout << "I3=0.0"<<std::endl;
					I3 = 0.0;
					break;
					case 'i':
					std::cout << "I4=0.0 TF disinhibithed"<<std::endl;
					if(I2 == 1.0)
					I2 = 0.0;
					else if(I2 == 0.0)
					I2 = 1.0;	
					break;
				}
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


  	for (unsigned int i = 0; i < DUNGBEETLE_SENSOR_MAX; i++) {
    	x.at(i) = x_[i];
  	}


  	//Locomotion control Single CPG
  	if(!mul_cpgs)
  	{
	//update neurons output for visualization in the GUI
  		updateGui();
  	
  		cpg->setInputNeuronInput(I_l,I_r,I3);
		cpg->update();
	
	//CT Signal 
		pattern1CT=cpg->getPsnOutput(10);
		pattern2CT= -pattern1CT;
		pattern3CT= pattern1CT;
		pattern4CT= -pattern1CT;

	//FT Signal
		pattern1FT=cpg->getPsnOutput(10);
		pattern2FT=-pattern1FT;

	//TC Signal
  		pattern1TC=cpg->getVrnRightOut();
		pattern2TC=cpg->getVrnLeftOut();

		if(I2==1.0){
	
	  //left back
	  		y_[5]=(pattern2TC-0.2)*0.3;
	  		y_[11]=(pattern2CT+0.3)*0.8;
	  		y_[17]=0;//0;//-pattern1FT+0.4;

	  //right back
	  		y_[2]=(-pattern1TC-0.2)*0.3;
	  		y_[8]=(pattern1CT+0.3)*0.8;
	  		y_[14]=0;//pattern1FT+0.4;

	  //left middle
	  		y_[4]=(-pattern2TC-0.2)*0.3;
	  		y_[10]=(pattern1CT+0.3)*0.8;
	  		y_[16]=0;//pattern1FT+0.4;

	  //right middle
	  		y_[1]=(pattern1TC-0.2)*0.3;
	  		y_[7]=(pattern2CT+0.3)*0.8;
	  		y_[13]=0;//-pattern1FT+0.4;

	  //top left
	  		y_[3]=(pattern2TC+0.1)*0.3;
	  		y_[9]=-0.1+(pattern2CT-0.05)*0.6;
	  		y_[15]=0;//-pattern1FT+0.4;

	  //top right
	  		y_[0]=(-pattern1TC+0.1)*0.3;
	  		y_[6]=-0.1+(pattern1CT-0.05)*0.6;
	  		y_[12]=0;//pattern1FT+0.4;

	  //backbone joint
	    	y_[18]=0;
	    }
	    else{
	    		  //left back
	  		y_[5]=0;//(pattern2TC-0.2)*0.3;
	  		y_[11]=0;//(pattern2CT+0.3)*0.8;
	  		y_[17]=0;//0;//-pattern1FT+0.4;

	  //right back
	  		y_[2]=0;//(-pattern1TC-0.2)*0.3;
	  		y_[8]=0;//(pattern1CT+0.3)*0.8;
	  		y_[14]=0;//pattern1FT+0.4;

	  //left middle
	  		y_[4]=0;//(-pattern2TC-0.2)*0.3;
	  		y_[10]=0;//(pattern1CT+0.3)*0.8;
	  		y_[16]=0;//pattern1FT+0.4;

	  //right middle
	  		y_[1]=0;//(pattern1TC-0.2)*0.3;
	  		y_[7]=0;//(pattern2CT+0.3)*0.8;
	  		y_[13]=0;//-pattern1FT+0.4;

	  //top left
	  		y_[3]=-0.10;//(pattern2TC+0.1)*0.3;
	  		y_[9]=0;//-0.1+(pattern2CT-0.05)*0.6;
	  		y_[15]=0;//-pattern1FT+0.4;

	  //top right
	  		y_[0]=-0.1;//(0;//-pattern1TC+0.1)*0.3;
	  		y_[6]=0;//-0.1+(pattern1CT-0.05)*0.6;
	  		y_[12]=0;//pattern1FT+0.4;

	  //backbone joint
	    	y_[18]=0;
	    }
	}else if(mul_cpgs)
	{	
	//filtering of the feedback signal
	feedback0 =joint_R0->update(x.at(0));
    feedback1 =joint_R1->update(-x.at(7));
    feedback2 =joint_R2->update(-x.at(8));
    
    feedback3 =joint_L0->update(x.at(3));
    feedback4 =joint_L1->update(-x.at(10));
    feedback5 =joint_L2->update(-x.at(11));
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
	cpg->update(feedback4,4);    
	cpg->update(feedback5,5);
	
	  //left back
	  		y_[5]=(cpg->getPsnOutput(5,10)*1.2+0.25)-0.8;
	  		y_[11]=-cpg->getCpgOut1(5);
	  		y_[17]=-cpg->getCpgOut1(5);

	  //right back
	  		y_[2]=(cpg->getPsnOutput(2,10)*1.2+0.25)-0.8;
	  		y_[8]=-cpg->getCpgOut1(2);
	  		y_[14]=-cpg->getCpgOut1(2);
	 
	  //left middle
	  		y_[4]=(cpg->getPsnOutput(4,10)*1.2+0.25)-0.8;
	  		y_[10]=-cpg->getCpgOut1(4);
	  		y_[16]=-cpg->getCpgOut1(4);

	  //right middle
	  		y_[1]=(cpg->getPsnOutput(1,10)*1.2+0.25)-0.8;
	  		y_[7]=-cpg->getCpgOut1(1);
	  		y_[13]=-cpg->getCpgOut1(1);

	  //top left
	  		y_[3]=cpg->getCpgOut1(3);
	  		y_[9]=-cpg->getPsnOutput(3,10)*1.2-0.20;
	  		y_[15]=-cpg->getCpgOut1(3);

	  //top right
	  		y_[0]=cpg->getCpgOut1(0);
	  		y_[6]=-cpg->getPsnOutput(0,10)*1.2-0.20;
	  		y_[12]=-cpg->getCpgOut1(0);

// 	  //backbone joint
	    	y_[18]=0;

	    	updateGui();
		}  	
  	/***Don't touch****Set Motor outputs begin *******************/
  	for(unsigned int j=0; j<y_MCPGs.size();j++)
  		for(unsigned k=0; k< 3; k++)//index of angel joints
  	 		y_[j+6*k] = y_MCPGs.at(j).at(j+6*k);

	
  	t++;
}

/** stores the controller values to a given file. */
bool modularNeuroController::store(FILE* f) const {
  return true;
}

/** loads the controller values from a given file. */
bool modularNeuroController::restore(FILE* f) {
  //	Configurable::parse(f);
  return true;
}
