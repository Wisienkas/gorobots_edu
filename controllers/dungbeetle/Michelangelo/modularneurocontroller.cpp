#include <math.h>
#include "modularneurocontroller.h"

using namespace matrix;
using namespace std;





void modularNeuroController::updateGui(){
	if(!multiple){
	o0 = mod->getCpgOut0();
	o1 = mod->getCpgOut1();
	o2 = mod->getCpgOut2();
	omega = mod->getCpgFrequency();
	}else if(multiple){
	o0 = cpg_0->getCpgOut0();
	o1 = cpg_0->getCpgOut1();
	o2 = cpg_0->getCpgOut2();
	perturbation = cpg_0->getP();
	omega = cpg_0->getCpgFrequency();
	omega1 = cpg_1->getCpgFrequency();
	omega2 = cpg_3->getCpgFrequency();
	omega3 = cpg_4->getCpgFrequency();

	input = cpg_3->getCpgOut0();
    input1 =cpg_3->getCpgOut1();
    input2 =cpg_3->getCpgOut2();
    input3 =cpg_3->getP();

	L1_H0 = cpg_4->getCpgOut0();
	L1_H1 = cpg_4->getCpgOut1();
	L1_H2 = cpg_4->getCpgOut2();
	L1_Per = cpg_4->getP();
    
    L2_H0 = cpg_5->getCpgOut0();
	L2_H1 = cpg_5->getCpgOut1();
	L2_H2 = cpg_5->getCpgOut2();
	L2_Per = cpg_5->getP();

	R2_H0 = cpg_2->getCpgOut0();
	R2_H1 = cpg_2->getCpgOut1();
	R2_H2 = cpg_2->getCpgOut2();
	R2_Per = cpg_2->getP();


	R1_H0 = cpg_1->getCpgOut0();
	R1_H1 = cpg_1->getCpgOut1();
	R1_H2 = cpg_1->getCpgOut2();
	R1_Per = cpg_1->getP();
	}

}




modularNeuroController::modularNeuroController():AbstractController("modularNeuroController", "$Id: modularneurocontroller.cpp,v 0.1 $"){
	initialize(2,false,false);
}

modularNeuroController::modularNeuroController(int dungBeetletype,bool mCPGs,bool mMuscleModelisEnabled):AbstractController("modularNeuroController", "$Id: modularneurocontroller.cpp,v 0.1 $"){
	initialize(dungBeetletype, mCPGs, mMuscleModelisEnabled);
	if(!multiple){
	o0 = mod->getCpgOut0();
	o1 = mod->getCpgOut1();
	o2 = mod->getCpgOut2();
	omega = mod->getCpgFrequency();
	perturbation = mod->getP();
	 addInspectableValue("frequency",&omega,"frequency");
     addInspectableValue("VRN_LEFT",&pattern2TC,"VRN_LEFT");
     addInspectableValue("VRN_RIGHT",&pattern1TC,"VRN_RIGHT");
     addInspectableValue("PSN_PATTERN1",&pattern1CT,"PSN_PATTERN1");
     addInspectableValue("PSN_PATTERN2",&pattern1FT,"PSN_PATTERN2");

     addInspectableValue("outputH0",&o0,"outputH0");
	 addInspectableValue("outputH1",&o1,"outputH1");
     addInspectableValue("outputH2",&o2,"outputH2");
     addInspectableValue("perturbation",&perturbation,"perturbation");
	}else if(multiple){
    input = cpg_3->getCpgOut0();
    input1 =cpg_3->getCpgOut1();
    input2 =cpg_3->getCpgOut2();
    input3 =cpg_3->getP();
	o0 = cpg_0->getCpgOut0();
	o1 = cpg_0->getCpgOut1();
	o2 = cpg_0->getCpgOut2();
	perturbation = cpg_0->getP();

	omega = cpg_0->getCpgFrequency();
	omega1 = cpg_1->getCpgFrequency();
	omega2 = cpg_3->getCpgFrequency();
	omega3 = cpg_4->getCpgFrequency();

	L1_H0 = cpg_4->getCpgOut0();
	L1_H1 = cpg_4->getCpgOut1();
	L1_H2 = cpg_4->getCpgOut2();
	L1_Per = cpg_4->getP();

	L2_H0 = cpg_5->getCpgOut0();
	L2_H1 = cpg_5->getCpgOut1();
	L2_H2 = cpg_5->getCpgOut2();
	L2_Per = cpg_5->getP();


	R1_H0 = cpg_1->getCpgOut0();
	R1_H1 = cpg_1->getCpgOut1();
	R1_H2 = cpg_1->getCpgOut2();
	R1_Per = cpg_1->getP();

	R2_H0 = cpg_2->getCpgOut0();
	R2_H1 = cpg_2->getCpgOut1();
	R2_H2 = cpg_2->getCpgOut2();
	R2_Per = cpg_2->getP();

	 addInspectableValue("R0_outputH0",&o0,"R0_outputH0");
	 addInspectableValue("R0_outputH1",&o1,"R0_outputH1");
     addInspectableValue("R0_outputH2",&o2,"R0_outputH2");
     addInspectableValue("R0_perturbation",&perturbation,"R0_perturbation");
     addInspectableValue("frequency_R0",&omega,"frequency_R0");
     addInspectableValue("frequency_L0",&omega2,"frequency_L0");
     addInspectableValue("frequency_R1",&omega1,"frequency_R1");
     addInspectableValue("frequency_L1",&omega3,"frequency_L1");

	 addInspectableValue("L0_outputH0",&input,"L0_outputH0");
	 addInspectableValue("L0_outputH1",&input1,"L0_L0_ooutputH1");
     addInspectableValue("L0_outputH2",&input2,"L0_outputH2");
     addInspectableValue("L0_perturbation",&input3,"L0_perturbation");

     addInspectableValue("L1_outputH0",&L1_H0,"L1_outputH0");
	 addInspectableValue("L1_outputH1",&L1_H1,"L1_ooutputH1");
     addInspectableValue("L1_outputH2",&L1_H2,"L1_outputH2");
     addInspectableValue("L1_perturbation",&L1_Per,"L1_perturbation");

     addInspectableValue("R1_outputH0",&R1_H0,"R1_outputH0");
	 addInspectableValue("R1_outputH1",&R1_H1,"R1_ooutputH1");
     addInspectableValue("R1_outputH2",&R1_H2,"R1_outputH2");
     addInspectableValue("R1_perturbation",&R1_Per,"R1_perturbation");


     addInspectableValue("R2_outputH0",&R2_H0,"R2_outputH0");
	 addInspectableValue("R2_outputH1",&R2_H1,"R2_ooutputH1");
     addInspectableValue("R2_outputH2",&R2_H2,"R2_outputH2");
     addInspectableValue("R2_perturbation",&R2_Per,"R2_perturbation");


     addInspectableValue("L2_outputH0",&L2_H0,"L2_outputH0");
	 addInspectableValue("L2_outputH1",&L2_H1,"L2_ooutputH1");
     addInspectableValue("L2_outputH2",&L2_H2,"L2_outputH2");
     addInspectableValue("L2_perturbation",&L2_Per,"L2_perturbation");
	}



   

}

void modularNeuroController::initialize(int aAMOSversion,bool mCPGs,bool mMuscleModelisEnabled)
{	
	//multiple = mCPGs;
	I_l = 1.0;
	I_r = 1.0;
	I3 = 0.0;
	t = 0;
	if(!multiple)
		//mCPGs = false;
		mod = new ModularNeural();
	else if(multiple){
		//mCPGs = true;
	timer = 0;
	cpg_0 = new ModularNeural();
	cpg_1 = new ModularNeural();
	cpg_2 = new ModularNeural();
	cpg_3 = new ModularNeural();
	cpg_4 = new ModularNeural();
	cpg_5 = new ModularNeural();
	}
}

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


  	//Locomotion control
  	//std::cout<<"timer:"<<t<<std::endl;
  	if(!multiple)
  	{
	//update neurons output for visualization in the GUI
  		updateGui();
  	
  		mod->setInputNeuronInput(I_l,I_r,I3);
		mod->update(0);
	
	//right legs CT
		pattern1CT=mod->getPsnOutput(11);
		// if(pattern1CT > 0.1)
		// pattern1CT=0.1;
		// if(pattern1CT < -0.3)
		// pattern1CT=-0.3;	
	//left legs	 CT
	

	//if(I2 == 0.0)
		pattern1FT=mod->getPsnOutput(10);
		pattern2FT=-pattern1FT;

	//right legs TC
  		pattern1TC=mod->getVrnRightOut();
  		// if(pattern1TC < -0.3)
  		// 	pattern1TC=-0.3;
  		// if(pattern1TC > 0.3)
  		// 	pattern1TC=0.3;
  	//left legs	 TC
		pattern2TC=mod->getVrnLeftOut();
		// if(pattern2TC < -0.3)
		// 	pattern2TC=-0.3;
		// if(pattern2TC > 0.3)
		// 	pattern2TC=0.3;
		if(I2 == 0.0){
	  //left back
	  		y_[5]=-0.4;
	  		y_[11]=0;
	  		y_[17]=0;

	  //right back
	  		y_[2]=-0.4;
	  		y_[8]=0;
	 		 y_[14]=0;
	 //left middle
	 		 y_[4]=-0.4;
	 		 y_[10]=0;
	 		 y_[16]=0;

	  //right middle
	  		y_[1]=-0.4;
	  		y_[7]=0;
	  		y_[13]=0;

	  	//front legs have opposite direction : - is down,+ is up
	  //top left
	  		y_[3]=0;
	  		y_[9]=0;
	  		y_[15]=0;

	  //top right
	  		y_[0]=0;
	  		y_[6]=0;
	  		y_[12]=0;
	
	   		y_[18]=0;
		}else if(I2 == 1.0){

	
	  //left back
	  		y_[5]=-0.4-pattern1CT*0.7;
	  		y_[11]=-pattern2TC*0.4;
	  		y_[17]=0;

	  //right back
	  		y_[2]=-0.4+pattern1CT*0.7;
	  		y_[8]=pattern1TC*0.4;
	  		y_[14]=0;

	  //left middle
	  		y_[4]=-0.4+pattern1CT*0.8;
	  		y_[10]=pattern2TC*0.4;
	  		y_[16]=0;

	  //right middle
	  		y_[1]=-0.4-pattern1CT*0.8;
	  		y_[7]=-pattern1TC*0.4;
	  		y_[13]=0;


	
	  //top left
	  		y_[3]=pattern2TC*0.1;
	  		y_[9]=pattern1CT*1.1;
	  		y_[15]=-pattern1FT;

	  //top right
	  		y_[0]=-pattern1TC*0.1;
	  		y_[6]=-pattern1CT*1.1;
	  		y_[12]=-pattern2FT-0.5;

	  //backbone joint
	    	y_[18]=0;
	    }

}else if(multiple && I2 == 0.0)
{		updateGui();
		//R0
	    cpg_0->setInputNeuronInput(I_l,I_r,I3);
		cpg_0->update(x.at(0)/1.4);

		cpg_3->setInputNeuronInput(I_l,I_r,I3);
		cpg_3->update(x.at(3)/1.4);
		cpg_1->setInputNeuronInput(I_l,I_r,I3);
		cpg_1->update(x.at(7)/1.2);
		cpg_2->setInputNeuronInput(I_l,I_r,I3);
		cpg_2->update(x.at(8)/1.2);
		cpg_4->setInputNeuronInput(I_l,I_r,I3);
		cpg_4->update(x.at(10)/1.2);
		cpg_5->setInputNeuronInput(I_l,I_r,I3);
		cpg_5->update(x.at(11)/1.2);
		// else{
		// cpg_0->setInputNeuronInput(I_l,I_r,I3);
		// cpg_0->update(cpg_0->getCpgOut1());
		// cpg_3->setInputNeuronInput(I_l,I_r,I3);
		// cpg_3->update(cpg_3->getCpgOut1());
		// cpg_1->setInputNeuronInput(I_l,I_r,I3);
		// cpg_1->update(cpg_1->getCpgOut1());
		// cpg_4->setInputNeuronInput(I_l,I_r,I3);
		// cpg_4->update(cpg_4->getCpgOut1());
		// }
		// cpg_5->setInputNeuronInput(I_l,I_r,I3);
		// cpg_5->update(filterJoint5->update(x.at(5)));


		//STANCE POSITION
	  //left back
	  		y_[5]=cpg_5->getCpgOut1()*1.4;
	  		y_[11]=cpg_5->getCpgOut1()*1.2;
	  		y_[17]=0;

	  //right back
	  		y_[2]=cpg_2->getCpgOut1()*1.4;
	  		y_[8]=cpg_2->getCpgOut1()*1.2;
	  		y_[14]=0;

	  //left middle
	  		y_[4]=-0.7;
	  		y_[10]=0;
	  		y_[16]=0;

	  //right middle
	  		y_[1]=-0.7;
	  		y_[7]=0;
	  		y_[13]=0;


	
	  //top left
	  		y_[3]=0;//input2;//
	  		y_[9]=0;//
	  		y_[15]=0;

	  //top right
	  		y_[0]=0;//-input;
	  		y_[6]=0;//-input;
	  		y_[12]=0;

// 	  //backbone joint
	    	y_[18]=0;
//   	//cout << t << " " << pattern1TC << " " << pattern1CT << " "<< pattern2TC << " " << pattern2CT << endl;

}  	
  	// /***Don't touch****Set Motor outputs begin *******************/
  	// for(unsigned int j=0; j<y_MCPGs.size();j++)
  	// 	for(unsigned k=0; k< 3; k++)//index of angel joints
  	//  		y_[j+6*k] = y_MCPGs.at(j).at(j+6*k);


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
