#include "aslcontroller.h"
#include <iomanip>
#include <math.h>  //exp for softmax
/**
 * Action-Sequence-Learning Controller for 
 * FourWheeldRPos_Gripper(Nimm4 with added sensors and gripper)
 * The controller gets a number of input sensor values each timestep
 * and has to generate a number of output motor values.
 *
 * Dominik Steven Weickgenannt (dowei14@student.sdu.dk 2015/2016)
 */
 
/**************************************
* defines the mode
* 0 = FSM
* 1 = Trigger + RNN
* 2 = Trigger + LSTM
* 3 = Trigger + ESN
* 4 = Trigger + DNF
* 5 = LSTM
************************************/
#define MODE 5

ASLController::ASLController(const std::string& name, const std::string& revision)
	: AbstractController(name, revision){

	// initialize parameters
	for (int i=0;i<number_ir_sensors;i++) irSmooth[i]=0;	
	dropBoxCounter = 0;
	haveTarget = false;
	prevHaveTarget = false;
	dropStuff = false;
	reset = false;
	counter = 0;	
	boxTouching = 0.00115;
	irFloorDistance = 0.5;
	irFrontClearDistance = 0.8;
	smoothingFactor = 1.0;
	state = -1;
	runNumber = 0;
	currentBox = 0;
	prevMotorLeft = 0;
	prevMotorRight = 0;
	sequenceCounter = 0;
	alreadyDone = false;

	// things for plotting
	parameter.resize(8);
	addInspectableValue("parameter1", &parameter.at(0),"parameter1");
	addInspectableValue("parameter2", &parameter.at(1),"parameter2");
	addInspectableValue("parameter3", &parameter.at(2),"parameter3");
	addInspectableValue("parameter4", &parameter.at(3),"parameter4");
	addInspectableValue("parameter5", &parameter.at(4),"parameter5");
	addInspectableValue("parameter6", &parameter.at(5),"parameter6");
	addInspectableValue("parameter7", &parameter.at(6),"parameter7");
	addInspectableValue("parameter8", &parameter.at(7),"parameter8");	
	
	
	// Action-Sequence-Learning Trigger NN
	aslt = new ASLT;
	asltf = new ASLTF;
	
	// RNN
	for (int i=0; i<8; i++){
		triggers[i] = 0.0;
		triggersUnfiltered[i] = 0.0;		
		neurons[i] = 0.0;
		neuronsPrev[i] = 0.0;
		weightsRecurrent[i] = 0.99;
	}
	weights[0]=1.0;	weights[1]=1.0;	weights[2]=1.0;	weights[3]=0.1;	weights[4]=0.1;	weights[5]=0.05;	weights[6]=0.2;	weights[7]=0.1;

	// lstm
	if (MODE ==2) lstmMode =2;
	else lstmMode =1;
	if (lstmMode ==1) lstm.setup(11,10,8, 1.0); // inputs - hidden - outputs - bias
	else lstm.setup(8,10,8, 1.0);

	std::string filename;
	if (lstmMode ==1) filename = "lstm_sensor_11-10-8.jsn"; // for mode 1
	else filename = "lstm_trigger_8-10-8.jsn"; // for mode 2b
	lstm.loadWeights(filename.c_str());
	
	// ESN
	esn = new ASLESN();
	esn->load();
	
	// DNF
	dnf = new DNF();
	int size = 90;
	double tau = 1.0;
	double h = -5.0;
	double beta = 4.0;
	dnf->setup(size,tau,h,beta);

	double sigmaExc 		= 5.0;
    double amplitudeExc 	= 50.0;
    double sigmaInh 		= 12.5;
    double amplitudeInh 	= 50.0;
    double amplitudeGlobal 	= 0.0;
	double cutoffFactor		= 5.0;
	dnf->setupLateral(sigmaExc,amplitudeExc,sigmaInh,amplitudeInh,amplitudeGlobal,cutoffFactor);

	int numStims = 8;
	double sigma_input = 2;
	for (int i=0;i<numStims;i++) {
		dnf->addStim((i+1)*10-1,sigma_input);
	}
	


}


/*************************************************************************************************
*** performs one step (includes learning).
*** Calculates motor commands from sensor inputs.
***   @param sensors sensors inputs scaled to [-1,1]
***   @param sensornumber length of the sensor array
***   @param motors motors outputs. MUST have enough space for motor values!
***   @param motornumber length of the provided motor array
*************************************************************************************************/
void ASLController::step(const sensor* sensors, int sensornumber,
      motor* motors, int motornumber){
      assert(number_sensors == sensornumber);
      assert(number_motors == motornumber);


	/*****************************************************************************************/
	// motors 0-4
	// motor 0 = left front motor
	// motor 1 = right front motor
	// motor 2 = left hind motor
	// motor 3 = right hind motor

	// sensors 0-3: wheel velocity of the corresponding wheel
	// sensor 0 = wheel velocity left front
	// sensor 1 = wheel velocity right front
	// sensor 2 = wheel velocity left hind
	// sensor 3 = wheel velocity right hind

	// sensors 4-9: IR Sensors
	// sensor 4 = front middle IR
	// sensor 5 = front middle top IR
	// sensor 6 = front right long range IR
	// sensor 7 = front left long range IR
	// sensor 8 = front right short range IR
	// sensor 9 = front left short range IR

	// sensors 10-33: distance to obstacles local coordinates (x,y,z)
	// sensor 10 = x direction to the first object (goal detection sensor)
	// sensor 11 = y direction to the first object (goal detection sensor)
	// sensor 12 = z direction to the first object (goal detection sensor)
      
	// 10-12 : Box 1		(0)
	// 13-15 : Box 2		(1)
	// 16-18 : Box 3		(2)
      
	/*****************************************************************************************/
	// add grippables
	vehicle->addGrippables(grippables);
	
	// calculate relative distances and angles from sensor value, normalized to 0..1 for distance and -1..1 for angle
	calculateDistanceToGoals(sensors);
	calculateAnglePositionFromSensors(sensors);
			
	// smooth ir sensors
//	for (int i=0;i<number_ir_sensors;i++) irSmooth[i] += (sensors[4+i]-irSmooth[i])/smoothingFactor;
	for (int i=0;i<number_ir_sensors;i++) irSmooth[i] = sensors[4+i];



/********************************************************************************************
*** store previous values
********************************************************************************************/

	prevMotorLeft = motors[0];
	prevMotorRight = motors[1];
	prevState = state;
	prevHaveTarget = haveTarget;
	
/********************************************************************************************
*** set up sensors and parameters
********************************************************************************************/
	// if there is no target set target sensors accordingly	
	if (haveTarget) { 
		distanceCurrentBox = distances[currentBox];
		angleCurrentBox = angles[currentBox];
	} else {
		distanceCurrentBox = -1.0;
		angleCurrentBox = 0.0;
	}
	irLeftLong = irSmooth[3];
	irRightLong = irSmooth[2];
	irLeftShort = irSmooth[5];
	irRightShort = irSmooth[4];
	irFront = irSmooth[0];
	if (irSmooth[0] > 0.89) touchGripper = 1.0;
	else touchGripper = 0.0;
	parameter.at(0) = irLeftLong;
	parameter.at(1) = irRightLong;
	parameter.at(2) = irLeftShort;
	parameter.at(3) = irRightShort;	
	parameter.at(4) = touchGripper;
	parameter.at(5) = irFront;	
	parameter.at(6) = distanceCurrentBox;	
	parameter.at(7) = angleCurrentBox;			

	if (reset) resetParameters();	
		

	
/********************************************************************************************
*** run controller step
********************************************************************************************/	
	if (!reset && (counter > 5) && (state < 8)) {

		// FSM to update state
		if (MODE == 0) fsmStep(motors);
		
		// Learned triggers + hand designed RNN
		else if (MODE == 1) {
			calcTriggersFull();
			rnnStep(motors);
		}
		
		// LSTM (motors, mode)
		else if (MODE ==2){				
			calcTriggers();
			lstmStep(motors,lstmMode);
		}
		// learned triggers + trained ESN
		else if (MODE == 3){
			calcTriggersFull();
			esnStep(motors);
			if (!haveTarget) setTarget(haveTarget);
		}
		// learned triggers + dnf
		else if (MODE == 4){
			calcTriggersFull();
			dnfStep(motors);
		}

		// LSTM without Trigger
		else if (MODE == 5) lstmStep(motors,lstmMode);		
	
		// execute action based on current state of the system
		executeAction(motors);	
			
	} 
	counter++; // increase counter

	// capping motor speed at +- 1.0 - was an issue at some point
	for (int i=0;i<4;i++){
		if (motors[i]>1.0) motors[i]=1.0;
		if (motors[i]<-1.0) motors[i]=-1.0;	
	}

	/*** STOP FORREST
	for (int i=0;i<4;i++) motors[i]=0.0; // stop robot
	*/

	// store left and right motor values
	motorLeft = motors[0]; motorRight = motors[1]; 	

	// stops the simulation if the robot reaches outside or falls into and stores the results
	comparison();

};


void ASLController::stepNoLearning(const sensor* , int number_sensors,motor* , int number_motors){
	
};
/********************************************************************************************
*** reset paramters after a simulation finished
********************************************************************************************/
void ASLController::resetParameters(){
	haveTarget = false;
	prevHaveTarget = false;
	distanceCurrentBox = -1.0;
	angleCurrentBox = 0.0;	
	dropStuff = false;
	dropBoxCounter = 0;
	state = -1;
	counter = 0;
	
	// RNN reset
	for (int i=0; i<8; i++){
		triggers[i] = 0.0;
		neurons[i] = 0.0;
		neuronsPrev[i] = 0.0;
	}
	neurons[0]=1.0;
	
	// LSTM reset
	lstm.reset();
}

/********************************************************************************************
*** Calculate triggers with FF NN
********************************************************************************************/
void ASLController::calcTriggers(){
	// set inputs to the 8 Trigger NN
	aslt->getASLT0()->setInput(  0 , prevMotorLeft);
	aslt->getASLT0()->setInput(  1 , prevMotorRight);
	aslt->getASLT0()->setInput(  2 , distanceCurrentBox);
	aslt->getASLT0()->setInput(  3 , angleCurrentBox);
	aslt->getASLT0()->setInput(  4 , irLeftLong);
	aslt->getASLT0()->setInput(  5 , irRightLong);
	aslt->getASLT0()->setInput(  6 , irLeftShort);
	aslt->getASLT0()->setInput(  7 , irRightShort);
	aslt->getASLT0()->setInput(  8 , irFront);
	aslt->getASLT0()->setInput(  9 , touchGripper);
	aslt->getASLT0()->setInput( 10 , (double)prevHaveTarget);
	aslt->getASLT1()->setInput(  0 , prevMotorLeft);
	aslt->getASLT1()->setInput(  1 , prevMotorRight);
	aslt->getASLT1()->setInput(  2 , distanceCurrentBox);
	aslt->getASLT1()->setInput(  3 , angleCurrentBox);
	aslt->getASLT1()->setInput(  4 , irLeftLong);
	aslt->getASLT1()->setInput(  5 , irRightLong);
	aslt->getASLT1()->setInput(  6 , irLeftShort);
	aslt->getASLT1()->setInput(  7 , irRightShort);
	aslt->getASLT1()->setInput(  8 , irFront);
	aslt->getASLT1()->setInput(  9 , touchGripper);
	aslt->getASLT1()->setInput( 10 , (double)prevHaveTarget);
	aslt->getASLT2()->setInput(  0 , prevMotorLeft);
	aslt->getASLT2()->setInput(  1 , prevMotorRight);
	aslt->getASLT2()->setInput(  2 , distanceCurrentBox);
	aslt->getASLT2()->setInput(  3 , angleCurrentBox);
	aslt->getASLT2()->setInput(  4 , irLeftLong);
	aslt->getASLT2()->setInput(  5 , irRightLong);
	aslt->getASLT2()->setInput(  6 , irLeftShort);
	aslt->getASLT2()->setInput(  7 , irRightShort);
	aslt->getASLT2()->setInput(  8 , irFront);
	aslt->getASLT2()->setInput(  9 , touchGripper);
	aslt->getASLT2()->setInput( 10 , (double)prevHaveTarget);
	aslt->getASLT3()->setInput(  0 , prevMotorLeft);
	aslt->getASLT3()->setInput(  1 , prevMotorRight);
	aslt->getASLT3()->setInput(  2 , distanceCurrentBox);
	aslt->getASLT3()->setInput(  3 , angleCurrentBox);
	aslt->getASLT3()->setInput(  4 , irLeftLong);
	aslt->getASLT3()->setInput(  5 , irRightLong);
	aslt->getASLT3()->setInput(  6 , irLeftShort);
	aslt->getASLT3()->setInput(  7 , irRightShort);
	aslt->getASLT3()->setInput(  8 , irFront);
	aslt->getASLT3()->setInput(  9 , touchGripper);
	aslt->getASLT3()->setInput( 10 , (double)prevHaveTarget);
	aslt->getASLT4()->setInput(  0 , prevMotorLeft);
	aslt->getASLT4()->setInput(  1 , prevMotorRight);
	aslt->getASLT4()->setInput(  2 , distanceCurrentBox);
	aslt->getASLT4()->setInput(  3 , angleCurrentBox);
	aslt->getASLT4()->setInput(  4 , irLeftLong);
	aslt->getASLT4()->setInput(  5 , irRightLong);
	aslt->getASLT4()->setInput(  6 , irLeftShort);
	aslt->getASLT4()->setInput(  7 , irRightShort);
	aslt->getASLT4()->setInput(  8 , irFront);
	aslt->getASLT4()->setInput(  9 , touchGripper);
	aslt->getASLT4()->setInput( 10 , (double)prevHaveTarget);
	aslt->getASLT5()->setInput(  0 , prevMotorLeft);
	aslt->getASLT5()->setInput(  1 , prevMotorRight);
	aslt->getASLT5()->setInput(  2 , distanceCurrentBox);
	aslt->getASLT5()->setInput(  3 , angleCurrentBox);
	aslt->getASLT5()->setInput(  4 , irLeftLong);
	aslt->getASLT5()->setInput(  5 , irRightLong);
	aslt->getASLT5()->setInput(  6 , irLeftShort);
	aslt->getASLT5()->setInput(  7 , irRightShort);
	aslt->getASLT5()->setInput(  8 , irFront);
	aslt->getASLT5()->setInput(  9 , touchGripper);
	aslt->getASLT5()->setInput( 10 , (double)prevHaveTarget);
	aslt->getASLT6()->setInput(  0 , prevMotorLeft);
	aslt->getASLT6()->setInput(  1 , prevMotorRight);
	aslt->getASLT6()->setInput(  2 , distanceCurrentBox);
	aslt->getASLT6()->setInput(  3 , angleCurrentBox);
	aslt->getASLT6()->setInput(  4 , irLeftLong);
	aslt->getASLT6()->setInput(  5 , irRightLong);
	aslt->getASLT6()->setInput(  6 , irLeftShort);
	aslt->getASLT6()->setInput(  7 , irRightShort);
	aslt->getASLT6()->setInput(  8 , irFront);
	aslt->getASLT6()->setInput(  9 , touchGripper);
	aslt->getASLT6()->setInput( 10 , (double)prevHaveTarget);
	aslt->getASLT7()->setInput(  0 , prevMotorLeft);
	aslt->getASLT7()->setInput(  1 , prevMotorRight);
	aslt->getASLT7()->setInput(  2 , distanceCurrentBox);
	aslt->getASLT7()->setInput(  3 , angleCurrentBox);
	aslt->getASLT7()->setInput(  4 , irLeftLong);
	aslt->getASLT7()->setInput(  5 , irRightLong);
	aslt->getASLT7()->setInput(  6 , irLeftShort);
	aslt->getASLT7()->setInput(  7 , irRightShort);
	aslt->getASLT7()->setInput(  8 , irFront);
	aslt->getASLT7()->setInput(  9 , touchGripper);
	aslt->getASLT7()->setInput( 10 , (double)prevHaveTarget);				

	// do FF step
	aslt->allSteps();
	
	// store outputs
	double val0 = aslt->getASLT0()->getOutput(16);
	double val1 = aslt->getASLT1()->getOutput(16);
	double val2 = aslt->getASLT2()->getOutput(16);
	double val3 = aslt->getASLT3()->getOutput(16);
	double val4 = aslt->getASLT4()->getOutput(16);
	double val5 = aslt->getASLT5()->getOutput(16);
	double val6 = aslt->getASLT6()->getOutput(16);
	double val7 = aslt->getASLT7()->getOutput(16);	
	
	// triggersUnfiltered is a work in progress, testing stuff with LSTM training
	triggersUnfiltered[0]=val0;	triggersUnfiltered[1]=val1;	triggersUnfiltered[2]=val2;	triggersUnfiltered[3]=val3;	
	triggersUnfiltered[4]=val4;triggersUnfiltered[5]=val5;	triggersUnfiltered[6]=val6;	triggersUnfiltered[7]=val7;
	
	// this is in essence a thresholding layer - will be added to the ASLT class later
	for (int i=0; i<8; i++)	triggers[i] = 0;
	if ((round(val0)>0) || (round(val1)>0) || (round(val2)>0) || (round(val3)>0) || (round(val4)>0) || (round(val5)>0) || (round(val6)>0) || (round(val7)>0)){
		//cout<<std::setprecision(5)<<val0<<" "<<val1<<" "<<val2<<" "<<val3<<" "<<val4<<" "<<val5<<" "<<val6<<" "<<val7<<endl;
		triggers[0]=val0;	triggers[1]=val1;	triggers[2]=val2;	triggers[3]=val3;	triggers[4]=val4;	triggers[5]=val5;	triggers[6]=val6;	triggers[7]=val7;
	}
}


/********************************************************************************************
*** Calculate triggers with FF NN all in 1 architecture
********************************************************************************************/
void ASLController::calcTriggersFull(){
	// set inputs to the 8 Trigger NN
	asltf->setInput(  0 , prevMotorLeft);
	asltf->setInput(  1 , prevMotorRight);
	asltf->setInput(  2 , distanceCurrentBox);
	asltf->setInput(  3 , angleCurrentBox);
	asltf->setInput(  4 , irLeftLong);
	asltf->setInput(  5 , irRightLong);
	asltf->setInput(  6 , irLeftShort);
	asltf->setInput(  7 , irRightShort);
	asltf->setInput(  8 , irFront);
	asltf->setInput(  9 , touchGripper);
	asltf->setInput( 10 , (double)prevHaveTarget);	

	// do FF step
	asltf->step();
	
	// store outputs
	for (int i=0;i<8;i++) {
		triggersUnfiltered[i] = asltf->getOutput(31+i);
	}
	
	
	// this is in essence a thresholding layer - will be added to the ASLT class later
	bool aboveThresh = false;
	for (int i=0; i<8; i++)	{
		triggers[i] = 0.0;
		if ( round(triggersUnfiltered[i]) > 0 ) aboveThresh = true;
	}
	if (aboveThresh){
		for (int i=0; i<8; i++)	
			if (MODE==4) {if (round(triggersUnfiltered[i]) > 0) triggers[i] = round(triggersUnfiltered[i]);}
			else triggers[i] = triggersUnfiltered[i];				
	}
	
}

/********************************************************************************************
*** trained LSTM to do a step
********************************************************************************************/
std::vector<float> ASLController::createLstmSensorVector(){
	std::vector<float> inputVector;
	inputVector.push_back(prevMotorLeft);
	inputVector.push_back(prevMotorRight);
	inputVector.push_back(distanceCurrentBox);
	inputVector.push_back(angleCurrentBox);
	inputVector.push_back(irLeftLong);
	inputVector.push_back(irRightLong);
	inputVector.push_back(irLeftShort);
	inputVector.push_back(irRightShort);
	inputVector.push_back(irFront);
	inputVector.push_back(touchGripper);
	if (prevHaveTarget) inputVector.push_back(1.0);
	else inputVector.push_back(0.0);
	return inputVector;
}
std::vector<float> ASLController::createLstmTriggerVector(){
	std::vector<float> inputVector;
	for (int i=0;i<8;i++) inputVector.push_back(triggers[i]);	
	return inputVector;
}

void ASLController::lstmStep(motor* motors, int modeIn){
	if (modeIn ==1) lstm.setInput(createLstmSensorVector());
	else lstm.setInput(createLstmTriggerVector());
	lstm.step();
	state=lstm.getState();						
}


/********************************************************************************************
*** trained ESN do a step
********************************************************************************************/
void ASLController::esnStep(motor* motors){
	std::vector<double> inputs;
	for (int i=0;i<8;i++) inputs.push_back(triggers[i]);
	std::vector<double> targets;
	for (int o=0;o<8;o++) targets.push_back(0.0);
	state = esn->RecurrentNetwork(inputs,targets, false);				
}

/********************************************************************************************
*** DNF do a step
********************************************************************************************/
int ASLController:: dnfCalcState(std::vector<double> inputVec){
	if (inputVec.size() != 90) {
		std::cout<<"--- Wrong Size ---"<<std::endl;
		return -1;
	}
	double max = -9999;
    int maxID = 0;
    for (int i=0; i<8;i++){    
        int start = 4+i*10;
        int stop  = 4+(i+1)*10+1;
		double currentSum = 0.0;
        for (int s=start;s<stop;s++) currentSum += inputVec[s];
        if (currentSum > max){
            max = currentSum;
            maxID = i;
		}
	}
    return maxID;
}


void ASLController::dnfStep(motor* motors){
	std::vector<double> inputs;
	for (int i=0;i<8;i++) inputs.push_back(triggers[i]*50.0);

	dnf->setAmplitudes(inputs);
	dnf->step();
	state = dnfCalcState(dnf->getOutput());
}


/********************************************************************************************
*** Hand designed RNN
********************************************************************************************/
void ASLController::rnnStep(motor* motors){

	/* 1 Layer RNN - weights are hand chosen to work with the trigger network
	and set up in the constructor of this controller 
	This is to be moved into a seperate class and implemented in the ANN Framework*/
	for (int i=0; i<8; i++){
		neuronsPrev[i] = neurons[i];
		neurons[i] = (neuronsPrev[i] * weightsRecurrent[i]) + (weights[i]*triggers[i]);
	}
					
	// softmax layer
	int max = 0;
	float maxNum = 0;
	for (int i=0; i<8; i++) {
		if (neurons[i]>maxNum){
			max = i; maxNum = neurons[i];
		}
	}
	state = max;
	
	double sum = 0.0;;
	for (int i=0; i<8; i++) sum += exp(neurons[i]);
	for (int i=0; i<8; i++) softMax[i] = exp(neurons[i]) / sum;
}

/********************************************************************************************
*** Finite State Machine to determine next state
*** triggers here are for training purposes, they are not used in the actual control
********************************************************************************************/
void ASLController::fsmStep(motor* motors){

	// reset triggers to 0
	for (int i=0; i<8; i++)	triggers[i] = 0;

	// determine new state based on previous state and sensor values (FSM)
	if (state==-1) {
		triggers[0] = 1.0;
		state++;
	} else
	if (state==0) {
		if (haveTarget)	{
			state++;
			triggers[1] = 1.0;
		}
	} else if (state==1){
		if (touchGripper){
			state++;
			triggers[2] = 1.0;
		}
	} else if (state==2){
		if ( motorLeft < -0.5 ){
			if (touchGripper < 1){			
				state = 0;
				triggers[0] = 1.0;
			} else {
				state++;
				triggers[3] = 1.0;
			}	
		}
	} else if (state ==3){
		if (irLeftLong < irFloorDistance || irRightLong < irFloorDistance) {
			state++;
			triggers[4] = 1.0;
		}
	} else if (state ==4){
		if ((irLeftLong < irFloorDistance) && (irRightLong < irFloorDistance) && ((irLeftShort > irFloorDistance) || (irRightShort > irFloorDistance))) {
			state++;
			triggers[5] = 1.0;
		}
	} else if (state ==5){
		if (irFront < irFrontClearDistance){		
			triggers[6] = 1.0;
			state++;
		}
	} else if (state ==6){
		if (irFront > irFrontClearDistance){
			triggers[7] = 1;
			state++;
		}
	} 
}

/********************************************************************************************
*** execute action based on state
********************************************************************************************/
void ASLController::executeAction(motor* motors){
	// execute action
	if (state==0) {
		setTarget(haveTarget);
	} else if (state==1){
		goToRandomBox(distanceCurrentBox,angleCurrentBox,motors);
	} else if (state==2){
		testBox(distanceCurrentBox,motors, haveTarget);
	} else if (state==3){
		moveToEdge(irLeftLong,irRightLong,motors);
	} else if (state==4){
		orientAtEdge(irLeftLong,irRightLong,irLeftShort,irRightShort,motors);
	} else if (state==5){
		dropBox(vehicle, dropBoxCounter, dropStuff,motors);
	} else if (state==6){
		crossGap(motors);
	} else if (state==7){
		//reset = true;
		//runNumber++;
		//state=-1;
	}
}

/*****************************************************************************************
*** Behaviours
*****************************************************************************************/
void ASLController::setTarget(bool& haveTarget){
	// aint nobody got time for dat
	if (counter < 1000){
		std::random_device rd; // obtain a random number from hardware
		std::mt19937 eng(rd()); // seed the generator
		std::uniform_int_distribution<> distr(0, 2); // define the range
		currentBox = distr(eng);
	} else {
		currentBox = 0;
	}	
	haveTarget = true;
//	cout<<"getting new Target: "<<currentBox<<endl;
}

void ASLController::goToRandomBox(double boxDistance, double boxAngle, motor* motors)
{
	double left,right;
	left = 0.5 + boxAngle;
	right = 0.5 - boxAngle;
	motors[0]=left; motors[2] = left;
	motors[1]=right; motors[3] = right;
}

void ASLController::testBox(double boxDistance, motor* motors, bool& haveTarget){
	double speed;
	haveTarget = false; // remove target
	
	// stop and accelerate backwards
	if(motors[0] >0) speed = 0;
	else speed = motors[0] - 0.005;
	motors[0]=speed; motors[2] = speed;
	motors[1]=speed; motors[3] = speed;
}

void ASLController::moveToEdge(double irLeft, double irRight, motor* motors){
	double left,right;
	
	left = 0.3; right = 0.3;

	motors[0]=left; motors[2] = left;
	motors[1]=right; motors[3] = right;
}

void ASLController::orientAtEdge(double irLeftLong, double irRightLong, double irLeftShort, double irRightShort, motor* motors){
	double left = 0.0;
	double right = 0.0;
	double threshold = 0.5;
	double speed = 0.05;
	if( (irLeftLong < threshold) && (irRightLong < threshold) && ((irLeftShort < threshold) || (irRightShort < threshold)) ){
		left = -speed*2; right = -speed*2; // overshoot
	} else if ( (irLeftLong > threshold) && (irRightLong > threshold) && (irLeftShort > threshold) && (irRightShort > threshold) ){
		left = speed;		right = speed; // undershoot
	} else if ( (irLeftLong > threshold) && (irRightLong < threshold) && (irLeftShort > threshold) && (irRightShort < threshold) ){
		left = speed;		right = -speed; // turn right
	} else if ( (irLeftLong < threshold) && (irRightLong > threshold) && (irLeftShort < threshold) && (irRightShort > threshold)){
		left = -speed;	right = speed; // turn left
	} else if ( (irLeftLong < threshold) && (irRightLong > threshold) && (irLeftShort > threshold) && (irRightShort > threshold)){
		left = -speed/2;	right = speed; // turn left with slower back movement
	} else if ( (irLeftLong > threshold) && (irRightLong < threshold) && (irLeftShort > threshold) && (irRightShort > threshold)){
		left = speed;	right = -speed/2; // turn right with slower back movement
	} 

	motors[0]=left; motors[2] = left;
	motors[1]=right; motors[3] = right;
}

void ASLController::dropBox(lpzrobots::FourWheeledRPosGripper* vehicle, int& dropBoxCounter, bool& dropStuff, motor* motors){
	double speed = 0.0;
	motors[0]=speed; motors[2] = speed;
	motors[1]=speed; motors[3] = speed;
	// the counter is for the robot to stop moving before dropping, otherwise it will fuck up from time to time
	// TODO: replace this with a "sensor based option"
	dropBoxCounter++;
	if (dropBoxCounter > 50) {
		dropStuff = true;
		vehicle->removeGrippables(grippables);
	}
}

void ASLController::crossGap(motor* motors){
	double speed = 0.5;
	motors[0]=speed; motors[2] = speed;
	motors[1]=speed; motors[3] = speed;
}


/*****************************************************************************************
*** Pass grippables and vehicle after reset
*****************************************************************************************/
void ASLController::setGrippablesAndVehicle(lpzrobots::FourWheeledRPosGripper* vehicleIn, std::vector<lpzrobots::Primitive*> grippablesIn){
	vehicle = vehicleIn;
	grippables = grippablesIn;
}

/*****************************************************************************************
*** Sensor Data => relative angle/distance
*****************************************************************************************/
//calculate distances
void ASLController::calculateDistanceToGoals(const sensor* x_)
{
	double distance_scale = 0.0005; // should normalize it to 0..1
	double distance;
	for (int i=0; i<number_relative_sensors; i++){
		distance = (sqrt(pow(x_[11+(i*3) ],2)+pow(x_[10+(i*3)],2)));
		distances[i] = distance * distance_scale;//85;//max_dis;
	}
}

//calculate relative angles
void ASLController::calculateAnglePositionFromSensors(const sensor* x_)
{
	int i=0;
	for (int counter=0; counter<number_relative_sensors; ++counter)
	{
		double alpha_value = 0;
		if (sign(x_[10 + i])>0)
			alpha_value = atan(x_[11 + i]/x_[10 + i]) * 180 / M_PI; // angle in degrees
		else {
			double alpha_value_tmp = -1*atan (x_[11 + i]/x_[10 + i]) * 180 / M_PI; // angle in degrees
			if (alpha_value_tmp<=0)
				alpha_value = (-90 + (-90-alpha_value_tmp));
			else alpha_value = ( 90 + ( 90-alpha_value_tmp));
		}
		angles[counter] = alpha_value/180.*M_PI / M_PI;
		i+=3;
	}
}

/*****************************************************************************************
*** Storing function used to create datasets
***
*** 	store 					-> standard without any seperation of data
*** 	storeTriggerBalance		-> seperates by trigger into + and - samples 
*** 	storeTransitionBalance	-> seperates by transitions into + and - samples
***		storebyState			-> seperates by state
***		storeSingleTrigger		-> the output is a single scalar, seperated in trainig and test set
*****************************************************************************************/
void ASLController::store(){
	// Open Files
	std::string in18name = "../data/in18.txt";
	in18.open (in18name.c_str(), ios::app);
	in18.precision(5);
	in18<<fixed;
	
	std::string in11name = "../data/in11.txt";
	in11.open (in11name.c_str(), ios::app);
	in11.precision(5);
	in11<<fixed;
	
	std::string in12name = "../data/in12.txt";
	in12.open (in12name.c_str(), ios::app);
	in12.precision(5);
	in12<<fixed;

	std::string out7name = "../data/out7.txt";	
	out7.open (out7name.c_str(), ios::app);
	out7.precision(5);
	out7<<fixed;

	std::string out1name = "../data/out1.txt";	
	out1.open (out1name.c_str(), ios::app);
	out1.precision(5);
	out1<<fixed;
	
	std::string outTname = "../data/outT.txt";	
	outT.open (outTname.c_str(), ios::app);
	outT.precision(5);
	outT<<fixed;
	
	// add binary states to in18 and out7
	for (int i=0;i<7;i++){
		if (i == prevState)	in18<<"1";
		else in18<<"0";
		in18<<" ";
	
		if (i == state)	out7<<"1";
		else out7<<"0";
		if (i<6) out7<<" ";
		if (i==6) out7<<"\n";		
	}

	// add scalar state to in12 and out1
	double multiplier = 0.1;
	in12<<prevState*multiplier<<" ";
	out1<<state*multiplier<<"\n";
	
	// add sensor values to all input files 
	in18<<prevMotorLeft<<" "<<prevMotorRight;	
	in18<<" ";
	in18<<distanceCurrentBox<<" "<<angleCurrentBox;
	in18<<" ";
	in18<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in18<<" ";
	if (prevHaveTarget) in18<<"1";
	else in18<<"0";
	in18<<"\n";

	in11<<prevMotorLeft<<" "<<prevMotorRight;	
	in11<<" ";
	in11<<distanceCurrentBox<<" "<<angleCurrentBox;
	in11<<" ";
	in11<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in11<<" ";
	if (prevHaveTarget) in11<<"1";
	else in11<<"0";
	in11<<"\n";

	in12<<prevMotorLeft<<" "<<prevMotorRight;	
	in12<<" ";
	in12<<distanceCurrentBox<<" "<<angleCurrentBox;
	in12<<" ";
	in12<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in12<<" ";
	if (prevHaveTarget) in12<<"1";
	else in12<<"0";
	in12<<"\n";
	
	
	// add triggers to outT
	for (int i=0; i<7; i++) {
		outT<<triggers[i]<<" ";
	}
	outT<<"\n";

	
	// close files	
  	in11.close();
  	in12.close();	
  	in18.close();	
	out1.close();
	out7.close();
	outT.close();
}

void ASLController::storeTriggerBalance(){
	int p = 0; // positive sample
	for (int i=0; i<8; i++) {
		if (triggers[i] > 0) p = 1;
	}

	// Open Files
	std::string in18name = "../data/T/TEST/in11.txt";
	in18.open (in18name.c_str(), ios::app);
	in18.precision(5);
	in18<<fixed;
	
	std::string in12name = "../data/T/TEST/outT.txt";
	in12.open (in12name.c_str(), ios::app);
	in12.precision(5);
	in12<<fixed;

	std::string in11name = "../data/T/" + std::to_string(p) + "/in11.txt";
	in11.open (in11name.c_str(), ios::app);
	in11.precision(5);
	in11<<fixed;
	


	std::string outTname = "../data/T/" + std::to_string(p) + "/outT.txt";	
	outT.open (outTname.c_str(), ios::app);
	outT.precision(5);
	outT<<fixed;

	in11<<prevMotorLeft<<" "<<prevMotorRight;	
	in11<<" ";
	in11<<distanceCurrentBox<<" "<<angleCurrentBox;
	in11<<" ";
	in11<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in11<<" ";
	if (prevHaveTarget) in11<<"1";
	else in11<<"0";
	in11<<"\n";
	
	in18<<prevMotorLeft<<" "<<prevMotorRight;	
	in18<<" ";
	in18<<distanceCurrentBox<<" "<<angleCurrentBox;
	in18<<" ";
	in18<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in18<<" ";
	if (prevHaveTarget) in18<<"1";
	else in18<<"0";
	in18<<"\n";
	
/*
	in12<<prevMotorLeft<<" "<<prevMotorRight;	
	in12<<" ";
	in12<<distanceCurrentBox<<" "<<angleCurrentBox;
	in12<<" ";
	in12<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in12<<" ";
	if (prevHaveTarget) in12<<"1";
	else in12<<"0";
	in12<<"\n";
*/	
	
	// add triggers to outT
	for (int i=0; i<8; i++) {
		outT<<triggers[i]<<" ";
		in12<<triggers[i]<<" ";		
	}
	outT<<"\n";
	in12<<"\n";



	// close files	
  	in11.close();
  	in12.close();	
  	in18.close();	
	outT.close();
}


void ASLController::storeTransitionBalance(){
	// training data
	int p = 0; // positive sample
	if (state != prevState) p =1;
	// Open Files
	std::string in11name = "../data/TRANS/" + std::to_string(p) + "/in11.txt";
	in11.open (in11name.c_str(), ios::app);
	in11.precision(5);
	in11<<fixed;

	std::string in12name = "../data/TRANS/" + std::to_string(p) + "/in12.txt";
	in12.open (in12name.c_str(), ios::app);
	in12.precision(5);
	in12<<fixed;
	
	std::string in18name = "../data/TRANS/" + std::to_string(p) + "/in18.txt";
	in18.open (in18name.c_str(), ios::app);
	in18.precision(5);
	in18<<fixed;
	
	std::string out7name = "../data/TRANS/" + std::to_string(p) + "/out7.txt";	
	out7.open (out7name.c_str(), ios::app);
	out7.precision(5);
	out7<<fixed;
	
	// add binary states to in18 and out7
	for (int i=0;i<7;i++){
		if (i == prevState)	in18<<"1";
		else in18<<"0";
		in18<<" ";
	
		if (i == state)	out7<<"1";
		else out7<<"0";
		if (i<6) out7<<" ";
		if (i==6) out7<<"\n";		
	}

	// add scalar state to in12 and out1
	double multiplier = 0.1;
	in12<<prevState*multiplier<<" ";
	
	// add sensor values to all input files 
	in18<<prevMotorLeft<<" "<<prevMotorRight;	
	in18<<" ";
	in18<<distanceCurrentBox<<" "<<angleCurrentBox;
	in18<<" ";
	in18<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in18<<" ";
	if (prevHaveTarget) in18<<"1";
	else in18<<"0";
	in18<<"\n";	
	
	in12<<prevMotorLeft<<" "<<prevMotorRight;	
	in12<<" ";
	in12<<distanceCurrentBox<<" "<<angleCurrentBox;
	in12<<" ";
	in12<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in12<<" ";
	if (prevHaveTarget) in12<<"1";
	else in12<<"0";
	in12<<"\n";	

	in11<<prevMotorLeft<<" "<<prevMotorRight;	
	in11<<" ";
	in11<<distanceCurrentBox<<" "<<angleCurrentBox;
	in11<<" ";
	in11<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in11<<" ";
	if (prevHaveTarget) in11<<"1";
	else in11<<"0";
	in11<<"\n";
	

	// close files
  	in18.close();  	
  	in12.close();   		
  	in11.close();
	out7.close();
	
	// testing data
	p = 0; // positive sample
	if (state != prevState) p =1;

	// Open Files
	in11name = "../data/TRANS/TEST/in11.txt";
	in11.open (in11name.c_str(), ios::app);
	in11.precision(5);
	in11<<fixed;
	
	in12name = "../data/TRANS/TEST/in12.txt";
	in12.open (in12name.c_str(), ios::app);
	in12.precision(5);
	in12<<fixed;
	
	in18name = "../data/TRANS/TEST/in18.txt";
	in18.open (in18name.c_str(), ios::app);
	in18.precision(5);
	in18<<fixed;	
	
	out7name = "../data/TRANS/TEST/out7.txt";	
	out7.open (out7name.c_str(), ios::app);
	out7.precision(5);
	out7<<fixed;

	// add binary states to in18 and out7
	for (int i=0;i<7;i++){
		if (i == prevState)	in18<<"1";
		else in18<<"0";
		in18<<" ";
	
		if (i == state)	out7<<"1";
		else out7<<"0";
		if (i<6) out7<<" ";
		if (i==6) out7<<"\n";		
	}

	// add scalar state to in12 and out1
	in12<<prevState*multiplier<<" ";
		
	in18<<prevMotorLeft<<" "<<prevMotorRight;	
	in18<<" ";
	in18<<distanceCurrentBox<<" "<<angleCurrentBox;
	in18<<" ";
	in18<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in18<<" ";
	if (prevHaveTarget) in18<<"1";
	else in18<<"0";
	in18<<"\n";	
	
	in12<<prevMotorLeft<<" "<<prevMotorRight;	
	in12<<" ";
	in12<<distanceCurrentBox<<" "<<angleCurrentBox;
	in12<<" ";
	in12<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in12<<" ";
	if (prevHaveTarget) in12<<"1";
	else in12<<"0";
	in12<<"\n";	
	
	in11<<prevMotorLeft<<" "<<prevMotorRight;	
	in11<<" ";
	in11<<distanceCurrentBox<<" "<<angleCurrentBox;
	in11<<" ";
	in11<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in11<<" ";
	if (prevHaveTarget) in11<<"1";
	else in11<<"0";
	in11<<"\n";
	

	// close files	
  	in18.close();  	
  	in12.close();   	
  	in11.close();
	out7.close();	
}


void ASLController::storebyState(){

	// Open Files
	std::string in18name = "../data/S/" + std::to_string(prevState) + "/in18.txt";
	in18.open (in18name.c_str(), ios::app);
	in18.precision(5);
	in18<<fixed;
	
	std::string in11name = "../data/S/" + std::to_string(prevState) + "/in11.txt";
	in11.open (in11name.c_str(), ios::app);
	in11.precision(5);
	in11<<fixed;
	
	std::string in12name = "../data/S/" + std::to_string(prevState) + "/in12.txt";
	in12.open (in12name.c_str(), ios::app);
	in12.precision(5);
	in12<<fixed;

	std::string out7name = "../data/S/" + std::to_string(prevState) + "/out7.txt";	
	out7.open (out7name.c_str(), ios::app);
	out7.precision(5);
	out7<<fixed;

	std::string out1name = "../data/S/" + std::to_string(prevState) + "/out1.txt";	
	out1.open (out1name.c_str(), ios::app);
	out1.precision(5);
	out1<<fixed;
	
	std::string outTname = "../data/S/" + std::to_string(prevState) + "/outT.txt";	
	outT.open (outTname.c_str(), ios::app);
	outT.precision(5);
	outT<<fixed;
	
	// add binary states to in18 and out7
	for (int i=0;i<7;i++){
		if (i == prevState)	in18<<"1";
		else in18<<"0";
		in18<<" ";
	
		if (i == state)	out7<<"1";
		else out7<<"0";
		if (i<6) out7<<" ";
		if (i==6) out7<<"\n";		
	}

	// add scalar state to in12 and out1
	double multiplier = 0.1;
	in12<<prevState*multiplier<<" ";
	out1<<state*multiplier<<"\n";
	
	// add sensor values to all input files 
	in18<<prevMotorLeft<<" "<<prevMotorRight;	
	in18<<" ";
	in18<<distanceCurrentBox<<" "<<angleCurrentBox;
	in18<<" ";
	in18<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in18<<" ";
	if (prevHaveTarget) in18<<"1";
	else in18<<"0";
	in18<<"\n";

	in11<<prevMotorLeft<<" "<<prevMotorRight;	
	in11<<" ";
	in11<<distanceCurrentBox<<" "<<angleCurrentBox;
	in11<<" ";
	in11<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in11<<" ";
	if (prevHaveTarget) in11<<"1";
	else in11<<"0";
	in11<<"\n";

	in12<<prevMotorLeft<<" "<<prevMotorRight;	
	in12<<" ";
	in12<<distanceCurrentBox<<" "<<angleCurrentBox;
	in12<<" ";
	in12<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in12<<" ";
	if (prevHaveTarget) in12<<"1";
	else in12<<"0";
	in12<<"\n";
	
	
	// add triggers to outT
	for (int i=0; i<7; i++) {
		outT<<triggers[i]<<" ";
	}
	outT<<"\n";

	// close files	
  	in11.close();
  	in12.close();	
  	in18.close();	
	out1.close();
	out7.close();
	outT.close();
}

void ASLController::storeSingleTrigger(int action){

	// training data
	int p = 0; // positive sample
	if (triggers[action] > 0) p = 1;

	// Open Files
	std::string in11name = "../data/ST/" + std::to_string(action) + "/" + std::to_string(p) + "/in11.txt";
	in11.open (in11name.c_str(), ios::app);
	in11.precision(5);
	in11<<fixed;

	std::string in12name = "../data/ST/" + std::to_string(action) + "/" + std::to_string(p) + "/in12.txt";
	in12.open (in12name.c_str(), ios::app);
	in12.precision(5);
	in12<<fixed;
	
	std::string in18name = "../data/ST/" + std::to_string(action) + "/" + std::to_string(p) + "/in18.txt";
	in18.open (in18name.c_str(), ios::app);
	in18.precision(5);
	in18<<fixed;
	
	std::string outTname = "../data/ST/" + std::to_string(action) + "/"  + std::to_string(p) + "/outT.txt";	
	outT.open (outTname.c_str(), ios::app);
	outT.precision(5);
	outT<<fixed;
	
	// add binary states to in18 and out7
	for (int i=0;i<7;i++){
		if (i == prevState)	in18<<"1";
		else in18<<"0";
		in18<<" ";		
	}

	// add scalar state to in12 and out1
	double multiplier = 0.1;
	in12<<prevState*multiplier<<" ";
	
	// add sensor values to all input files 
	in18<<prevMotorLeft<<" "<<prevMotorRight;	
	in18<<" ";
	in18<<distanceCurrentBox<<" "<<angleCurrentBox;
	in18<<" ";
	in18<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in18<<" ";
	if (prevHaveTarget) in18<<"1";
	else in18<<"0";
	in18<<"\n";	
	
	in12<<prevMotorLeft<<" "<<prevMotorRight;	
	in12<<" ";
	in12<<distanceCurrentBox<<" "<<angleCurrentBox;
	in12<<" ";
	in12<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in12<<" ";
	if (prevHaveTarget) in12<<"1";
	else in12<<"0";
	in12<<"\n";	

	in11<<prevMotorLeft<<" "<<prevMotorRight;	
	in11<<" ";
	in11<<distanceCurrentBox<<" "<<angleCurrentBox;
	in11<<" ";
	in11<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in11<<" ";
	if (prevHaveTarget) in11<<"1";
	else in11<<"0";
	in11<<"\n";
	
	// add triggers to outT
	outT<<triggers[action]<<" "<<"\n";

	// close files
  	in18.close();  	
  	in12.close();   		
  	in11.close();
	outT.close();
	
	// testing data
	p = 0; // positive sample
	if (triggers[action] > 0) p = 1;

	// Open Files
	in11name = "../data/ST/" + std::to_string(action) + "/TEST/in11.txt";
	in11.open (in11name.c_str(), ios::app);
	in11.precision(5);
	in11<<fixed;
	
	in12name = "../data/ST/" + std::to_string(action) + "/TEST/in12.txt";
	in12.open (in12name.c_str(), ios::app);
	in12.precision(5);
	in12<<fixed;
	
	in18name = "../data/ST/" + std::to_string(action) + "/TEST/in18.txt";
	in18.open (in18name.c_str(), ios::app);
	in18.precision(5);
	in18<<fixed;	
	
	outTname = "../data/ST/" + std::to_string(action) + "/TEST/outT.txt";	
	outT.open (outTname.c_str(), ios::app);
	outT.precision(5);
	outT<<fixed;

	// add binary states to in18 and out7
	for (int i=0;i<7;i++){
		if (i == prevState)	in18<<"1";
		else in18<<"0";
		in18<<" ";		
	}

	// add scalar state to in12 and out1
	in12<<prevState*multiplier<<" ";
		
	in18<<prevMotorLeft<<" "<<prevMotorRight;	
	in18<<" ";
	in18<<distanceCurrentBox<<" "<<angleCurrentBox;
	in18<<" ";
	in18<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in18<<" ";
	if (prevHaveTarget) in18<<"1";
	else in18<<"0";
	in18<<"\n";	
	
	in12<<prevMotorLeft<<" "<<prevMotorRight;	
	in12<<" ";
	in12<<distanceCurrentBox<<" "<<angleCurrentBox;
	in12<<" ";
	in12<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in12<<" ";
	if (prevHaveTarget) in12<<"1";
	else in12<<"0";
	in12<<"\n";	
	
	in11<<prevMotorLeft<<" "<<prevMotorRight;	
	in11<<" ";
	in11<<distanceCurrentBox<<" "<<angleCurrentBox;
	in11<<" ";
	in11<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in11<<" ";
	if (prevHaveTarget) in11<<"1";
	else in11<<"0";
	in11<<"\n";
	
	// add triggers to outT
	outT<<triggers[action]<<" "<<"\n";

	// close files	
  	in18.close();  	
  	in12.close();   	
  	in11.close();
	outT.close();	
}

void ASLController::storeTriggerAccuracy(bool fsm){

	std::string in11name;
	if (fsm) in11name = "../data/TriggerAccuracy/predicted.txt";
	else in11name = "../data/TriggerAccuracy/actual.txt"; 
	in11.open (in11name.c_str(), ios::app);
	in11.precision(5);
	in11<<fixed;

	// add triggers to outT
	for (int i=0; i<7; i++) {
		in11<<triggers[i]<<" ";
	}
	in11<<"\n";

	// close files	
  	in11.close();
}

void ASLController::storeRNN(){

	std::string in11name = "../data/RNN/triggers.txt";
	in11.open (in11name.c_str(), ios::app);
	in11.precision(5);
	in11<<fixed;

	// add triggers to outT
	for (int i=0; i<8; i++) {
		in11<<triggers[i]<<" ";
	}
	in11<<"\n";

	// close files	
  	in11.close();
  	
  	
	std::string in12name = "../data/RNN/neurons.txt";
	in12.open (in12name.c_str(), ios::app);
	in12.precision(5);
	in12<<fixed;

	// add triggers to outT
	for (int i=0; i<8; i++) {
		in12<<neurons[i]<<" ";
	}
	in12<<"\n";

	// close files	
  	in12.close();
  	
	std::string in18name = "../data/RNN/action.txt";
	in18.open (in18name.c_str(), ios::app);
	in18.precision(5);
	in18<<fixed;

	// add triggers to outT
	for (int i=0; i<8; i++) {
		if (i == state)	in18<<"1";
		else in18<<"0";
		if (i<7) in18<<" ";
	}
	in18<<"\n";

	// close files	
  	in18.close();  	
  	
	std::string out1name = "../data/RNN/softmax.txt";
	out1.open (out1name.c_str(), ios::app);
	out1.precision(5);
	out1<<fixed;

	// add triggers to outT
	for (int i=0; i<8; i++) {
		out1<<softMax[i]<<" ";
	}
	out1<<"\n";

	// close files	
  	out1.close(); 
}

void ASLController::storeLSTMTrain(){
	std::string in11name = "../data/LSTMTrain/sensors.txt";
	in11.open (in11name.c_str(), ios::app);
	in11.precision(5);
	in11<<fixed;

	in11<<prevMotorLeft<<" "<<prevMotorRight;	
	in11<<" ";
	in11<<distanceCurrentBox<<" "<<angleCurrentBox;
	in11<<" ";
	in11<<irLeftLong<<" "<<irRightLong<<" "<<irLeftShort<<" "<<irRightShort<<" "<<irFront<<" "<<touchGripper;
	in11<<" ";
	if (prevHaveTarget) in11<<"1";
	else in11<<"0";
	in11<<"\n";

	// close files	
  	in11.close();
  	
	in11name = "../data/LSTMTrain/triggerOut.txt";
	in11.open (in11name.c_str(), ios::app);
	in11.precision(5);
	in11<<fixed;

	// add triggers to outT
	for (int i=0; i<8; i++) {
		in11<<triggers[i]<<" ";
	}
	in11<<"\n";

	// close files	
  	in11.close();
  	
	in11name = "../data/LSTMTrain/triggerOutUnfiltered.txt";
	in11.open (in11name.c_str(), ios::app);
	in11.precision(5);
	in11<<fixed;

	// add triggers to outT
	for (int i=0; i<8; i++) {
		in11<<triggersUnfiltered[i]<<" ";
	}
	in11<<"\n";

	// close files	
  	in11.close();
  	
	in11name = "../data/LSTMTrain/class.txt";
	in11.open (in11name.c_str(), ios::app);
	in11.precision(5);
	in11<<fixed;

	in11<<state<<"\n";

	// close files	
  	in11.close();
 
 	sequenceCounter++;
	if (alreadyDone){
		in11name = "../data/LSTMTrain/seqTagsLength.txt";
		in11.open (in11name.c_str(), ios::app);
		in11.precision(5);
		in11<<fixed;

		in11<<runNumber<<" "<<sequenceCounter<<"\n";

		// close files	
		in11.close();
		sequenceCounter = 0;
	}
}

void ASLController::storeState(){
	std::string in11name = "../data/state.txt";
	in11.open (in11name.c_str(), ios::app);
	in11<<state<<"\n";
	in11.close();
}


void ASLController::comparison(){
	double xAbs = abs(vehicle->getPosition().x);
	double yAbs = abs(vehicle->getPosition().y);
	double z = vehicle->getPosition().z;
	if (reset && !alreadyDone){
		std::string in11name = "../results/comparison.txt";
		in11.open (in11name.c_str(), ios::app);
		in11<<MODE<<" 0"<<"\n";
		in11.close();
		std::cout<<"failure"<<std::endl;
	}		
	if (xAbs > 12.0 || yAbs > 12.0) {
		if (!alreadyDone) {
			std::cout<<"success"<<std::endl;
			std::string in11name = "../results/comparison.txt";
			in11.open (in11name.c_str(), ios::app);
			in11<<MODE<<" 1"<<"\n";
			in11.close();
		}
		alreadyDone = true;
		reset = true;				
	}
	if (z < 1.0 or counter > 20000){
		if (!alreadyDone) {
			std::string in11name = "../results/comparison.txt";
			in11.open (in11name.c_str(), ios::app);
			in11<<MODE<<" 0"<<"\n";
			in11.close();
			std::cout<<"failure"<<std::endl;
		}
		reset = true;			
		alreadyDone = true;
	}
}

