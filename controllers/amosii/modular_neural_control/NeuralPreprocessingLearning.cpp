/*
 * NeuralPreprocessingLearning.cpp
 *
 *  Created on: May 2, 2011
 *      Author: poramate
 *
 *      Edited by Dennis Goldschmidt
 *      Apr 27, 2012
 *
 *      Edited by Eduard Grinke
 *      May 10, 2012
 */

#include "NeuralPreprocessingLearning.h"
#include <iomanip>
#include <time.h>
///-------------------------------------------------------------------------------------------------------------------------

US_Obstacleavoidance::US_Obstacleavoidance(){
//2013 by Eduard Grinke, BA-Thesis

	//write OA values on the console if TRUE
	debug=0;

	//log files with proper date as name!!
	time_t rawtime;
	struct tm * timeinfo;
	char buffer [80];

	time (&rawtime);
	timeinfo = localtime (&rawtime);

	strftime (buffer,80,"OAValues_%F_%H-%M-%S.txt",timeinfo);
	puts (buffer);

	outOAValues.open(buffer);
	outTezinhib.open("outTezinhib.txt"); //Just to write a File which is always overwritten for fast evaluation of the OA values
	//end logfiles

	//Neuron 0 and 1 are input layer neurons, 2 and 3 are processing neurons. Neurons 5 and 6 are output layer.
	setNeuronNumber(6);
	setTransferFunction(getNeuron(0),identityFunction());
	setTransferFunction(getNeuron(1),identityFunction());
	setTransferFunction(getNeuron(4),identityFunction());
	setTransferFunction(getNeuron(5),identityFunction());

	//modes
	//mode=1 sum each recurrent and each inhibitory  weights;;
	//mode=2 sum only recurrent weights;;
	//mode=3 sum only inhibitory weights;
	//mode 4 learn recurrent, inhibitory weights static!!!
	//mode 5 learn inhibitory, recurrent weights static!!!
	//mode 6 full static MRC
	//mode 7 BRAITENBERG
	//mode 8 winner takes it all
	//mode=0 default: sum nothing
	mode=3;

	neuron_weightsum_inhib=0;
	//learning and forgetting parameters for adaptive MRC
	mu    =0.0065;
	gamma =0.0003;

	mu2   =0.015;
	gamma2=0.0003;

	//Reflex cut, below this limit the incoming signal is ignored
	reflex_cut=0.2;

	//Initialization of the input output layer
	u1=0;
	u2=0;
	u3=0;
	u4=0;
	v1=0;
	v2=0;
	v3=0;
	v4=0;

	//Initial synapse values of the adaptive MRC for neuron 2 and 3
	weight_neuron1=2;
	weight_neuron2=2;
	weight_neuron3=3;
	weight_neuron4=3;
	//Weight for input layer for neuron 0 and 1
	gain=4.7;

	//Threshold for cutting off output signals of neuron 2 and 3.
	e=0.009;


	//Scaling term in my tezlaff learning!
	//Negative will lead to a fixpoint for the synaptic weight at 0, so the agent forgets all learned weights
	vt = -0.01;
	vt2= -0.01;

	//For aborting the simulation after xxx steps, if set to Zero, the simulation will not abort and run continuously
	steps=0; //only count global running steps
	runsteps=0; //aborts the programm after xxx steps, if 0 the programm does not abort


	//Write 2 data streams
	outOAValues<<"#OA preset values: mode:"<<mode<<" gain:"<<gain<<"  vt:"<<vt<<"  vt2:"<<vt2<<"  mu:"<<mu<<"  gamma:"<<gamma<<"  mu2:"<<mu2<<"  gamma2:"<<gamma2<<"  weight_neuron1:"<<weight_neuron1<<"  weight_neuron2:"<<weight_neuron2<<"  weight_neuron3:"<<weight_neuron3<<"  weight_neuron4:"<<weight_neuron4<<" e:"<<e<<" reflex_cut:"<<reflex_cut<<endl;
	outOAValues<<"#runsteps<<" "<<u1<<" "<< v1 <<" "<<weight_neuron1<<" "<< ANN::getWeight(2,2)<<" "<<u2<<" " << v2 <<" "<<weight_neuron2<<" "<< ANN::getWeight(3,3)<<" "<<u3<<" " << v3 <<" "<<weight_neuron3<<" "<<ANN::getWeight(3,2)<<" "<<u4<<" " << v4 <<" "<<weight_neuron4<<" "<<ANN::getWeight(2,3)<<" "<<i1<<" "<<i2<<" "<<i1_refl<<" "<<i2_refl"<<endl;

	outTezinhib<<"#OA preset values: mode:"<<mode<<" gain:"<<gain<<"  vt:"<<vt<<"  vt2:"<<vt2<<"  mu:"<<mu<<"  gamma:"<<gamma<<"  mu2:"<<mu2<<"  gamma2:"<<gamma2<<"  weight_neuron1:"<<weight_neuron1<<"  weight_neuron2:"<<weight_neuron2<<"  weight_neuron3:"<<weight_neuron3<<"  weight_neuron4:"<<weight_neuron4<<" e:"<<e<<" reflex_cut:"<<reflex_cut<<endl;
	outTezinhib<<"#runsteps<<" "<<u1<<" "<< v1 <<" "<<weight_neuron1<<" "<< ANN::getWeight(2,2)<<" "<<u2<<" " << v2 <<" "<<weight_neuron2<<" "<< ANN::getWeight(3,3)<<" "<<u3<<" " << v3 <<" "<<weight_neuron3<<" "<<ANN::getWeight(3,2)<<" "<<u4<<" " << v4 <<" "<<weight_neuron4<<" "<<ANN::getWeight(2,3)<<" "<<i1<<" "<<i2<<" "<<i1_refl<<" "<<i2_refl"<<endl;


	//Initialize OA CPG like network
	w(2,0,gain);
	w(3,1,gain);
	w(4,2,	1);
	w(5,3,	1);
	//Produce first step, to avoid (stupid?) first oscillation of the OA values
	ANN::step();
}


void US_Obstacleavoidance::step_oa(){
//2013 by Eduard Grinke, BA-Thesis

	//Input conversion of neuron 0 and 1 to 0...1
	i1=0.5*(ANN::getOutput(0)+1);
	i2=0.5*(ANN::getOutput(1)+1);
	//Output conversion of neuron 2 and 3 to 0...1
	u1=0.5*(ANN::getOutput(2)+1);
	u2=0.5*(ANN::getOutput(3)+1);

	//Different "modes" the network can be connected. Described in the constructor above.
	switch (mode) {

	case 1:
		neuron_weightsum_inhib=(weight_neuron3+weight_neuron4)*0.5;
		//recurrent
		w(2, 2,  0.5*(weight_neuron1+weight_neuron2));
		w(3, 3,  0.5*(weight_neuron1+weight_neuron2));
		//inhibitory
		w(3, 2,  -neuron_weightsum_inhib);
		w(2, 3,  -neuron_weightsum_inhib);
		break;

	case 2:
		//recurrent
		w(2, 2,  0.5*(weight_neuron1+weight_neuron2));
		w(3, 3,  0.5*(weight_neuron1+weight_neuron2));
		//inhibitory
		w(3, 2,  -weight_neuron3);
		w(2, 3,  -weight_neuron4);
		break;

	case 3:
		neuron_weightsum_inhib=(weight_neuron3+weight_neuron4)*0.5;
		//recurrent
		w(2, 2,  weight_neuron1);
		w(3, 3,  weight_neuron2);
		//inhibitory
		w(3, 2,  -neuron_weightsum_inhib);
		w(2, 3,  -neuron_weightsum_inhib);
		break;

	case 4:
		//recurrent
		w(2, 2,  weight_neuron1);
		w(3, 3,  weight_neuron2);
		//inhibitory
		w(3, 2,  -3.5);
		w(2, 3,  -3.5);
		break;

	case 5:
		//recurrent
		w(2, 2,  2.4);
		w(3, 3,  2.4);
		//inhibitory
		w(3, 2,  -weight_neuron3);
		w(2, 3,  -weight_neuron4);
		break;

	case 6:
		//recurrent
		w(2, 2,  2.4);//2.4
		w(3, 3,  2.4);//2.4
		//inhibitory
		w(3, 2,  -3.5);//-3.5
		w(2, 3,  -3.5);//-3.5
		break;

	case 7:
		//recurrent
		w(2, 2,  0);
		w(3, 3,  0);
		//inhibitory
		w(3, 2,  0);
		w(2, 3,  0);
		break;

	case 8:
		//recurrent
		w(2, 2,  0);
		w(3, 3,  0);
		//inhibitory
		w(3, 2,  -weight_neuron3);
		w(2, 3,  -weight_neuron4);
		break;

	default:
		//recurrent
		w(2, 2,  weight_neuron1);
		w(3, 3,  weight_neuron2);
		//inhibitory
		w(3, 2,  -weight_neuron3);
		w(2, 3,  -weight_neuron4);
		break;
	}

	//Reflexcut for input signal
	if(i1>reflex_cut){i1_refl=1;} else{i1_refl=0;}
	if(i2>reflex_cut){i2_refl=1;} else{i2_refl=0;}
	//Cut-off for neurons 2 and 3 output at time "t"
	if(u1<e)u1=0;
	if(u2<e)u2=0;

	ANN::step();

	//Output of neuron 2 and 3 conversion to 0...1
	v1=0.5*(ANN::getOutput(2)+1);
	v2=0.5*(ANN::getOutput(3)+1);
	//Cut-off for neurons 2 and 3 output at time "t+1"
	if(v1<e)v1=0;
	if(v2<e)v2=0;

	//Applying tetzlaffs modified adaptive algorithm
	weight_neuron1+=mu *u1*v1*i1_refl+ gamma* (vt-v1)  *weight_neuron1* weight_neuron1;
	weight_neuron2+=mu *u2*v2*i2_refl+ gamma* (vt-v2)  *weight_neuron2* weight_neuron2;
	weight_neuron3+=mu2*u1*v1*i1_refl+ gamma2*(vt2-v1) *weight_neuron3* weight_neuron3;
	weight_neuron4+=mu2*u2*v2*i2_refl+ gamma2*(vt2-v2) *weight_neuron4* weight_neuron4;

	//OA values to Console output
	if(debug==1){
		cout<<setprecision(8)<<"i1:"<<i1<<" "<<"u1: "<<u1<<"\t v1: " << v1 <<"\t w1: "<< ANN::getWeight(2,2)<<endl;
		cout<<setprecision(8)<<"i2:"<<i2<<" "<<"u2: "<<u2<<"\t v2: " << v2 <<"\t w2: "<< ANN::getWeight(3,3)<<endl;
		cout<<setprecision(8)<<"u3: "<<u3<<"\t v3: " << v3 <<"\t w3: "<< ANN::getWeight(3,2)<<endl;
		cout<<setprecision(8)<<"u4: "<<u4<<"\t v4: " << v4 <<"\t w4: "<< ANN::getWeight(2,3)<<endl;
		cout<<"steps"<<steps<<endl;
	}

	outOAValues<<runsteps<<" "<<u1<<" "<< v1 <<" "<<weight_neuron1<<" "<< ANN::getWeight(2,2)<<" "<<u2<<" " << v2 <<" "<<weight_neuron2<<" "<< ANN::getWeight(3,3)<<" "<<u3<<" " << v3 <<" "<<weight_neuron3<<" "<<ANN::getWeight(3,2)<<" "<<u4<<" " << v4 <<" "<<weight_neuron4<<" "<<ANN::getWeight(2,3)<<" "<<i1<<" "<<i2<<" "<<i1_refl<<" "<<i2_refl<<endl;
	outTezinhib<<runsteps<<" "<<u1<<" "<< v1 <<" "<<weight_neuron1<<" "<< ANN::getWeight(2,2)<<" "<<u2<<" " << v2 <<" "<<weight_neuron2<<" "<< ANN::getWeight(3,3)<<" "<<u3<<" " << v3 <<" "<<weight_neuron3<<" "<<ANN::getWeight(3,2)<<" "<<u4<<" " << v4 <<" "<<weight_neuron4<<" "<<ANN::getWeight(2,3)<<" "<<i1<<" "<<i2<<" "<<i1_refl<<" "<<i2_refl<<endl;

	//Count steps for running simulation and abort if specified a maximum number for abort
	steps++;
	if(steps>runsteps && runsteps!=0) abort();

}

US_Obstacleavoidance::~US_Obstacleavoidance(){
	outTezinhib.close();
	outOAValues.close();
}
;

//1) Class for Neural preprocessing------------


NeuralPreprocessingLearning::NeuralPreprocessingLearning() {

	//Save files
	outFilenpp1.open("Neuralpreprocessing.txt");

	//---Set vector size----//
	mappingsensor.resize(AMOSII_SENSOR_MAX);// 111 sensors (Mar 29 2012)
	sensor_activity.resize(AMOSII_SENSOR_MAX);
	sensor_output.resize(AMOSII_SENSOR_MAX);

	preprosensor.resize(AMOSII_SENSOR_MAX);
	for(int i=0;i<AMOSII_SENSOR_MAX;i++){preprosensor[i].resize(2);}

	//preproobjvect.resize(AMOSII_SENSOR_MAX);
	//for(int i = 0;i < AMOSII_SENSOR_MAX;i++) preproobjvect[i].resize(2);

	ir_reflex_activity.resize(AMOSII_SENSOR_MAX);
	ir_predic_activity.resize(AMOSII_SENSOR_MAX);
	ir_reflex_output.resize(AMOSII_SENSOR_MAX);
	ir_predic_output.resize(AMOSII_SENSOR_MAX);
	ir_reflex_w.resize(AMOSII_SENSOR_MAX);
	ir_predic_w.resize(AMOSII_SENSOR_MAX);
	d_reflex_output.resize(AMOSII_SENSOR_MAX);
	rho1.resize(AMOSII_SENSOR_MAX);
	drho1.resize(AMOSII_SENSOR_MAX);
	us_delayline.resize(AMOSII_SENSOR_MAX);
	irs_delayline.resize(AMOSII_SENSOR_MAX);
	irlearn_output.resize(AMOSII_SENSOR_MAX);
	irlearn_output_prolong.resize(AMOSII_SENSOR_MAX);
	counter_fs.resize(AMOSII_SENSOR_MAX);
	dcounter_fs.resize(AMOSII_SENSOR_MAX);
	test_output.resize(5);

	//---Initialize your values

	lowpass = 0.1;
	sensor_w_pfs_rfs = 5.0;
	sensor_w_pfs_pfs = 2.0;//0.5;//1.0;//1.5;//2.0;
	irsensor_w_pfs_rfs = 1.0;//0.05;
	irsensor_w_pfs_pfs = 0.5;//1.0 - irsensor_w_pfs_rfs;
	threshold = 0.9;//0.7;
	ir_learnrate = 1.0;//0.35;//1.0//1.5 if > 0.10
	switchon_IRlearning = false;
	if (!switchon_IRlearning) {
		rho1.at(FR_us) = 2.0;
		rho1.at(FL_us) = 2.0;
	}
	delay = 250;
	delay_irs = 50;

	//Delayline constructor
	for (unsigned int i = FR_us; i < (FL_us + 1); i++) {
		us_delayline.at(i) = new Delayline(2 * delay + 1);
	}
	for (unsigned int i = R2_irs; i < (L0_irs + 1); i++) {
		irs_delayline.at(i) = new Delayline(2 * delay_irs + 1);
	}


	//Obstacleavoidance constructor
	OA = new US_Obstacleavoidance();

	//Include forn IR and set mix rate for IR and US signals
	FRONT_IR=false;
	rate=1;

}
;

NeuralPreprocessingLearning::~NeuralPreprocessingLearning() {

	//Save files
	outFilenpp1.close();

}
;

//1)  Step function of Neural preprocessing------------
std::vector< vector<double> > NeuralPreprocessingLearning::step_npp(const std::vector<double> in0) {


	//1)****Prepro Foot sensors for searching reflexes********

	for (unsigned int i = R0_fs; i < (L2_fs + 1); i++) {

		//Mapping foot sensor to +-1
		mappingsensor.at(i) = (((in0.at(i) - 1) / (0 - 1)) * 2.0 - 1.0);

		if (mappingsensor.at(i) > 0.9) {
			mappingsensor.at(i) = 1;
		}
		if (mappingsensor.at(i) <= 0.9) {
			mappingsensor.at(i) = -1;
		}

		//Preprocessing
		sensor_activity.at(i) = mappingsensor.at(i) * sensor_w_pfs_rfs + sensor_output.at(i) * sensor_w_pfs_pfs;//*presyFL3+biasFL3+ac_OutPostprocessFL3*recurrentFL3;
		sensor_output.at(i) = tanh(sensor_activity.at(i));
		preprosensor.at(i).at(0) = sensor_output.at(i);

	}


	// >> i/o operations here <<
	//outFilenpp1 << in0.at(L0_fs) << ' ' << in0.at(L1_fs) << ' ' << ' ' << in0.at(L2_fs) << endl;


	//3)****Prepro IR leg sensors for elevator reflexes********

	for (unsigned int i = R2_irs; i < (L0_irs + 1); i++) {
		if (in0.at(i) > 0.4) {
			sensor_activity.at(i) = 1.0;
		} else {
			sensor_activity.at(i) = 0.0;
		}
		sensor_output.at(i) = sensor_activity.at(i) * irsensor_w_pfs_rfs + sensor_output.at(i) * irsensor_w_pfs_pfs;

		if (sensor_output.at(i) > 0.05)
			preprosensor.at(i).at(0) = 1.0;
		else
			preprosensor.at(i).at(0) = 0.0;
	}

	//4)****Prepro Speed Sensors for landing reflexes**********

	for (unsigned int i = BX_spd; i < (BZ_spd + 1); i++) {
		preprosensor.at(i).at(0) = in0.at(i);
	}

	//5)****Prepro BJ angle sensors for BJC**********

	sensor_activity.at(BJ_as) = in0.at(BJ_as);
	sensor_output.at(BJ_as) = lowpass * sensor_activity.at(BJ_as) + (1.0 - lowpass) * sensor_output.at(BJ_as);
	preprosensor.at(BJ_as).at(0) = sensor_output.at(BJ_as);

	//6)****Learning US Sensors for BJC********

	//dividing into two signals

	for (unsigned int i = FR_us; i < (FL_us + 1); i++) {
		if (in0.at(i) < threshold) {
			ir_predic_activity.at(i) = /*(1.0/0.7)**/in0.at(i);//sensor_output.at(i); //early signal (CS)
			ir_reflex_activity.at(i) = 0.0; //late signal (US)
		} else {
			ir_predic_activity.at(i) = 0.0; //early signal (CS)
			ir_reflex_activity.at(i) = in0.at(i);//sensor_output.at(i); //late signal (US)
		}

		//2)****Prepro US sensors for BJC********

		//lowpass of both signals
		ir_predic_w.at(i) = 0.1;//0.05;
		ir_reflex_w.at(i) = 0.1;//0.05;

		//neural output (and derivative of reflex output)
		d_reflex_output.at(i) = -ir_reflex_output.at(i); //derivative 1st step

		ir_predic_output.at(i) = ir_predic_w.at(i) * ir_predic_activity.at(i) + (1.0 - ir_predic_w.at(i))
        		* ir_predic_output.at(i);
		ir_reflex_output.at(i) = ir_reflex_w.at(i) * ir_reflex_activity.at(i) + (1.0 - ir_reflex_w.at(i))
        		* ir_reflex_output.at(i);

		d_reflex_output.at(i) += ir_reflex_output.at(i); //derivative 2nd step

		//learning rule
		if (switchon_IRlearning) {
			if (d_reflex_output.at(i) > 0.0) {
				drho1.at(i) = ir_learnrate * ir_predic_output.at(i) * d_reflex_output.at(i);
			} else {
				drho1.at(i) = 0.0;
			}
			if (drho1.at(i) < 0.0) {
				drho1.at(i) = 0.0;
			}
			rho1.at(i) += drho1.at(i);
		} else {
			drho1.at(i) = 0.0;
			ir_reflex_output.at(i) = 0.0;
			//      rho1.at(i) = 2.0;
		}

		irlearn_output.at(i) = /*rho0 * irreflex_output.at(i)*/+rho1.at(i) * ir_predic_output.at(i); //first neuron
		irlearn_output_prolong.at(i) = 0.1 * irlearn_output.at(i) + (1.0 - 0.1) * irlearn_output_prolong.at(i); //prolongation via lowpass

		//TEST
		for (unsigned int j = 0; j < 5; j++) {
			test_output.at(j) = (j + 1) * 0.2 * ir_predic_output.at(25);
		}

		//learning neuron output
		us_delayline.at(i)->Write(irlearn_output_prolong.at(i));
		preprosensor.at(i).at(0) = 2.0 * us_delayline.at(i)->Read(delay);
		if (preprosensor.at(i).at(0) > 1.0) {
			preprosensor.at(i).at(0) = 1.0;
		}
		us_delayline.at(i)->Step();
	}

	for (unsigned int i = TR0_as; i < BJ_as; i++) {
		preprosensor.at(i).at(0) = in0.at(i);
	}



	//Eduard Grinke 2013 BA-Thesis
		//Obstacle avoidance, both sensors have the same network so I just use the FR_us
			if(FRONT_IR){
				sensor_output.at(FR_us)=(rate*in0.at(FR_us)+(1-rate)*in0.at(R0_irs));
				sensor_output.at(FL_us)=(rate*in0.at(FL_us)+(1-rate)*in0.at(L0_irs));
			}
			else{
				sensor_output.at(FL_us)=(in0.at(FL_us));
				sensor_output.at(FR_us)=(in0.at(FR_us));
			}
			//non crossed US config
			OA->setInput(0,sensor_output.at(FL_us)*2-1);
			OA->setInput(1,sensor_output.at(FR_us)*2-1);

			//Return output of OA to preprosensor
			preprosensor.at(FL_us).at(1)=(OA->ANN::getOutput(5));
			preprosensor.at(FR_us).at(1)=(OA->ANN::getOutput(4));

			//Run ANN step and update all neurons of OA network
			OA->step_oa();
		//Eduard Grinke ******************END



	//---------------------------//

	return preprosensor;

}
;

///-------------------------------------------------------------------------------------------------------------------------


