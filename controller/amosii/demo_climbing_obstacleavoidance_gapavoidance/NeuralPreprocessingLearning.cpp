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

///-------------------------------------------------------------------------------------------------------------------------

US_Obstacleavoidance::US_Obstacleavoidance(){

	setNeuronNumber(4);

	setTransferFunction(getNeuron(2),thresholdFunction());
	setTransferFunction(getNeuron(3),thresholdFunction());
	ThresholdFunction().setTheta(0.5);

	// synaptic weights
	w(0, 0,  2.4);//2.2
	w(0, 1,  -3.6);
	w(1, 0,  -3.6);
	w(1, 1,  2.4);//2.2

	w(3,0,      1.0);
	w(2,1,      1.0);
};

double US_Obstacleavoidance::getOutput(int i)
{
	if(ANN::getOutput(2)<0.05)
		setOutput(2,-1);
	else
		output.at(0)=ANN::getOutput(2);

	if(ANN::getOutput(3)<0.05)
		setOutput(3,-1);
	else
		output.at(1)=ANN::getOutput(3);

	return output.at(i);
}


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

	preproobjvect.resize(AMOSII_SENSOR_MAX);
	for(int i = 0;i < AMOSII_SENSOR_MAX;i++) preproobjvect[i].resize(2);

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


	//Neuronal Preprocessing Objects
	//for(int i = 0;i < AMOSII_SENSOR_MAX;i++){ if(i!=FR_us||FL_us){preproobjvect.at(i).at(0)        = new OneNeuron();}}
	//for(int i = 0;i < AMOSII_SENSOR_MAX;i++){preproobjvect.at(i).at(0) = new OneNeuron();}
	// preproobjvect.at(FR_us).at(0) = new BJC_ANN;
	preproobjvect.at(FR_us).at(1) = new US_Obstacleavoidance();


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



	//Eduard
	//Obstacle avoidance, both sensors have the same network so I just use the FR_us
	sensor_output.at(FR_us)=(in0.at(FR_us)*2.0-1.0);
	sensor_output.at(FL_us)=(in0.at(FL_us)*2.0-1.0);
	preproobjvect.at(FR_us).at(1)->setInput(0,4.3*sensor_output.at(FR_us));//4.1
	preproobjvect.at(FR_us).at(1)->setInput(1,4.3*sensor_output.at(FL_us));//4.1 is good
	preprosensor.at(FR_us).at(1)=preproobjvect.at(FR_us).at(1)->getOutput(1);
	preprosensor.at(FL_us).at(1)=preproobjvect.at(FR_us).at(1)->getOutput(0);

	// outFilenpp2 << sensor_output.at(FR_us) << ' ' << sensor_output.at(FL_us) << ' '<< preprosensor.at(FR_us).at(1) << ' '<<preprosensor.at(FL_us).at(1) <<' '<<preproobjvect.at(FR_us).at(1)->getOutput(0)<<' '<<preproobjvect.at(FR_us).at(1)->getOutput(1)<<  endl;
	//END obstacle-avoidance




	//Do the Timestep for every Neuron!
	// for(int i = 0;i < AMOSII_SENSOR_MAX;i++) preproobjvect.at(i).at(0)->step();
	preproobjvect.at(FR_us).at(1)->step();
	//Eduard END



	//---------------------------//

	return preprosensor;

}
;

///-------------------------------------------------------------------------------------------------------------------------


