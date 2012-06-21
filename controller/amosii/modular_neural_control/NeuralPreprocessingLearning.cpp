/*
 * NeuralPreprocessingLearning.cpp
 *
 *  Created on: May 2, 2011
 *      Author: poramate
 *
 *      Edited by Dennis Goldschmidt
 *      Apr 27, 2012
 */

#include "NeuralPreprocessingLearning.h"

///-------------------------------------------------------------------------------------------------------------------------

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
  switchon_IRlearning = true;
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

  //Neuronal Preprocessing
  sensorprepro_us = new SO2CPG();

  //set neural weights
  sensorprepro_us->setWeight(0, 0, 3.3);
  sensorprepro_us->setWeight(0, 1, -3.5);
  sensorprepro_us->setWeight(1, 0, -3.5);
  sensorprepro_us->setWeight(1, 1, 3.3);

}
;

NeuralPreprocessingLearning::~NeuralPreprocessingLearning() {

  //Save files
  outFilenpp1.close();

}
;

//1)  Step function of Neural preprocessing------------
std::vector< vector<double> > NeuralPreprocessingLearning::step_npp(const std::vector<double> in0) {

  //enum for new us sensor prepro
  enum {
    FR_us2 = 111, FL_us2 = 112,
  };

  //1)****Prepro Foot sensors for searching reflexes********

  for (unsigned int i = R0_fs; i < (L2_fs + 1); i++) {

    //    dcounter_fs.at(i) = -counter_fs.at(i);
    //    if(in0.at(i) < 0.6) {
    //      counter_fs.at(i) += 1;
    //    }
    //    else {
    //      counter_fs.at(i) = 0;
    //    }
    //    dcounter_fs.at(i) += counter_fs.at(i);
    //    if(dcounter_fs.at(i) < 0) {
    //      cout << i << "\t" << -dcounter_fs.at(i) << endl;
    //      if(i == L2_fs) {
    //        cout << endl;
    //      }
    //    }

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

    ////    //Mapping foot sensor to +-1
    //    mappingsensor.at(i) = (((in0.at(i) - 0) / (1 - 0)) * (-5.0) + 1.0); //amplify (e.g., 2.5) and invert signals (-1)
    //
    //    //Preprocessing
    //    sensor_activity.at(i) = mappingsensor.at(i) * sensor_w_pfs_rfs + sensor_output.at(i) * sensor_w_pfs_pfs;//*presyFL3+biasFL3+ac_OutPostprocessFL3*recurrentFL3;
    //    sensor_output.at(i) = tanh(sensor_activity.at(i));
    //    preprosensor.at(i) = sensor_output.at(i);

  }

  //cout << "FOOT CONTACT SENSORS --- R0: "<< contactmsg.at(R0_fs) << ", L0: "<< contactmsg.at(L0_fs) << " @ " << zeit << "s" << endl;

  // >> i/o operations here <<
  outFilenpp1 << in0.at(L0_fs) << ' ' << in0.at(L1_fs) << ' ' << ' ' << in0.at(L2_fs) << endl;

  //*******************************************************
  //Preprocessing US sensors FRONT (Eduard)
  //*******************************************************




  //3)****Prepro IR leg sensors for elevator reflexes********

  for (unsigned int i = R2_irs; i < (L0_irs + 1); i++) {
    if (in0.at(i) > 0.4) {
      sensor_activity.at(i) = 1.0;
    } else {
      sensor_activity.at(i) = 0.0;
    }
    sensor_output.at(i) = sensor_activity.at(i) * irsensor_w_pfs_rfs + sensor_output.at(i) * irsensor_w_pfs_pfs;
    //    irs_delayline.at(i)->Write(sensor_output.at(i));
    //irs_delayline.at(i)->Read(delay_irs);
    if (sensor_output.at(i) > 0.05)
      preprosensor.at(i).at(0) = 1.0;
    else
      preprosensor.at(i).at(0) = 0.0;
    //    irs_delayline.at(i)->Step();
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

    //    for (unsigned int i = FR_us; i < (FL_us + 1); i++) {
    //      //lowpass preprocessing of raw signal
    //      sensor_activity.at(i) = lowpass * in0.at(i) + (1.0 - lowpass) * sensor_output.at(i);
    //      sensor_output.at(i) = sensor_activity.at(i);
    //    }

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
  //---------------------------//

  return preprosensor;

}
;

///-------------------------------------------------------------------------------------------------------------------------


