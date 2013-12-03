/***************************************************************************
 *   Copyright (C) 2012 by Dennis Goldschmidt                              *
 *    goldschmidt@physik3.gwdg.de                                          *
 *                                                                         *
 * This is a general forward model!                                        *
 *                                                                         *
 **************************************************************************/

#include "forwardmodel.h"
#include "../utils/ann-framework/ann.h"
#include <ode_robots/amosiisensormotordefinition.h>
#include <vector>
using namespace std;

/******************************************************************************
 *     Construct forward model
 *****************************************************************************/

class ForwardModelANN : public ANN {
  public:

    ForwardModelANN(double r_w, double b_w) {

      setNeuronNumber(2);

      // synaptic weights
      w(0, 0, r_w);
      w(1, 0, 50.0);

      // neuron biases
      b(0, b_w);

    }

};

Forwardmodel::Forwardmodel(double rec_weight, double weight, double b, bool learning) { //Hier ANN generieren
  counter = 0;
  lowpass_error_gain = 0.9;
  lowpass_error = 0.0;
  output = 0.0;
  lowpass_error_old = 0.0;
  output_old = 0.0;
  error_threshold = 0.15;
  error_threshold_2 = 0.2;//0.25;
  rec_w = rec_weight;
  input_w = weight;
  bias = b;
  finish = false;

  FModelANN = new ForwardModelANN(rec_w, bias);

  if (learning) // No learning
  {
    learnrate = 0.0;
  }
  else { //learning
    learnrate = 0.01;
  }

}


Forwardmodel::~Forwardmodel() {
  // delete neuronal net
  delete FModelANN;
}


double Forwardmodel::step(double motor_output, double sensor_input, bool learning, bool purefoot) { //Hier ANN step, danach Fehler akkumulieren: return acc_error

  //Previous values
  lowpass_error_old = lowpass_error;
  output_old = output;
  input = sensor_input;

  //ANN--------------------------------------------

//  cout << input_w << " " << rec_w << " " << bias << endl;
  //Input from CTR joint
  FModelANN->setInput(0, input_w * motor_output);


  //Forward model of Coxa right motor signals //Recurrent single neuron
  FModelANN->step();

  //Outputs
  activity = FModelANN->getActivity(0);
  output = FModelANN->getOutput(0);
  outputfinal = FModelANN->getOutput(1);

  //-------------------------------------------------


  //Calculate error for learning
  learning_error = sensor_input - output;//-fmodel_cmr_outputfinal.at(i) /*regulate error*/; //target - output // only positive error

  //For elevator reflex
  error_old = error;

  //Calculate error
  error = sensor_input - outputfinal; //target - output // only positive error

  //lowpass error
  lowpass_error = lowpass_error_gain * lowpass_error_old + (1 - lowpass_error_gain) * error;

  if (purefoot)//Only foot contact signal
  {
    error = sensor_input + 1.0; //target - output // only positive error
  }

  // Error threshold
//  if (error < error_threshold) {
//    error = 0.0;
//  }

  //Accumulating positive errors for controlling searching reflexes
  acc_error += abs(error);

  if (abs(error) < 0.05) {
    acc_error_small = 0.0;
  }
    acc_error_small += error;

  //Negative Error signal for controlling elevator reflexes
  if (error_old < 0.0) {
    error_neg += abs(error_old);
    error_elev = 1.0;
  }
  if (error_old > 0.0) {
    error_neg = 0.0;
    error_elev = 0.0;
  }


  //reset at swing phase
  if (motor_output > -0.7) {//Reset error every Swing phase by detecting CR motor signal
    acc_error = 0.0;
  }

  /****************LEARNING*****************/// Later in updateWeights()

  //--------Stop learning process if error not appear for 500 time steps!
  //Count distance between two error peaks

  if (abs(lowpass_error) < error_threshold_2)//abs(fmodel_cmr_error.at(i)) == 0.0)
  {
    counter++;
  }
  if (abs(lowpass_error) > error_threshold_2)//abs(fmodel_cmr_error.at(i)) > error_threshold)
  {
    counter = 0;
  }
  /*1) TO DO NEED TO BE ADJUSTED THIS NUMBER "500"*/

  if (counter > 300)//500)//1000) // 500 time steps !! need to be adaptive !! later
  {
    learnrate = 0.0;
    lowpass_error = 0.0;
    finish = true;
  }


  //learning delta rule with gradient descent

  //Recurrent weight, fmodel_cmr_outputfinal.at(i)
  FModelANN->setWeight(0,0,FModelANN->getWeight(0, 0) + learnrate * (learning_error * (1.0 - output * output) * output_old));
  if (FModelANN->getWeight(0, 0) < 0.0) {
    FModelANN->setWeight(0,0,0.0);
  }
  rec_w = FModelANN->getWeight(0,0);

  //Input weight, fmodel_cmr_outputfinal.at(i)
  input_w += learnrate * (learning_error * (1 - output * output) * motor_output);
  if (input_w < 0.0) {
    input_w = 0.0;
  }

  //Bias
  FModelANN->setBias(0, FModelANN->getBias(0) + learnrate * (learning_error * (1 - output * output)));
  if (FModelANN->getBias(0) < 0.0) {
    FModelANN->setBias(0, 0.0);
  }
  bias = FModelANN->getBias(0);

 return acc_error;
}

