/***************************************************************************
 *   Copyright (C) 2012 by Dennis Goldschmidt                              *
 *    goldschmidt@physik3.gwdg.de                                          *
 *                                                                         *
 *    BackboneJointControl.cpp                                             *
 *                                                                         *
 * Controller for backbone joint motion!                                   *
 *                                                                         *
 **************************************************************************/

#include "BackboneJointControl.h"
#include "../utils/ann-framework/ann.h"
#include <ode_robots/amosiisensormotordefinition.h>
#include <vector>
using namespace std;



    BackboneJointControl::BackboneJointControl(){
      lowpass = 0.95;

      setNeuronNumber(6);

      setTransferFunction(0, identityFunction());
      for(unsigned int i = 1; i < 4; i++) {
      setTransferFunction(i, thresholdFunction()); //threshold at 0.9
      }
      setTransferFunction(4, identityFunction());
      setTransferFunction(5, identityFunction());

      //Synaptic weights
      w(1, 0, -10.0); //Inhibition, if 0 active then 1 inactive
      w(2, 0, -10.0); //Inhibition, if 0 active then 2 inactive
      w(2, 1, -10.0); //Inhibition, if 1 active then 2 inactive
      w(3, 0, -10.0); //Inhibition, if 0 active then 3 inactive
      w(3, 1, -10.0); //Inhibition, if 1 active then 3 inactive

      //Biases for normalizing (threshold usually at 0.0)
      b(2, 0.8);
      b(3, 0.8);

      //Lowpass neuron
      w(4, 1, -(1.0 - lowpass));
      w(4, 2, (1.0 - lowpass));
      w(4, 3, -(1.0 - lowpass));
      w(4, 4, lowpass);

      //Additive output neuron
      w(5, 0, 1.0);
      w(5, 4, 1.0);


    }


