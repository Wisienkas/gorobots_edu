/*
 * test_neuron.cpp
 *
 *  Created on: Nov 1, 2013
 *      Author: timo
 */

#include "gtest/gtest.h"
#include "utils/ann-framework/neuron.h"
#include "utils/ann-framework/synapse.h"

namespace {

  TEST(NeuronTest, Constructor) {
    Neuron neuron();
  }

  TEST(NeuronTest, addSynapse) {
    Neuron neuron;
    Synapse synapse(&neuron, &neuron, false);
    ASSERT_EQ(neuron.getSynapsesIn().size(), 0);
    ASSERT_EQ(neuron.getSynapsesOut().size(), 0);
    neuron.addSynapseIn(&synapse);
    ASSERT_EQ(neuron.getSynapsesIn().size(), 1);
    ASSERT_EQ(neuron.getSynapsesOut().size(), 0);
    neuron.addSynapseOut(&synapse);
    ASSERT_EQ(neuron.getSynapsesIn().size(), 1);
    ASSERT_EQ(neuron.getSynapsesOut().size(), 1);
  }

}

