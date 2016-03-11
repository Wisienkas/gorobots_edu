#ifndef MODULARNEURAL_H_
#define MODULARNEURAL_H_
#include <selforg/controller_misc.h>
#include <selforg/configurable.h>
#include <selforg/types.h>

#include <assert.h>
#include <cmath>
#include <stdlib.h>
#include <string.h>
#include <vector>

#include <selforg/matrix.h>
#include "cpg.cpp"
#include "utils/ann-library/pcpg.cpp"
#include "utils/ann-framework/neuron.h"
#include "utils/ann-framework/synapse.h"
#include "utils/ann-framework/ann.h"
#include "utils/ann-library/vrn.h"
#include "utils/ann-library/psn.cpp"


class CPG;
class PCPG;
class VRN;
class PSN;

class ModularNeural : public ANN{
			public:
					ModularNeural();
					~ModularNeural(){}

			double getVrnLeftOut();
			double getVrnRightOut();
			void update(double p);
			void setInputNeuronInput(double l,double r,double i);
			double getCpgOut0();
			double getCpgOut1();
			double getCpgOut2();
			double getCpgFrequency();
			double getPsnOutput(int neuron_index);
			double getP();
			private:
				PCPG *pcpg;
				CPG *cpg;
				VRN *vrn_left;
				VRN *vrn_right;
				PSN *psn;
				vector<Neuron*> inputNeurons;


				
					

};
#endif