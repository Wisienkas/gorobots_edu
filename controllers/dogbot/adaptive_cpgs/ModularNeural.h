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
					ModularNeural(bool multiple,int nCPGs);
					~ModularNeural(){}

			double getVrnLeftOut();
			double getVrnRightOut();
			double getVrnOut(int ID);
			void update();
			void update(double p,int ID);
			void setInputNeuronInput(double l,double r,double i);
			double getCpgOut0();
			double getCpgOut1();
			double getCpgOut2();
			double getCpgOut0(int ID);
			double getCpgOut1(int ID);
			double getCpgOut2(int ID);
			double getCpgFrequency();
			double getCpgFrequency(int ID);
			double getPsnOutput(int neuron_index);
			double getPsnOutput(int ID,int neuron_index);
			double getP();
			double getP(int ID);
			private:
				PCPG *pcpg;
				CPG *cpg;
				VRN *vrn_left;
				VRN *vrn_right;
				PSN *psn;
				vector<Neuron*> inputNeurons;
				std::vector<CPG*> mCPGs;
				std::vector<PSN*> mPSNs;


				double ** delta;
				double ** k;
				std::vector<double> a_t,a_t1;
				double n_CPGs;

				double osc_couple0,osc_couple1;


				
					

};
#endif