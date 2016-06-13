#ifndef EMG_APPROX_RBF_H
#define EMG_APPROX_RBF_H

#include "includes/rbf_network.h"
#include "DelimitedFileReader.h"
#include "SimpleRNG.h"

#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <cstdlib>

#include "math.h"

using namespace std;
using namespace RBFnetwork;


class EMGapproxRBF {

    public:

	EMGapproxRBF( 
                string,                                 /// input path
                string,                                 /// target path
                string = "results/validation/def.txt",  /// result path
                string = "save_files/def.txt",          /// save path
                unsigned int = 1,                       /// # of inputs
                unsigned int = 1,                       /// # of outputs
                unsigned int = 10,                      /// # of hidden units
                double = 0.3,                           /// learning rate
                double = -1.2,                          /// min
                double = 1.2,                           /// max 
                bool = false,                           /// randomizePosition
                bool = false,                           /// randomizeSTD
                double = 0.5,                           /// std
                double = 0.0,                           /// std_min,
                double = 0.0,                           /// std_max
                double = 80.0,                          /// percentage_1
                double = 50.0,                          /// percentage_2
                unsigned int = 1,                       /// iteration limit
                unsigned int = 0 );                     /// verboseLevel
        
        EMGapproxRBF( 
                string,                                 /// input path
                string,                                 /// target path
                string = "results/validation/def.txt",  /// result path
                string = "save_files/def.txt",          /// load path
                double = 80.0,                          /// percentage_1
                double = 50.0,                          /// percentage_2
                unsigned int = 0 );                     /// verboseLevel
        
        ~EMGapproxRBF();
        
        void train( double = 0.0000005 );
	void test( bool = true );
        
        void save( string = "save_files/def.txt" );
        void load( string = "save_files/def.txt" );

    private:

	unsigned int numberOfInputs, numberOfHiddenUnits, numberOfOutputs;
	double learningRate, percentage_1, percentage_2, mse;
	RBFNetwork* RBFnet;
	vector<LearningUnit*> trainingUnits, validationUnits, testingUnits;
        
        string savePath;
        
        unsigned int iterationLimit, verboseLevel;
        
        ofstream resultsFile; // File to save the output-target-error vector

        void readData( string, string, char  = '\t', unsigned int = 0 );
	void separateData( vector< vector< double > >&, vector< vector< double > >& );
        
        void activate( vector<LearningUnit*>& );
        
        RBFneuron* generateRBFneuron( double, double, bool = true, unsigned int = 0, bool = false, double = 0.5, double = 0.1, double = 3.0 );

};

#endif
