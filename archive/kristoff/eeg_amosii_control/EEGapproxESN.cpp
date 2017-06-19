/**
 * EEGapproxESN.cpp
 *
 * Contains the EEGapproxESN class member function definitions
 *
 * Created on:          ?? ??, ????
 * Last Modified on:    May 23, 2017
 *
 * Author: Kristoffer Honore
 *
 * Remodeling of EMGapproxESN
 *
 */
#include "EEGapproxESN.h"
EEGapproxESN::EEGapproxESN()
{
	std::cout << "Initialize EEGapproxESN" << std::endl;
	/// Load a network
    ESN                     = new ESNetwork( 1, "Network/TRC6_M11_30_S80" );
	//    load( _num, _dir );
	//    std::cout << "Network Loaded" << std::endl;

    /// Extracting ANN parameters
    numberOfInputs          = ESN->inputNeurons;
    numberOfHiddenUnits     = ESN->networkNeurons;
    numberOfOutputs         = ESN->outputNeurons;

    /// Create ESN input vector
    inputValues             = new float[numberOfInputs];
    /// Create ESN target output vector
    targetValues            = new float[numberOfOutputs];
	
	std::cout << "number Of Inputs: " << numberOfInputs << std::endl;
	std::cout << "number Of HiddenUnits: " << numberOfHiddenUnits << std::endl;
	std::cout << "number Of Outputs: " << numberOfOutputs << std::endl;
	
    /// Reset input and target values
    resetInputValues();
    resetOutputValues();
    resetMSE();
}
/*
EEGapproxESN::EEGapproxESN(
        string _inputPath,
        string _targetPath,
        string _resultPath,
        string _dir,
        unsigned int _num,
        double _percentage_1,
        double _percentage_2,
        unsigned int _verboseLevel,
        char _delimiter,
        unsigned int _numOfHeaderLines ) {

    verboseLevel            = _verboseLevel;

    /// Load a network
    ESN                     = new ESNetwork( _num, _dir );
//    load( _num, _dir );
//    std::cout << "Network Loaded" << std::endl;

    /// Extracting ANN parameters
    numberOfInputs          = ESN->inputNeurons;
    numberOfHiddenUnits     = ESN->networkNeurons;
    numberOfOutputs         = ESN->outputNeurons;

    /// Create ESN input vector
    inputValues             = new float[numberOfInputs];
    /// Create ESN target output vector
    targetValues            = new float[numberOfOutputs];

    percentage_1            = _percentage_1;
    percentage_2            = _percentage_2;

    /// Reset input and target values
    resetInputValues();
    resetOutputValues();
    resetMSE();

    ESN->verboseLevel       = _verboseLevel;

    readData( _inputPath, _targetPath, _delimiter, _numOfHeaderLines );

    /// Open file for saving results
    resultsFile.open( _resultPath + ".txt", ios::out );

    test( false );

    //    string greenBold = "\033[32;1m";
    //    string reset = "\033[0m";
    //    cout << "before" + greenBold + " green " + reset + "after" << endl;

}
*/

EEGapproxESN::~EEGapproxESN(){

    delete[] ESN;
    delete[] inputValues;
    delete[] targetValues;

    //resultsFile.close();

}

void EEGapproxESN::trainnetwork()
{
	std::cout << "hey it works" << std::endl;
}

float EEGapproxESN::run(float * signalPoint, float * targetPoint)
{
	//std::cout << "hey it works " << signalPoint[0] << std::endl;
	
	/// ...set inputs...
	for( unsigned int j = 0; j < numberOfInputs; j++ ) {
                inputValues[j] = signalPoint[j];
		
	}
	/// ...set outputs...
	for( unsigned int j = 0; j < numberOfOutputs; j++ ) {
		targetValues[j] = targetPoint[j];
	}
	
	activate();
	
	return output_ESN;
}

/*
void EEGapproxESN::save( unsigned int num, string dir ) {

    ESN->writeParametersToFile( num, dir );
    ESN->writeStartweightsToFile( num, dir );
    ESN->writeInnerweightsToFile( num, dir );
    ESN->writeEndweightsToFile( num, dir );
    ESN->writeNoiseToFile( num, dir );

}

void EEGapproxESN::load( unsigned int num, string dir ) {

    ESN->readParametersFromFile( num, dir );
    ESN->readStartweightsFromFile( num, dir );
    ESN->readInnerweightsFromFile( num, dir );
    ESN->readEndweightsFromFile( num, dir );
    ESN->readNoiseFromFile( num, dir );

//    cout << "network loaded:" << endl;
//    cout << ESN->inputNeurons << endl;
//    cout << ESN->networkNeurons << endl;
//    cout << ESN->outputNeurons << endl;
}
*/
/*
void EEGapproxESN::readData(
        string pathToInput,
        string pathToTarget,
        char delimiter,
        unsigned int numberOfHeaderLines ) {

    //if( verboseLevel >= 1 ) cout << "Reading data...";

    vector< vector< double > > inputData, targetData;

    DelimitedFileReader::read( pathToInput, delimiter, inputData, numberOfHeaderLines  );
    DelimitedFileReader::read( pathToTarget, delimiter, targetData, numberOfHeaderLines  );

    //if( verboseLevel >= 1 ) cout << "\t\t\t\t[OK]" << endl;

    /// Separating training data from testing data.
    separateData( inputData, targetData );

}
*/
/*
 * 
 * Divides the data available into two batches. One for training and one for
 * testing the network
 *
 * trainingUnits and testingUnits will contain data following this pattern:
 *     trainingUnits[0][0][0] = input1(t0)
 *     trainingUnits[0][0][1] = input2(t0)
 *     trainingUnits[0][1][0] = target1(t0)
 *     trainingUnits[0][1][1] = target2(t0)
 *     trainingUnits[1][0][0] = input1(t1)
 *     trainingUnits[1][0][1] = input2(t1)
 *     ...
 *
 * @param inputData
 * @param targetData
 * @param percentage
 */
/*
void EEGapproxESN::separateData(
        vector< vector< double > > & inputData,
        vector< vector< double > > & targetData ) {

    //if( verboseLevel >= 1 ) cout << "Separating data...";

    // The number of iterations after which the learning units created are set aside for
    // cross evaluation and testing purposes
    unsigned int index_1 = ( percentage_1 / 100.0 ) * inputData.size();
    unsigned int index_2 = index_1 + ( percentage_2 / 100.0 ) * ( inputData.size() - index_1 );

    /// Even if the number of inputs and outputs are not the same,
    /// the time steps are equal thus the outer loop can be common
    for( unsigned int i = 0; i < inputData.size(); i++ ) {

        vector< double > inputs;
        for( unsigned int j = 0; j < numberOfInputs; j++ ) {

            inputs.push_back( inputData[i][j] );

        }

        vector< double > targets;
        for( unsigned int j = 0; j < numberOfOutputs; j++ ) {

            targets.push_back( targetData[i][j] );

        }

        vector< vector< double > > unit{ inputs, targets };




        if ( i <= index_1 )
          trainingUnits.push_back( unit );
        else if( i <= index_2 )
          validationUnits.push_back( unit );
        else
          testingUnits.push_back( unit );

    }

    //if( verboseLevel >= 1 ) cout << "\t\t\t\t[OK]" << endl;

}
*/
/*
 * 
 * Sets the values given as parameter to be the inputs of the ANN
 *
 * @param values
 */
/*
void  EEGapproxESN::writeInput( float * values ) {

    for( unsigned int i = 0; i < numberOfInputs; i++ ) {

        inputValues[i] = values[i];

    }

}
*/
/*
 * 
 * @return the latest output of the ANN
 */
/*
float * EEGapproxESN::readOutput() const {

    float * temp = new float[numberOfOutputs];

    for( unsigned int i = 0; i < numberOfOutputs; i++ ) {

        temp[i] = targetValues[i];

    }

    return temp;

}
*/
void EEGapproxESN::activate() {

	//std::cout << "Start activation" << std::endl;
	//std::cout << "input " << inputValues[0] << std::endl;
	
	//learningFlag    = false;
    //iteration++;

    ESN->setInput( inputValues, numberOfInputs/* no. input*/ ); // Call ESN
	// should take one input

    // ESN Learning function
    ESN->takeStep( targetValues, 0.99 /*0.9 RLS*/, 1 /*no td = 1 else td_error*/, false/* true= learn, false = not learning learn_critic*/, 0/*0*/);

    // TODO temporary solution
    target_ESN = targetValues[0];
    output_ESN = ESN->outputs->val(0, 0); // Read out the output of ESN
    input_ESN = inputValues[0];

    // ESN->printMatrix(ESN->endweights); //print weight matrix on screen

    /// Calculate online error at each time step
    squaredError = ( target_ESN - output_ESN ) * ( target_ESN - output_ESN );

    mse += squaredError;

}

/*
void EEGapproxESN::train( string resultPath) {

    learningFlag    = true;
    iteration       = 0;

    if( verboseLevel >= 1 ) {

        cout << endl;
        cout << "#########################################################################" << endl;
        cout << "##                          Training started...                        ##" << endl;
        cout << "#########################################################################" << endl;
    }

    /// Repeat training as many times as specified or until iterationLimit is reached
    for( unsigned int r = 0; r < repetition; r++ ) {

        //cout << "repetition: " << r << endl;
        resultsFile.open( resultPath + "_" + to_string(r) + ".txt", ios::out );

        /// Iterate through the time steps...
        for( unsigned int i = 0; i < trainingUnits.size(); i++ ) {


            /// Escape condition
            if( iteration > iterationLimit )
            {
                resultsFile.close();
                break;
            }

            /// ...set inputs...
            for( unsigned int j = 0; j < numberOfInputs; j++ ) {

                inputValues[j] = trainingUnits[i][0][j];

            }

            /// ...set outputs...
            for( unsigned int j = 0; j < numberOfOutputs; j++ ) {

                targetValues[j] = trainingUnits[i][1][j];

            }

            activate(  );

        }
        
        //calculate MSE here and output it. use mse and reset it each time
        mse = mse / trainingUnits.size();
        resultsFile << "\n" << "MSE" << "\t" << mse << "\n" << flush;
        resultsFile.close();
        cout << "repitition: " << r + 1 << " MSE: " << mse << endl;
        resetMSE();
        /// Escape condition
        if( iteration > iterationLimit ) break;

    }

    if( verboseLevel >= 1 ) {

        cout << "##                          Training finished.                         ##" << endl;
        cout << endl;

    }

}
*/
/*
void EEGapproxESN::test( bool isValidation ) {

    learningFlag    = false;
    resetMSE();

    if( verboseLevel >= 1 ) {

        cout << "#########################################################################" << endl;
        cout << "##                           Testing started...                        ##" << endl;
        cout << "#########################################################################" << endl;

    }

    /// Adding header
    // resultsFile << "Output" << "\t" << "Target" << "\t" << "Squared Error" << "\n" << flush;

    /// Iterate through the timesteps...

    for( unsigned int i = 0; i < validationUnits.size(); i++ ) {
    //for( unsigned int i = 0; i < testingUnits.size(); i++ ) {
    //for( unsigned int i = 0; i <  trainingUnits.size(); i++ ) {

    /// ...set inputs...
        for( unsigned int j = 0; j < numberOfInputs; j++ ) {

            if( isValidation ) {

                inputValues[j] = validationUnits[i][0][j];

            } else {

                inputValues[j] = testingUnits[i][0][j];

            }

        }

        /// ...set outputs...
        for( unsigned int j = 0; j < numberOfOutputs; j++ ) {

            if( isValidation ) {

                targetValues[j] = validationUnits[i][1][j];

            } else {

                targetValues[j] = testingUnits[i][1][j];

            }

        }

        activate();

    }

    /// Calculate MSE
    if( isValidation ) {

        mse = mse / validationUnits.size();

    } else {

        mse = mse / testingUnits.size();

    }

    for( unsigned int j = 0; j < numberOfInputs; j++ ) {
        resultsFile << "Input" << "\t";
    }
    for( unsigned int j = 0; j < numberOfOutputs; j++ ) {
        resultsFile << "Output" << "\t";
    }
    for( unsigned int j = 0; j < numberOfOutputs; j++ ) {
        resultsFile << "Target" << "\t";
    }
       resultsFile << "Squared Error" << "\n" << flush;

//       resultsFile << "Input" << "\t" << "Output" << "\t" << "Target" << "\t" << "Squared Error" << "\n" << flush;
       /// Save MSE
       resultsFile << "\n" << "MSE" << "\t" << mse << "\n" << flush;
       //resultsFile << "\n" << "MSE" << "\t" << mse << "\n" << validationUnits.size() << "\n" << testingUnits.size()<<flush;

    if( verboseLevel >= 1 ) {

        cout << "##                           Testing finished.                         ##" << endl;
        cout << endl;

        cout << "MSE: " << mse << " Hidden unit: "<< numberOfHiddenUnits << " Iteration: "<< iterationLimit<< endl;
        cout << "Training data: " << trainingUnits.size() << " Validation data: "<< validationUnits.size() << " Total data: "<< trainingUnits.size()+validationUnits.size() << endl;

    }

}
*/
