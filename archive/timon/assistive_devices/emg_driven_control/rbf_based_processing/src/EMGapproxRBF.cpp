#include "EMGapproxRBF.h"

/**
 * Main constructor
 * 
 * Used for parameter tuning during cross-validation
 * 
 * @param _inputPath
 * @param _targetPath
 * @param _resultPath
 * @param _savePath
 * @param _numberOfInputs
 * @param _numberOfOutputs
 * @param _numberOfHiddenUnits
 * @param _learningRate
 * @param _min
 * @param _max
 * @param _randomizePosition
 * @param _randomizeSTD
 * @param _std
 * @param _std_min
 * @param _std_max
 * @param _percentage_1
 * @param _percentage_2
 * @param _iterationLimit
 * @param _verboseLevel
 */
EMGapproxRBF::EMGapproxRBF(
        string _inputPath,
        string _targetPath,
        string _resultPath,
        string _savePath,
        unsigned int _numberOfInputs,
        unsigned int _numberOfOutputs,
        unsigned int _numberOfHiddenUnits,
        double _learningRate,
        double _min,
        double _max,
        bool _randomizePosition,
        bool _randomizeSTD,
        double _std,
        double _std_min,
        double _std_max,        
        double _percentage_1,
        double _percentage_2,
        unsigned int _iterationLimit,
        unsigned int _verboseLevel ) {

    numberOfInputs          = _numberOfInputs;
    numberOfHiddenUnits     = _numberOfHiddenUnits;
    numberOfOutputs         = _numberOfOutputs;
    iterationLimit          = _iterationLimit;
    learningRate            = _learningRate;
    percentage_1            = _percentage_1;
    percentage_2            = _percentage_2;
    mse                     = 0.0;

    savePath                = _savePath;
    verboseLevel            = _verboseLevel;
    
    readData( _inputPath, _targetPath );

//     for ( unsigned int i = 0; i < trainingUnits.size(); i++ )
//     	cout << trainingUnits[i]->inputs[0] << endl;

//    cout << "trainingUnits.size(): " << trainingUnits.size() << ", testingUnits.size(): " << testingUnits.size() << endl;
    
    
    
//              This is the old method

//    vector<InputConfigs*> inputs_confis;
//
//    // setting up a configuration for each dimension TODO SET IT UP MANUALLY!!!
//    // # of hidden neurons = ( # of centers per dimension on )^( # of dimensions )
//    InputConfigs ic1(
//            1.200, 	//  max number
//            -1.200, //  min number
//            30 );  	// number of centers on this dimension
//
//    // InputConfigs ic2( 0.000, -0.334, 8 );
//    // InputConfigs ic3( 0.000, -0.334, 8 );
//    // InputConfigs ic4( 0.000, -0.334, 8 );
//    // InputConfigs ic5( 0.000, -0.334, 5 );
//    // InputConfigs ic6( 0.000, -0.334, 5 );
//    // InputConfigs ic7( 0.000, -0.334, 5 );
//    // InputConfigs ic8( 0.000, -0.334, 5 );
//
//    inputs_confis.push_back( &ic1 ); // add them to the list of dimensions
//    // inputs_confis.push_back( &ic2 ); // add them to the list of dimensions
//    // inputs_confis.push_back( &ic3 ); // add them to the list of dimensions
//    // inputs_confis.push_back( &ic4 ); // add them to the list of dimensions
//    // inputs_confis.push_back( &ic5 ); // add them to the list of dimensions
//    // inputs_confis.push_back( &ic6 ); // add them to the list of dimensions
//    // inputs_confis.push_back( &ic7 ); // add them to the list of dimensions
//    // inputs_confis.push_back( &ic8 ); // add them to the list of dimensions
//
//    // Instantiate the network with the configurations specified above
//    RBFnet = new RBFNetwork( numOfInputs, numOfOutputs, inputs_confis );
    
    
    
    // Generating hidden units
    vector<RBFneuron*> neurons;
    for( unsigned int i = 0; i < numberOfHiddenUnits; i++ ) {
        
//        cout << i << ": ";
        neurons.push_back( generateRBFneuron( _min, _max, _randomizePosition, i, _randomizeSTD, _std, _std_min, _std_max ) );
        
    }
    
    // Instantiate the network with the neurons generated above
    RBFnet = new RBFNetwork( numberOfInputs, numberOfOutputs, neurons );
    
    // Open file for saving results
    resultsFile.open( _resultPath, ios::out );
    
    // RBFnet.print_RBF_hidden_neurons_information();
    
    train();
    test( true );

}

/**
 * Main constructor
 * 
 * Used for testing
 * 
 * @param _inputPath
 * @param _targetPath
 * @param _resultPath
 * @param _loadPath
 * @param _percentage_1
 * @param _percentage_2
 * @param _verboseLevel
 */
EMGapproxRBF::EMGapproxRBF( 
        string _inputPath,
        string _targetPath,
        string _resultPath,
        string _loadPath,
        double _percentage_1,                     
        double _percentage_2, 
        unsigned int _verboseLevel ) {
    
    /// Load a network
    RBFnet = new RBFNetwork( _loadPath );
    
    /// Extracting ANN parameters
    int* neuronCount = RBFnet->getNeuronCount();
    
    numberOfInputs      = neuronCount[0];
    numberOfHiddenUnits = neuronCount[1];
    numberOfOutputs     = neuronCount[2];
    
//    cout << "EMGapproxRBF: " << numberOfInputs << ", " << numberOfHiddenUnits << ", " << numberOfOutputs << endl;
    
    percentage_1            = _percentage_1;
    percentage_2            = _percentage_2;
    
    verboseLevel            = _verboseLevel;
    
    readData( _inputPath, _targetPath );
    
    // Open file for saving results
    resultsFile.open( _resultPath, ios::out );
    
    test( false );
    
}

EMGapproxRBF::~EMGapproxRBF() {
    
//    resultsFile.close();
    
}

void EMGapproxRBF::save( string path ) {
    
    RBFnet->save( path );
    
}

void EMGapproxRBF::load( string path ) {
    
    RBFnet->load( path );
    
    int* neuronCount = RBFnet->getNeuronCount();
    
    numberOfInputs      = neuronCount[0];
    numberOfHiddenUnits = neuronCount[1];
    numberOfOutputs     = neuronCount[2];
    
}

/**
 * 
 * 
 * If the data is organized in plain .txt files as follows:
 *     input1(t0)    input2(t0)    input3(t0)    ...
 *     input1(t1)    input2(t1)    input3(t1)    ...
 *          ...                 ...                 ...
 * where the separator is '\t', then it will be read to 
 * inputData and targetData as:
 *     [
 *          [ input1(t0)    input2(t0)    input3(t0)    ... ],
 *          [ input1(t1)    input2(t1)    input3(t1)    ... ],
 *          ...
 *     ]
 * so that e.g. 
 *     inputData[0][0] = input1(t0)
 *     inputData[0][1] = input2(t0)
 *     inputData[1][0] = input1(t1)
 *     ...
 * 
 * @param pathToInput
 * @param pathToTarget
 * @param delimiter
 * @param numberOfHeaderLines
 */
void EMGapproxRBF::readData( 
    string pathToInput, 
    string pathToTarget, 
    char delimiter, 
    unsigned int numberOfHeaderLines ) {
    
    if( verboseLevel >= 1 ) cout << "Reading data...";
    
    vector< vector< double > > inputData, targetData;
    
    DelimitedFileReader::read( pathToInput, delimiter, inputData, numberOfHeaderLines  );
    DelimitedFileReader::read( pathToTarget, delimiter, targetData, numberOfHeaderLines );
    
    if( verboseLevel >= 1 ) cout << "\t\t\t\t[OK]" << endl;
            
    /// Separating training data from testing data.
    separateData( inputData, targetData );
    
}

/**
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
 */
void EMGapproxRBF::separateData(
        vector< vector< double > >& inputData,
        vector< vector< double > >& targetData ) {
    
    if( verboseLevel >= 1 ) cout << "Separating data...";

    // The number of iterations after which the learning units created are set aside for 
    // cross evaluation and testing purposes
    unsigned int index_1 = ( percentage_1 / 100.0 ) * inputData.size();
    unsigned int index_2 = index_1 + ( percentage_2 / 100.0 ) * ( inputData.size() - index_1 );

    LearningUnit* ln;
    for( unsigned int i = 0; i < inputData.size(); i++ ) {

        ln = new LearningUnit();

        for( unsigned int j = 0; j < numberOfInputs; j++ ) {
            
            ln->inputs.push_back( inputData[i][j] );
            
        }
        
        for( unsigned int j = 0; j < numberOfOutputs; j++ ) {
            
            ln->outputs.push_back( targetData.at(i).at(j) );
            
        }
        
        if ( i <= index_1 ) 
            trainingUnits.push_back( ln );
        else if( i <= index_2 )
            validationUnits.push_back( ln );
        else
            testingUnits.push_back( ln );

    }

    if( verboseLevel >= 1 ) cout << "\t\t\t[OK]" << endl;
    
}

void EMGapproxRBF::activate( vector<LearningUnit*>& units ) {
    
    for ( unsigned int i = 0; i < units.size(); i++ ) {
        
        double error = 0.0;
       
        // Output results to console
//        cout << "Network output = " << RBFnet->get_output( testingUnits[i]->inputs )[0]
//                 << " Desired output = " << testingUnits[i]->outputs[0] << endl;

        stringstream ss;
        
        /// Saving input data
        for( unsigned int j = 0; j < numberOfInputs; j++ ) {
            
            ss << units[i]->inputs[j] << "\t";
            
        }
        
        vector<double> outputs = RBFnet->get_output( units[i]->inputs );
        vector<double> targets = units[i]->outputs;
        double squaredError = 0.0;
        for( unsigned int k = 0; k < numberOfOutputs; k++ ) {
            
//          cout << "target - output: " << targets[j] << "-" << outputs[j]  << endl;
            error += ( targets[k] - outputs[k] );
            squaredError += ( error * error );
            ss << outputs[k] << "\t" << targets[k] << "\t" << ( error * error );

        }
        
        ss << "\n";
        resultsFile << ss.str();
        
        mse += ( squaredError / numberOfOutputs );

    }
    /// Calculate MSE
    mse = mse / units.size();
    
}

void EMGapproxRBF::train( double targetError ) {
    
    bool console = false;

    if( verboseLevel >= 1 ) {
        
        cout << "/////////////////////////////////////////////////" << endl;
        cout << "//                  learning                   //" << endl;
        cout << "/////////////////////////////////////////////////" << endl;
        
        console = true;
        
    }
    
    RBFnet->learn( trainingUnits, targetError, iterationLimit, learningRate, console );

}

void EMGapproxRBF::test( bool isValidation ) {

    if( verboseLevel >= 1 ) {
        
        if( isValidation ) {
            
            cout << "/////////////////////////////////////////////////" << endl;
            cout << "//               cross-validation              //" << endl;
            cout << "/////////////////////////////////////////////////" << endl;
            
//            save();
            
        } else {
            
            cout << "/////////////////////////////////////////////////" << endl;
            cout << "//                   testing                   //" << endl;
            cout << "/////////////////////////////////////////////////" << endl;
            
        }
        
    }

    // Writing header to file
    stringstream header;
    for( unsigned int i = 0; i < numberOfInputs; i++ ){
        
        header << "Input " << ( i + 1 ) << "\t";
        
    }
    for( unsigned int i = 0; i < numberOfOutputs; i++ ){
        
        header << "Output " << ( i + 1 ) << "\t" << "Target " << ( i + 1 ) << "\t" << "Squared error " << ( i + 1 );
        
    }
    header << "\n";
    resultsFile << header.str();
    
    if( isValidation )
        activate( validationUnits );
    else
        activate( testingUnits );

    resultsFile << "\nMSE\t" << mse << "\n";
    
    resultsFile.close();
    
    if( isValidation ) save( savePath );

}

RBFneuron* EMGapproxRBF::generateRBFneuron( 
        double min, 
        double max, 
        bool randomizePosition, 
        unsigned int index, 
        bool randomizeSTD, 
        double std, 
        double std_min, 
        double std_max ) {
    
    vector<double> centers, variances;
    
    // Setting centroids
    if( randomizePosition ) {
        
//        centers = SimpleRNG::getUniDouble( min, max, numberOfInputs );
        centers = SimpleRNG::getNormalDouble( min, max, numberOfInputs, 0.0, 0.8 );
        
    } else {
        
        double interval = ( ( max - min ) / numberOfHiddenUnits ); // Defines the width of the interval within which the steps are taken
        
        double step = ( interval / ( numberOfInputs + 1 ) );
        
        unsigned int counter = 0;
        for( double d = ( min + ( interval * index ) + step ); d < ( min + ( interval * ( index + 1 ) ) ); d += step ) {
            
            if( counter >= numberOfInputs ) break; // Exit condition compensating for the rounding errors
            
            centers.push_back( d );
            
            counter++;
            
        }
        
    }
    
    //Setting variances
    if( randomizeSTD ) {
        
//        variances = SimpleRNG::getUniDouble( std_min, std_max, numberOfInputs );
        variances = SimpleRNG::getNormalDouble( std_min, std_max, numberOfInputs, 0.0, 0.8 );
        
    } else {
        
        for( unsigned int i = 0; i < numberOfInputs; i++ ) {
            
            variances.push_back( std );
            
        }
        
    }

    RBFneuron* neuron = new RBFneuron( centers, variances );
    
    return neuron;
    
}
