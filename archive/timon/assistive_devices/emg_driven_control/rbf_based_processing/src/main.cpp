#include <iostream>
#include <vector>
#include "EMGapproxRBF.h"


using namespace RBFnetwork;

int main( int argc, char ** argv ) {
    
//    cout << "argc: " << argc << endl;
    
    if( argc == 7 ) {
        
        string inputPath                    = argv[1]; 
        string targetPath                   = argv[2];
        string resultPath                   = argv[3];
        string loadPath                     = argv[4];
        double percentage_1                 = atof( argv[5] );
        double percentage_2                 = atof( argv[6] );
        
        unsigned int verboseLevel           = 0;
        
        if( verboseLevel >= 1 ) {
          
            cout << "Initializing network with the following parameters: " << endl;
            cout << "    Input path\t\t\t"              << inputPath << endl; 
            cout << "    Target path\t\t\t"             << targetPath << endl; 
            cout << "    Result path\t\t\t"             << resultPath << endl;
            cout << "    Load path\t\t\t"               << loadPath << endl;
            cout << "    percentage_1\t\t"              << percentage_1 << endl; 
            cout << "    percentage_2\t\t"              << percentage_2 << endl; 
            cout << "    Verbose level\t\t"             << verboseLevel << endl; 
            
        }
        
        EMGapproxRBF * app1 = new EMGapproxRBF(
            inputPath,              // 1
            targetPath,             // 2
            resultPath,             // 3
            loadPath,               // 4
            percentage_1,           // 5
            percentage_2,           // 6
            verboseLevel );         // 7
    
    } else if( argc == 19 ) {
        
        bool randomizePosition, randomizeSTD;
        
        string inputPath                    = argv[1]; 
        string targetPath                   = argv[2]; 
        string resultPath                   = argv[3]; 
        string savePath                     = argv[4]; 
        unsigned int numberOfInputs         = atoi( argv[5] ); 
        unsigned int numberOfOutputs        = atoi( argv[6] ); 
        unsigned int numberOfHiddenUnits    = atoi( argv[7] ); 
        double learningRate                 = atof( argv[8] ); 
        double min                          = atof( argv[9] ); 
        double max                          = atof( argv[10] ); 
        if( atoi( argv[11] ) == 0 ) { randomizePosition = false; } else { randomizePosition = true; } 
        if( atoi( argv[12] ) == 0 ) { randomizeSTD = false; } else { randomizeSTD = true; } 
        double std                          = atof( argv[13] ); 
        double std_min                      = atof( argv[14] ); 
        double std_max                      = atof( argv[15] ); 
        double percentage_1                 = atof( argv[16] ); 
        double percentage_2                 = atof( argv[17] ); 
        unsigned int iterationLimit         = atoi( argv[18] ); 
        
        unsigned int verboseLevel           = 0; 
        
        if( verboseLevel >= 1 ) {
          
            cout << "Initializing network with the following parameters: " << endl;
            cout << "    Input path\t\t\t"              << inputPath << endl; 
            cout << "    Target path\t\t\t"             << targetPath << endl; 
            cout << "    Result path\t\t\t"             << resultPath << endl;
            cout << "    Save path\t\t\t"               << savePath << endl;
            cout << "    # of inputs\t\t\t"             << numberOfInputs << endl; 
            cout << "    # of outputs\t\t"              << numberOfOutputs << endl; 
            cout << "    # of hidden neurons\t\t"       << numberOfHiddenUnits << endl;
            cout << "    Learning rate\t\t"             << learningRate << endl; 
            cout << "    minimum\t\t\t"                 << min << endl; 
            cout << "    maximum\t\t\t"                 << max << endl;
            cout << "    Randomize position\t\t"        << randomizePosition << endl; 
            cout << "    Randomize STD\t\t"             << randomizeSTD << endl; 
            cout << "    STD\t\t\t\t"                   << std << endl; 
            cout << "    STD min\t\t\t"                 << std_min << endl; 
            cout << "    STD max\t\t\t"                 << std_max << endl; 
            cout << "    percentage_1\t\t"              << percentage_1 << endl; 
            cout << "    percentage_2\t\t"              << percentage_2 << endl; 
            cout << "    Iteration limit\t\t"           << iterationLimit << endl; 
            cout << "    Verbose level\t\t"             << verboseLevel << endl; 
            
        }
        
        EMGapproxRBF * app1 = new EMGapproxRBF(
            inputPath,              // 1
            targetPath,             // 2
            resultPath,             // 3
            savePath,               // 4
            numberOfInputs,         // 5
            numberOfOutputs,        // 6
            numberOfHiddenUnits,    // 7
            learningRate,           // 8
            min,                    // 9
            max,                    // 10
            randomizePosition,      // 11
            randomizeSTD,           // 12
            std,                    // 13
            std_min,                // 14
            std_max,                // 15
            percentage_1,           // 16
            percentage_2,           // 17
            iterationLimit,         // 18
            verboseLevel );         // 19                
        
    } else {
        
        cout << "[ERROR] Not enough arguments." << endl;
        cout << "\tusage: ./start inputPath resultPath numberOfInputs numberOfOutputs numberOfHiddenUnits"
                << " learningMode internalNonlinearity outputNonlinearity inputSparsity internalSparsity learningRate "
                << " leak percentage repetition iterationLimit" << endl;
        
    }
    
//    EMGapproxRBF app1(
//            "resources/synthetic/sin.txt",  // inputPath
//            "resources/synthetic/sin.txt",  // targetPath
//            "results/results.txt",          // resulPath
//            1,                              // # of inputs
//            1,                              // # of outputs
//            10,                             // # of hidden units
//            0.5,                            // learning rate
//            -1.2,                           // min 
//            1.2,                            // max
//            false,                          // randomizePosition
//            false,                          // randomizeSTD
//            0.3                             // std
//            0.3                             // std_min
//            0.8                             // std_max
//            30.0,                           // percentage
//            10,                             // iterationLimit
//            1 );                            // verboseLevel

    return 0;

}
