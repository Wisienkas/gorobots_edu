#include <iostream>
#include <vector>
#include "EMGapproxRBF.h"


using namespace RBFnetwork;

unsigned int MODE = 3; /// MODE 1: training, MODE 2: testing, MODE 3: training and testing

int main( int argc, char ** argv ) {
    
    if( argc == 2 ) MODE = atoi( argv[1] );
    
    cout << "Running in MODE: " << MODE << "\n\n";
    
    bool randomizePosition = false, randomizeSTD = false;
    
    string inputPath                    = "resources/cos_f_7.0Hz_fs_10240_2.txt"; 
    string targetPath                   = "resources/cos_f_7.0Hz_fs_10240_2.txt"; 
    string resultName                   = "results/r_1";                            // The program will append "_training.txt" or "_testing.txt"
    string savePath                     = "save_files/def.txt"; 
    string loadPath                     = savePath;
    unsigned int numberOfInputs         = 1; 
    unsigned int numberOfOutputs        = 1; 
    unsigned int numberOfHiddenUnits    = 200; 
    double learningRate                 = 0.5; 
    double min                          = -1.2; 
    double max                          = 1.2; 
    if( atoi( argv[11] ) == 0 ) { randomizePosition = false; } else { randomizePosition = true; } 
    if( atoi( argv[12] ) == 0 ) { randomizeSTD = false; } else { randomizeSTD = true; } 
    double std                          = 0.5; 
    double std_min                      = 0.3; 
    double std_max                      = 0.8; 
    double percentage_1                 = 80.0; 
    double percentage_2                 = 50.0; 
    unsigned int iterationLimit         = 10; 

    unsigned int verboseLevel           = 1; 
    
    if( MODE == 1 ) { // XXX Training mode
        
        if( verboseLevel >= 1 ) {

        cout << "Initializing network with the following parameters: " << endl;
        cout << "    Input path\t\t\t"              << inputPath << endl; 
        cout << "    Target path\t\t\t"             << targetPath << endl; 
        cout << "    Result path\t\t\t"             << resultName << "_training.txt"  << endl;
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
            resultName,             // 3
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
        
    } else if( MODE == 2 ) { // XXX Testing mode
        
        if( verboseLevel >= 1 ) {
          
            cout << "Initializing network with the following parameters: " << endl;
            cout << "    Input path\t\t\t"              << inputPath << endl; 
            cout << "    Target path\t\t\t"             << targetPath << endl; 
            cout << "    Result path\t\t\t"             << resultName << "_testing.txt" << endl;
            cout << "    Load path\t\t\t"               << loadPath << endl;
            cout << "    percentage_1\t\t"              << percentage_1 << endl; 
            cout << "    percentage_2\t\t"              << percentage_2 << endl; 
            cout << "    Verbose level\t\t"             << verboseLevel << endl; 
            
        }
        
        EMGapproxRBF * app1 = new EMGapproxRBF(
            inputPath,              // 1
            targetPath,             // 2
            resultName,             // 3
            loadPath,               // 4
            percentage_1,           // 5
            percentage_2,           // 6
            verboseLevel );         // 7
        
    } else if( MODE == 3 ) { // XXX Combined mode
    
        if( verboseLevel >= 1 ) {

        cout << "Initializing network with the following parameters: " << endl;
        cout << "    Input path\t\t\t"              << inputPath << endl; 
        cout << "    Target path\t\t\t"             << targetPath << endl; 
        cout << "    Result name\t\t\t"             << resultName << endl;
        cout << "    Save path\t\t\t"               << savePath << endl;
        cout << "    Load path\t\t\t"               << loadPath << endl;
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
            resultName,             // 3
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
        
        delete app1;
        
        app1 = new EMGapproxRBF(
            inputPath,              // 1
            targetPath,             // 2
            resultName,             // 3
            loadPath,               // 4
            percentage_1,           // 5
            percentage_2,           // 6
            verboseLevel );         // 7
        
        
    } else {
        
        cout << "[ERROR] You must specify the mode of operation:" << endl;
        cout << "\tusage: ./start [MODE]" << endl;
        cout << "\t, where MODE 1: training, MODE 2: testing, MODE 3: training and testing" << endl;
        
    }
    


    return 0;

}
