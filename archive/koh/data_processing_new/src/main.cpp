#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include "EMGapproxESN.h"


unsigned int MODE = 1; /// MODE 1: training, MODE 2: testing, MODE 3: training and testing

int main( int argc, char ** argv ) {
    
    if( argc == 2 ) MODE = atoi( argv[1] );
    
    cout << "Running in MODE: " << MODE << "\n\n";
    
    string inputPath                    = "resources/Inputs12.txt"; //cos_f_7.0Hz_fs_10240_2.txt"; // 1   //
    string targetPath                   = "resources/TrainInput3.txt"; //cos_f_7.0Hz_fs_10240_2.txt"; // 2 TrainInput3.txt"; //
    string resultPath                   = "results/r_1"; // 3                            // The program will append "_training.txt" or "_testing.txt"
    string saveDir                      = "save_files/"; // 4
    unsigned int saveNum                = 1;    // 5
    unsigned int numberOfInputs         = 2;    // 6
    unsigned int numberOfOutputs        = 1;    // 7
    unsigned int numberOfHiddenUnits    = 100;  // 8
    unsigned int learningMode           = 1;    // 9
    unsigned int internalNonlinearity   = 2;    // 10
    unsigned int outputNonlinearity     = 0;    // 11
    double inputSparsity                = 50.0; // 12
    double internalSparsity             = 50.0; // 13
    double learningRate                 = 0.99; // 14
    double leak                         = 0.33; // 15
    double percentage_1                 = 50.0; // 16
    double percentage_2                 = 50.0; // 17
    unsigned int repetition             = 1;    // 18
    unsigned int iterationLimit         = 100000; // 19

    unsigned int verboseLevel           = 1;    // 20
    
    char delimiter                      = '\t'; // 21
    unsigned int numOfHeaderLines       = 0;    // 22

    if( MODE == 1 ) { // XXX Training mode

        if( verboseLevel >= 1 ) {

            cout << "Initializing network with the following parameters: " << endl;
            cout << "    Input path\t\t\t"              << inputPath << endl;
            cout << "    Target path\t\t\t"             << targetPath << endl;
            cout << "    Result path\t\t\t"             << resultPath << endl;
            cout << "    Save directory\t\t"            << saveDir << endl;
            cout << "    Save file number\t\t"          << saveNum << endl;
            cout << "    # of inputs\t\t\t"             << numberOfInputs << endl;
            cout << "    # of outputs\t\t"              << numberOfOutputs << endl;
            cout << "    # of hidden neurons\t\t"       << numberOfHiddenUnits << endl;
            cout << "    Learning mode\t\t"             << learningMode << endl;
            cout << "    Internal nonlinearity\t"       << internalNonlinearity << endl;
            cout << "    Output nonlinearity\t\t"       << outputNonlinearity << endl;
            cout << "    Input sparsity\t\t"            << inputSparsity << endl;
            cout << "    Internal sparsity\t\t"         << internalSparsity << endl;
            cout << "    learningRate\t\t"              << learningRate << endl;
            cout << "    Leakage\t\t\t"                 << leak << endl;
            cout << "    Percentage_1\t\t"              << percentage_1 << endl;
            cout << "    Percentage_2\t\t"              << percentage_2 << endl;
            cout << "    Repetition\t\t\t"              << repetition << endl;
            cout << "    Iteration limit\t\t"           << iterationLimit << endl;
            cout << "    Verbose level\t\t"             << verboseLevel << endl;

        }
        
        EMGapproxESN * app1 = new EMGapproxESN(
                inputPath,              // 1
                targetPath,             // 2
                resultPath,             // 3
                saveDir,                // 4
                saveNum,                // 5
                numberOfInputs,         // 6
                numberOfOutputs,        // 7
                numberOfHiddenUnits,    // 8
                learningMode,           // 9
                internalNonlinearity,   // 10
                outputNonlinearity,     // 11
                inputSparsity,          // 12
                internalSparsity,       // 13
                learningRate,           // 14
                leak,                   // 15
                percentage_1,           // 16
                percentage_2,           // 17
                repetition,             // 18
                iterationLimit,         // 19
                verboseLevel,           // 20
                delimiter,              // 21
                numOfHeaderLines );     // 22

    } else if( MODE == 2 ) { // XXX Testing mode

        if( verboseLevel >= 1 ) {

            cout << "Initializing network with the following parameters: " << endl;
            cout << "    Input path\t\t\t"              << inputPath << endl;
            cout << "    Target path\t\t\t"             << targetPath << endl;
            cout << "    Result path\t\t\t"             << resultPath << endl;
            cout << "    Save directory\t\t"            << saveDir << endl;
            cout << "    Save file number\t\t"          << saveNum << endl;
            cout << "    Percentage_1\t\t"              << percentage_1 << endl;
            cout << "    Percentage_2\t\t"              << percentage_2 << endl;
            cout << "    Verbose level\t\t"             << verboseLevel << endl;

        }

        EMGapproxESN * app1 = new EMGapproxESN(
                inputPath,              // 1
                targetPath,             // 2
                resultPath,             // 3
                saveDir,                // 4
                saveNum,                // 5
                percentage_1,           // 16
                percentage_2,           // 17
                verboseLevel,           // 20
                delimiter,              // 21
                numOfHeaderLines );     // 22

    } else if( MODE == 3 ) { // XXX Combined mode

        if( verboseLevel >= 1 ) {

            cout << "Initializing network with the following parameters: " << endl;
            cout << "    Input path\t\t\t"              << inputPath << endl;
            cout << "    Target path\t\t\t"             << targetPath << endl;
            cout << "    Result path\t\t\t"             << resultPath << endl;
            cout << "    Save directory\t\t"            << saveDir << endl;
            cout << "    Save file number\t\t"          << saveNum << endl;
            cout << "    # of inputs\t\t\t"             << numberOfInputs << endl;
            cout << "    # of outputs\t\t"              << numberOfOutputs << endl;
            cout << "    # of hidden neurons\t\t"       << numberOfHiddenUnits << endl;
            cout << "    Learning mode\t\t"             << learningMode << endl;
            cout << "    Internal nonlinearity\t"       << internalNonlinearity << endl;
            cout << "    Output nonlinearity\t\t"       << outputNonlinearity << endl;
            cout << "    Input sparsity\t\t"            << inputSparsity << endl;
            cout << "    Internal sparsity\t\t"         << internalSparsity << endl;
            cout << "    learningRate\t\t"              << learningRate << endl;
            cout << "    Leakage\t\t\t"                 << leak << endl;
            cout << "    Percentage_1\t\t"              << percentage_1 << endl;
            cout << "    Percentage_2\t\t"              << percentage_2 << endl;
            cout << "    Repetition\t\t\t"              << repetition << endl;
            cout << "    Iteration limit\t\t"           << iterationLimit << endl;
            cout << "    Verbose level\t\t"             << verboseLevel << endl;

        }

        EMGapproxESN * app1 = new EMGapproxESN(
                inputPath,              // 1
                targetPath,             // 2
                resultPath,             // 3
                saveDir,                // 4
                saveNum,                // 5
                numberOfInputs,         // 6
                numberOfOutputs,        // 7
                numberOfHiddenUnits,    // 8
                learningMode,           // 9
                internalNonlinearity,   // 10
                outputNonlinearity,     // 11
                inputSparsity,          // 12
                internalSparsity,       // 13
                learningRate,           // 14
                leak,                   // 15
                percentage_1,           // 16
                percentage_2,           // 17
                repetition,             // 18
                iterationLimit,         // 19
                verboseLevel,           // 20
                delimiter,              // 21
                numOfHeaderLines );     // 22

        delete app1;
        
        app1 = new EMGapproxESN(
                inputPath,              // 1
                targetPath,             // 2
                resultPath,             // 3
                saveDir,                // 4
                saveNum,                // 5
                percentage_1,           // 16
                percentage_2,           // 17
                verboseLevel,           // 20
                delimiter,              // 21
                numOfHeaderLines );     // 22
        
    }

    return 0;

}
