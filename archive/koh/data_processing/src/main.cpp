#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include "EMGapproxESN.h"

int main( int argc, char ** argv ) {

    if( argc == 1 ) { // XXX Manual mode

        //string inputPath                    = "resources/withoutCF/maturingsalmon/Input13_166_salmon_cod.txt";//"resources/withoutCF/maturingsalmon/Input13_89.txt"; //"resources/emg_data/Input_emg_24576.txt";
        //string targetPath                   = "resources/withoutCF/maturingsalmon/Target13_166_salmon_cod.txt";//"resources/withoutCF/maturingsalmon/Target13_89.txt"; //"resources/emg_data/Target_emg_24576.txt";

    	string inputPath                    = "resources/fishdata_combinedmodels/Table1/ModelE/Input13_Data3_5_test_35_124full.txt";
	    string targetPath                   = "resources/fishdata_combinedmodels/Table1/ModelE/Target_Data3_5_test_35_124full.txt";
        string resultPath                   = "results/result.txt";
        string saveDir                      = "save_files/";
        unsigned int saveNum                = 1;
        unsigned int numberOfInputs         = 2;//3;
        unsigned int numberOfOutputs        = 1;
        unsigned int numberOfHiddenUnits    = 50;
        unsigned int learningMode           = 1;
        unsigned int internalNonlinearity   = 2;
        unsigned int outputNonlinearity     = 0;
        double inputSparsity                = 50.0;
        double internalSparsity             = 50.0;
        double learningRate                 = 0.99;
        double leak                         = 1.0; //0.3
        double percentage_1                 = 13.0; //  85% of total data for "training" (save)
        double percentage_2                 = 100.0; // 100% of the remaining data (15% of total) for "validationUnits" (save & final) and another 0% of the remaining data (15% of total)  for "testing" (not use!)
        unsigned int repetition             = 3;
        unsigned int iterationLimit         = 5000;
        unsigned int verboseLevel           = 1;
        char delimiter                      = '\t';
        unsigned int numOfHeaderLines       = 0;

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

    } else if( argc == 8 ) { // XXX Automation mode - Loading network from save file

        string inputPath                    = argv[1];
        string targetPath                   = argv[2];
        string resultPath                   = argv[3];
        string saveDir                      = argv[4];
        unsigned int saveNum                = atoi( argv[5] );
        double percentage_1                 = atof( argv[6] );
        double percentage_2                 = atof( argv[7] );

        unsigned int verboseLevel           = 1;

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
                inputPath,
                targetPath,
                resultPath,
                saveDir,
                saveNum,
                percentage_1,
                percentage_2,
                verboseLevel );

    } else if( argc == 20 ) { // XXX Automation mode - Creating new network

        string inputPath                    = argv[1];
        string targetPath                   = argv[2];
        string resultPath                   = argv[3];
        string saveDir                      = argv[4];
        unsigned int saveNum                = atoi( argv[5] );
        unsigned int numberOfInputs         = atoi( argv[6] );
        unsigned int numberOfOutputs        = atoi( argv[7] );
        unsigned int numberOfHiddenUnits    = atoi( argv[8] );
        unsigned int learningMode           = atoi( argv[9] );
        unsigned int internalNonlinearity   = atoi( argv[10] );
        unsigned int outputNonlinearity     = atoi( argv[11] );
        double inputSparsity                = atof( argv[12] );
        double internalSparsity             = atof( argv[13] );
        double learningRate                 = atof( argv[14] );
        double leak                         = atof( argv[15] );
        double percentage_1                 = atof( argv[16] );
        double percentage_2                 = atof( argv[17] );
        unsigned int repetition             = atoi( argv[18] );
        unsigned int iterationLimit         = atoi( argv[19] );

        unsigned int verboseLevel           = 1;

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
            cout << "    Percentage_1\t\t"            << percentage_1 << endl;
            cout << "    Percentage_2\t\t"            << percentage_2 << endl;
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
                verboseLevel );         // 20

    }

    return 0;

}
