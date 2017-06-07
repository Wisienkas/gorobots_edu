#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include "EMGapproxESN.h"

#define networknumber 2

#include <unistd.h>

int main( int argc, char ** argv ) {

    cout << "#################################################" << endl;
    cout << "#        Welcome to EMGapproxESN training       #" << endl;
    cout << "#           for training an RC network          #" << endl;
    cout << "#################################################" << endl;


    string MyString = "Train_test_muIn_muout.txt";

    string stringContainer[networknumber];
    stringContainer[0] = "Train_test.txt";
	stringContainer[1] = "Train_test_muIn_muout.txt";


    for (int i = 0; i < networknumber; i++)
    {
        MyString = stringContainer[i];
        cout << endl;
        cout << "Using Training parameters: \n" << stringContainer[i] << endl;
        cout << endl;

        string inputPath = "";
        string targetPath = "";
        string resultPath = "";
        string saveDir = "";
        int startup = 3;

        if( startup == 1 ) { // XXX Manual mode

            string msg;
            string settings[19];

            ifstream myfile( MyString );
            if(myfile)
            {
                cout << "Reading settings file" << endl;
                int lineCount = 0;
                while(getline( myfile, msg ))
                {
                    std::string msg2 = msg;
                    int i = 0;
                    while(msg2[i++] != '{');
                    msg2 = msg2.substr(i, msg2.size()-i);
                    i = 0;
                    while(msg2[i++] != '}');
                    msg2 = msg2.substr(0, i-1);

                    settings[lineCount] = msg2;
                    lineCount++;
                    //cout << lineCount << endl;
                }
                myfile.close();
            }
            else
            {
                cout << "ERROR READING!!" << endl;
                cout << MyString << endl;
            }

            int temp_count = 0;
            while(temp_count <= 5)
            {
                cout << temp_count << ": " << settings[temp_count] << endl;
                temp_count++;
            }

            cout << " Train network Manual mode" << endl;

            inputPath                    = settings[0];
            targetPath                   = settings[1];
            resultPath                   = settings[2];
            saveDir                      = settings[3];
            unsigned int saveNum                = stoi(settings[4]);
            unsigned int numberOfInputs         = stoi(settings[5]);
            unsigned int numberOfOutputs        = stoi(settings[6]);
            unsigned int numberOfHiddenUnits    = stoi(settings[7]);
            unsigned int learningMode           = stoi(settings[8]);
            unsigned int internalNonlinearity   = stoi(settings[9]);
            unsigned int outputNonlinearity     = stoi(settings[10]);
            double inputSparsity                = stof(settings[11]);
            double internalSparsity             = stof(settings[12]);
            double learningRate                 = stof(settings[13]);
            double leak                         = stof(settings[14]);
            double percentage_1                 = stof(settings[15]);
            double percentage_2                 = stof(settings[16]);
            unsigned int repetition             = stoi(settings[17]);
            unsigned int iterationLimit         = stoi(settings[18]);
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

        } else if( startup == 2 ) { // XXX Automation mode - Loading network from save file

            string msg;
            string settings[8];

            ifstream myfile( MyString );
            if(myfile)
            {
                cout << "Reading settings file" << endl;
                int lineCount = 0;
                while(getline( myfile, msg ))
                {
                    std::string msg2 = msg;
                    int i = 0;
                    while(msg2[i++] != '{');
                    msg2 = msg2.substr(i, msg2.size()-i);
                    i = 0;
                    while(msg2[i++] != '}');
                    msg2 = msg2.substr(0, i-1);

                    settings[lineCount] = msg2;
                    lineCount++;
                    cout << lineCount << endl;
                }
                myfile.close();
            }
            else
            {
                cout << "ERROR READING!!" << endl;
                cout << MyString << endl;
            }

            int temp_count = 0;
            while(temp_count <= 5)
            {
                cout << temp_count << ": " << settings[temp_count] << endl;
                temp_count++;
            }

            cout << " validate network " << endl;

            inputPath                    = settings[0];
            targetPath                   = settings[1];
            resultPath                   = settings[2];
            saveDir                      = settings[3];
            unsigned int saveNum         = stoi(settings[4]);
            double percentage_1          = stof(settings[5]);
            double percentage_2          = stof(settings[6]);
            unsigned int verboseLevel    = 1;

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
                        percentage_1,           // 6
                        percentage_2,           // 7
                        verboseLevel );         // 8

        } else if( startup == 3 ) { // XXX Automation mode - Creating new network

            string msg;
            string settings[19];

            ifstream myfile( MyString );
            if(myfile)
            {
                cout << "Reading settings file" << endl;
                int lineCount = 0;
                while(getline( myfile, msg ))
                {
                    std::string msg2 = msg;
                    int i = 0;
                    while(msg2[i++] != '{');
                    msg2 = msg2.substr(i, msg2.size()-i);
                    i = 0;
                    while(msg2[i++] != '}');
                    msg2 = msg2.substr(0, i-1);

                    settings[lineCount] = msg2;
                    lineCount++;
                    cout << lineCount << endl;
                }
                myfile.close();
            }
            else
            {
                cout << "ERROR READING!!" << endl;
                cout << MyString << endl;
            }

            //        int temp_count = 0;
            //        while(temp_count <= 5)
            //        {
            //            cout << temp_count << ": " << settings[temp_count] << endl;
            //            temp_count++;
            //        }

            cout << " Train network Automation Mode " << endl;

            inputPath                    = settings[0];
            targetPath                   = settings[1];
            resultPath                   = settings[2];
            saveDir                      = settings[3];
            unsigned int saveNum                = stoi(settings[4]);
            unsigned int numberOfInputs         = stoi(settings[5]);
            unsigned int numberOfOutputs        = stoi(settings[6]);
            unsigned int numberOfHiddenUnits    = stoi(settings[7]);
            unsigned int learningMode           = stoi(settings[8]);
            unsigned int internalNonlinearity   = stoi(settings[9]);
            unsigned int outputNonlinearity     = stoi(settings[10]);
            double inputSparsity                = stof(settings[11]);
            double internalSparsity             = stof(settings[12]);
            double learningRate                 = stof(settings[13]);
            double leak                         = stof(settings[14]);
            double percentage_1                 = stof(settings[15]);
            double percentage_2                 = stof(settings[16]);
            unsigned int repetition             = stoi(settings[17]);
            unsigned int iterationLimit         = stoi(settings[18]);
            unsigned int verboseLevel           = 0;
            //        char delimiter                      = '\t';
            //        unsigned int numOfHeaderLines       = 0;

            //if( verboseLevel >= 1 ) {

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

            //}

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
                        verboseLevel/*,           // 20
                            delimiter,              // 21
                            numOfHeaderLines*/        // 22
                        );
        }
    }
    return 0;

}
