#include <iostream>
#include <vector>

#include "EMGapproxRBF.h"

#define verboseLevel 1
#define networknumber 1
using namespace std;

int main( int argc, char ** argv )
{

    cout << "#################################################" << endl;
    cout << "#        Welcome to EMGapproxRBF training       #" << endl;
    cout << "#           for training an RBF network         #" << endl;
    cout << "#################################################" << endl;

    string MyString = "Train_test";

    string stringContainer[networknumber];
    stringContainer[0] = "Train_test_xor.txt";
    //stringContainer[0] = "Train_test.txt";

    for (int i = 0; i < networknumber; i++)
    {
        MyString = stringContainer[i];
        cout << endl;
        cout << "Using Training parameters: \n" << stringContainer[i] << endl;
        cout << endl;

        string inputPath = "";
        string targetPath = "";
        string resultPath = "";
        string loadPath = "";
        string savePath = "";

        int startup = 1;

        if (startup == 2)
        {
            string msg;
            string settings[6];

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

            inputPath = settings[0];
            targetPath = settings[1];
            resultPath = settings[2];
            loadPath = settings[3];
            double percentage_1                 = stof(settings[4]);
            double percentage_2                 = stof(settings[5]);

            if( verboseLevel >= 1 )
            {

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

        }
        else if(startup == 1)
        {
            string msg;
            string settings[18];

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
                }
                myfile.close();
            }
            else
            {
                cout << "ERROR READING!!" << endl;
            }

            int temp_count = 0;
            while(temp_count <= 17)
            {
                cout << temp_count << ": " << settings[temp_count] << endl;
                temp_count++;
            }

            bool randomizePosition, randomizeSTD;

            cout << " Train network " << endl;

            inputPath = settings[0];
            targetPath = settings[1];
            resultPath = settings[2];
            savePath = settings[3];
            unsigned int numberOfInputs         = stoi(settings[4]);
            unsigned int numberOfOutputs        = stoi(settings[5]);
            unsigned int numberOfHiddenUnits    = stoi(settings[6]);
            double learningRate                 = stof(settings[7]);
            double min                          = stof(settings[8]);
            double max                          = stof(settings[9]);
            randomizePosition                   = stoi(settings[10]);
            randomizeSTD                        = stoi(settings[11]);
            double std                          = stof(settings[12]);
            double std_min                      = stof(settings[13]);
            double std_max                      = stof(settings[14]);
            double percentage_1                 = stof(settings[15]);
            double percentage_2                 = stof(settings[16]);
            unsigned int iterationLimit         = stoi(settings[17]);

            if( verboseLevel >= 1 )
            {
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

        }
    }
    return 0;
}
