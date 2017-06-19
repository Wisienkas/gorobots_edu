#include "dataCollection.hpp"
#define ERROR std::cout << "passed over here.. \n";

#define DATA_FOLDER "/home/gimait/Programs/lpzrobots/gorobots/projects/millipede/data/"

using namespace std;

/*/ return the filenames of all files that have the specified extension
// in the specified directory and all subdirectories
void get_all(const boost::filesystem::path& root, const string& ext, vector<boost::filesystem::path>& ret)
{
    if(!boost::filesystem::exists(root) || !boost::filesystem::is_directory(root)) return;

    boost::filesystem::recursive_directory_iterator it(root);
    boost::filesystem::recursive_directory_iterator endit;

    while(it != endit)
    {
        if(boost::filesystem::is_regular_file(*it) && it->path().extension() == ext) ret.push_back(it->path().filename());
        ++it;

    }

}
//*/

// return a string with the current date and time on the computer
void getTime() {
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    std::cout << (now->tm_year + 1900) << '-'
         << (now->tm_mon + 1) << '-'
         <<  now->tm_mday
         << std::endl;

}


void saveWeights( std::vector<double> w, std::vector<double> lastw, double *input, int lineIndex, int fileIndex)
{
    //Get change in weights
    double deltaWeight = 0;

    for(int i = 0; i < w.size(); i++){
        deltaWeight+=(w[i]-lastw[i])*(w[i]-lastw[i]);
    }

    deltaWeight/=(w.size()*w.size());

    ofstream wPlot;
    std::stringstream ss;
    ss << "/home/rsd-group1/weightData" << fileIndex << ".txt";
    string toSave = ss.str();
    wPlot.open(toSave.c_str(), fstream::app);
    wPlot << lineIndex << " ";

    for(int i = 0; i < w.size(); i++){
        wPlot << w[i] << " ";
    }
    wPlot << deltaWeight << " ";            //Weight variation

    for(int i = 0; i < w.size(); i++)
        wPlot << input[i] << " ";

    wPlot << endl;

    wPlot.close();


return;
}


void loadWeights(std::vector<double> *w, int *lineIndex, int fileIndex){
    std::stringstream ss;
    ss << "/home/rsd-group1/weightData" << fileIndex << ".txt";
    string saved = ss.str();
    string line, last;
    ifstream weightData(saved);


    while (getline(weightData,line))
    {
       last=line;
    }

    std::stringstream iss(last);

    iss >> *lineIndex >> w[0][0]>> w[0][1]>> w[0][2]>> w[0][3]>> w[0][4]>> w[0][5]>> w[0][6]>> w[0][7];
}



void saveSensorMotorData(std::vector<double> sensor, std::vector<double> motor, int lineIndex, int fileIndex){

    ofstream wPlot;
    std::stringstream ss;
    ss << DATA_FOLDER << "sensorMotordata" << fileIndex << ".txt";
    string toSave = ss.str();
    wPlot.open(toSave.c_str(), fstream::app);
    wPlot << lineIndex << " ";

    for(int i = 0; i < sensor.size(); i++){
        wPlot << sensor[i] << " ";
    }

    for(int i = 0; i < motor.size(); i++)
        wPlot << motor[i] << " ";

    wPlot << endl;

    wPlot.close();


return;
}

void saveAllData(const std::vector<double> &sensor, const std::vector<double> &motor, const std::vector<double> &odom, int lineIndex, int fileIndex){

    ofstream wPlot;
    std::stringstream ss;
    ss << DATA_FOLDER << "posVeldata" << fileIndex << ".txt";
//    string toSave = ss.str();
    string root = DATA_FOLDER, name = "posVeldata", extension = ".txt";

    wPlot.open(root + name + std::to_string(fileIndex) + extension , fstream::app);
    wPlot << lineIndex << " ";

    for(int i = 0; i < odom.size(); i++){
        wPlot << odom[i] << " ";
    }

    for(int i = 0; i < sensor.size(); i++){
        wPlot << sensor[i] << " ";
    }

    for(int i = 0; i < motor.size(); i++)
        wPlot << motor[i] << " ";



    wPlot << endl;

    wPlot.close();


return;
}















