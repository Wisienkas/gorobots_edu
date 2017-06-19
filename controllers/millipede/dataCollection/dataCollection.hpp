#ifndef _DATA_COLLECTION
#define _DATA_COLLECTION

/*/
#define BOOST_FILESYSTEM_VERSION 3
#define BOOST_FILESYSTEM_NO_DEPRECATED
#include <boost/filesystem.hpp> /**/
#include <ctime>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdlib.h>     /* atof */
#include <vector>


void get_time();

void saveSensorMotorData(std::vector<double> sensor, std::vector<double> motor, int lineIndex, int fileIndex);

// Not used in this simulation
//void get_all(const boost::filesystem::path& root, const std::string& ext, std::vector<boost::filesystem::path>& ret);
void saveWeights(std::vector<double> w, std::vector<double> lastw, double *input, int lineIndex, int fileIndex);
void loadWeights(std::vector<double> *w, int *lineIndex, int fileIndex);
void saveAllData(const std::vector<double> &sensor, const std::vector<double> &motor, const std::vector<double> &odom, int lineIndex, int fileIndex);


#endif
