#include "grid.h"
#include "SLAMSolver.h"
#include "tools.h"
#include "rangeFinder.h"
#include "Models/occupancyGridMap.h"
#include "Models/occupancyGridBresenham.h"
#include "Models/odometryModel.h"
#include "Models/velocityModel.h"
#include "Models/likelihoodField.h"
#include "scanMatcher.h"

#include "alphanum.hpp"
#include "dirent.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/assign/list_of.hpp>
#include <cctype>


using namespace SLAM;
using namespace std;
using namespace boost;

//Get all files in given directory as string vector
vector<string> getFiles(string directory){
	vector<string> files;
	DIR *dir;
	struct dirent *ent;
	if ((dir = opendir (directory.c_str())) != NULL) {
		/* print all the files and directories within directory */
		while ((ent = readdir (dir)) != NULL)
			files.push_back(ent->d_name);
		closedir (dir);
	}
	else {
		/* could not open directory */
		perror ("Could not open directory");
	}
	return files;
}

//Extract first number from given string
int getNumberFromString(string str){
	int number = 0;
	string temp;
	for (unsigned int i=0; i < str.size(); i++)
	{
		//iterate the string to find the first "number" character
		//if found create another loop to extract it
		//and then break the current one
		//thus extracting the FIRST encountered numeric block
		if (isdigit(str[i]))
		{
			for (unsigned int a=i; a<str.size(); a++)
			{
				temp += str[a];
			}
			//the first numeric block is extracted
			break;
		}
	}

	istringstream stream(temp);
	stream >> number;

	return number;
}

//Read a single lrf scan
vector<double> readSingleLRF(string fileName, int& indexAcc){
	ifstream myfile(fileName.c_str());
	string line;
	vector<string> strs;
	vector<double> z;
	int measurement;
	indexAcc = -1;
	while(getline(myfile,line))
	{
		if(line[0]=='#')
			continue;
		if(line[0]=='!'){
			split(strs,line, boost::is_any_of("\t"));
			indexAcc = lexical_cast<int>(strs[1]);
			continue;
		}
		split(strs,line, boost::is_any_of("\t"));
		//Store measurement
		measurement = lexical_cast<int>(strs[1]);
		int index = lexical_cast<int>(strs[0]);
//		if(lexical_cast<int>(strs[0]) % 3 == 0){
		if(index >= 86 && index <= 595 && index % 2 == 0)
			//Convert to meter
			z.push_back((double)measurement / 1000.);
//		}
	}
	myfile.close();
	return z;
}

//Read all lrf scans
vector<int> readLRF(string dir, vector<vector<double> >& measurements, vector<int>& indicesAcc){

	measurements.clear();
	indicesAcc.clear();

	vector<int> indices;

	vector<string> files = getFiles(dir);
	vector<string> lrfFiles;
	int indexAcc;

	for(int i = 0; i < (int)files.size(); i++){
		size_t found  = files[i].find("lrf");
		if(found != string::npos)
			lrfFiles.push_back(files[i]);
	}

	sort(lrfFiles.begin(), lrfFiles.end(), doj::alphanum_less<std::string>());

	for(int i = 0; i < (int)lrfFiles.size(); i++){
		indices.push_back(getNumberFromString(lrfFiles[i]));
		vector<double> z = readSingleLRF(dir + lrfFiles[i], indexAcc);
		indicesAcc.push_back(indexAcc);
		measurements.push_back(z);
	}

	return indices;
}

vector<int> readAccelerations(string fileName, vector<vector<double> >& accelerations){

	accelerations.clear();

	ifstream myfile(fileName.c_str());
	string line;
	vector<string> strs;
	vector<int> indices;
	vector<double> a(3);
	while(getline(myfile,line))
	{
		if(line[0]=='#')
			continue;
		split(strs,line, boost::is_any_of("\t"));
		indices.push_back(lexical_cast<int>(strs[0]));
		//Forward velocity
		a[0] = lexical_cast<double>(strs[1]);
		a[1] = lexical_cast<double>(strs[2]);
		//Rotation (convert from deg/sec to rad/sec)
		a[2] = lexical_cast<double>(strs[6]) * M_PI/180.;
//		if(std::abs(a[2]) < 20. * M_PI/180.)
//			a[2] = 0;
		accelerations.push_back(a);
	}
	myfile.close();
	return indices;
}

vector<int> readVelocities(string fileName, vector<vector<double> >& velocities){

	velocities.clear();

	ifstream myfile(fileName.c_str());
	string line;
	vector<string> strs;
	vector<int> indices;
	vector<double> v(3);
	while(getline(myfile,line))
	{
		if(line[0]=='#')
			continue;
		split(strs,line, boost::is_any_of("\t"));
		indices.push_back(lexical_cast<int>(strs[0]));
		//Forward velocity
		v[0] = lexical_cast<double>(strs[4]);
		v[1] = lexical_cast<double>(strs[5]);
		//Rotation (convert from deg/sec to rad/sec)
		v[2] = lexical_cast<double>(strs[7]) * M_PI/180.;
		velocities.push_back(v);
	}
	myfile.close();
	return indices;
}

/*
 * Example of the FastSLAM algorithm for occupancy grid maps. The data is created via simulation in the LPZ robots
 * framework with the AMOSII robot. Use plotSLAM.py to visualize the results
 */

int main(int argc, char *argv[])
{
	Tools tools(1);
	vector<vector<double> > measurements;
	vector<int> indicesAcc;
	vector<int> indicesLRF = readLRF("../data/", measurements, indicesAcc);
//	vector<vector<double> > velocities;
//	vector<int> indices = readVelocities("../data/velocities.dat", velocities);
	vector<vector<double> > accelerations;
	vector<int> indices = readAccelerations("../data/dataIMU.dat", accelerations);

	//Generate map
	vector<double> start = boost::assign::list_of(-1.2)(-0.2);
	vector<double> end = boost::assign::list_of(0.36)(1.96);
	vector<double> resolution = boost::assign::list_of(0.05)(0.05);
	Grid<double> m(start,end,resolution,0.5);
	//
	//	vector<double> p;
	//	for(double y = 0.35; y > -2.25; y -= 0.01){
	//		p = boost::assign::list_of(-0.39)(y);
	//		m.setCellValue(p,1.);
	//		p = boost::assign::list_of(0.4)(y);
	//		m.setCellValue(p,1.);
	//	}
	//
	//	for(double x = -0.39; x < 0.4; x+=0.01){
	//		p = boost::assign::list_of(x)(-2.25);
	//		m.setCellValue(p,1);
	//	}
	//	for(double x = -0.02; x < 1.41; x+=0.01){
	//		double r = 0.81;
	//		double xM = 0.67;
	//		double yM = 1.45;
	//		double y = sqrt(r*r - (x - xM)*(x - xM)) + yM;
	//		p = boost::assign::list_of(x)(y);
	//		m.setCellValue(p,1);
	//	}
	//
	//	m.writeToFile("tunnel_highRes.map");
	//	return 1;

//	Grid<double> m("Complex_highRes.map");

	//Load parameters
	const char* cfgFileName =  "parameters.cfg";
	RangeFinder rf(cfgFileName);
	OdometryModel odometryModel(cfgFileName);
	LikelihoodField lField(cfgFileName,&rf);
	OccupancyGridBresenham occMap(cfgFileName,&rf);
	KLDParameters kldParam(cfgFileName);
	ScanMatcher sm(&rf, cfgFileName);
	
	//Set initial belief
	double x,y;
	//Boost random stuff
	double variance = 0.;

	double startX = 0.0;
	double startY = 0.;
	double orientation = M_PI/2.;
	int numberParticles = 1000;
	vector<Particle> particles(numberParticles);
	vector<double> s;
	for(int i = 0; i < numberParticles; i++){
		//    				x = tools.sampleUniform(-7,27);
		//    				y = tools.sampleUniform(-22,22);
		x = tools.sampleNormal(startX, variance);
		y = tools.sampleNormal(startY, variance);
		s =boost::assign::list_of(x)(y)(orientation);
		particles[i] = Particle(s,1./numberParticles,m);
	}

	//Init robot
	SLAMSolver mySLAM;
	mySLAM = SLAMSolver(particles,&lField, &odometryModel, &occMap, &kldParam);
//	mySLAM.disableKLD();
	ostringstream oss;
	clock_t t_begin = clock();
	//Use plotSLAM.py
	ofstream out("data/particles.dat");
	out << "#Index\tPosX\tPosY\tYaw"<<endl;

	int indexLRF = 1;
	double dt = 1./235.;//0.074;
	vector<double> currState = boost::assign::list_of(startX)(startY)(orientation);
	vector<double> refState = currState;
	vector<double> control;
	for(int i = 0; i < (int)accelerations.size();i++){
		double aX = cos(currState[2]) * accelerations[i][0] + sin(currState[2]) * accelerations[i][1];
		double aY = -sin(currState[2]) * accelerations[i][0] + cos(currState[2]) * accelerations[i][1];
		currState[0] += 0.5 * aX * dt * dt;
		currState[1] += 0.5 * aY * dt * dt;
		currState[2] += accelerations[i][2] * dt;
		
		if(indicesAcc[indexLRF] == indices[i]){
			cout << "Step "<<indexLRF<<" of "<<indicesLRF.size()<<endl;
			sm.matchScans(measurements[indexLRF], measurements[indexLRF-1], currState, refState);
			control = currState;
			control.insert(control.end(), refState.begin(), refState.end());
			mySLAM.fastSlamStep(control,measurements[indexLRF]);
//			mySLAM.MCL_step(control, measurements[indexLRF],m);

			particles = mySLAM.getBelief();
			double maxWeight = 0;
			int index = -1;
			cout <<"Number particles: " << particles.size()<<endl;
			for(unsigned int j = 0; j < particles.size(); j++){
				if(particles[j].weight > maxWeight){
					maxWeight = particles[j].weight;
					index = j;
				}
			}
			oss << "data/map_"<<indexLRF<<".map";
			particles[index].map.writeToFile(oss.str());
			oss.str("");
			out <<indexLRF<<"\t"<< particles[index].state[0] << "\t"<<particles[index].state[1]<<"\t"<< particles[index].state[2]<<endl;
			oss << "data/particles_"<<indexLRF<<".dat";
			mySLAM.writeBelief(oss.str());
			oss.str("");

			currState = particles[index].state;
			indexLRF++;
			refState = currState;
		}
	}
	out.close();
	clock_t t_end = clock();
	double elapsed_secs = double(t_end - t_begin) / CLOCKS_PER_SEC;
	cout<<"Time: "<<elapsed_secs<<endl;
}
