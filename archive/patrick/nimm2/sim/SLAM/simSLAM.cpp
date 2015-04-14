#include "grid.h"
#include "SLAMSolver.h"
#include "tools.h"
#include "rangeFinder.h"
#include "occupancyGridMap.h"
#include "occupancyGridBresenham.h"
#include "odometryModel.h"
#include "velocityModel.h"
#include "likelihoodField.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/assign/list_of.hpp>

using namespace SLAM;
using namespace std;
using namespace boost;

void addMeasurement(double v1, double v2, double v3, vector<vector<double> >& z){
	double arr[] = {v1,v2,v3};
	vector<double> vec (arr, arr + sizeof(arr) / sizeof(arr[0]) );
	z.push_back(vec);
}

vector<int> readAcceleration(Tools* tools, string fileName, double varianceAcc,vector<vector<double> >& acceleration){
	acceleration.clear();
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
		a[0] = lexical_cast<double>(strs[1]) + tools->sampleNormal(varianceAcc);
		a[1] = lexical_cast<double>(strs[2]) + tools->sampleNormal(varianceAcc);
		a[2] = lexical_cast<double>(strs[8]) + tools->sampleNormal(varianceAcc);
		acceleration.push_back(a);
	}
	myfile.close();
	return indices;
}

vector<int> readData(Tools* tools, string fileName, 	double variancePos, double varianceVelo, double varianceRot,
		vector<vector<double> >& states, vector<vector<double> >& measurements,
		vector<vector<double> >& velocities){

	states.clear();
	measurements.clear();
	velocities.clear();

	ifstream myfile(fileName.c_str());
	string line;
	vector<string> strs;
	vector<double> z;
	vector<int> indices;
	vector<double> s(3);
	vector<double> v(2);
	while(getline(myfile,line))
	{
		if(line[0]=='#')
			continue;
		split(strs,line, boost::is_any_of("\t"));
		indices.push_back(lexical_cast<int>(strs[0]));
		s[0] = lexical_cast<double>(strs[1]) + tools->sampleNormal(variancePos);
		s[1] = lexical_cast<double>(strs[2]) + tools->sampleNormal(variancePos);
		s[2] =  lexical_cast<double>(strs[3]);
		states.push_back(s);
		v[0] = lexical_cast<double>(strs[4]) + tools->sampleNormal(varianceVelo);
		//Entry with index 5 is velocity perpendicular to forward
		v[1] = lexical_cast<double>(strs[6]) + tools->sampleNormal(varianceRot);
		velocities.push_back(v);
		for(int j = 9; j < 24;j++){
			try{
				z.push_back(lexical_cast<double>(strs[j]));
			}
			catch(bad_lexical_cast const&){
				cout << "Error casting value to double"<< endl;
				continue;
			}
		}
		measurements.push_back(z);
		z.clear();

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

	//Filenames
	string fileNameSensor = "sensorData/sensorData.dat";
	string fileNameAcc = "sensorData/accData.dat";

	//Which motion model
	int useVelocity = false;

	//Model noise
	double variancePos = 0.2;
	double varianceVelo = 0.0;
	double varianceRot = 0.0;
	double varianceAcc = 0.0;

	vector<vector<double> > states;
	vector<vector<double> > measurements;
	vector<vector<double> > angularVelocities;
	vector<vector<double> > acceleration;

	vector<int> indicesAcc = readAcceleration(&tools, fileNameAcc, varianceAcc, acceleration);
	vector<int> indices = readData(&tools, fileNameSensor, variancePos, varianceVelo, varianceRot,
			states, measurements, angularVelocities);

	//Load Map
//	Grid<double> m("circle.map");

	//Generate map
	vector<double> start = boost::assign::list_of(-7.)(-22.);
	vector<double> end = boost::assign::list_of(27.)(22.);
	vector<double> resolution = boost::assign::list_of(1.)(1.);
	Grid<double> m(start,end,resolution,0.5);

	//Load parameters
	const char* cfgFileName =  "parameters.cfg";
	RangeFinder rf(cfgFileName);
	OdometryModel odometryModel(cfgFileName);//
	VelocityModel velocityModel(cfgFileName);
	LikelihoodField lField(cfgFileName,&rf);
	//	OccupancyGridMap occMap(cfgFileName,&rf);
	OccupancyGridBresenham occMap(cfgFileName,&rf);
	KLDParameters kldParam(cfgFileName);

	//Set initial belief
	double x,y;
	//Boost random stuff
	double variance = 0.;
	double startX = 0.0;
	double startY = 0.10;
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
	if(useVelocity)
		mySLAM = SLAMSolver(particles,&lField, &velocityModel, &occMap, &kldParam);
	else
		mySLAM = SLAMSolver(particles,&lField, &odometryModel, &occMap, &kldParam);
	//	mySLAM. disableKLD();
	ostringstream oss;
	clock_t t_begin = clock();
	//Use plotSLAM.py
	ofstream out("data/particles.dat");
	out << "#Index\tPosX\tPosY\tTruePosX\tTruePosY\tYaw\tTrueYaw"<<endl;

	//Use acceleration values
//	ofstream testOut("orientationAcc.dat");
//	vector<vector<double> > velocities;
//	velocities.push_back(boost::assign::list_of(angularVelocities[0][0])(angularVelocities[0][1]));
//	vector<double> control;
//	double dt = 0.01;
//	double v;
//	for(int i = 1, indexSens = 1; i < (int)indicesAcc.size(); i++){
//		v = velocities[i-1][0] + acceleration[i-1][0] * dt;
//		velocities.push_back(boost::assign::list_of(v)(acceleration[i-1][2]));
//		if(indicesAcc[i] == indices[indexSens]){
//			cout << "Step "<<indexSens<<" of "<<states.size()<<endl;
//			cout <<"Number particles: " << particles.size()<<endl;
//			control = velocities[i];
//			mySLAM.fastSlamStep(control,measurements[indexSens]);
////			mySLAM.MCL_step(control, measurements[indexSens],m);
//			particles = mySLAM.getBelief();
//			double maxWeight = 0;
//			int index = -1;
//			for(int j = 0; j < (int)particles.size(); j++){
//				if(particles[j].weight > maxWeight){
//					maxWeight = particles[j].weight;
//					index = j;
//				}
//			}
//			testOut << particles[index].state[2]<<endl;
//			oss << "data/map_"<<indexSens<<".map";
//			particles[index].map.writeToFile(oss.str());
//			oss.str("");
//			out <<indexSens<<"\t"<< particles[index].state[0] << "\t"<<particles[index].state[1]<<"\t"<<
//					states[indexSens][0]<<"\t"<<states[indexSens][1]<<"\t"<<particles[index].state[2]<<"\t"<<states[indexSens][2]<<endl;
//
//			indexSens++;
//		}
//	}

		vector<double> control;
		ofstream testOut("orientationVelo.dat");
		for(unsigned int i = 0; i < states.size();i++){
			cout << "Step "<<i<<" of "<<states.size()<<endl;
			if(i != 0){
				if(useVelocity){
					control = angularVelocities[i];
				}
				else{
					control = states[i];
					control.insert(control.end(), states[i-1].begin(), states[i-1].end());
				}

				mySLAM.fastSlamStep(control, measurements[i]);
//				mySLAM.MCL_step(control, measurements[i],m);
			}
			particles = mySLAM.getBelief();
			double maxWeight = 0;
			int index = -1;
			//		cout << "\rPercent: "<<(double)i/(double)states.size()<<;
			cout <<"Number particles: " << particles.size()<<endl;
			for(unsigned int j = 0; j < particles.size(); j++){
				if(particles[j].weight > maxWeight){
					maxWeight = particles[j].weight;
					index = j;
				}
			}
			testOut << particles[index].state[2]<<endl;
			oss << "data/map_"<<i<<".map";
			particles[index].map.writeToFile(oss.str());
			oss.str("");
			out <<i<<"\t"<< particles[index].state[0] << "\t"<<particles[index].state[1]<<"\t"<<
					states[i][0]<<"\t"<<states[i][1]<<"\t"<<particles[index].state[2]<<"\t"<<states[i][2]<<endl;
			oss << "data/particles_"<<i<<".dat";
			mySLAM.writeBelief(oss.str());
			oss.str("");

		}
	out.close();
	clock_t t_end = clock();
	double elapsed_secs = double(t_end - t_begin) / CLOCKS_PER_SEC;
	cout<<"Time: "<<elapsed_secs<<endl;
}
