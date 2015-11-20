/*
 * NeuralNavigationControl.cpp
 *
 *  Created on: 28.04.2014
 *      Author: Dennis Goldschmidt
 */

#include "NeuralNavigationControl.h"
using namespace std;


NeuralNavigationControl::NeuralNavigationControl(bool navi_opt){
	pi = new PathIntegration(18, 0, correlated, 0);
	compass_input = 0;
	speed_input = 0;
	//HDmat.set(18, 1);

	rand_dir = 0;
	steering_command = 0;
	navi_on = navi_opt;
	pi_only = false;
	global_count = 0;

	nnc_data.open("nnc_data.dat");
	nnc_data << "#timestep\t#HV_x\t#HV_y\t#HV_angle\n";

	hd_array.open("hd_array.dat");
}

NeuralNavigationControl::~NeuralNavigationControl(){
	nnc_data.close();
	hd_array.close();
}

double NeuralNavigationControl::getNormalRandom(double mean, double std){
	static random_device e { };
	static normal_distribution<double> d(mean, std);
	return d(e);
}

double NeuralNavigationControl::step_nnc(const vector<double> in_sensor, const vector< vector<double> > in_prepro){
	/// write into file
	nnc_data << global_count << "\t" << pi->getHV().x() << "\t" << pi->getHV().y() << "\t" << pi->getHVAngle() << endl;

	for(unsigned int index = 0; index < pi->getSubnet(HD)->N(); index++){
		//HDmat.val(index, 0) = pi->getOutput(index);
		//HDmat.norm_sqr();
		hd_array << pi->getSubnet(HD)->getOutput(index) << "\t";
	}
	hd_array << endl;

	if(!navi_on)
		return steering_command;

	if(global_count%turn_interval == 0)
		rand_dir = getNormalRandom(0.0, 1.0);

	compass_input = -atan2(in_sensor.at(BY_ori), in_sensor.at(BX_ori));
	if(compass_input < 0.)
		compass_input += 2*M_PI;
	speed_input = sqrt(pow(in_sensor.at(BX_spd),2) + pow(in_sensor.at(BY_spd),2));
	//printf("Angle = %f, Speed = %f\n", 180*compass_input/M_PI, speed_input);
	pi->step(compass_input, speed_input);
	//printf("HV angle = %f\n", 180*pi->getHVAngle()/M_PI);
	steering_command = rand_dir;

	global_count++;
	return steering_command;
}
