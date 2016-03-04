#include "cnnet.h"
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
//#include "console.h"
#include <fcntl.h>

////Save files /read file
//#include <iostream>
//#include <fstream>
//#include <string.h>


using namespace std;

using namespace runbot;

const double cNNet::expp = exp(-1);     // 1 ms .. means xep(-1/tau) ,for knee neurons,
// tau is bigger than that of hips, in order to
// eliminate the noise of motor neuron output.
const double cNNet::exppp = exp(-1);    //  1 ms.. for hip neurons

const double cNNet::eps = 1e-12;        // Precision for comparisons of floats.

const unsigned int cNNet::writer_columns = 13;
const string cNNet::writer_column_names("time u_hl_em u_hl_fm u_hr_em u_hr_fm u_kl_em u_kl_fm u_kr_em u_kr_fm u_gl u_gr u_al u_ar");


double alph;
double phi;
double WeightH1_H1;
double WeightH2_H2;
double WeightH1_H2;
double WeightH2_H1;

double BiasH1;
double BiasH2;

double activityH1;
double activityH2;

double outputH1;
double outputH2;


cNNet::cNNet ( cGaitProfile* profile_ )
: gait(profile_),
  array_writer( "nnet_out", writer_columns, writer_column_names ) {

	serialPlot2.open("savecnnet.dat");
	//std::cout<<"cnnet function "<<std::endl;

	alph = 1.5;//1.5;
	phi = 0.25;

	activityH1=0;
	activityH2=0;

	outputH1 = 0.01;
	outputH2 = 0.01;


	countpiezo = 0;

	u_al = 0;
	u_ar = 0;
	elf_al = 100;//4
	elf_ar = 100;//4
	u_gl = 1;
	u_gr = 0;


	angle_hl_pre=0;
	angle_hl_now=0;
	angle_hr_pre=0;
	angle_hr_now=0;


	motorvolt_hl = 0;
	motorvolt_hr = 0;
	motorvolt_kl = 0;
	motorvolt_kr = 0;

	angle_hl_low_pass = 90;
	angle_hr_low_pass = 90;
	angle_kl_low_pass = 180;
	angle_kr_low_pass = 180;

	u_kl_es_low_pass_pre = 0;
	u_kl_es_low_pass = 0;
	u_kr_es_low_pass_pre = 0;
	u_kr_es_low_pass = 0;
}



void cNNet::update_thresholds() {


}


void cNNet::update_nnet ( std::valarray< double > input_data ) {

	threshold_al = 100; //degrees
	threshold_ar = 100; //degrees


	// Transfer new array to old names
	int leftpiezo   = (int)(input_data[2]);
	int rightpiezo  = (int)(input_data[3]);
	double angle_hl =       input_data[4];
	double angle_hr =       input_data[5];
	double angle_kl =       input_data[6];
	double angle_kr =       input_data[7];


	// Another low pass filter

	double gain_low_pass = 0.55;
	angle_hl_low_pass_pre = angle_hl_low_pass;
	angle_hl_low_pass = (1-gain_low_pass)*angle_hl+ angle_hl_low_pass_pre*gain_low_pass;

	angle_hr_low_pass_pre = angle_hr_low_pass;
	angle_hr_low_pass = (1-gain_low_pass)*angle_hr+ angle_hr_low_pass_pre*gain_low_pass;

	angle_kl_low_pass_pre = angle_kl_low_pass;
	angle_kl_low_pass = (1-gain_low_pass)*angle_kl+ angle_kl_low_pass_pre*gain_low_pass;

	angle_kr_low_pass_pre = angle_kr_low_pass;
	angle_kr_low_pass = (1-gain_low_pass)*angle_kr+ angle_kr_low_pass_pre*gain_low_pass;

	angle_kl_now = angle_kl;
	angle_kr_now = angle_kr;

	if (u_gl==1) {
		//3) Enter here
		//u_al = 1./(1+exp(elf_al*(threshold_al - angle_hl_low_pass))); // Stretch receptor sensor left
		// u_al starts to decrease from 1 to zero if it below the threshold
		if(angle_hl_low_pass>threshold_al) // Stretch receptor sensor left
			u_al = 1;
		else
			u_al=0;

	} else {
			if (((int(angle_hl_low_pass*10))>(int(angle_hl_low_pass_pre*10))) &&
					((int(angle_hl_low_pass*10))>(int(threshold_al*10)))) {
				u_al=1;
			}
	}

	if (u_gr==1) {
		if(angle_hr_low_pass>threshold_ar) // Stretch receptor sensor right
					u_ar = 1;
				else
					u_ar = 0;

		} else {
			if (((int(angle_hr_low_pass*10))>(int(angle_hr_low_pass_pre*10))) &&
					((int(angle_hr_low_pass*10))>(int(threshold_ar*10)))) {
				u_ar=1;
			}
	}



	//State machine control///

	// piezos : 2048 = 0v (foot contact the ground) .... 4096 (off ground)

	// Left leg at front
	if (u_al>0) {                               // Touch the ground of left leg
		if (int(angle_kl)>170) {                        // knee left has to be Straigth
			if (abs(leftpiezo-2048)<300) {              // Foot signal ~ ground contact if it touch the ground then enter here!
				u_gl=1;
				u_gr=0;

			}
		}
	}


	// Right leg at front
	if (u_ar>0) {                               // Touch the ground of right leg
		//1)  Enter this loop
		if (int(angle_kr)>170) {                        // knee right has to be Straigth
			//2)  Enter this loop
			if (abs(rightpiezo-2048)<300) {             // Foot signal ~ ground contact if it touch the ground then enter here!
				u_gl=0;
				u_gr=1;

			}
		}
	}


	d_u_gr = u_gr;
	d_u_gl = u_gl;



	//Hip joint control/////////////////
	if (u_gr==1){
		state_u_hr_em = 0.0;
		state_u_hr_fm = 1.0;

		state_u_hl_em = 1.0;
		state_u_hl_fm = 0.0;
	}


	if (u_gl==1){
		state_u_hr_em = 1.0;
		state_u_hr_fm = 0.0;

		state_u_hl_em = 0.0;
		state_u_hl_fm = 1.0;
	}

	//Knee joint control/////////////////

	// Hip Left move forward beyond the threshold then the knee left is extending
	//if (int(angle_hl*10)>int(threshold_al*10)) {
	//if (u_al>0.5) {
	if (angle_hl_low_pass>threshold_al){
		state_u_kl_em = 1.0;
		state_u_kl_fm = 0.0;
	}

	// Hip Left move backward and below the threshold then the knee left is hold
	//if (int(angle_hl_now*10)<int(angle_hl_pre*10)){//&&int(angle_hl*10)<int(threshold_al*10)) {
	if (angle_hl_low_pass<angle_hl_low_pass_pre){//&&int(angle_hl*10)<int(threshold_al*10)) {
		state_u_kl_em = 1.0; // holding power
		state_u_kl_fm = 0.0;
	}

	// Hip Left move forward and below the threshold then the knee left is flexing
	//if (int(angle_hl_now*10)>int(angle_hl_pre*10)&&int(angle_hl*10)<int(threshold_al*10)&&(u_gr==1)) {
	if (angle_hl_low_pass>angle_hl_low_pass_pre&&angle_hl_low_pass<threshold_al&&(u_gr==1)) {
		state_u_kl_em = 0.0;
		state_u_kl_fm = 1.0;
	}

	// Hip Right move forward beyond the threshold then Extend the knee left
	//if (int(angle_hr*10)>int(threshold_ar*10)) {
	if (angle_hr_low_pass>threshold_ar) {
		state_u_kr_em = 1.0;
		state_u_kr_fm = 0.0;
	}
	// Hip Right move backward and below the threshold then the knee left is off
	//if (int(angle_hr_now*10)<int(angle_hr_pre*10)){//&&int(angle_hr*10)<int(threshold_ar*10)) {
	if (angle_hr_low_pass<angle_hr_low_pass_pre){//&&int(angle_hr*10)<int(threshold_ar*10)) {
		state_u_kr_em = 1.0; // holding power
		state_u_kr_fm = 0.0;
	}
	// Hip Right move forward and below the threshold then the knee right is flexing
	//if (int(angle_hr_now*10)>int(angle_hr_pre*10)&&int(angle_hr*10)<int(threshold_ar*10)&&(u_gl==1)) {
	if (angle_hr_low_pass>angle_hr_low_pass_pre&&angle_hr_low_pass<threshold_ar&&(u_gl==1)) {
		state_u_kr_em = 0.0;
		state_u_kr_fm = 1.0;
	}



	/*std::cout << "u_gr>>>>" << u_gr << std::endl;
	std::cout << "u_gl>>>>" << u_gl << std::endl;
	std::cout << "u_al>>>>" << u_al << std::endl;
	std::cout << "u_ar>>>>" << u_ar << std::endl;*/


	state_motorvolt_hl = gait->gain_hl_ext()  * state_u_hl_em -
			gait->gain_hl_flex() * state_u_hl_fm;
	state_motorvolt_hr = gait->gain_hr_ext()  * state_u_hr_em -
			gait->gain_hr_flex() * state_u_hr_fm;


	state_motorvolt_kl = gait->gain_kl_ext()  * state_u_kl_em -
			gait->gain_kl_flex() * state_u_kl_fm;
	state_motorvolt_kr = gait->gain_kr_ext()  * state_u_kr_em -
			gait->gain_kr_flex() * state_u_kr_fm;



	serialPlot2<<angle_hl<<' '<<angle_hr<<' '<<u_hr_fm<<' '<<u_hr_em<<' '<<u_kl_fm<<' '<<u_kl_em<<' '<<u_kr_fm<<' '<<u_kr_em<<' '<<u_al<<' '<<u_ar<<' '<<u_gl<<' '<<u_gr<<' '<<motorvolt_kl<<' '<<motorvolt_kr<<' '<<motorvolt_hl<<' '<<motorvolt_hr<<' '<<input_data[6]<<' '<<input_data[7]<<endl;


}


std::valarray< double > cNNet::update_motorvoltages() {

	motorvolt_hl = state_motorvolt_hl;
	motorvolt_hr = state_motorvolt_hr;


	motorvolt_kl = state_motorvolt_kl;
	motorvolt_kr = state_motorvolt_kr;

	valarray<double> result(4);
	result[0] = motorvolt_hl; // correct! +1 = forward, -1 = backward
	result[1] = motorvolt_hr; // correct! +1 = forward, -1 = backward

	result[2] = motorvolt_kl; // correct! +1 = forward, -1 = backward
	result[3] = motorvolt_kr; // correct! +1 = forward, -1 = backward


	std::cout<<"threshold_al: "<<threshold_al<<std::endl;

	return result;
}


valarray< double >  cNNet::update(valarray< double > input_data, int time) {

	// Update possible gait profile changes
	update_thresholds();

	// Do one iteration of neural network update
	update_nnet(input_data);

	// Dump the output vector to a file
	dump_output_vector(time);

	// Translate neuron activation to motor voltages
	return update_motorvoltages();
}


void cNNet::dump_output_vector ( int time ) {

	valarray<double> output_vector(writer_columns);
	output_vector[ 0] = time;
	output_vector[ 1] = u_hl_em;
	output_vector[ 2] = u_hl_fm;
	output_vector[ 3] = u_hr_em;
	output_vector[ 4] = u_hr_fm;
	output_vector[ 5] = u_kl_em;
	output_vector[ 6] = u_kl_fm;
	output_vector[ 7] = u_kr_em;
	output_vector[ 8] = u_kr_fm;
	output_vector[ 9] = u_gl;
	output_vector[10] = u_gr;
	output_vector[11] = u_al;
	output_vector[12] = u_ar;

	array_writer.write(output_vector);
}


std::valarray< double > runbot::cNNet::get_neuron_outputs() {
	valarray<double> output_vector(12);
	output_vector[ 0] = u_hl_em;
	output_vector[ 1] = u_hl_fm;
	output_vector[ 2] = u_hr_em;
	output_vector[ 3] = u_hr_fm;
	output_vector[ 4] = u_kl_em;
	output_vector[ 5] = u_kl_fm;
	output_vector[ 6] = u_kr_em;
	output_vector[ 7] = u_kr_fm;
	output_vector[ 8] = u_gl;
	output_vector[ 9] = u_gr;
	output_vector[10] = u_al;
	output_vector[11] = u_ar;
	return output_vector;
}


