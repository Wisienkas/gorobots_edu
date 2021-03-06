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

	elf_knee = 4;//2
	elf_hip_es=4;//2
	elf_hip_fs=4;//2

	//Hip Left
	y_hl_ei=0;
	y_hl_fi=0;
	y_hl_em=0;
	y_hl_fm=0;
	u_hl_es=0;
	u_hl_fs=0;
	u_hl_ei=0;
	u_hl_fi=0;
	u_hl_em=0;
	u_hl_fm=0;
	w_hl_es_em=10;
	w_hl_fs_fm=10;
	w_hl_ei_em=0.1;
	w_hl_fi_fm=0.1;
	w_hl_ei_fm=-10;
	w_hl_fi_em=-10;
	elf_hl_es=elf_hip_es;
	elf_hl_fs=elf_hip_fs;

	//Hip Right
	y_hr_ei=0;
	y_hr_fi=0;
	y_hr_em=0;
	y_hr_fm=0;
	u_hr_es=0;
	u_hr_fs=0;
	u_hr_ei=0;
	u_hr_fi=0;
	u_hr_em=0;
	u_hr_fm=0;
	w_hr_es_em=10;
	w_hr_fs_fm=10;
	w_hr_ei_em=0.1;
	w_hr_fi_fm=0.1;
	w_hr_ei_fm=-10;
	w_hr_fi_em=-10;
	elf_hr_es=elf_hip_es;
	elf_hr_fs=elf_hip_fs;

	//Knee Left
	y_kl_ei=0;
	y_kl_fi=0;
	y_kl_em=0;
	y_kl_fm=0;
	u_kl_es=0;
	u_kl_fs=0;
	u_kl_ei=0;
	u_kl_fi=0;
	u_kl_em=0;
	u_kl_fm=0;
	w_kl_es_em=10;
	w_kl_fs_fm=10;
	w_kl_ei_em=0.1;
	w_kl_fi_fm=0.1;
	w_kl_ei_fm=-10;
	w_kl_fi_em=-10;
	elf_kl_es=elf_knee;
	elf_kl_fs=elf_knee;

	//Knee Right
	y_kr_ei=0;
	y_kr_fi=0;
	y_kr_em=0;
	y_kr_fm=0;
	u_kr_es=0;
	u_kr_fs=0;
	u_kr_ei=0;
	u_kr_fi=0;
	u_kr_em=0;
	u_kr_fm=0;
	w_kr_es_em=10;
	w_kr_fs_fm=10;
	w_kr_ei_em=0.1;
	w_kr_fi_fm=0.1;
	w_kr_ei_fm=-10;
	w_kr_fi_em=-10;
	elf_kr_es=elf_knee;
	elf_kr_fs=elf_knee;


	y_pre_hl_ei=0;
	y_pre_hl_fi=0;
	y_pre_hl_em=0;
	y_pre_hl_fm=0;


	y_pre_hr_ei=0;
	y_pre_hr_fi=0;
	y_pre_hr_em=0;
	y_pre_hr_fm=0;


	y_pre_kl_ei=0;
	y_pre_kl_fi=0;
	y_pre_kl_em=0;
	y_pre_kl_fm=0;


	y_pre_kr_ei=0;
	y_pre_kr_fi=0;
	y_pre_kr_em=0;
	y_pre_kr_fm=0;

	u_al = 0;
	w_al_kl_ei = 10;
	w_al_kl_fi = -20;
	u_ar = 0;
	w_ar_kr_ei = 10;
	w_ar_kr_fi = -20;
	elf_al = 100;//4
	elf_ar = 100;//4

	u_gl = 1;
	w_gl_hl_fi = 10;
	w_gl_kl_ei = 10;
	w_gl_hr_ei = 10;
	w_gl_kr_fi = 10;
	u_gr = 0;
	w_gr_hl_ei = 10;
	w_gr_kl_fi = 10;
	w_gr_hr_fi = 10;
	w_gr_kr_ei = 10;


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

	// Angle sensor neurons
	threshold_hl_es = gait->maxhip_left();
	threshold_hl_fs = gait->minhip_left();
	threshold_hr_es = gait->maxhip_right();
	threshold_hr_fs = gait->minhip_right();
	threshold_kl_es = gait->maxknee_left();
	threshold_kl_fs = gait->minknee_left();
	threshold_kr_es = gait->maxknee_right();
	threshold_kr_fs = gait->minknee_right();

	/*int maxhip=105;//(130 slope)//110 (actual); // maximum permitted angle of hip
	int minhip=78;//(115 slope)//80(actual);  // minimum permitted angle of hip joints

	int maxknee=175;//(170 slope)//175(actual);  // maximum permitted angle of knee joints
	int minknee=115;//(105 slope) //110(actual); // minimum permitted angle of knee joints*/

	// Swing phase neurons
	/* threshold_al = gait->maxhip_left()
        - (double)(gait->maxhip_left() - gait->minhip_left())*3./100;
    threshold_ar =gait->maxhip_right()
        - (double)(gait->maxhip_right() - gait->minhip_right())*3./100; */


	threshold_al = gait->maxhip_left();
	threshold_ar =gait->maxhip_right();


	// Motor neurons
	threshold_em = 5;           // 5 for smooth function !!!!
	threshold_fm = 5;           // What ever this is supposed to mean !!!!
	threshold_ei = 5;
	threshold_fi = 5;

}


void cNNet::update_nnet ( std::valarray< double > input_data ) {

	// Transfer new array to old names
	int leftpiezo   = (int)(input_data[2]);
	int rightpiezo  = (int)(input_data[3]);
	double angle_hl =       input_data[4];
	double angle_hr =       input_data[5];
	double angle_kl =       input_data[6];
	double angle_kr =       input_data[7];
cout << " foot sensor" << leftpiezo <<"  "<< rightpiezo<< std::endl; 

	// Simple low pass filter.
	++countpiezo;
	if (countpiezo == 6) {
		countpiezo=0;
		angle_hl_pre=angle_hl_now;
		angle_hl_now=angle_hl;
		angle_hr_pre=angle_hr_now;
		angle_hr_now=angle_hr;
	}

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
		//u_al = 1./(1+exp(elf_al*(threshold_al - angle_hl))); // Stretch receptor sensor left
		u_al = 1./(1+exp(elf_al*(threshold_al - angle_hl_low_pass))); // Stretch receptor sensor left
		// u_al starts to decrease from 1 to zero if it below the threshold

	} else {
		if (int(20*u_al)>5) {
			u_al=1;
		} else {
			// u_al = 1 during moving backward from the most forward to the threshold
			//if (((int(angle_hl_now*10))>(int(angle_hl_pre*10))) &&
				//	((int(angle_hl*10))>(int(threshold_al*10)))) {
			if (((int(angle_hl_low_pass*10))>(int(angle_hl_low_pass_pre*10))) &&
							((int(angle_hl_low_pass*10))>(int(threshold_al*10)))) {
				u_al=1;
			}
		}
	}

	if (u_gr==1) {
		//u_ar = 1./(1+exp(elf_ar*(threshold_ar - angle_hr)));  // Stretch receptor sensor right ??
		u_ar = 1./(1+exp(elf_ar*(threshold_ar - angle_hr_low_pass)));  // Stretch receptor sensor right ??
	} else {
		if (int(20*u_ar)>5) {
			u_ar=1;
		} else {
			//if (((int(angle_hr_now*10))>(int(angle_hr_pre*10))) &&
				//	((int(angle_hr*10))>(int(threshold_ar*10)))) {
			if (((int(angle_hr_low_pass*10))>(int(angle_hr_low_pass_pre*10))) &&
					((int(angle_hr_low_pass*10))>(int(threshold_ar*10)))) {
				u_ar=1;
			}
		}
	}


	////////////////////////Old Network not like in IJRR////////////////////////////////////////////

	// hip left
	y_pre_hl_ei=y_hl_ei;
	y_pre_hl_fi=y_hl_fi;
	y_pre_hl_em=y_hl_em;
	y_pre_hl_fm=y_hl_fm;

	// activation of internal neuron from Input sensor GR(0,1) to Ei (a(t))
	y_hl_ei = activation(y_hl_ei, exppp, w_gr_hl_ei, u_gr);

	// activation of internal neuron from Input sensor GL(0,1) to Fi (a(t))
	y_hl_fi = activation(y_hl_fi, exppp, w_gl_hl_fi, u_gl);

	u_hl_ei = 1/(1+exp(threshold_ei-y_hl_ei)); //Output of internal neuron Ei (O(t)) == (0,1)
	u_hl_fi = 1/(1+exp(threshold_fi-y_hl_fi)); //Output of internal neuron Fi (O(t)) == (0,1)



	///Preprocessing of angle sensors==> convert to ES, FS sigmoid transfer function//
	u_hl_es = 1/(1+exp(elf_hl_es*(angle_hl_low_pass-threshold_hl_es)));   // Output of Sensor neuron  Hip left-Extensor********
	u_hl_fs = 1/(1+exp(elf_hl_fs*(threshold_hl_fs-angle_hl_low_pass)));   // Output of Sensor neuron  Hip left-Flexsor********
	// Output between 0,1

	//********** Motor neuron ********//
	// Activitiy of Hip joint EM
	y_hl_em = y_pre_hl_em*exppp+(1-exppp)*(w_hl_es_em*u_hl_es+w_hl_ei_em*u_hl_ei+w_hl_fi_em*u_hl_fi);

	// Activitiy of Hip joint FM
	y_hl_fm = y_pre_hl_fm*exppp+(1-exppp)*(w_hl_fs_fm*u_hl_fs+w_hl_fi_fm*u_hl_fi+w_hl_ei_fm*u_hl_ei);

	u_hl_em = 1/(1+exp(threshold_em-y_hl_em));   //Output motor neuron hip_extensor_left **
	u_hl_fm = 1/(1+exp(threshold_fm-y_hl_fm));   //Output motor neuron hip_flexsor_left  **

	// knee, left
	y_pre_kl_ei=y_kl_ei;
	y_pre_kl_fi=y_kl_fi;
	y_pre_kl_em=y_kl_em;
	y_pre_kl_fm=y_kl_fm;

	y_kl_ei=y_pre_kl_ei*expp+(1-expp)*(w_gl_kl_ei*u_gl+w_al_kl_ei*u_al); // (10*u_gl+10*u_al)
	y_kl_fi=y_pre_kl_fi*expp+(1-expp)*(w_gr_kl_fi*u_gr+w_al_kl_fi*u_al);
	u_kl_ei=1/(1+exp(threshold_ei-y_kl_ei));
	u_kl_fi=1/(1+exp(threshold_fi-y_kl_fi));

	u_kl_es=1/(1+exp(elf_kl_es*(angle_kl_low_pass-threshold_kl_es)));

	double gain_low_pass_es = 0.9;
	u_kl_es_low_pass_pre = u_kl_es_low_pass;

	u_kl_es_low_pass = (1-gain_low_pass_es)*u_kl_es+ u_kl_es_low_pass_pre*gain_low_pass_es;
	u_kl_fs=1/(1+exp(elf_kl_fs*(threshold_kl_fs-angle_kl_low_pass)));

	y_kl_em=y_pre_kl_em*expp+(1-expp)*(w_kl_es_em*u_kl_es+w_kl_ei_em*u_kl_ei+w_kl_fi_em*u_kl_fi);
	//y_kl_em=y_pre_kl_em*expp+(1-expp)*(w_kl_es_em*u_kl_es_low_pass+w_kl_ei_em*u_kl_ei+w_kl_fi_em*u_kl_fi);
	y_kl_fm=y_pre_kl_fm*expp+(1-expp)*(w_kl_fs_fm*u_kl_fs+w_kl_fi_fm*u_kl_fi+w_kl_ei_fm*u_kl_ei);

	u_kl_em=1/(1+exp(threshold_em-y_kl_em));
	u_kl_fm=1/(1+exp(threshold_fm-y_kl_fm));

	// hip, right
	y_pre_hr_ei=y_hr_ei;
	y_pre_hr_fi=y_hr_fi;
	y_pre_hr_em=y_hr_em;
	y_pre_hr_fm=y_hr_fm;
	y_hr_ei=y_pre_hr_ei*exppp+(1-exppp)*(w_gl_hr_ei*u_gl);
	y_hr_fi=y_pre_hr_fi*exppp+(1-exppp)*(w_gr_hr_fi*u_gr);
	u_hr_ei=1/(1+exp(threshold_ei-y_hr_ei));
	u_hr_fi=1/(1+exp(threshold_fi-y_hr_fi));
	u_hr_es=1/(1+exp(elf_hr_es*(angle_hr_low_pass-threshold_hr_es)));
	u_hr_fs=1/(1+exp(elf_hr_fs*(threshold_hr_fs-angle_hr_low_pass)));
	y_hr_em=y_pre_hr_em*exppp+(1-exppp)*(w_hr_es_em*u_hr_es+w_hr_ei_em*u_hr_ei+w_hr_fi_em*u_hr_fi);
	y_hr_fm=y_pre_hr_fm*exppp+(1-exppp)*(w_hr_fs_fm*u_hr_fs+w_hr_fi_fm*u_hr_fi+w_hr_ei_fm*u_hr_ei);

	u_hr_em=1/(1+exp(threshold_em-y_hr_em));
	u_hr_fm=1/(1+exp(threshold_fm-y_hr_fm));

	// knee, right
	y_pre_kr_ei=y_kr_ei;
	y_pre_kr_fi=y_kr_fi;
	y_pre_kr_em=y_kr_em;
	y_pre_kr_fm=y_kr_fm;

	y_kr_ei=y_pre_kr_ei*expp+(1-expp)*(w_gr_kr_ei*u_gr+w_ar_kr_ei*u_ar);
	y_kr_fi=y_pre_kr_fi*expp+(1-expp)*(w_gl_kr_fi*u_gl+w_ar_kr_fi*u_ar);
	u_kr_ei=1/(1+exp(threshold_ei-y_kr_ei));
	u_kr_fi=1/(1+exp(threshold_fi-y_kr_fi));


	u_kr_es=1/(1+exp(elf_kr_es*(angle_kr_low_pass-threshold_kr_es)));

	u_kr_es_low_pass_pre = u_kr_es_low_pass;
	u_kr_es_low_pass = (1-gain_low_pass_es)*u_kr_es+ u_kr_es_low_pass_pre*gain_low_pass_es;

	u_kr_fs=1/(1+exp(elf_kr_fs*(threshold_kr_fs-angle_kr_low_pass)));

	y_kr_em=y_pre_kr_em*expp+(1-expp)*(w_kr_es_em*u_kr_es+w_kr_ei_em*u_kr_ei+w_kr_fi_em*u_kr_fi);
	//y_kr_em=y_pre_kr_em*expp+(1-expp)*(w_kr_es_em*u_kr_es_low_pass+w_kr_ei_em*u_kr_ei+w_kr_fi_em*u_kr_fi);
	y_kr_fm=y_pre_kr_fm*expp+(1-expp)*(w_kr_fs_fm*u_kr_fs+w_kr_fi_fm*u_kr_fi+w_kr_ei_fm*u_kr_ei);

	u_kr_em=1/(1+exp(threshold_em-y_kr_em));
	u_kr_fm=1/(1+exp(threshold_fm-y_kr_fm));



	//State machine control///

	// piezos : 2048 = 0v (foot contact the ground) .... 4096 (off ground)

	// Left leg at front
	if (int(10*u_al)>5) {                               // Touch the ground of left leg
		if (int(angle_kl)>170) {                        // knee left has to be Straigth
			if (abs(leftpiezo-2048)<300) {              // Foot signal ~ ground contact if it touch the ground then enter here!
				u_gl=1;
				u_gr=0;

			}
		}
	}


	// Right leg at front
	if (int(10*u_ar)>5) {                               // Touch the ground of right leg

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


	//If the robot does not touch the ground at all switch off knee motor
	/*if (abs(rightpiezo-2048)>300 && abs(leftpiezo-2048)>300)
	{
		state_u_kl_em = 0.007;
		state_u_kl_fm = 0.00;
		state_u_kr_em = 0.007;
		state_u_kr_fm = 0.00;
		std::cout << "off ground"<< std::endl;


	}*/



	/*std::cout << "u_gr>>>>" << u_gr << std::endl;
	std::cout << "u_gl>>>>" << u_gl << std::endl;
	std::cout << "u_al>>>>" << u_al << std::endl;
	std::cout << "u_ar>>>>" << u_ar << std::endl;

	std::cout << "u_kl_em>>>>" << u_kl_em << std::endl;
	std::cout << "u_kl_fm>>>>" << u_kl_fm << std::endl;
	std::cout << "u_kr_em>>>>" << u_kr_em << std::endl;
	std::cout << "u_kr_fm>>>>" << u_kr_fm << std::endl;


	std::cout << "u_hl_em>>>>" << u_hl_em << std::endl;
	std::cout << "u_hl_fm>>>>" << u_hl_fm << std::endl;
	std::cout << "u_hr_em>>>>" << u_hr_em << std::endl;
	std::cout << "u_hr_fm>>>>" << u_hr_fm << std::endl;


	std::cout << "motorvolt_kl>>>>" << motorvolt_kl << std::endl;
	std::cout << "motorvolt_kr>>>>" << motorvolt_kr << std::endl;
	std::cout << "motorvolt_hl>>>>" << motorvolt_hl << std::endl;
	std::cout << "motorvolt_hr>>>>" << motorvolt_hr << std::endl;*/
	/*state_motorvolt_hl = gait->gain_hl_ext()  * state_u_hl_em -
      gait->gain_hl_flex() * state_u_hl_fm;
  state_motorvolt_hr = gait->gain_hr_ext()  * state_u_hr_em -
      gait->gain_hr_flex() * state_u_hr_fm;
	 */

	state_motorvolt_kl = gait->gain_kl_ext()  * state_u_kl_em -
			gait->gain_kl_flex() * state_u_kl_fm;
	state_motorvolt_kr = gait->gain_kr_ext()  * state_u_kr_em -
			gait->gain_kr_flex() * state_u_kr_fm;



	serialPlot2<<angle_hl<<' '<<angle_hr<<' '<<u_hr_fm<<' '<<u_hr_em<<' '<<u_kl_fm<<' '<<u_kl_em<<' '<<u_kr_fm<<' '<<u_kr_em<<' '<<u_al<<' '<<u_ar<<' '<<u_gl<<' '<<u_gr<<' '<<motorvolt_kl<<' '<<motorvolt_kr<<' '<<motorvolt_hl<<' '<<motorvolt_hr<<' '<<input_data[6]<<' '<<input_data[7]<<endl;


}


std::valarray< double > cNNet::update_motorvoltages() {

	motorvolt_hl = gait->gain_hl_ext()  * u_hl_em -
			gait->gain_hl_flex() * u_hl_fm;
	motorvolt_hr = gait->gain_hr_ext()  * u_hr_em -
			gait->gain_hr_flex() * u_hr_fm;
	//motorvolt_kl = gait->gain_kl_ext()  * u_kl_em -
			//gait->gain_kl_flex() * u_kl_fm;
	//motorvolt_kr = gait->gain_kr_ext()  * u_kr_em -
			//gait->gain_kr_flex() * u_kr_fm;

	motorvolt_kl = state_motorvolt_kl;
	motorvolt_kr = state_motorvolt_kr;


	valarray<double> result(4);
	result[0] = motorvolt_hl; // correct! +1 = forward, -1 = backward
	result[1] = motorvolt_hr; // correct! +1 = forward, -1 = backward

	result[2] = motorvolt_kl; // correct! +1 = forward, -1 = backward
	result[3] = motorvolt_kr; // correct! +1 = forward, -1 = backward




	std::cout<<"threshold_al: "<<threshold_al<<std::endl;
	//std::cout<<"u_hl_fm: "<<u_hl_fm<<std::endl;

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


