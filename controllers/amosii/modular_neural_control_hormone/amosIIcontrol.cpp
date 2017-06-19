/***************************************************************************
 *   Copyright (C) 2010 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *                                                                         *
 *   LICENSE:                                                              *
 *   This work is licensed under the Creative Commons                      *
 *   Attribution-NonCommercial-ShareAlike 2.5 License. To view a copy of   *
 *   this license, visit http://creativecommons.org/licenses/by-nc-sa/2.5/ *
 *   or send a letter to Creative Commons, 543 Howard Street, 5th Floor,   *
 *   San Francisco, California, 94105, USA.                                *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  *
 *                                                                         *
 *   $Log: tripodgate18dof.cpp,v $                                         *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/

/*The new controller (AmosIIControl(int aAMOStype,bool mMCPGs,bool mMuscleModelisEnabled)) has numerous privileges:
 *1) selection between AMOSv1 (aAMOStype=1) and AMOSv2 (aAMOStype=2). [DEFAULT = 2]
 *2) selection between single CPG-based controller (mMCPGs=false) and Multiple CPGs-based control (mMCPGs=true). [DEFAULT = false]
 *3) possibility to utilize muscle model (mMuscleModelisEnabled=true). [DEFAULT = false]
 *4) selection between integrated navigation controller (mNNC=true) and pure locomotion controller (mNNC=false). [DEFAULT = false]
 * */

#include <selforg/controller_misc.h>
#include <math.h>
#include "amosIIcontrol.h"
//#include <ode_robots/amosII.h>
using namespace matrix;
using namespace std;

AmosIIControl::AmosIIControl(int aAMOSversion, bool mMCPGs, bool mMuscleModelisEnabled) :
    AbstractController("AmosIIControl", "$Id: amosIIcontrol.cpp,v 0.1 $") {
  //data_wr.open("results_n.dat");
  //data_sr.open("sensor_n.dat");
  //data_mr.open("motor_n.dat");

  outFilenlc1.open("control.txt");
  outFilenlc2.open("gait_ana.txt");
  outFilenlc3.open("B-Check2.txt");
  outFilenlc1 << "H0 H1 H2 H3 H4 H5 Gait0 Gait1 Gait2 Gait3 Gait4 Gait5" << endl;
  outFilenlc2
      << "pos_body_x pos_body_y pos_body_z vel_body_x vel_body_y vel_body_z acc_body_x acc_body_y acc_body_z ori_body_x ori_body_y ori_body_z time FC_R3 FC_R2 FC_R1 FC_L3 FC_L2 FC_L1 MI H1"
      << endl;
  outFilenlc3 << "Lift H[6] Incl_x Incl_y Average_Current" << endl;
  //---ADD YOUR initialization here---//
  t = 0; // step counter
  MCPGs = mMCPGs;
  if (MCPGs)
    num_cpgs = 6;
  else
    num_cpgs = 1;

  control_adaptiveclimbing = new NeuralLocomotionControlAdaptiveClimbing(aAMOSversion, mMCPGs,
      mMuscleModelisEnabled/*, mNNC*/);

  artificial_endocrine = new hormone_control();

  if (MCPGs == true) {
    // press Ctrl M to display matrix
    for (unsigned int i_cpg = 0; i_cpg < num_cpgs; i_cpg++) {
      string lr = (i_cpg < 3) ? "R" : "L";
      addInspectableValue("CPG[" + to_string(i_cpg) + "]", &control_adaptiveclimbing->cpg_output.at(i_cpg).at(0),
          lr + to_string(i_cpg % 3) + "_0"); //to_string() is C++11
      addInspectableValue("CPG[" + to_string(i_cpg) + "]", &control_adaptiveclimbing->cpg_output.at(i_cpg).at(1),
          lr + to_string(i_cpg % 3) + "_1");
      addInspectableValue("Gait[" + to_string(i_cpg) + "]", &control_adaptiveclimbing->nlc.at(i_cpg)->Control_input,
          "_0"); //to_string() is C++11
    }
  }

  //double precent_gait = control_adaptiveclimbing->nlc.at(0)->Control_input/1.2;
  if (MCPGs != true)
    addInspectableValue("Gait", &control_adaptiveclimbing->nlc.at(0)->Control_input, "_0"); //to_string() is C++11

  for (unsigned int i_cpg = 0; i_cpg < num_cpgs; i_cpg++) {
    addInspectableValue("HTank_" + to_string(i_cpg), &artificial_endocrine->HTank[i_cpg].hormoneValue, "_0"); //to_string() is C++11
    control_adaptiveclimbing->nlc.at(i_cpg)->changeControlInput(0.03);//ฺB-Edit alpha = 0.07
    //control_adaptiveclimbing->nlc.at(i_cpg)->changeControlInput(0.1);//ฺB-Edit alpha = 0.08
    //control_adaptiveclimbing->nlc.at(i_cpg)->changeControlInput(0.13);//ฺB-Edit alpha = 0.08
    //control_adaptiveclimbing->nlc.at(i_cpg)->changeControlInput(0.14);//ฺB-Edit alpha = 0.08
    control_adaptiveclimbing->lift_value = 10;
  }
  addInspectableValue("HTank_" + to_string(6), &artificial_endocrine->HTank[6].hormoneValue, "_0");
  addInspectableValue("Lift", &control_adaptiveclimbing->lift_value, "_0");
  //Add edit parameter on terminal

  //Configurable::addParameter("cin", &control_adaptiveclimbing->nlc.at(0)->Control_input,  /*minBound*/ -10,  /*maxBound*/ 10, "test description" );

  Configurable::addParameter("cin", &control_adaptiveclimbing->Control_input, /*minBound*/-10, /*maxBound*/10,
      "test description");
  Configurable::addParameter("input3", &control_adaptiveclimbing->input.at(3), /*minBound*/-1, /*maxBound*/1,
      "test description");
  Configurable::addParameter("input4", &control_adaptiveclimbing->input.at(4), /*minBound*/-1, /*maxBound*/1,
      "test description");
  /*for(int i = 0;i<6;i++){
	  addInspectableValue("CP[" + to_string(i), &artificial_endocrine->cp[i], "_0");
  }*/ //error


  addInspectableValue("mpre", &artificial_endocrine->m_pre_edit.at(CR0_m),"_0");
}

AmosIIControl::~AmosIIControl() {
  //data_wr.close();
  //data_sr.close();
  //data_mr.close();

  //Save files
  outFilenlc1.close();
  outFilenlc2.close();
}
void AmosIIControl::enableContactForceMech() //enable sensory feedback mechanism
{
  for (unsigned int i = 0 && MCPGs; i < num_cpgs; i++) {
    control_adaptiveclimbing->nlc.at(i)->enableContactForce(MCPGs);
    std::cout << "[" << i << "] " << "contactForce is enabled -> "
        << control_adaptiveclimbing->nlc.at(i)->contactForceIsEnabled << "\n";
  }
}

void AmosIIControl::disableContactForceMech() //disable sensory feedback mechanism
{
  for (unsigned int i = 0 && MCPGs; i < num_cpgs; i++)
    control_adaptiveclimbing->nlc.at(i)->disableContactForce();
  std::cout << "contact force is disabled \n";
}

// Frequency increases by increasing modulatory input.
void AmosIIControl::increaseFrequency() {
  for (unsigned int i = 0 && MCPGs; i < num_cpgs; i++)
    control_adaptiveclimbing->nlc.at(i)->changeControlInput(control_adaptiveclimbing->nlc.at(i)->Control_input + 0.01);
  std::cout << "Frequency increases" << endl;
  std::cout << "Modulatory input:" << control_adaptiveclimbing->nlc.at(0)->Control_input << endl;
}
// Frequency decreases by decreasing modulatory input.
void AmosIIControl::decreaseFrequency() {
  for (unsigned int i = 0 && MCPGs; i < num_cpgs; i++)
    control_adaptiveclimbing->nlc.at(i)->changeControlInput(control_adaptiveclimbing->nlc.at(i)->Control_input - 0.01);
  std::cout << "Frequency decreases" << endl;
  std::cout << "Modulatory input:" << control_adaptiveclimbing->nlc.at(0)->Control_input << endl;
}
void AmosIIControl::enableOscillatorCoupling() //enable oscillator coupling (fully connected network)
{
  for (unsigned int i = 0 && MCPGs; i < num_cpgs; i++)
    control_adaptiveclimbing->nlc.at(i)->enableoscillatorsCoupling(MCPGs);
  std::cout << "Oscillator Coupling is enabled \n";
}
void AmosIIControl::disableOscillatorCoupling() //disable oscillator coupling (fully connected network)
{
  for (unsigned int i = 0 && MCPGs; i < num_cpgs; i++)
    control_adaptiveclimbing->nlc.at(i)->disableoscillatorsCoupling();
  std::cout << "Oscillator Coupling is disabled \n";
}

void AmosIIControl::enableTripodGait() {
  for (unsigned int i = 0 && MCPGs; i < num_cpgs; i++) {
    int gaitPattern = 0; //Tripod
    control_adaptiveclimbing->nlc.at(i)->changeGaitpattern(gaitPattern);
  }
  std::cout << "Tripod \n";
}
void AmosIIControl::enableTetrapodGait() {
  for (unsigned int i = 0 && MCPGs; i < num_cpgs; i++) {
    int gaitPattern = 1; //Tetrapod
    control_adaptiveclimbing->nlc.at(i)->changeGaitpattern(gaitPattern);
  }
  std::cout << "Tetrapod \n";
}

void AmosIIControl::enableWaveGait() {
  for (unsigned int i = 0 && MCPGs; i < num_cpgs; i++) {
    int gaitPattern = 2; //wave
    control_adaptiveclimbing->nlc.at(i)->changeGaitpattern(gaitPattern);
  }
  std::cout << "wave \n";
}
void AmosIIControl::enableIrregularGait() {
  for (unsigned int i = 0 && MCPGs; i < num_cpgs; i++) {
    int gaitPattern = 3; //irregular gait.
    control_adaptiveclimbing->nlc.at(i)->changeGaitpattern(gaitPattern);
  }
  std::cout << "irregularEnable \n";
}

void AmosIIControl::init(int sensornumber, int motornumber, RandGen* randGen) {
  numbersensors = sensornumber;
  numbermotors = motornumber;
  x.resize(sensornumber);
  y.resize(AMOSII_MOTOR_MAX);
}

/// performs one step (includes learning). Calulates motor commands from sensor inputs.
void AmosIIControl::step(const sensor* x_, int number_sensors, motor* y_, int number_motors) {

  assert(number_sensors == numbersensors);
  assert(number_motors == numbermotors);

  /************   0) Sensor inputs   ************/
  static unsigned int counter = 0;

  //static double vel[3];
  double acc[3];

  for (unsigned int i = 0; i < 3; i++) {
    double temp;
    acc[i] = (x.at(i + BX_spd) - x_[i + BX_spd]) / 0.1;
  }

  for (unsigned int i = 0; i < AMOSII_SENSOR_MAX; i++) {
    x.at(i) = x_[i];
  }

  outFilenlc2 << x[BX_pos] << ' ' << x[BY_pos] << ' ' << x[BZ_pos] << ' '\
 << x[BX_spd] << ' ' << x[BY_spd] << ' '
      << x[BZ_spd] << ' '\
 /*            << vel[0]    << ' ' << vel[1]    << ' ' << vel[2]    << ' '\*/
      << acc[0] << ' ' << acc[1] << ' ' << acc[2] << ' '\
 << x[BX_ori] << ' ' << x[BY_ori] << ' ' << x[BZ_ori] << ' '\

      << counter << ' ';
//	                << x[R2_fs]  << ' ' << x[R1_fs]  << ' ' << x[R0_fs]  << ' '\
	                << x[L2_fs]  << ' ' << x[L1_fs]  << ' ' << x[L0_fs]  << ' '\
	                << endl;
  counter++;

  /******   1) Neural preprocessing and learning   ******/

  std::vector < vector<double> > x_prep = preprocessing_learning.step_npp(x);

  /**********   3) Neural locomotion control   **********/

  y = control_adaptiveclimbing->step_nlc(x, x_prep, false);

  /****************   4) Motor outputs   ****************/

  for (unsigned int i = 0; i < AMOSII_MOTOR_MAX; i++) {
    y_[i] = y.at(i);

  }

  outFilenlc2 << y[CR2_m] << ' ' << y[CR1_m] << ' ' << y[CR0_m] << ' '\
 << y[CL2_m] << ' ' << y[CL1_m] << ' ' << y[CL0_m]
      << ' '\

      //<< ctrl_input << ' '<< artificial_endocrine->h1->hormoneValue

      << endl;
  /*

   //change six leg to 4leg
   y_[TR1_as]=0;
   y_[TL1_as]=0;

   y_[CR1_as]=1;
   y_[CL1_as]=1;

   y_[FR1_as]=-1;
   y_[FL1_as]=-1;
   */

  // create array MI
  if (MCPGs){
    Leg_Gait.clear();
    Leg_Gait.resize(num_cpgs);

    for (unsigned char i = 0; i < num_cpgs; i++) {
      Leg_Gait.push_back(control_adaptiveclimbing->nlc.at(i)->Control_input);
    }
  }
  outFilenlc3 << control_adaptiveclimbing->lift_value << ' ' << artificial_endocrine->HTank[6].hormoneValue << ' ' << x[In_x] << ' ' << x[In_y] << ' ' << x[A_cs] << endl;
  // add to hormone_control inline (x param mean sensor , y param mean =motor)
  artificial_endocrine->StimulateInput(x, y);
  //artificial_endocrine->StimulateInput(x, y, Leg_Gait);

  //std::cout << "Y[14] = " << y.at(14) <<endl;
  //std::cout << "x[14] = " << x.at(14) <<endl;

  outFilenlc1 << artificial_endocrine->HTank[0].hormoneValue << ' '\
 << artificial_endocrine->HTank[1].hormoneValue
      << ' '\
 << artificial_endocrine->HTank[2].hormoneValue << ' '\
 << artificial_endocrine->HTank[3].hormoneValue << ' '\

      << artificial_endocrine->HTank[4].hormoneValue << ' '\
 << artificial_endocrine->HTank[5].hormoneValue << ' ';
  	  artificial_endocrine->m_pre_1 = control_adaptiveclimbing->m_pre ;
  //outFilenlc1 << artificial_endocrine->g1->data.OUTPUT << ' ' << artificial_endocrine->h1->hormoneValue << ' ' << artificial_endocrine->sum_error ;
 if (MCPGs){
    for (unsigned int i = 0; i < 6; i++) {
      double receptor = artificial_endocrine->getReceptor(i);
      double ctrl_input = control_adaptiveclimbing->nlc.at(i)->Control_input * receptor;
      //double ctrl_input = 0.19;

      if (ctrl_input > 0.19) {
        ctrl_input = 0.19;
      } else if (ctrl_input < 0.02) {
        ctrl_input = 0.02;
      }

      control_adaptiveclimbing->nlc.at(i)->changeControlInput(ctrl_input);
      //control_adaptiveclimbing->
      outFilenlc1 << ctrl_input << ' ';
    }

  }else{
    double receptor = artificial_endocrine->getReceptor(0);
    //double ctrl_input = control_adaptiveclimbing->nlc.at(0)->Control_input * receptor;
  } //B-Edit
 //if(artificial_endocrine->cp == 1)
 {
	 double receptor = artificial_endocrine->getReceptor(6);
	 outFilenlc1 << endl;
	 double lift = 35.0;

	 // Use hormone !! uncomment this
	 lift = (receptor*33.0);
	 //If you want to fix the height then comment this one!

	 if(lift>35){
		 lift = 35;
	 }
	 else if(lift<1){
		 lift = 1;
	 }
	 control_adaptiveclimbing->lift_value = lift;
	 // outFilenlc1 << ' ' << receptor << ' ' << ctrl_input <<endl;
	 //inject motor
	 //y_[14] = 1;
 }
  // update step counter
  t++;
}
;

/** stores the controller values to a given file. */
bool AmosIIControl::store(FILE* f) const {
  return true;
}

/** loads the controller values from a given file. */
bool AmosIIControl::restore(FILE* f) {
  //	Configurable::parse(f);
  return true;
}

