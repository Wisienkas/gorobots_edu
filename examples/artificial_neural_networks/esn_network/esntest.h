/**
 * @author Sakya & Poramate 02.12.2013
 */

#ifndef __TestESN_H__
#define __TestESN_H__

// #################### definitions #####################

#include <math.h>
#include <string.h>
#include <stdio.h>



///////// Save text////////////////////////
#include <iostream>
#include <fstream>
#include <string.h>
using namespace std;
///////////////////////////////////////////


//Set parameters (for Students)
const int num_input_ESN = 1;
const int num_output_ESN = 1;
const int num_hidden_ESN = 20; // Student Adjust!***

const int learning_mode = 1;
//set learning_mode = 1 for RLS (learning rate needs to be large, e.g., 0.99)
//set learning mode =2  for LMS (learning rate needs to be small, e.g., 0.01)

const double learning_rate_ESN = 0.9; // Student Adjust!***

const double leak = 0.33;
const double input_sparsity = 70;

const int testing_start = 4000; /*training after, e.g., 4000 steps for learning modes 1-4 & "TestingData_4900.txt"*/
//const int testing_start = 1500; /*training after, e.g., 1500 steps for "TestingData_2000.txt"*/



class TestESN
{
  public:

    TestESN();
    ~TestESN();


    // --- Save text------------//
    ofstream saveFile1;
    //-------------------------//
    double RecurrentNetwork(double i0, double d);
    double target_ESN;
    double input_ESN;
    double output_ESN;
    bool learn;
    double error;
    int washout_time;

};

#endif
