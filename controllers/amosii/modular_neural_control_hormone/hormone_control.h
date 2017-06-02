// for controller

#ifndef HORMONE_CONTROL_H_
#define HORMONE_CONTROL_H_

#include <ode_robots/amosiisensormotordefinition.h>

#include "utils/hormone-framework/hormone.h"
#include <algorithm>
#include <vector>

//Save files
#include <iostream>
#include <fstream>
#include <string.h>

using namespace std;
using namespace Hormone;

class hormone_control {

  public:

    hormone_control(); //hormone mapping and config
    ~hormone_control();
    void StimulateInput(const vector<double> x, const vector<double> y);
    void StimulateInput(const vector<double> x, const vector<double> y, const vector<double> MI);

    DATATYPE getReceptor(unsigned int index);
    void setBuffersize(unsigned int);

    gland* HGland;
    hormone* HTank;
    receptor* HReceptor;

    //---Start Save files---//
    ofstream outFilenlc1;
    ofstream outFilenlc2;
    //---End Save files---//
    vector<double> m_pre_1 ;
    vector<double> m_pre_edit ;
    vector<double> m_pre_edit_p ;

    double sum_error;

  private:
    //Angle sensors
    std::vector<std::vector<double>> in0;
    std::vector<std::vector<double>> out0;
    //Control input
    std::vector<std::vector<double>> Gait;

    void UpdateInput();

    DATATYPE Convolution(const vector<double> x, const vector<double> y);

    DATATYPE calcSD(vector<double> inputVector);DATATYPE calcMean(vector<double> inputVector);DATATYPE calcCov(
        const vector<double> x, const vector<double> y);DATATYPE Correlation(const vector<double> x,
        const vector<double> y);

    //unsigned int BufferNow;
    unsigned int BufferSize;
    unsigned int LegSize;
    unsigned int JointSize;
    unsigned int FootSize;
    unsigned int GlandSize;
    unsigned int HormoneSize;
    unsigned int ReceptorSize;
    unsigned int t ;
    unsigned int t_i[6];
    unsigned int t_f[6];
    double dt[6];
    int t_ref[6];
    int d_case[6][2];
    int check_fall[6][2];
    int cp[6][2];
    double dt_cp[6];
    double error[6];
};

#endif
