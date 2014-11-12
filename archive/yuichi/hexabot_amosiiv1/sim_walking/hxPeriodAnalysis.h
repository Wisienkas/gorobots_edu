#ifndef HXPERIODANALYSIS_H
#define HXPERIODANALYSIS_H 100

//! Version Control
#define VERSION_HPA "HPA_Ver 1.00_20140217"
/*
Ver 1.00 20140217
   Start to make


***********************************************

This class is for analyzing and logging one periodic walking

We should call

start : to start analyze
stop : to stop analyze
work : to work ()



***********************************************
*/
#include <iostream>
#include "hexabotController.h"


class hxPeriodAnalysis
{
public:
    hxPeriodAnalysis(std::string filename_, bool log_flag_, int log_freq_ = 10);
    ~hxPeriodAnalysis();

private:
    std::ofstream ofs;

    double touch_down_time[CPGNUM];
    double touch_down_phase[CPGNUM];
    double duty_rate[CPGNUM];
    double lift_off_time[CPGNUM];
    bool prv_isLegContact[CPGNUM];


    double period;
    double start_time;
    bool start_flag;

    int log_freq;
    double prv_logTime;

public:
    // start the log and analysis
    bool start(HexabotController& hCtr, double time);
    // stop the log and analysis
    bool stop(HexabotController& hCtr, double time);

    // This is called every step
    void work(HexabotController& hCtr, double time);

    // get real Duty Rate
    double get_realDutyrate(int ch);
    // get real period
    double get_realPeriod();
    // get touch down phase
    double get_touchDownPhase(int ch);

};

#endif // HXPERIODANALYSIS_H
