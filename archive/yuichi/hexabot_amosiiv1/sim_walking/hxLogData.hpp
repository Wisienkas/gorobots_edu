#ifndef HXLOGDATA_HPP
#define HXLOGDATA_HPP

//! Version Control
#define VERSION_HLD "HLD_Ver 1.00_20140508"

/*
Ver 1.00 20140508
   Start to make

***********************************************

This class is for logging the data from start to end

We should call

start : to start analyze
stop : to stop analyze
work : to work ()

***********************************************
*/

#include <iostream>
#include "hexabotController.h"


class hxLogData
{

private:
    std::ofstream ofs;

    double start_time;
    bool start_flag;

    int log_freq;
    double prv_logTime;

public:
    // constructor
    hxLogData(std::string filename_, bool log_flag_, int log_freq_ = 10){
        if(log_flag_) ofs.open(filename_);

        log_freq = log_freq_;
        prv_logTime = 0.;

        if(ofs.is_open()){
            ofs << "## Log data for walking!!  Ver." << VERSION_HLD << std::endl;
            ofs << "##  1:time, 2:phase_0, 3:is_contact_0, 4:phase_1, 5:is_contact_1, 6:phase_2, 7:is_contact_2, "
                << "8:phase_3, 9:is_contact_3, 10:phase_4, 11:is_contact_4, 12:phase_5, 13:is_contact_5, "
                << "14:roll, 15:pitch, 16:yaw, 17: phase1- phase0, 18:phase2 - phase1" << std::endl;
        }
        start_flag = false;

        return;
    }

    // destructor
    ~hxLogData(){
        if(ofs.is_open()) ofs.close();
    }

    // start the log and analysis
    //  if there are errors, return true
    bool start(HexabotController& hCtr, double time){
        start_flag = true;
        start_time = time;
        prv_logTime = -1.;

        // file check
        if(ofs.is_open()) return false;
        else return true;
    }

    // stop the log and analysis
    bool stop(HexabotController& hCtr, double time){
        start_flag = false;
        return false;
    }

    // This is called every step
    void work(HexabotController& hCtr, double time){
        double time2;
        time2 = time - start_time;

        if(start_flag){
            // logging
            if(ofs.is_open()){
                if( time2 - prv_logTime > 1./(double)log_freq ){
                    // update time
                    prv_logTime = time2;

                    double dPhase1 = hCtr.eGetPhase(1) - hCtr.eGetPhase(0);
                    HexabotController::checkPhase(dPhase1);
                    double dPhase2 = hCtr.eGetPhase(2) - hCtr.eGetPhase(1);
                    HexabotController::checkPhase(dPhase2);


                    // logging
                    //ofs << "## 1:time, 2:phase_0, 3:is_contact_0, 4:phase_1, 5:is_contact_1, 6:phase_2, 7:is_contact_2, "
                    //    << "8:phase_3, 9:is_contact_3, 10:phase_4, 11:is_contact_4, 12:phase_5, 13:is_contact_5, "
                    //    << "14:roll, 15:pitch, 16:yaw, 17:p1-p0, 18:p2-p1" << std::endl;
                    ofs << time2 << " ";
                    for(int i =0; i< CPGNUM; i++){
                        ofs << hCtr.eGetPhase(i) << " " << hCtr.eIsLegContact(i) << " ";
                    }
                    ofs << hCtr.eGetSensedData().pose.x() << " " << hCtr.eGetSensedData().pose.y() << " " << hCtr.eGetSensedData().pose.z() << " ";
                    ofs << dPhase1 << " ";
                    ofs << dPhase2 << " ";
                    ofs << std::endl;
                }
            }

        }

        return;
    }

};

#endif // HXLOGDATA_HPP
