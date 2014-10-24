#ifndef HXPERIODANALYSIS_CPP
#define HXPERIODANALYSIS_CPP 100


#include "hxPeriodAnalysis.h"

// constructor
hxPeriodAnalysis::hxPeriodAnalysis(std::string filename_, bool log_flag_, int log_freq_):start_flag(false)
{
    if(log_flag_) ofs.open(filename_);

    for (int i=0;i<CPGNUM;++i){
        touch_down_time[i] = 0.;
        touch_down_phase[i]=0.;
        lift_off_time[i]=0.;
        duty_rate[i] = 0.;
    }
    period = 0.;
    log_freq = log_freq_;
    prv_logTime = 0.;

    if(ofs.is_open()){
        ofs << "## Log data for 1 periodic walking!!  Ver." << VERSION_HPA << std::endl;
        ofs << "##   Cog pos means that the position of the COG from the center of the robot " << std::endl;
        ofs << "##  1:time, 2:phase_0, 3:is_contact_0, 4:phase_1, 5:is_contact_1, 6:phase_2, 7:is_contact_2, "
            << "8:phase_3, 9:is_contact_3, 10:phase_4, 11:is_contact_4, 12:phase_5, 13:is_contact_5, "
            << "14:roll, 15:pitch, 16:yaw, 17:cog_x, 18:cog_y, 19:cog_z" << std::endl;
    }

    return;
}

// destructor
hxPeriodAnalysis::~hxPeriodAnalysis(){
    if(ofs.is_open()) ofs.close();
}

// start
bool hxPeriodAnalysis::start(HexabotController &hCtr, double time){
    start_flag = true;
    start_time = time;
    prv_logTime = -1.;

    // contact situation
    for(int i=0;i<CPGNUM;i++){
        prv_isLegContact[i] = hCtr.eIsLegContact(i);
    }

    // file check
    if(ofs.is_open()) return false;
    else return true;
}

// stop
bool hxPeriodAnalysis::stop(HexabotController &hCtr, double time){
    start_flag = false;

    // period
    period = time - start_time;

    for (int i =0; i< CPGNUM; i++){
        // calculate duty
        double stanceT = lift_off_time[i] - touch_down_time[i];
        if(stanceT < 0) stanceT = stanceT + period;
        duty_rate[i] = stanceT / period;

        // touch down phase
        touch_down_phase[i] = hCtr.eGetTouchDownPhase(i);
    }

    return false;
}

// work function
void hxPeriodAnalysis::work(HexabotController &hCtr, double time){
    double time2;
    time2 = time - start_time;

    if(start_flag){
        // check leg contact
        for (int i = 0; i < CPGNUM; i++){
            // touch down  or  lift off
            if(hCtr.eIsLegContact(i) != prv_isLegContact[i]){
                if(hCtr.eIsLegContact(i)){
                    touch_down_time[i] = time2;
                }else lift_off_time[i] = time2;
            }
            // prv info update
            prv_isLegContact[i] = hCtr.eIsLegContact(i);
        }

        // logging
        if(ofs.is_open()){
            if( time2 - prv_logTime > 1./(double)log_freq ){
                // update time
                prv_logTime = time2;

                // logging
                //ofs << "## 1:time, 2:phase_0, 3:is_contact_0, 4:phase_1, 5:is_contact_1, 6:phase_2, 7:is_contact_2, "
                //    << "8:phase_3, 9:is_contact_3, 10:phase_4, 11:is_contact_4, 12:phase_5, 13:is_contact_5, "
                //    << "14:roll, 15:pitch, 16:yaw, 17:cog_x, 18:cog_y, 19:cog_z" << std::endl;
                ofs << time2 << " ";
                for(int i =0; i< CPGNUM; i++){
                    ofs << hCtr.eGetPhase(i) << " " << hCtr.eIsLegContact(i) << " ";
                }
                ofs << hCtr.eGetSensedData().pose.x() << " " << hCtr.eGetSensedData().pose.y() << " " << hCtr.eGetSensedData().pose.z() << " ";
                ofs << hCtr.eGetSensedData().g_cogPos.x() - hCtr.eGetSensedData().g_roboPos.x() << " ";
                ofs << hCtr.eGetSensedData().g_cogPos.y() - hCtr.eGetSensedData().g_roboPos.y() << " ";
                ofs << hCtr.eGetSensedData().g_cogPos.z() - hCtr.eGetSensedData().g_roboPos.z() << " ";
                ofs << std::endl;
            }
        }

    }

    return;
}

// get real duty rate
double hxPeriodAnalysis::get_realDutyrate(int ch){
    if(start_flag) return -1.;
    if(ch < 0 || ch >= CPGNUM) return -1;
    return duty_rate[ch];
}

// get real period
double hxPeriodAnalysis::get_realPeriod(){
    if(start_flag) return -1.;
    return period;
}

// get touvh down
double hxPeriodAnalysis::get_touchDownPhase(int ch){
    if(start_flag) return -1.;
    if(ch < 0 || ch >= CPGNUM) return -1;
    return touch_down_phase[ch];
}

#endif
