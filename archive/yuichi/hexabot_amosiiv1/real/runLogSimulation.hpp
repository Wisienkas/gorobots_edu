#ifndef RUNLOGSIMULATION_HPP
#define RUNLOGSIMULATION_HPP 200

#include <iostream>
#include <sstream>
#include "itfOneProcSimulation.h"
#include "hexabotController.h"
#include "itfTimedChangePrmAgt.hpp"

/*
 This is the class to run simulation with log.
20140815 Ver 2.00
    change the programs for the real exp, add the log data to debug

20140608 Ver 1.01
    add the legPhase to choose poincare section's leg

20140515 Ver 1.00
    start to make

*/

// logging Basic Data
class logBasicData{
private:
    std::ofstream ofs;

    double startTime;
    bool startFlag;
    int logFreq;
    double prvLogTime;
    int logCnt;

public:
    // constructor
    logBasicData(std::string name_, std::string prmName_, bool logFlag_, int logFreq_ = 10){
        if(logFlag_) ofs.open(name_);

        logFreq = logFreq_;
        prvLogTime = 0.;

        if(ofs.is_open()){
            ofs << "## Log data for RunLogSimulation!!  Ver."  << RUNLOGSIMULATION_HPP  << std::endl;
            ofs << "##  The name of the file is  [ " << name_ << " ]" << std::endl;
            ofs << "##  1:time, 2: number, 3:phase_0, 4:is_contact_0, 5:phase_1, 6:is_contact_1, "
                << "7:phase_2, 8:is_contact_2, "
                << "9:phase_3, 10:is_contact_3, 11:phase_4, 12:is_contact_4, 13:phase_5, 14:is_contact_5, "
                << "15:roll [deg], 16:pitch [deg], 17:Current [A], 18: phase1- phase0, 19:phase2 - phase1, "
                << "20:" << prmName_ << " 21-: realValues of foot sensor <1, 2, 3, 4, 5, 6> " <<std::endl;
        }
        startFlag = false;
        logCnt = 0;

        return;
    }

    // destructor
    ~logBasicData(){
        if(ofs.is_open()) ofs.close();
    }

    // start the log and analysis
    //  if there are errors, return true
    bool start(HexabotController& hCtr, double time){
        startFlag = true;
        startTime = time;
        prvLogTime = -1.;
        logCnt = 0;

        // file check
        if(ofs.is_open()) return false;
        else return true;
    }

    // stop the log and analysis
    bool stop(HexabotController& hCtr, double time){
        startFlag = false;
        return false;
    }

    // This is called every step to log
    //  ( log Freq is automatically controlled )
    void work(HexabotController& hCtr, std::string prmValue,  double time){
        double time2;
        time2 = time - startTime;
        logCnt++;

        if(startFlag){
            // logging
            if(ofs.is_open()){
                if( time2 - prvLogTime > 1./(double)logFreq ){
                    // update time
                    prvLogTime = time2;

                    double dPhase1 = hCtr.eGetPhase(1) - hCtr.eGetPhase(0);
                    HexabotController::checkPhase(dPhase1);
                    double dPhase2 = hCtr.eGetPhase(2) - hCtr.eGetPhase(1);
                    HexabotController::checkPhase(dPhase2);


                    // logging
                    //ofs << "## 1:time, 2:cnt 3:phase_0, 4:is_contact_0, 5:phase_1, 6:is_contact_1, 7:phase_2, 8:is_contact_2, "
                    //    << "9:phase_3, 10:is_contact_3, 11:phase_4, 12:is_contact_4, 13:phase_5, 14:is_contact_5, "
                    //    << "15:roll, 16:pitch, 17:yaw, 18:p1-p0, 19:p2-p1" << std::endl;
                    ofs << time2 << " " << logCnt << " ";
                    for(int i =0; i< CPGNUM; i++){
                        ofs << hCtr.eGetPhase(i) << " " << hCtr.eIsLegContact(i) << " ";
                    }
                    ofs << hCtr.eGetSensedData().pose.x() << " " << hCtr.eGetSensedData().pose.y() << " " << hCtr.eGetSensedData().pose.z() << " ";
                    ofs << dPhase1 << " ";
                    ofs << dPhase2 << " ";
                    ofs << prmValue << "   ";
                    for (int i=0;i<6;i++){
                        ofs << hCtr.eGetSensedData().forceData[i] << " ";
                    }
                    ofs << std::endl;
                }
            }

        }

        return;
    }

};

// run log simulation
//  this is the program for the real robot experiment
class runLogSimulation : public itfOneProcSimulation<HexabotController>
{
public:
    // configs
    struct runLogSimConf{
        // the initial state to begin search
        osg::Vec2d initialState;

        // poincare section
        double poincarePhase;
        int poincarePhaseLegNum;

        // time for wait for initializing [s]
        int waitTime;

        // max time to do simulation( in case for something errors)
        int maxSimulationTime;
    };

    // get default configs
    static runLogSimConf getDefaultConf(){
        runLogSimConf conf;
        conf.initialState = osg::Vec2d(3.14, 3.14);
        conf.poincarePhase = 0.;
        conf.poincarePhaseLegNum = 0;
        conf.waitTime = 10;
        conf.maxSimulationTime = 5000;
        return conf;
    }

private:
    // configs
    runLogSimConf& conf;

    // timedChangeParameterAgent
    itfTimedChangePrmAgent* prmAgent;

    //log file
    logBasicData* timedLog;
    bool isTimedLog;

    logBasicData* mappedLog;
    bool isMappedLog;

    std::string logLocation;
    std::string logName;
    double logFreq;

    // status flag
    enum STATUS {IDLE, START, WAIT, RUN_LOG, FIN};
    enum STATUS status;

    // simNum
    int simNum;

    // simulation time
    //  the time to run simulation (it does not contains wait time!!)
    double simulationTime;
    // simulation start time
    double simStartTime;
    // waitMode start time
    double waitStartTime;

    // poincare
    bool poincareFlag;
    int periodSteps;
    int poincareSecCount;

public:
    // constructor
    runLogSimulation(runLogSimConf& conf_, itfTimedChangePrmAgent* prmAgent_, double simulationTime_,  bool logMapFlag_, bool logTimedWholeFlag_, std::string logLocation_ = "", std::string logName_ = "", double logFreq_ = 10.)
        :itfOneProcSimulation<HexabotController>(), prmAgent(prmAgent_), conf(conf_), logLocation(logLocation_), logName(logName_), status(IDLE),
          isTimedLog(logTimedWholeFlag_), isMappedLog(logMapFlag_), simulationTime(simulationTime_), logFreq(logFreq_), timedLog(0), mappedLog(0)
    {
        // configs updates
        conf = conf_;

        // whether log mapping or not
        logLocation = logLocation_;
        logName = logName_;

        // ini parameter
        simNum = 0;
        poincareFlag = false;
        periodSteps = 1;
        poincareSecCount = 0;
    }

    // destructor
    ~runLogSimulation(){
        //if(timedLog) delete timedLog;
        //if(mappedLog) delete mappedLog;
    }

    // interface
    // this is called when one periodic simulation starts
    void start_oneProc(int simNum_, HexabotController& tC_){
        // start process;
        simNum = simNum_;
        status = IDLE;

        // logging preparation
        std::stringstream ss, ss2;
        ss  << logLocation <<  simNum << "_runLog_" << logName << "_timed.dat";
        timedLog = new logBasicData(ss.str(), prmAgent->getPrmName(), isTimedLog, logFreq);

        ss2 << logLocation <<  simNum << "_runLog_" << logName << "_mapped.dat";
        mappedLog = new logBasicData(ss2.str(), prmAgent->getPrmName(), isMappedLog, 1000);

        // ini param
        poincareFlag = false;
        poincareSecCount = 0;

        // change status
        status = START;
    }

    // this is called every simulation step
    void calcStep(HexabotController& tC_, double time_){
        // check the time error
        if(time_ > static_cast<double>(conf.maxSimulationTime)){
            std::cout << "@runLogSim :: Error -- timed out !!" << std::endl;
            status = FIN;
        }
        // switch depending on the mode which we use
        if(status == START){
            // we calc the simulation step nums of a period
            double contFreq = (double)tC_.eGetHexabotCtrConf().controlFreq;
            double oscFreq = (double)tC_.eGetHexabotCtrConf().oscConf[0].freq;
            periodSteps = (int)(contFreq / oscFreq);

            // set the phase difference to robot
            double phase1 = 0.; // the phase of the Leg 1
            double phase2 = M_PI; // the phase of the Leg 2

            // stop phase modulation
            tC_.eActivatePhaseReset_all(false);

            // set initial phase Diff
            for(int i=0;i<3;i++){
                // set the Phase of CPG
                tC_.eSetPhase(phase1,i);
                tC_.eSetPhase(phase2,i+3);

                // update the phase
                if( i < 2){
                    phase1 = phase1 + conf.initialState[i];
                    phase2 = phase2 + conf.initialState[i];
                }
                // check phase
                HexabotController::checkPhase(phase1);
                HexabotController::checkPhase(phase2);
            }

            // initialize prmAgent
            prmAgent->ini();

            waitStartTime = time_;
            status = WAIT;

            std::cout << "@runLogSim :: START -- !!" << std::endl;
            std::cout << "@runLogSim :: ini vector ( " << conf.initialState[0] << ", "<<conf.initialState[1] << " ) "<< std::endl;
        }
        else if(status == WAIT){
            // wait for a while
            double dPhase3;
            bool detectPoincareSection = false;

            // Poincre section checking
            if(fabs(conf.poincarePhase) < 0.001){
                // detect by ground contact of Leg 1
                if( !tC_.eIsLegContact(conf.poincarePhaseLegNum) ){
                    poincareFlag = true;
                }else if( tC_.eIsLegContact(conf.poincarePhaseLegNum) && poincareFlag){
                    poincareFlag = false;
                    detectPoincareSection = true;
                    dPhase3 = tC_.eGetPhase(conf.poincarePhaseLegNum);
                }
            }else{
                // detect by only phase
                dPhase3 = tC_.eGetPhase(conf.poincarePhaseLegNum) - conf.poincarePhase;
                if( fabs(dPhase3) < 2. * (2. * M_PI / (double)periodSteps) ){
                    if(dPhase3 < 0.){
                        poincareFlag = true;
                    }
                    // Poincre section is determined the place where the +- has changed
                    else if(dPhase3 > 0. && poincareFlag){
                        poincareFlag = false;
                        detectPoincareSection = true;
                    }
                }
            }

            // If detect poincre section
            if(detectPoincareSection){
                std::cout << "@runLogSim :: WAIT -- detect poincare sec !!" << std::endl;
                std::cout << "@runLogSim ::  time : " << time_ << " [s]"<< std::endl;

                // time is over, we will go to next step
                if( (time_  - waitStartTime) > static_cast<double>(conf.waitTime) ){
                    // go to next phase
                    status = RUN_LOG;

                    // start the run log phase
                    tC_.eActivatePhaseReset_all(true);
                    simStartTime = time_;
                    poincareSecCount = 0;

                    // log start
                    if(isTimedLog) timedLog->start(tC_, time_ - simStartTime);
                    if(isMappedLog) mappedLog->start(tC_, time_- simStartTime);

                    // changeAgent start
                    if(prmAgent) prmAgent->start(time_- simStartTime);

                    std::cout << "@runLogSim :: WAIT -- Simulation start !!" << std::endl;
                    std::cout << "@runLogSim ::  time : " << time_ << " [s],  param " << prmAgent->getPrmName() << ": " << prmAgent->getPrmValStr() <<  std::endl;

                    if(isMappedLog) mappedLog->work(tC_, prmAgent->getPrmValStr(), time_ - simStartTime);
                }
                // reset flag
                detectPoincareSection = false;
            }
        }
        else if(status == RUN_LOG){
            // proc of the simulation run log
            double simTime = time_ - simStartTime;

            //********* logging phase **********************
            double dPhase3;
            bool detectPoincareSection = false;

            // Poincre section checking
            if(fabs(conf.poincarePhase) < 0.001){
                // detect by ground contact of Leg 1
                if( !tC_.eIsLegContact(conf.poincarePhaseLegNum) ){
                    poincareFlag = true;
                }else if( tC_.eIsLegContact(conf.poincarePhaseLegNum) && poincareFlag){
                    poincareFlag = false;
                    detectPoincareSection = true;
                    dPhase3 = tC_.eGetPhase(conf.poincarePhaseLegNum);
                }
            }else{
                // detect by only phase
                dPhase3 = tC_.eGetPhase(conf.poincarePhaseLegNum) - conf.poincarePhase;
                if( fabs(dPhase3) < 2. * (2. * M_PI / (double)periodSteps) ){
                    if(dPhase3 < 0.){
                        poincareFlag = true;
                    }
                    // Poincre section is determined the place where the +- has changed
                    else if(dPhase3 > 0. && poincareFlag){
                        poincareFlag = false;
                        detectPoincareSection = true;
                    }
                }
            }

            // If detect poincre section
            if(detectPoincareSection){
                poincareSecCount++;

                double dp1 = tC_.eGetPhase(1) - tC_.eGetPhase(0);
                double dp2 = tC_.eGetPhase(2) - tC_.eGetPhase(1);
                HexabotController::checkPhase(dp1);
                HexabotController::checkPhase(dp2);

                // cout
                std::cout << "@runLogSim :: RUN_LOG -- Simulation start !!" << std::endl;
                std::cout << "@runLogSim ::  time : " << time_ << " [s],  param " << prmAgent->getPrmName() << ": " << prmAgent->getPrmValStr() <<  std::endl;
                std::cout << "@runLogSim ::  phase diff : ( " << dp1 << ", "<< dp2 << " )" << std::endl;

                // log Mapped Data
                if(isMappedLog) mappedLog->work(tC_, prmAgent->getPrmValStr(), simTime);
                detectPoincareSection = false;
            }

            // log Timed data
            if(isTimedLog) timedLog->work(tC_, prmAgent->getPrmValStr(), simTime);

            //********* parameter change *******************
            if(prmAgent) prmAgent->update(simTime);

            //********* check the time reached or not ******
            if(simTime > simulationTime){
                // finalize
                if(prmAgent) prmAgent->end(simTime);
                if(isTimedLog) timedLog->stop(tC_, simTime);
                if(isMappedLog) mappedLog->stop(tC_, simTime);

                //change status
                status = FIN;

                std::cout << "@runLogSim :: RUN_LOG -- Simulation finish!!" << std::endl;
            }

        }else{
            // do nothing
            return;
        }
    }

    // this is called every simulation step
    //  if it returns true, this procedure of simulation will end
    bool isFinished_oneProc(HexabotController& tC_, double time_){
        // check the situation
        if(status == FIN){
            return true;
        }
        return false;
    }

    // this is called when the procedure ends
    void finish_oneProc(HexabotController& tC_, double time_){

        // one proc simulation has finished
        if(isTimedLog){
            if(timedLog){
                delete timedLog;
                timedLog = 0;
            }

        }
        if(isMappedLog){
            if(mappedLog){
                delete mappedLog;
                mappedLog = 0;
            }
        }

        return;
    }

    // this is called when simulation restarts
    //  if it is true, the simulation will end
    void inform_restart(int simNum_){
        // initialize for restart
        // no need to do
        return;
    }

    // this is called when the simulation ends
    void finish_simulation(int simNum_){
        // close files
        /*
        if(timedLog){
            delete timedLog;
            timedLog = 0;
        }
        if(mappedLog){
            delete mappedLog;
            mappedLog = 0;
        }
        */

        return;
    }

};

#endif // RUNLOGSIMULATION_H

