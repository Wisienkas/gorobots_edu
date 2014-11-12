#ifndef FINDFIXEDPOINTS_HPP
#define FINDFIXEDPOINTS_HPP 103

/*
THis is the class to search the fixed points
20140610 Ver 1.03
    Add the conf to change poincare section's phase (leg 1,2,3)

20140508 Ver 1.02
    Add the time limit to analize

20140508 Ver 1.01
    Add the function to get the sequence data of the phase diff on Poincare section in the FindfixedPoint class
    Add the class to log whole data

*/
#include <iostream>
#include <sstream>
#include "hexabotController.h"
#include "hxPeriodAnalysis.h"
#include "hxLogData.hpp"
#include <math.h>
#include <osg/Matrix>

#define FINDFIXEDPOINT_DEBUG

// Class to find fixed point (only one)
class FindFixedPoint{
public:
    typedef std::vector<osg::Vec2d> vec2dVector;

    typedef struct{
        // The phase When the Poincre plane is devided (phi = ??)
        double poincrePhase;
        // the leg number of the phase which is used for the poincare section
        int poincarePhaseLegNum; // 0,1,2 (otherwise it will casue memory collaptions)
        // How many steps will we wait before evaluating the fixed point
        int waitPeriodNum;
        // Max waiting steps to find a fixed point
        int maxPeriodNum;

        // How much Difference of phase between poincre sections can we afford to regard the phase as fixed points
        double phaseThreshold;
        // How many periods solution we search??
        int maxSolutionPeriod;

        // The maximum time which we try to search
        int maxSimulationTime;

    }FindFixedPointConf;

private:
    FindFixedPointConf conf;
    hxPeriodAnalysis* pAnaly;
    hxLogData* pLog;

    // fixedPhase
    osg::Vec2d fPhase;
    // solution period
    int fPeriod;

    // ini phase
    osg::Vec2d iniPhase;
    // diff Phase
    osg::Vec2d diffFixedPhase;
    // fixedPhase log
    vec2dVector logPhase;

    // maxTimeFlag
    bool maxStepFlag;
    // count
    int count;
    // period steps for 1 period
    double periodSteps;
    bool poincreFlag;

    // finish Flag
    bool finishFlag;

    // time out flag
    bool timeoutFlag;

    // status value
    enum STATUS_ {IDLE, START, SET_WAIT, WAIT, SEARCH, ANALYSIS, FIN, TIMEOUT};
    enum STATUS_ STATUS;

    // start time
    double startTime;

public:
    // this is the interface to serve
    //default Const
    FindFixedPoint():
        finishFlag(false), timeoutFlag(false), STATUS(IDLE), count(0), pAnaly(0), pLog(0)
    {}

    // starts the simulation
    bool set(osg::Vec2d _iniPhase, hxPeriodAnalysis* pAnal_, hxLogData* pLog_, FindFixedPointConf& conf_){
        finishFlag = false;
        timeoutFlag = false;
        STATUS = START;
        count = 0;
        poincreFlag = false;
        iniPhase = _iniPhase;
        maxStepFlag = false;
        logPhase.clear();
        pAnaly = pAnal_; // it is not NULL till getResult
        pLog = pLog_; // it is not NULL till get result
        conf = conf_;
        fPeriod = 0;
        return true;
    }

    // CalcFunc ; it is called every time step
    bool calcStep(HexabotController& tC, double time){
        if(STATUS == START){
            // we calc the simulation step nums of a period
            double contFreq = (double)tC.eGetHexabotCtrConf().controlFreq;
            double oscFreq = (double)tC.eGetHexabotCtrConf().oscConf[0].freq;
            periodSteps = (int)(contFreq / oscFreq);

            // set the phase difference to robot
            double phase1 = 0.; // the phase of the Leg 1
            double phase2 = M_PI; // the phase of the Leg 2

            // stop phase modulation
            tC.eActivatePhaseReset_all(false);

            // set initial phase Diff
            for(int i=0;i<3;i++){
                // set the Phase of CPG
                tC.eSetPhase(phase1,i);
                tC.eSetPhase(phase2,i+3);

                // update the phase
                if( i < 2){
                    phase1 = phase1 + iniPhase[i];
                    phase2 = phase2 + iniPhase[i];
                }
                // check phase
                HexabotController::checkPhase(phase1);
                HexabotController::checkPhase(phase2);
            }

            // setting is finished -> Next, WAIT mode
            STATUS = SET_WAIT;
            startTime = time;

            // for debug
            #ifdef FINDFIXEDPOINT_DEBUG
                std::cout << "@@ DEBUG @@@@FixedPoint START:: Finish the STATUS < START > "  << std::endl;
                std::cout << "@@ DEBUG @@@@FixedPoint START:: nowTime:" << time << "[s], iniPhase:" << iniPhase[0] << " " << iniPhase[1]
                          << "[rad]" << std::endl;
            #endif

        }else if(STATUS == SET_WAIT){
            // we wait till step reaches setted limit

            double dPhase3;
            bool detectPoincreSection = false;
            // Poincre section checking
            if(fabs(conf.poincrePhase) < 0.001){
                // detect by ground contact of Leg 1
                if( !tC.eIsLegContact(conf.poincarePhaseLegNum) ){
                    poincreFlag = true;
                }else if( tC.eIsLegContact(conf.poincarePhaseLegNum) && poincreFlag){
                    poincreFlag = false;
                    detectPoincreSection = true;
                    dPhase3 = tC.eGetPhase(conf.poincarePhaseLegNum);
                }
            }else{
                // detect by only phase
                dPhase3 = tC.eGetPhase(conf.poincarePhaseLegNum) - conf.poincrePhase;
                if( fabs(dPhase3) < 2. * (2. * M_PI / (double)periodSteps) ){
                    if(dPhase3 < 0.){
                        poincreFlag = true;
                    }
                    // Poincre section is determined the place where the +- has changed
                    else if(dPhase3 > 0. && poincreFlag){
                        poincreFlag = false;
                        detectPoincreSection = true;
                    }
                }
            }

            // If detect poincre section
            if(detectPoincreSection){
                // Counting the period
                count++;

                // get the phase diff data
                fPhase[0] = tC.eGetPhase(1) - tC.eGetPhase(0);
                fPhase[1] = tC.eGetPhase(2) - tC.eGetPhase(1);
                HexabotController::checkPhase(fPhase[0]);
                HexabotController::checkPhase(fPhase[1]);

                // for debug
                #ifdef FINDFIXEDPOINT_DEBUG
                    std::cout << "@@ DEBUG @@@@ FixedPoint SET_WAIT:: In the STATUS < SET_WAIT > "  << std::endl;
                    std::cout << "@@ DEBUG @@@@ FixedPoint SET_WAIT::   nowTime:" << time << "[s], poincre section dPhase:" << dPhase3
                              << "[rad]  count:"<< count << std::endl;
                    std::cout << "@@ DEBUG @@@@ FixedPoint SET_WAIT::   dPhase1:" << fPhase[0] << "[rad], dPhase2:" << fPhase[1]
                              << "[rad]" << std::endl;
                #endif

                detectPoincreSection = false;
            }

            // count reaches 10th and 30s has passed
            if(count >= 10 && (time - startTime) > 30.){
                // activate the phase modulation methods
                tC.eActivatePhaseReset_all(true);

                // end of the SET_WAIT MODE
                count = 0;
                STATUS = WAIT;

                // start log if there are pLog
                if(pLog) pLog->start(tC, time);

                // DEBUG
                #ifdef FINDFIXEDPOINT_DEBUG
                    std::cout << "@@ DEBUG @@@@ FixedPoint SET_WAIT:: FINISH the STATUS < SET_WAIT > "  << std::endl;
                #endif
            }

        }else if(STATUS == WAIT){
            // we wait till step reaches setted limit

            // logging
            if(pLog) pLog->work(tC, time);

            bool detectPoincreSection2 = false;
            double dPhase;
            // Poincre section checking
            if(fabs(conf.poincrePhase) < 0.001){
                // detect by ground contact of Leg 1
                if( !tC.eIsLegContact(conf.poincarePhaseLegNum) ){
                    poincreFlag = true;
                }else if( tC.eIsLegContact(conf.poincarePhaseLegNum) && poincreFlag){
                    dPhase = tC.eGetPhase(conf.poincarePhaseLegNum);
                    poincreFlag = false;
                    detectPoincreSection2 = true;
                }
            }else{
                // detect by only phase
                dPhase = tC.eGetPhase(conf.poincarePhaseLegNum) - conf.poincrePhase;
                if( fabs(dPhase) < 2. * (2. * M_PI / (double)periodSteps) ){
                    if(dPhase < 0.){
                        poincreFlag = true;
                    }
                    // Poincre section is determined the place where the +- has changed
                    else if(dPhase > 0. && poincreFlag){
                        poincreFlag = false;
                        detectPoincreSection2 = true;
                    }
                }
            }

            // If detect poincre section
            if(detectPoincreSection2){
                    // Counting the period
                    count++;

                    // get the phase diff data
                    fPhase[0] = tC.eGetPhase(1) - tC.eGetPhase(0);
                    fPhase[1] = tC.eGetPhase(2) - tC.eGetPhase(1);
                    HexabotController::checkPhase(fPhase[0]);
                    HexabotController::checkPhase(fPhase[1]);
                    // log
                    logPhase.push_back(fPhase);

                    // for debug
                    #ifdef FINDFIXEDPOINT_DEBUG
                        std::cout << "@@ DEBUG @@@@ FixedPoint WAIT:: In the STATUS < WAIT > "  << std::endl;
                        std::cout << "@@ DEBUG @@@@ FixedPoint WAIT::   nowTime:" << time << "[s], poincre section dPhase:" << dPhase
                                  << "[rad]  count:"<< count << std::endl;
                        std::cout << "@@ DEBUG @@@@ FixedPoint WAIT::   dPhase1:" << fPhase[0] << "[rad], dPhase2:" << fPhase[1]
                                  << "[rad]" << std::endl;
                    #endif

                    detectPoincreSection2 = false;
            }

            // count reaches limit
            if(count >= conf.waitPeriodNum){
                // end of the WAIT MODE
                STATUS = SEARCH;

                // DEBUG
                #ifdef FINDFIXEDPOINT_DEBUG
                    std::cout << "@@ DEBUG @@@@ FixedPoint WAIT:: FINISH the STATUS < WAIT > "  << std::endl;
                #endif
            }
        }else if(STATUS == SEARCH){
            // we start to search the fixed point
            // logging
            if(pLog) pLog->work(tC, time);

            bool detectPoincreSection3 = false;
            double dPhase2;
            // Poincre section checking
            if(fabs(conf.poincrePhase) < 0.001){
                // detect by ground contact of Leg 1
                if( !tC.eIsLegContact(conf.poincarePhaseLegNum) ){
                    poincreFlag = true;
                }else if( tC.eIsLegContact(conf.poincarePhaseLegNum) && poincreFlag){
                    dPhase2 = tC.eGetPhase(conf.poincarePhaseLegNum);
                    poincreFlag = false;
                    detectPoincreSection3 = true;
                }
            }else{
                // detect by only phase
                dPhase2 = tC.eGetPhase(conf.poincarePhaseLegNum) - conf.poincrePhase;
                if( fabs(dPhase2) < 2. * (2. * M_PI / (double)periodSteps) ){
                    if(dPhase2 < 0.){
                        poincreFlag = true;
                    }
                    // Poincre section is determined the place where the +- has changed
                    else if(dPhase2 > 0. && poincreFlag){
                        poincreFlag = false;
                        detectPoincreSection3 = true;
                    }
                }
            }

            // If detect poincre section
            if(detectPoincreSection3){
                    // Counting the period
                    count++;

                    // get the phase diff data
                    //osg::Vec2d prvPhase = fPhase;
                    fPhase[0] = tC.eGetPhase(1) - tC.eGetPhase(0);
                    fPhase[1] = tC.eGetPhase(2) - tC.eGetPhase(1);
                    HexabotController::checkPhase(fPhase[0]);
                    HexabotController::checkPhase(fPhase[1]);

                    // calculate the difference
                    auto itr = logPhase.end();
                    for(int i=0; i< std::min((int)(logPhase.size()), conf.maxSolutionPeriod); ++i){
                        // change itr
                        --itr;

                        // get the diff between poincre sections
                        diffFixedPhase[0] = fabs((*itr).x() -  fPhase.x());
                        diffFixedPhase[1] = fabs((*itr).y() -  fPhase.y());

                        // detect fixed point
                        if(diffFixedPhase[0] < conf.phaseThreshold && diffFixedPhase[1] < conf.phaseThreshold){
                            STATUS = ANALYSIS;
                            maxStepFlag = false;
                            fPeriod = i + 1; // solution period
                            // start logging
                            if(pAnaly) pAnaly->start(tC, time);
                            if(pAnaly) pAnaly->work(tC, time);

                            // for debug
                            #ifdef FINDFIXEDPOINT_DEBUG
                                std::cout << "@@ DEBUG @@@@ FixedPoint SEARCH:: FINISH the STATUS < SEARCH > "  << std::endl;
                                std::cout << "@@ DEBUG @@@@ FixedPoint SEARCH::   nowTime:" << time << "[s], poincre section dPhase:" << dPhase2
                                          << "[rad]  count:"<< count << std::endl;
                                std::cout << "@@ DEBUG @@@@ FixedPoint SEARCH::   dPhase1:" << fPhase[0] << "[rad], dPhase2:" << fPhase[1]
                                          << "[rad]" << std::endl;
                                std::cout << "@@ DEBUG @@@@ FixedPoint SEARCH::   ddPhase1:" << diffFixedPhase[0] << "[rad], ddPhase2:" << diffFixedPhase[1]
                                          << "[rad]" << std::endl;
                            #endif
                            return true;

                        }
                    }

                    // When we did not find the fixed point
                    // log
                    logPhase.push_back(fPhase);

                    // for debug
                    #ifdef FINDFIXEDPOINT_DEBUG
                        std::cout << "@@ DEBUG @@@@ FixedPoint SEARCH:: IN the STATUS < SEARCH > "  << std::endl;
                        std::cout << "@@ DEBUG @@@@ FixedPoint SEARCH::   nowTime:" << time << "[s], poincre section dPhase:" << dPhase2
                                  << "[rad]  count:"<< count << std::endl;
                        std::cout << "@@ DEBUG @@@@ FixedPoint SEARCH::   dPhase1:" << fPhase[0] << "[rad], dPhase2:" << fPhase[1]
                                  << "[rad]" << std::endl;
                        std::cout << "@@ DEBUG @@@@ FixedPoint SEARCH::   ddPhase1:" << diffFixedPhase[0] << "[rad], ddPhase2:" << diffFixedPhase[1]
                                  << "[rad]" << std::endl;
                    #endif

            }

            // count reaches limit
            if(count >= conf.maxPeriodNum){
                // end of the SEARCH MODE
                STATUS = FIN;
                maxStepFlag = true;

                // DEBUG
                #ifdef FINDFIXEDPOINT_DEBUG
                    std::cout << "@@ DEBUG @@@@ FixedPoint SEARCH:: FINISH the STATUS < SEARCH >  with TIMEOUT"  << std::endl;
                #endif
            }
        }else if(STATUS == ANALYSIS){
            // we start to analyze the  periodic walking
            // logging
            if(pLog) pLog->work(tC, time);

            // logging and analyzing
            if(pAnaly) pAnaly->work(tC, time);

            bool detectPoincreSection4 = false;
            double dPhase4;
            // Poincre section checking
            if(fabs(conf.poincrePhase) < 0.001){
                // detect by ground contact of Leg 1
                if( !tC.eIsLegContact(conf.poincarePhaseLegNum) ){
                    poincreFlag = true;
                }else if( tC.eIsLegContact(conf.poincarePhaseLegNum) && poincreFlag){
                    dPhase4 = tC.eGetPhase(conf.poincarePhaseLegNum);
                    poincreFlag = false;
                    detectPoincreSection4 = true;
                }
            }else{
                // detect by only phase
                dPhase4 = tC.eGetPhase(conf.poincarePhaseLegNum) - conf.poincrePhase;
                if( fabs(dPhase4) < 2. * (2. * M_PI / (double)periodSteps) ){
                    if(dPhase4 < 0.){
                        poincreFlag = true;
                    }
                    // Poincre section is determined the place where the +- has changed
                    else if(dPhase4 > 0. && poincreFlag){
                        poincreFlag = false;
                        detectPoincreSection4 = true;
                    }
                }
            }

            // If detect poincre section
            if(detectPoincreSection4){
                // Counting the period
                count++;

                if(count >= fPeriod){
                    // detect fixed points
                    STATUS = FIN;

                    // stop logging
                    if(pAnaly) pAnaly->stop(tC, time);

                    // for debug
                    #ifdef FINDFIXEDPOINT_DEBUG
                        std::cout << "@@ DEBUG @@@@ FixedPoint ANALYSIS:: FINISH the STATUS < SEARCH > "  << std::endl;
                        std::cout << "@@ DEBUG @@@@ FixedPoint ANALYSIS::   nowTime:" << time << "[s], poincre section dPhase:" << dPhase4
                                  << "[rad]  count:"<< count << std::endl;
                        std::cout << "@@ DEBUG @@@@ FixedPoint ANALYSIS::   dPhase1:" << fPhase[0] << "[rad], dPhase2:" << fPhase[1]
                                  << "[rad]" << std::endl;
                        std::cout << "@@ DEBUG @@@@ FixedPoint ANALYSIS::   ddPhase1:" << diffFixedPhase[0] << "[rad], ddPhase2:" << diffFixedPhase[1]
                                  << "[rad]" << std::endl;
                    #endif
                }
            }
        }else if(STATUS == FIN){
            // finish
            finishFlag = true;
            // stop log
            if(pLog) pLog->stop(tC, time);
        }else if(STATUS == IDLE){
            // do nothing
        }else if(STATUS == TIMEOUT){
            // set timeout flag and finish flag
            timeoutFlag = true;
            finishFlag = true;
            std::cout << " TIMEOUT in FindFIxedPoint :" << time - startTime << " [s]"<< std::endl;
        }else{
            std::cout << " STATUS is wrong in FindFIxedPoint "<< std::endl;
        }

        // check timeout
        if( (time - startTime) > conf.maxSimulationTime){
            STATUS = TIMEOUT;
        }
        return true;
    }

    // isFinished calc??
    bool isFinished(void){
        return finishFlag;
    }

    // get the result!! fixed phase is gotten
    bool getResult(osg::Vec2d&  _fixedPhase, osg::Vec2d& _diffFixedPhase, int& solutionPeriod_){
        pAnaly = 0;
        pLog = 0;
        if(!finishFlag || maxStepFlag || timeoutFlag) return false;
        else{
            _fixedPhase = fPhase;
            _diffFixedPhase = diffFixedPhase;
            solutionPeriod_ = fPeriod;

            STATUS = IDLE;
            return true;
        }
    }

    // get the log phase :: The phase on Poincare section. false means fail
    // This function shoulb be called after the simukation has finihsed
    bool getLogPhase(vec2dVector& _vecLogPhase){
        if(isFinished()){
            _vecLogPhase = logPhase;
            return true;
        }else{
            _vecLogPhase.clear();
            return false;
        }
    }

};

// class to find fixed points
class FindFixedPoints{
private:

public:
    typedef struct{
        FindFixedPoint::FindFixedPointConf fConf;
        // anything??
    }FindFixedPointsConf;

    // typedef
    typedef std::vector<osg::Vec2d> vec2dVector;
    typedef std::vector<double> vecDouble;
    typedef std::vector<int> vecInt;
    typedef std::vector<vec2dVector> vec2dVecVector;

    // set default conf func
    static FindFixedPointsConf getDefaultFindFixedPointsConf(void){
        FindFixedPointsConf conf;

        // Maximum steps for which we search
        conf.fConf.maxPeriodNum = 50;
        // wait steps before search
        conf.fConf.waitPeriodNum = 30;
        // phase diff threshold
        conf.fConf.phaseThreshold = 0.00001;
        // poincre section
        conf.fConf.poincrePhase = 0.;//39./ 20. *M_PI;
        // poincare section leg num 0, 1, 2
        conf.fConf.poincarePhaseLegNum = 0;
        // how many period of the solution you search
        conf.fConf.maxSolutionPeriod = 3;
        // max time to simulate [s]
        conf.fConf.maxSimulationTime = 10000;

        return conf;
    }

private:
    // Configuration params
    FindFixedPointsConf conf;
    // fixedPhase
    vec2dVector fPhaseVec;
    // diff phase between poincre section
    vec2dVector diffFixedPhaseVec;
    // ini Phase
    vec2dVector iniPhaseVec;

    // mapping data
    vec2dVecVector mapPhasesVec;

    // period , duty, touch down phase
    vecDouble periodVec;
    vecDouble dutyVec;
    vecDouble tdPhaseVec;
    vecInt fPeriod;

    // period analysis pointer
    hxPeriodAnalysis* pAnal;
    // whole data log pointer
    hxLogData* pLog;

    // count
    int count;
    // dataNum
    int dataNum;

    // simulation number
    int simNum;

    // loging ??
    bool logFlag_pAnal;
    // log Hz
    int logFreq_pAnal;

    // loging ??
    bool logFlag_pLog;
    // log Hz
    int logFreq_pLog;


    // logLocation
    std::string logLocation;
    // logNum
    std::string logName;

    // finish Flag
    bool finishFlag;
    // errorFlag
    bool errorFlag;

    // status value
    enum STATUS_ {IDLE, CALC, FIN};
    enum STATUS_ STATUS;

private:
    FindFixedPoint findPoint;

public:
    // this is the interface to serve
    //default Const
    FindFixedPoints(bool logFlag_periodData_, bool logFlag_wholeData_, std::string logLocation_ = std::string(" "), std::string logName_ = std::string(" "), int logFreq_periodData_ = 100, int logFreq_wholeData_ = 10)
        :findPoint(), STATUS(IDLE), finishFlag(false), count(0)
    {
        logFlag_pAnal = logFlag_periodData_;
        logFlag_pLog = logFlag_wholeData_;
        logFreq_pAnal = logFreq_periodData_;
        logFreq_pLog = logFreq_wholeData_;

        logLocation = logLocation_;
        logName = logName_;
    }

    // starts the simulation
    bool set(vec2dVector _iniPhase, int simNum_, FindFixedPointsConf& _conf){
        conf = _conf;
        simNum = simNum_;

        iniPhaseVec = _iniPhase;

        // Number of data
        dataNum = iniPhaseVec.size();
        if(dataNum == 0){
            std::cout << " There are no data!!!! in FindFixedPoints Check it" << std::endl;
            return false;
        }

        // reset the count
        count = 0;
        // reset the resulted data
        fPhaseVec.clear();
        diffFixedPhaseVec.clear();
        periodVec.clear();
        dutyVec.clear();
        tdPhaseVec.clear();
        fPeriod.clear();
        mapPhasesVec.clear();

        // create pAnal
        std::stringstream ss;
        ss << logLocation << simNum << "_" << count << "_pAnaly_" << logName << ".dat";
        pAnal = new hxPeriodAnalysis(ss.str(), logFlag_pAnal, logFreq_pAnal);

        // create pLog
        std::stringstream ss2;
        ss2 << logLocation << simNum << "_" << count << "_pLog_" << logName << ".dat";
        pLog = new hxLogData(ss2.str(), logFlag_pLog, logFreq_pLog);

        // start the simulation
        findPoint.set(iniPhaseVec.at(count), pAnal, pLog, conf.fConf);

        // change the status and flag
        finishFlag = false;
        errorFlag = false;
        STATUS = CALC;

        return true;
    }

    // CalcFunc ; it is called every time step
    bool calcStep(HexabotController& tC, double time){
        // check the finish flag
        if(finishFlag){
            return false;
        }else{
            // proc the inner class
            findPoint.calcStep(tC,time);

            // if the calc has finished
            if(findPoint.isFinished()) changeStatus();

            return true;
        }
    }

    // isFinished calc??
    bool isFinished(void){return finishFlag;}

    // get the periodic data
    // please call it before get result
    bool getMapPhases(vec2dVecVector& _mapPhasesVec){
        if(finishFlag){
            _mapPhasesVec = mapPhasesVec;
            return true;
        }
        return false;
    }

    // get the result!! fixed phase is gotten
    bool getResult(vec2dVector&  _fixedPhase, vecInt& _fPeriod,  vec2dVector& _diffFixedPhase, vecDouble& dutyRate_, vecDouble& realPeriod_, vecDouble& tdPhase_){
        if(finishFlag && !errorFlag){
            _fixedPhase = fPhaseVec;
            _fPeriod = fPeriod;
            _diffFixedPhase = diffFixedPhaseVec;
            dutyRate_ = dutyVec;
            realPeriod_ = periodVec;
            tdPhase_ = tdPhaseVec;

            // terminate this calc step
            finishFlag = false;
            errorFlag = false;
            STATUS = IDLE;

            return true;
        }else{
            _fixedPhase.clear();
            _fPeriod.clear();
            _diffFixedPhase.clear();
            dutyRate_.clear();
            realPeriod_.clear();
            tdPhase_.clear();
            return false;
        }
    }

private:
    // change status
    void changeStatus(){
        //
        if(STATUS == CALC){
            // finish the calculation
            // get Data
            vec2dVector tmpMaps;
            findPoint.getLogPhase(tmpMaps);

            osg::Vec2d tmpFP;
            osg::Vec2d tmpDFP;
            int tmpPr;
            errorFlag = errorFlag || !findPoint.getResult(tmpFP,tmpDFP,tmpPr);     

            // add to the result
            fPhaseVec.push_back(tmpFP);
            diffFixedPhaseVec.push_back(tmpDFP);
            periodVec.push_back(pAnal->get_realPeriod());
            dutyVec.push_back(pAnal->get_realDutyrate(0));
            tdPhaseVec.push_back(pAnal->get_touchDownPhase(0));
            fPeriod.push_back(tmpPr);
            mapPhasesVec.push_back(tmpMaps);

            // clear
            delete pAnal;
            delete pLog;

            count++;

            // check there are data or not to calculate
            if(count < dataNum){
                // create pAnal
                std::stringstream ss3;
                ss3 << logLocation << simNum << "_" << count << "_pAnaly_" << logName << ".dat";
                pAnal = new hxPeriodAnalysis(ss3.str(),logFlag_pAnal, logFreq_pAnal);

                // create pLog
                std::stringstream ss4;
                ss4 << logLocation << simNum << "_" << count << "_pLog_" << logName << ".dat";
                pLog = new hxLogData(ss4.str(),logFlag_pLog, logFreq_pLog);


                // repeat the simulation
                findPoint.set(iniPhaseVec.at(count), pAnal, pLog, conf.fConf);

                // cout
                std::cout <<">> sim" << simNum << "  " << count << "/"<< dataNum << " stepFinished >>  x" << std::endl;

                // for debug
                #ifdef FINDFIXEDPOINT_DEBUG
                    std::cout << "@@ DEBUG @@ FindFixedPoints:: Restart the simulation, count:" << count <<  std::endl;
                #endif

            }else{
                // finish simulation
                STATUS = FIN;
                finishFlag = true;

                // for debug
                #ifdef FINDFIXEDPOINT_DEBUG
                    std::cout << "@@ DEBUG @@ FindFIxedPoints:: Finish the simulation, count:" << count <<  std::endl;
                #endif
            }
        }else if(STATUS == IDLE){

        }else{
            std::cout << " status error occurs on FindFixedPoints " << std::endl;
        }
    }

};

#endif






