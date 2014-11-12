#ifndef FINDSOLUTION_HPP
#define FINDSOLUTION_HPP 101

#include <iostream>
#include <sstream>
#include <iomanip>
#include "itfOneProcSimulation.h"
#include "findFixedPoints.hpp"
#include "hexabotController.h"

/*
 This is the class to find the solution and log data.

20140515 Ver 1.01
    fix the bug that the initial guessing is not valid

20140508 Ver 1.00
    start to make

*/

class findSolution : public itfOneProcSimulation<HexabotController>
{
public:
    typedef std::vector<osg::Vec2d> vec2dVector;
    typedef std::vector<double> vecDouble;
    typedef std::vector<int> vecInt;
    typedef std::array<std::array<double, 2>,2> Matrix2_2;
    typedef std::vector<Matrix2_2> Matrix2_2Vector;

    // configs
    struct findSolConf{
        // configs for find fixed point
        FindFixedPoints::FindFixedPointsConf findFpConf;

        // the initial state to begin serach
        osg::Vec2d initialState;
    };

    // get default configs
    static findSolConf getDefaultConf(){
        findSolConf conf;
        conf.findFpConf = FindFixedPoints::getDefaultFindFixedPointsConf();
        conf.initialState = osg::Vec2d(0., 0.);
        return conf;
    }

private:
    // configs
    findSolConf& conf;

    // flag for using initial guessing
    //   if this flag is true, the next initial states will be estimated by prv results automatically
    bool is_use_initialGuessing;

    // class for analysis
    FindFixedPoints findFixedPoints;

    // file for logging
    // this contains the initial val and fixed point and other informations
    std::ofstream ofs_main;
    // this contains the Mapping data of phase for each initial value
    std::ofstream ofs_maps;
    bool isLog_maps;

    std::string logLocation;
    std::string logName;

    // status flag
    enum STATUS {IDLE, CALC_FINDPOINT, FIN};
    enum STATUS status;

    // simNum
    int simNum;

    // is success in this step to cac
    bool is_success;

    // initial and final states (1 element)
    vec2dVector iniStateVec;

    // this is the vector for the result (1 element)
    vec2dVector fixedPoints;
    vecInt      fixedPointMappingPeriods;
    vec2dVector fixedPointAccuracies;
    vecDouble   fixedPointDuties;
    vecDouble   fixedRealPointPeriods;
    vecDouble   fixedPointTouchDownPhases;

    // the phases on poincare maps
    vec2dVector mappingPhases;

public:
    // constructor
    findSolution(findSolConf& conf_, bool is_use_initialGuessing_, bool logFlag_, bool logOneProcDat_, bool logMaps_, bool logTimedWholeData_, std::string logLocation_ = "", std::string logName_ = "")
        :itfOneProcSimulation<HexabotController>(), conf(conf_), findFixedPoints(logOneProcDat_, logTimedWholeData_, logLocation_, logName_, 100, 10), status(IDLE), is_use_initialGuessing(is_use_initialGuessing_), is_success(false)
    {
        // configs updates
        conf = conf_;
        // whether log mapping or not
        isLog_maps = logMaps_;
        logLocation = logLocation_;
        logName = logName_;

        // init states
        iniStateVec.clear();
        iniStateVec.push_back(conf.initialState);

        // setting for file. if you log
        if(logFlag_){
            std::stringstream str;
            str << logLocation_ << "fsol_" << logName_ << ".dat";
            ofs_main.open(str.str());

            // initial logging
            ofs_main << "# THis is the file which contains all of this simulation data in FindSolution" << std::endl;
            ofs_main << "#  Be notice that this mode is < POINCRE > " << std::endl;
            ofs_main << "# 1 simNum, 2 Freq, 3 Duty, 4 inhibiCoef, 5 iniPhaseDiff1, 6 iniPhaseDiff2,    7 fixedPhaseDiff1, 8 fixedPhaseDiff2, "
                << " 9 accuracyPhaseDiff1, 10 accuracy2,  11 realPeriod, 12 realDuty, 13 touchDownPhase, 14 solution_period  "<<std::endl;
        }

        // ini param
        simNum = 0;

    }

    // destructor
    ~findSolution(){
        if(ofs_main.is_open()) ofs_main.close();
        if(ofs_maps.is_open()) ofs_maps.close();
    }

    // interface
    // this is called when one periodic simulation starts
    void start_oneProc(int simNum_, HexabotController& tC_){
        // start process;
        simNum = simNum_;
        status = IDLE;

        // set initial value for the find fixed point
        findFixedPoints.set(iniStateVec, simNum_, conf.findFpConf);
        status = CALC_FINDPOINT;
    }

    // this is called every simulation step
    void calcStep(HexabotController& tC_, double time_){
        // switch depending on the mode which we use
        if(status == CALC_FINDPOINT){
            // calc find fixed opoint
            findFixedPoints.calcStep(tC_, time_);
        }else {
            // do nothing
            return;
        }
    }

    // this is called every simulation step
    //  if it returns true, this procedure of simulation will end
    bool isFinished_oneProc(HexabotController& tC_, double time_){
        // check the situation
        if(status == CALC_FINDPOINT){
            // find fixed points
            if( findFixedPoints.isFinished() ){
                // get the mapped data
                std::vector<vec2dVector> tmpVecVec;
                findFixedPoints.getMapPhases(tmpVecVec);
                if(tmpVecVec.size() == 1){
                    mappingPhases = tmpVecVec.at(0);
                }else mappingPhases.clear();

                // get the result
                is_success = findFixedPoints.getResult(fixedPoints, fixedPointMappingPeriods, fixedPointAccuracies, fixedPointDuties, fixedRealPointPeriods, fixedPointTouchDownPhases);

                return true;
            }
        }else if(status == FIN){
            return true;
        }

        return false;
    }

    // this is called when the procedure ends
    void finish_oneProc(HexabotController& tC_, double time_){

        // one proc simulation has finished
        // log main data
        if( ofs_main.is_open()){
            if(is_success){
                // logging
                for(uint i = 0;i< fixedPoints.size();i++){
                    // "# 1 simNum, 2 Freq, 3 Duty, 4 inhibiCoef, 5 iniPhaseDiff1, 6 iniPhaseDiff2,    7 fixedPhaseDiff1, 8 fixedPhaseDiff2, "
                    // << "  9 acuracy1, 10 acuracy2, 11 realPeriod, 12 realDuty, 13 touchDownPhase, 14 solution_period  "

                    ofs_main << std::setprecision(10) << simNum << " " << tC_.eGetHexabotCtrConf().oscConf[0].freq << " "<<  tC_.eGetHexabotCtrConf().oscConf[0].dutyRate << " "
                             << tC_.eGetHexabotCtrConf().oscConf[0].inhibiCoef << "  " << iniStateVec.at(i).x() << " " << iniStateVec.at(i).y() << "    "
                             <<fixedPoints.at(i).x() << " " << fixedPoints.at(i).y() << " "
                             << fixedPointAccuracies.at(i).x() << " " << fixedPointAccuracies.at(i).y() << " "
                             << fixedRealPointPeriods.at(i) << " " << fixedPointDuties.at(i) << " " << fixedPointTouchDownPhases.at(i) << " "
                             << fixedPointMappingPeriods.at(i)
                             << std::endl;
                }
            }else{
                ofs_main << "# fail fail fail  sim:" << simNum << std::endl;
            }
        }

        // log maps data
        if(isLog_maps){
            // is there data
            if(mappingPhases.size() == 0) return;

            std::stringstream str;
            str << logLocation << simNum << "_mapPhases_" << logName << ".dat";
            ofs_maps.open(str.str());
            if(ofs_maps.is_open()){
                // log data
                ofs_maps << "# This is the data contains the phase mapping histpry on Poincare section" << std::endl;
                ofs_maps << "#   Sim Num : " << simNum << ", initial Phase : (" << iniStateVec.at(0).x() << ", " << iniStateVec.at(0).y() << ") " ;
                if(is_success){
                    ofs_maps << ", fixed point : ( " << fixedPoints.at(0).x() << ", " << fixedPoints.at(0).y() << " ) " << std::endl;
                }else ofs_maps << std::endl;
                ofs_maps << "# 1:number of section,  2:phaseDiff1,  3:phaseDiff2 " << std::endl;

                // log every data
                for(uint i=0;i<mappingPhases.size();i++){
                    ofs_maps << std::setprecision(10) << i << "  " << mappingPhases.at(i).x() << "  " << mappingPhases.at(i).y() << std::endl;
                }
            }
            ofs_maps.close();
        }

        return;
    }

    // this is called when simulation restarts
    //  if it is true, the simulation will end
    void inform_restart(int simNum_){
        // initialize for restart
        if(is_use_initialGuessing){
            if(is_success){
                // update initial value
                iniStateVec = fixedPoints;
            }else{
                // if it is fail, just use same initial value
            }

        }else{
            // updates it from configulation
            iniStateVec.clear();
            iniStateVec.push_back(conf.initialState);
        }

        // cr lf for file.
        if( ofs_main.is_open() ){ ofs_main << std::endl;}

        return;
    }

    // this is called when the simulation ends
    void finish_simulation(int simNum_){
        // close the file
        if(ofs_main.is_open()) ofs_main.close();

        return;
    }

};



#endif // FINDSOLUTION_HPP
