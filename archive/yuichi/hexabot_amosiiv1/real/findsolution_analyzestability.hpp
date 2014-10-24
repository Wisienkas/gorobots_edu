#ifndef FINDSOLUTION_ANALYZESTABILITY_HPP
#define FINDSOLUTION_ANALYZESTABILITY_HPP

#include <iostream>
#include <sstream>
#include <iomanip>
#include "itfOneProcSimulation.h"
#include "findFixedPoints.hpp"
#include "stabilityAnalysis.h"
#include "hexabotController.h"

/*
 This is the class to find and analyze the solution
    Config file they do not have!!.

*/

class findSolution_analyzeStability : public itfOneProcSimulation<HexabotController>
{
public:
    typedef std::vector<osg::Vec2d> vec2dVector;
    typedef std::vector<double> vecDouble;
    typedef std::vector<int> vecInt;
    typedef std::array<std::array<double, 2>,2> Matrix2_2;
    typedef std::vector<Matrix2_2> Matrix2_2Vector;

    // configs
    struct findSolAnalyStabConf{
        // configs for stabanalysis
        StabAnaly::StabAnalyConf stabAnalyConf;
        // configs for find fixed point
        FindFixedPoints::FindFixedPointsConf findFpConf;

        // the initial state to begin serach
        vec2dVector initialStateVec;
    };

    // get default configs
    static findSolAnalyStabConf getDefaultConf(){
        findSolAnalyStabConf conf;
        conf.findFpConf = FindFixedPoints::getDefaultFindFixedPointsConf();
        conf.stabAnalyConf = StabAnaly::getDefaultStabAnalyConf();
        conf.initialStateVec.clear();
        //conf.initialStateVec.push_back(osg::Vec2d(0, 0));
        return conf;
    }

private:
    // configs
    findSolAnalyStabConf& conf;

    // flag for using initial guessing
    //   if this flag is true, the next initial states will be estimated by prv results automatically
    bool is_use_initialGuessing;

    // class for analysis
    StabAnaly stabAnaly;
    FindFixedPoints findFixedPoints;

    // file for logging
    std::ofstream ofs;

    // status flag
    enum STATUS {IDLE, CALC_FINDPOINT, CALC_STABANAL, FIN};
    enum STATUS status;

    // simNum
    int simNum;

    // is success in this step to cac
    bool is_success;

    // initial and final states
    vec2dVector iniStateVec;

    // this is the vector for the result
    vec2dVector fixedPoints;
    vecInt      fixedPointMappingPeriods;
    vec2dVector fixedPointAccuracies;
    vecDouble   fixedPointDuties;
    vecDouble   fixedRealPointPeriods;
    vecDouble   fixedPointTouchDownPhases;

    vec2dVector eigAbsVals;
    Matrix2_2Vector eigVals;
    Matrix2_2Vector eigVecs;
    Matrix2_2Vector jacobians;

public:
    // constructor
    findSolution_analyzeStability(findSolAnalyStabConf& conf_, bool is_use_initialGuessing_, bool logFlag_, bool logOneProcDat_, bool logTimedDat_, std::string logLocation_ = "", std::string logName_ = "")
        :itfOneProcSimulation<HexabotController>(), conf(conf_),
          stabAnaly(), findFixedPoints(logOneProcDat_, logTimedDat_, logLocation_, logName_, 100, 10), status(IDLE), is_use_initialGuessing(is_use_initialGuessing_), is_success(false)
    {
        // configs updates
        conf = conf_;

        // init states
        iniStateVec = conf.initialStateVec;

        // setting for file. if you log
        if(logFlag_){
            std::stringstream str;
            str << logLocation_ << "fsAnal_" << logName_ << ".dat";
            ofs.open(str.str());

            // initial logging
            ofs << "# THis is the file which contains all of this simulation data in FindSolution and Analyze" << std::endl;
            ofs << "#  Be notice that this mode is < POINCRE > " << std::endl;
            ofs << "# 1 simNum, 2 Freq, 3 Duty, 4 fixedPhaseDiff1, 5 fixedPhaseDiff2, 6 delta, "
                << "7 absolute Eigen Val(large), 8 absolute Eigen Val(small)" << std::endl;
            ofs << "# 9 Eigen Val 1 -real value,  10 Eigen Val 1 -imerginal value, "
                << "11 Eigen Val 2 -real value,  12 Eigen Val 2 -imerginal value,  13 diffFixedPhase1, 14 diffFixedPhase2 " << std::endl;
            ofs << "# 15 dPdx_00  16 dPdx_01  17 dPdx_10  18 dPdx_11  19 eVec_1x  20 eVec_1y  21 eVec_2x  22 eVec_2y  23 inhibiCoef"
                << "  24 realPeriod  25 realDuty  26 touchDownPhase 27 solution_period  "<<std::endl;
        }

        // ini param
        simNum = 0;

    }

    // destructor
    ~findSolution_analyzeStability(){
        if(ofs.is_open()) ofs.close();
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
        }else if(status == CALC_STABANAL){
            // calc stab
            stabAnaly.calcStep(tC_, time_);
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
                is_success = findFixedPoints.getResult(fixedPoints, fixedPointMappingPeriods, fixedPointAccuracies, fixedPointDuties, fixedRealPointPeriods, fixedPointTouchDownPhases);
                // is success ??
                if(!is_success){
                    // fail to calc
                    status = FIN;
                    return true;
                }else{
                    // success
                    // start to analyze stability
                    status = CALC_STABANAL;
                    stabAnaly.set(fixedPoints, simNum, conf.stabAnalyConf);
                }
            }
        }else if(status == CALC_STABANAL){
            // calc stab analysis
            if( stabAnaly.isFinished() ){
                //
                is_success = stabAnaly.getResult(eigAbsVals, eigVals, eigVecs, jacobians);
                // finish simulations
                status = FIN;
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
        if( ofs.is_open()){
            if(is_success){
                // logging
                for(uint i = 0;i< fixedPoints.size();i++){
                    ofs << simNum << " " << tC_.eGetHexabotCtrConf().oscConf[0].freq << " "<<  tC_.eGetHexabotCtrConf().oscConf[0].dutyRate << " "
                        << std::setprecision(10) <<fixedPoints.at(i).x() << " " << std::setprecision(10) << fixedPoints.at(i).y() << " "
                             << conf.stabAnalyConf.delta << " "
                             << eigAbsVals.at(i).x() << " "<<  eigAbsVals.at(i).y() << " "
                             << eigVals.at(i).at(0).at(0) << " " << eigVals.at(i).at(0).at(1) <<  " "
                             << eigVals.at(i).at(1).at(0) << " " << eigVals.at(i).at(1).at(1) <<  " "
                             << fixedPointAccuracies.at(i).x() << " " << fixedPointAccuracies.at(i).y() << "   "
                             << jacobians.at(i).at(0).at(0) << " " << jacobians.at(i).at(0).at(1) << " "
                             << jacobians.at(i).at(1).at(0) << " " << jacobians.at(i).at(1).at(1) << "   "
                             << eigVecs.at(i).at(0).at(0) << " " << eigVecs.at(i).at(0).at(1) << " "
                             << eigVecs.at(i).at(1).at(0) << " " << eigVecs.at(i).at(1).at(1) << " "
                             << tC_.eGetHexabotCtrConf().oscConf[0].inhibiCoef << " "
                             << fixedRealPointPeriods.at(i) << " " << fixedPointDuties.at(i) << " " << fixedPointTouchDownPhases.at(i) << " "
                             << fixedPointMappingPeriods.at(i)
                             << std::endl;
                }
            }else{
                ofs << "# fail fail fail  sim:" << simNum << std::endl;
            }
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
            iniStateVec = conf.initialStateVec;
        }

        // cr lf for file.
        if( ofs.is_open() ){ ofs << std::endl;}

        return;
    }

    // this is called when the simulation ends
    void finish_simulation(int simNum_){
        // close the file
        if(ofs.is_open()) ofs.close();

        return;
    }

};

#endif // FINDSOLUTION_ANALYZESTABILITY_H
