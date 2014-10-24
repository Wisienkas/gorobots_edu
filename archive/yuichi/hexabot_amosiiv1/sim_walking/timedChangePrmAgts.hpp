#ifndef TIMEDCHANGEPRMAGTS_HPP 100
#define TIMEDCHANGEPRMAGTS_HPP

// These classes manage to change parameters depending on time for the simulation
//   this class just changes the parameter depending on time. They do not control whether the simulation is end or not

/*
Ver 1.00 on 20140515
THis is the first version.

*/
#include "itfTimedChangePrmAgt.hpp"
#include <sstream>

// nothing to change class
class cNoChange_tpa:public itfTimedChangePrmAgent{
public:
    // constractor
    cNoChange_tpa():itfTimedChangePrmAgent(){}

    // destructor
    ~cNoChange_tpa(){}

    // *********** overroad funcs ************************
    void ini(){
        return;
    }

    void start(double time){
        return;
    }

    void update(double time){
        return;
    }

    void end(double time){
        return;
    }

    std::string getPrmName(){
        return "NoChange";
    }

    std::string getPrmValStr(){
        std::stringstream str;
        str << 0;
        return str.str();
    }

};

// change only 1 double parameter with linear function
class cLinearFunc_1prm_tpa:public itfTimedChangePrmAgent{
private:
    double& refParam;
    double  startVal;
    double  gradient;
    std::string prmName;
    double startTime;

public:
    // constractor
    cLinearFunc_1prm_tpa(double &refPrm_, double startVal_, double gradient_, std::string name_)
        :itfTimedChangePrmAgent(), refParam(refPrm_), startVal(startVal_), gradient(gradient_), prmName(name_){
        startTime = 0.;
    }

    // destructor
    ~cLinearFunc_1prm_tpa(){}

    // *********** overroad funcs ************************
    void ini(){
        startTime = 0;
        refParam = startVal;
        return;
    }

    void start(double time){
        startTime = time;
        return;
    }

    void update(double time){
        refParam = gradient * (time - startTime) + startVal;
        return;
    }

    void end(double time){
        return;
    }

    std::string getPrmName(){
        return prmName;
    }

    std::string getPrmValStr(){
        std::stringstream str;
        str << std::setprecision(10)  << refParam;
        return str.str();
    }

};

// change 6 double parameters at the same time with linear func
class cLinearFunc_6prm_tpa:public itfTimedChangePrmAgent{
private:
    double& refParam0;
    double& refParam1;
    double& refParam2;
    double& refParam3;
    double& refParam4;
    double& refParam5;

    double  startVal;
    double  gradient;
    std::string prmName;
    double startTime;

public:
    // constractor
    cLinearFunc_6prm_tpa(double &refPrm0_,double &refPrm1_,double &refPrm2_,double &refPrm3_,double &refPrm4_,double &refPrm5_, double startVal_, double gradient_, std::string name_)
        :itfTimedChangePrmAgent(), refParam0(refPrm0_),refParam1(refPrm1_),refParam2(refPrm2_),refParam3(refPrm3_),refParam4(refPrm4_),refParam5(refPrm5_), startVal(startVal_), gradient(gradient_), prmName(name_){
        startTime = 0.;
    }

    // destructor
    ~cLinearFunc_6prm_tpa(){}

    // *********** overroad funcs ************************
    void ini(){
        startTime = 0;
        refParam0 = startVal;
        refParam1 = startVal;
        refParam2 = startVal;
        refParam3 = startVal;
        refParam4 = startVal;
        refParam5 = startVal;
        return;
    }

    void start(double time){
        startTime = time;
        return;
    }

    void update(double time){
        refParam0 = gradient * (time - startTime) + startVal;
        refParam1 = gradient * (time - startTime) + startVal;
        refParam2 = gradient * (time - startTime) + startVal;
        refParam3 = gradient * (time - startTime) + startVal;
        refParam4 = gradient * (time - startTime) + startVal;
        refParam5 = gradient * (time - startTime) + startVal;

        return;
    }

    void end(double time){
        return;
    }

    std::string getPrmName(){
        return prmName;
    }

    std::string getPrmValStr(){
        std::stringstream str;
        str << std::setprecision(10)  << refParam0;
        return str.str();
    }

};

// change duty with constant swing time
class cLinearFunc_changeDuty_keepST_tpa:public itfTimedChangePrmAgent{
public:
    // typedef
    struct paramSet{
        double& duty_0;
        double& duty_1;
        double& duty_2;
        double& duty_3;
        double& duty_4;
        double& duty_5;

        double& freq_0;
        double& freq_1;
        double& freq_2;
        double& freq_3;
        double& freq_4;
        double& freq_5;

        paramSet(double& duty_0_, double& duty_1_, double& duty_2_, double& duty_3_, double& duty_4_, double& duty_5_,
                 double& freq_0_, double& freq_1_, double& freq_2_, double& freq_3_, double& freq_4_, double& freq_5_)
            :duty_0(duty_0_), duty_1(duty_1_), duty_2(duty_2_), duty_3(duty_3_), duty_4(duty_4_), duty_5(duty_5_),
              freq_0(freq_0_),freq_1(freq_1_),freq_2(freq_2_),freq_3(freq_3_),freq_4(freq_4_),freq_5(freq_5_)
        {}
    };

private:
    paramSet cParamSet;

    double  startVal;
    double  gradient;
    double startTime;
    double swingDuration;

public:
    // constractor
    cLinearFunc_changeDuty_keepST_tpa(paramSet& prmSet_, double swingDuration_, double iniDuty_, double gradient_)
        :itfTimedChangePrmAgent(), cParamSet(prmSet_), swingDuration(swingDuration_),startVal(iniDuty_),  gradient(gradient_){

        cParamSet.duty_0 = prmSet_.duty_0;
        cParamSet.duty_1 = prmSet_.duty_1;
        cParamSet.duty_2 = prmSet_.duty_2;
        cParamSet.duty_3 = prmSet_.duty_3;
        cParamSet.duty_4 = prmSet_.duty_4;
        cParamSet.duty_5 = prmSet_.duty_5;

        cParamSet.freq_0 = prmSet_.freq_0;
        cParamSet.freq_1 = prmSet_.freq_1;
        cParamSet.freq_2 = prmSet_.freq_2;
        cParamSet.freq_3 = prmSet_.freq_3;
        cParamSet.freq_4 = prmSet_.freq_4;
        cParamSet.freq_5 = prmSet_.freq_5;

        startTime = 0.;
    }

    // destructor
    ~cLinearFunc_changeDuty_keepST_tpa(){}

    // *********** overroad funcs ************************
    void ini(){
        startTime = 0;
        double freq =  (1. - startVal) / swingDuration;

        cParamSet.duty_0 = startVal;
        cParamSet.duty_1 = startVal;
        cParamSet.duty_2 = startVal;
        cParamSet.duty_3 = startVal;
        cParamSet.duty_4 = startVal;
        cParamSet.duty_5 = startVal;

        cParamSet.freq_0 = freq;
        cParamSet.freq_1 = freq;
        cParamSet.freq_2 = freq;
        cParamSet.freq_3 = freq;
        cParamSet.freq_4 = freq;
        cParamSet.freq_5 = freq;

        return;
    }

    void start(double time){
        startTime = time;
        return;
    }

    void update(double time){
        double duty = gradient * (time - startTime) + startVal;
        double freq = (1. - duty) / swingDuration;

        cParamSet.duty_0 = duty;
        cParamSet.duty_1 = duty;
        cParamSet.duty_2 = duty;
        cParamSet.duty_3 = duty;
        cParamSet.duty_4 = duty;
        cParamSet.duty_5 = duty;

        cParamSet.freq_0 = freq;
        cParamSet.freq_1 = freq;
        cParamSet.freq_2 = freq;
        cParamSet.freq_3 = freq;
        cParamSet.freq_4 = freq;
        cParamSet.freq_5 = freq;

        return;
    }

    void end(double time){
        return;
    }

    std::string getPrmName(){
        std::stringstream str;
        str <<  "duty(ST)" << swingDuration;
        return str.str();
    }

    std::string getPrmValStr(){
        std::stringstream str;
        str << std::setprecision(10)  << cParamSet.duty_0;
        return str.str();
    }

};

// combine two change class
class cCombineTwoAgents_tpa:public itfTimedChangePrmAgent{
private:
    itfTimedChangePrmAgent* firstAgt;
    itfTimedChangePrmAgent* secondAgt;
    double changeTime;
    bool changeFlag;
    double startTime;

public:
    // constractor
    cCombineTwoAgents_tpa(itfTimedChangePrmAgent* firstAgt_, itfTimedChangePrmAgent* secondAgt_, double durationFirst)
        :itfTimedChangePrmAgent(), firstAgt(firstAgt_), secondAgt(secondAgt_), changeTime(durationFirst)
    {
        changeFlag = false;
        startTime = 0.;
    }

    // destructor
    ~cCombineTwoAgents_tpa(){}

    // *********** overroad funcs ************************
    void ini(){
        changeFlag = false;
        firstAgt->ini();
        return;
    }

    void start(double time){
        startTime = time;
        firstAgt->start(time);
        return;
    }

    void update(double time){
        if( (time - startTime) < changeTime){
            firstAgt->update(time);
        }else{
            if(!changeFlag){
                firstAgt->end(time);
                secondAgt->ini();
                secondAgt->start(time);
                changeFlag = true;
            }else{
                secondAgt->update(time);
            }
        }
        return;
    }

    void end(double time){
        secondAgt->end(time);
        return;
    }

    std::string getPrmName(){
        std::stringstream ss;
        ss << "<cmbMode 1:" << firstAgt->getPrmName() << " 2:" << secondAgt->getPrmName() << " >" ;
        return ss.str();
    }

    std::string getPrmValStr(){
        std::stringstream str;
        str << firstAgt->getPrmValStr() << " " << secondAgt->getPrmValStr();
        return str.str();
    }

};



#endif // TIMEDCHANGEPRMAGTS_HPP


