#ifndef REPEATMODULATORS_HPP
#define REPEATMODULATORS_HPP

/*
 In this class, we design the general repest modulation

*/
#include <iostream>
#include <vector>
#include <list>
#include <osg/Matrix>
#include "itfRepeatModulator.h"

//! Version Control
#define VERSION_REPEATMODULATORS "RepModulators_Ver 1.04_20140701"
//
//
// 20140701 Ver 1.04
//   add class to calc phase plane
//
// 20140527 Ver 1.03
//   add class to give a deviation
//
//
// 20140423 Ver 1.02
//   add class for change two params
//
// 20140402 Ver 1.01
//   fixed the bugs , パラメタを減らす時に対応できていなかったバグの修正
//
//

// class to change nothing (this means one proc simulator)
class cNoParamRepMod:public itfRepeatModulator{
private:
    int repeatTimes;
    int count;
public:
    //constructer
    cNoParamRepMod(int repeatTimes_):itfRepeatModulator(), repeatTimes(repeatTimes_), count(0)
    {}

    //destructor
    ~cNoParamRepMod(){}

    // change configs
    void changeConfigs(int simNum_){
        // change parameters
        ++count;
    }

    // isFinshed
    bool isFinished(){
        if(count >= repeatTimes) return true;
        else return false;
    }

};


//class to change the 1 parameter
//    T_type should be able to add and compare
template<class T_type>
class cOneParamRepMod:public itfRepeatModulator{
private:
    // reference for the parametervalue to be changed
    T_type& refParam;
    T_type iniValue;
    T_type finValue;
    T_type step;

public:
    //constructer
    cOneParamRepMod(T_type& ref_parameterVal, T_type ini_, T_type fin_, T_type step_)
        :itfRepeatModulator(), refParam(ref_parameterVal)
    {
        refParam = ref_parameterVal;
        iniValue = ini_;
        finValue = fin_;
        step = step_;
    }

    //destructor
    ~cOneParamRepMod(){}

    // change configs
    void changeConfigs(int simNum_){
        // change parameters
        T_type value = iniValue;
        for(int i=1; i< simNum_; ++i){
            value = value + step;
        }
         // update params
        refParam = value;
    }

    // isFinshed
    bool isFinished(){
        if(step > 0){
            if(refParam >= finValue) return true;
        }else{
            if(refParam <= finValue) return true;
        }
        return false;
    }

};


// class to change several parameters
//  this class just execute several itfRepeatModulators
class cMultiRepMod:public itfRepeatModulator{
private:
    std::list<itfRepeatModulator*> RepModList;
public:
    // constructor
    cMultiRepMod():itfRepeatModulator()
    {
        RepModList.clear();
    }

    // destructor
    ~cMultiRepMod(){
        RepModList.clear();
    }

    // regist function
    bool regist(itfRepeatModulator* repMod_){
        if(repMod_==0) return true;
        RepModList.push_back(repMod_);
        return false;
    }

    // change configs
    void changeConfigs(int simNum_){
        for(auto itr = RepModList.begin(); itr != RepModList.end(); itr++){
            (*itr)->changeConfigs(simNum_);
        }
    }

    // is Finished
    bool isFinished(){
        if(RepModList.empty()) return true;

        // calc
        bool flag = false;
        for(auto itr = RepModList.begin(); itr != RepModList.end(); itr++){
            flag = (flag || (*itr)->isFinished());
        }
        return flag;
    }
};


// class to change array
//  this is the class to change the array value
template<class T_type>
class cOneArray6ParamRepMod:public itfRepeatModulator{
private:
    // reference for the parametervalue to be changed
    T_type& refParam0;
    T_type& refParam1;
    T_type& refParam2;
    T_type& refParam3;
    T_type& refParam4;
    T_type& refParam5;

    T_type iniValue;
    T_type finValue;
    T_type step;

public:
    //constructer
    cOneArray6ParamRepMod(T_type& ref_parameterVal0, T_type& ref_parameterVal1,T_type& ref_parameterVal2, T_type& ref_parameterVal3,T_type& ref_parameterVal4, T_type& ref_parameterVal5, T_type ini_, T_type fin_, T_type step_)
        :itfRepeatModulator(), refParam0(ref_parameterVal0), refParam1(ref_parameterVal1), refParam2(ref_parameterVal2), refParam3(ref_parameterVal3), refParam4(ref_parameterVal4), refParam5(ref_parameterVal5)
    {
        iniValue = ini_;
        finValue = fin_;
        step = step_;
    }

    //destructor
    ~cOneArray6ParamRepMod(){}

    // change configs
    void changeConfigs(int simNum_){
        // change parameters
        T_type value = iniValue;
        for(int i=1; i< simNum_; ++i){
            value = value + step;
        }

        // change array
        refParam0 = value;
        refParam1 = value;
        refParam2 = value;
        refParam3 = value;
        refParam4 = value;
        refParam5 = value;
    }

    // isFinshed
    bool isFinished(){
        if(step > 0){
            if(refParam0 >= finValue) return true;
        }else{
            if(refParam0 <= finValue) return true;
        }
        return false;
    }

};

// class to change duty rate keeping the duration of swing phase
class cChangeDuty_KeepSwingDuration_RepMod:public itfRepeatModulator{
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
    // reference for the parametervalue to be changed
    paramSet changingParamSet;
    double swingDuration;//(s)
    double ini_duty;
    double fin_duty;
    double step_duty;

public:
    //constructer
    cChangeDuty_KeepSwingDuration_RepMod(paramSet& paramSet_, double swingDuration_, double ini_duty_, double fin_duty_, double step_duty_)
        :itfRepeatModulator(), swingDuration(swingDuration_), ini_duty(ini_duty_), fin_duty(fin_duty_), step_duty(step_duty_), changingParamSet(paramSet_)
    {
        // update values
        changingParamSet.duty_0 = paramSet_.duty_0;
        changingParamSet.duty_1 = paramSet_.duty_1;
        changingParamSet.duty_2 = paramSet_.duty_2;
        changingParamSet.duty_3 = paramSet_.duty_3;
        changingParamSet.duty_4 = paramSet_.duty_4;
        changingParamSet.duty_5 = paramSet_.duty_5;

        changingParamSet.freq_0 = paramSet_.freq_0;
        changingParamSet.freq_1 = paramSet_.freq_1;
        changingParamSet.freq_2 = paramSet_.freq_2;
        changingParamSet.freq_3 = paramSet_.freq_3;
        changingParamSet.freq_4 = paramSet_.freq_4;
        changingParamSet.freq_5 = paramSet_.freq_5;
    }

    //destructor
    ~cChangeDuty_KeepSwingDuration_RepMod(){}

    // change configs
    void changeConfigs(int simNum_){
        // change parameters
        double duty = ini_duty;
        duty = duty + step_duty * static_cast<double>((simNum_ - 1));

        double freq = 1. / ( swingDuration / (1. - duty) );

        // change duty and freq
        changingParamSet.duty_0 = duty;
        changingParamSet.duty_1 = duty;
        changingParamSet.duty_2 = duty;
        changingParamSet.duty_3 = duty;
        changingParamSet.duty_4 = duty;
        changingParamSet.duty_5 = duty;

        changingParamSet.freq_0 = freq;
        changingParamSet.freq_1 = freq;
        changingParamSet.freq_2 = freq;
        changingParamSet.freq_3 = freq;
        changingParamSet.freq_4 = freq;
        changingParamSet.freq_5 = freq;
    }

    // isFinshed
    bool isFinished(){
        if(step_duty > 0){
            if(changingParamSet.duty_0  >= fin_duty) return true;
        }else{
            if(changingParamSet.duty_0  <= fin_duty) return true;
        }
        return false;
    }
};


// class to chage a initial vector to search basin
//  search the square from left bottom to right Top by step
class cChangeIniPhase_RepMod:public itfRepeatModulator{
private:
    // reference for the parametervalue to be changed
    osg::Vec2d& refParam;
    osg::Vec2d leftBottom;
    osg::Vec2d rightTop;
    osg::Vec2d step;

public:
    //constructer
    cChangeIniPhase_RepMod(osg::Vec2d& ref_iniPhase_, osg::Vec2d leftBottom_, osg::Vec2d rightTop_, osg::Vec2d step_)
        :itfRepeatModulator(), refParam(ref_iniPhase_), leftBottom(leftBottom_), rightTop(rightTop_), step(step_)
    {}

    //destructor
    ~cChangeIniPhase_RepMod(){}

    // change configs
    void changeConfigs(int simNum_){
        // change parameters
        double v_x = leftBottom.x();
        double v_y = leftBottom.y();

        for(int i=1; i< simNum_; ++i){
            v_x += step.x();
            if(v_x > rightTop.x()){
                v_x = leftBottom.x();
                v_y +=  step.y();
            }
        }
         // update params
        refParam.set(v_x, v_y);
    }

    // isFinshed
    bool isFinished(){
        if(refParam.y() >= rightTop.y()) return true;
        return false;
    }

};

// class to chage a initial vector to analyze stability (convergence)
//  search (x + dx, y) -> (x - dx, y) -> (x, y + dy) -> (x, y - dy) -> (x + dx, y + dy) -> (x - dx, y - dy)
class cChangeIniPhase_pmDelta_RepMod:public itfRepeatModulator{
private:
    // reference for the parametervalue to be changed
    osg::Vec2d& refParam;
    osg::Vec2d fixedPoint;
    double delta_x;
    double delta_y;
    bool finishFlag;

public:
    //constructer
    cChangeIniPhase_pmDelta_RepMod(osg::Vec2d& ref_iniPhase_, osg::Vec2d fixedPoint_, double delta_x_, double delta_y_)
        :itfRepeatModulator(), refParam(ref_iniPhase_), fixedPoint(fixedPoint_), delta_x(delta_x_), delta_y(delta_y_), finishFlag(false)
    {}

    //destructor
    ~cChangeIniPhase_pmDelta_RepMod(){}

    // change configs
    void changeConfigs(int simNum_){
        // change parameters
        double x = fixedPoint.x();
        double y = fixedPoint.y();

        if(simNum_ == 1){
            x += delta_x;
        }else if (simNum_ == 2){
            x -= delta_x;
        }else if (simNum_ == 3){
            y += delta_y;
        }else if (simNum_ == 4){
            y -= delta_y;
        }else if (simNum_ == 5){
            x += delta_x;
            y += delta_y;
        }else if (simNum_ == 6){
            x -= delta_x;
            y -= delta_y;
            finishFlag = true;
        }

         // update params
        refParam.set(x, y);
    }

    // isFinshed
    bool isFinished(){
        if(finishFlag) return true;
        return false;
    }

};


// class to chage a initial vector to analyze stability (convergence)
//  search (x + dx, y) -> (x - dx, y) -> (x, y + dy) -> (x, y - dy) -> (x + dx, y + dy) -> (x - dx, y - dy)
class cChangeIniPhase_circleDelta_RepMod:public itfRepeatModulator{
private:
    // reference for the parametervalue to be changed
    osg::Vec2d& refParam;
    osg::Vec2d fixedPoint;
    double delRadius;
    double delAngle;
    bool finishFlag;

public:
    //constructer
    cChangeIniPhase_circleDelta_RepMod(osg::Vec2d& ref_iniPhase_, osg::Vec2d fixedPoint_, double deltaRadius_, double deltaAngle_)
        :itfRepeatModulator(), refParam(ref_iniPhase_), fixedPoint(fixedPoint_), delRadius(deltaRadius_), delAngle(deltaAngle_), finishFlag(false)
    {}

    //destructor
    ~cChangeIniPhase_circleDelta_RepMod(){}

    // change configs
    void changeConfigs(int simNum_){
        // change parameters
        double x = fixedPoint.x();
        double y = fixedPoint.y();

        double angle = delAngle * (simNum_ - 1.);
        x += delRadius * cos(angle);
        y += delRadius * sin(angle);

        if(delAngle * (simNum_) > 2. * M_PI){
            finishFlag = true;
        }

         // update params
        refParam.set(x, y);
    }

    // isFinshed
    bool isFinished(){
        if(finishFlag) return true;
        return false;
    }

};

#endif // REPEATMODULATORS_HPP





