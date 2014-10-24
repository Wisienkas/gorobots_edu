#ifndef STABILITYANALYSIS_H
#define STABILITYANALYSIS_H

#include <iostream>
#include "hexabotController.h"
#include "hxPeriodAnalysis.h"
#include <math.h>
#include <osg/Matrix>
#include <array>

#define VERSION_STABANALY "StabilityAnalysis_Ver 1.10_20140422"

/*
****************************************************************
Revised History
Ver 1.11 20140610
  We change the program to add the leg num of the poincare Phase

Ver 1.10 20140422
  We change the program to use new calc Jacobian because the accuracy of the prv calc Jacobian is not so high and not so reliable

Ver 1.00 201403
  First version
   1, poincreProjection
    1 step poincre projection is calculated
   2, CalcJacobian
    Jacobian is calculated for fixed point
   3, stabAnaly
    Stability is calculated for fixed points

****************************************************************

In this file, 3 classes

  1, poincreProjection
    1 step poincre projection is calculated

  2, CalcJacobian
    Jacobian is calculated for fixed point

  3, stabAnaly
    Stability is calculated for fixed points

*/

#define CALCPOINCREMAP_DEBUG 1
#define CALCJACOBIAN_DEBUG 2
#define STABANALY_DEBUG 3

// The poincre projection calc
//  inival is changed by poincre projection
class CalcPoincreMap{
public:
    typedef struct{
        // The phase When the Poincre plane is devided (phi = ??)
        double poincrePhase;
        // The leg number of the poincare phase
        int poincarePhaseLegNum;

        // How many steps of period with phase reset will we wait to calculate the stability on Poincre Map
        int poincrePeriodNum;
        // How many steps of period without PR will we wait to execute the phase reset
        int waitPeriodNum;
    }CalcPoincreMapConf;
private:

    // configuration of this class
    CalcPoincreMapConf conf;

    // result Phase difference
    osg::Vec2d rPhase;

    // finish flag
    bool finishFlag;

    // sim num
    int simNum;


    // The function to change the status depending on the state....
    //  By using this, we control the status transition
    void changeStatus();

    //  The state transition is like "waitMode -> ExecuteMode -> CalcMode"
    //  By using this transition, we calculate the P(x,y) with one configuration
    // the mode interface class
    class itfMode{
    public:
        // this is the func which is called in every loop
        virtual bool proc(HexabotController& tC, double time);
        // we detect whethr this simulation is Finished or not
        virtual bool isFinished();
    };

    // the mode abstruct class
    class absMode:public itfMode{
    protected:
        // parent of this class
        CalcPoincreMap* pCal;
        // Flag which show that the status sequence has finished or not
        bool finishFlag;
    public:
        // constructor
        absMode(CalcPoincreMap* _pCal);//:pCal(_pCal){}
        // destructor
        virtual ~absMode();
        // define the real part of the virtual function
        bool isFinished();
    };

    // the mode to wait the stable walking to simulate without Phase resetting
    class WaitMode:public absMode{
    private:
        // initial Phase of this Poincre mapping
        osg::Vec2d iniDPhase;
        // How many periods should this system wait
        int periodTimes;
        // How much time should this simuration wait
        double waitTime;

        // internal Param to control
        enum STATUS_ {IDLE, INI, WAIT, FIN};
        enum STATUS_ STATUS;
        // start time
        double startTime;

    public:
        // constructor
        WaitMode(CalcPoincreMap* _pCal);
        // setFunction
        //  when we do this func, the simulation start to run
        bool set(osg::Vec2d iniP, int _periodTimes);
        // this is the func which is called in every loop
        bool proc(HexabotController& tC, double time);
        // This is called at the end of this state
        void terminate();
    };
    // activate the phase resetting and do simuration untill the simuration reaches Poincre Plane
    class ExeMode:public absMode{
    private:
        // how many times of period we wait to gt Poicre map with PR
        int poincrePeriodNum;
        // the phase which we deviede the poicre plane on
        double poincrePhase;
        // The leg number of the poincare phase
        int poincarePhaseLegNum;

        // The phase which is gotten as a result on poincre plane
        osg::Vec2d rPhase;
        // how many simulation steps are in one period of oscillation
        int periodSteps;
        // the time length of 1 period oscillation
        double oscPeriod;

        // internal param to control
        enum STATUS_ {IDLE, INI, WAIT, EXE, FIN};
        enum STATUS_ STATUS;
        // count of period times
        int periodCnt;
        // time when we detect the system pass through the poincre plane
        double pTime;
        // poincre pass Flag
        bool poincreFlag;

    public:
        // constructor
        ExeMode(CalcPoincreMap* _pCal);
        // set Function
        //  when we do this func, the simuation will starts
        bool set(int _poincrePeriodNum, double _poincrePhase, int _poincarePhaseLegNum);
        // this is the func which is called in every loop
        bool proc(HexabotController &tC, double time);
        // THis is called when you finish the smulation
        bool getP(osg::Vec2d &_P);
        // This is called at the end of this state
        void terminate();
    };

    // abst class for main loop
    absMode* aMode;
    // state classes
    WaitMode waitSta;
    ExeMode exeSta;

public:
    // constrctor
    CalcPoincreMap();
    // THis is called every step to calculate
    bool proc(HexabotController& tC, double time);
    // this is called to check whether the simuration is finished or not
    bool isFinished();
    // This is called to get (P1(x,y), P2(x,y))
    bool getP(osg::Vec2d& _P);
    // This is called to set the (x, y)
    //bool set(double _x, double _y);
    // set the (x_ini, y_ini) and configs  and start the calcu;lation
    bool set(osg::Vec2d x_ini_, int simNum_, CalcPoincreMapConf conf_);
    // This should be called before we set the parameters
   // bool setConf(CalcPoincreMapConf _conf);
};

// The sequences of poincare projection are calculated
//   till it reaches near of the fixed point
class CalcPoincreMaps{
public:
    typedef struct{
        // The phase When the Poincre plane is devided (phi = ??)
        double poincrePhase;
        // The leg number of the poincare phase
        int poincarePhaseLegNum;

        // How many steps of period without PR will we wait to execute the phase reset
        int waitPeriodNum;
        // max poincare steps to calculate
        int maxPeriodNum;
    }CalcPoincreMapsConf;

    typedef std::vector<osg::Vec2d> vec2dVector;
private:
    // configuration of this class
    CalcPoincreMapsConf conf;

    // fixed point
    osg::Vec2d fixedPhase;
    // delta (calculate till the difference from fixed point becomes less than delta)
    double delta;

    // result Phase difference
    vec2dVector rPhaseVec;

    // finish flag
    bool finishFlag;

    // sim num
    int simNum;


    // The function to change the status depending on the state....
    //  By using this, we control the status transition
    void changeStatus();

    //  The state transition is like "waitMode -> ExecuteMode -> CalcMode"
    //  By using this transition, we calculate the P(x,y) with one configuration
    // the mode interface class
    class itfMode{
    public:
        // this is the func which is called in every loop
        virtual bool proc(HexabotController& tC, double time);
        // we detect whethr this simulation is Finished or not
        virtual bool isFinished();
    };

    // the mode abstruct class
    class absMode:public itfMode{
    protected:
        // parent of this class
        CalcPoincreMaps* pCal;
        // Flag which show that the status sequence has finished or not
        bool finishFlag;
    public:
        // constructor
        absMode(CalcPoincreMaps* _pCal);//:pCal(_pCal){}
        // destructor
        virtual ~absMode();
        // define the real part of the virtual function
        bool isFinished();
    };

    // the mode to wait the stable walking to simulate without Phase resetting
    class WaitMode:public absMode{
    private:
        // initial Phase of this Poincre mapping
        osg::Vec2d iniDPhase;
        // How many periods should this system wait
        int periodTimes;
        // How much time should this simuration wait
        double waitTime;

        // internal Param to control
        enum STATUS_ {IDLE, INI, WAIT, FIN};
        enum STATUS_ STATUS;
        // start time
        double startTime;

    public:
        // constructor
        WaitMode(CalcPoincreMaps* _pCal);
        // setFunction
        //  when we do this func, the simulation start to run
        bool set(osg::Vec2d iniP, int _periodTimes);
        // this is the func which is called in every loop
        bool proc(HexabotController& tC, double time);
        // This is called at the end of this state
        void terminate();
    };
    // activate the phase resetting and do simuration untill the simuration reaches Poincre Plane
    class ExeMode:public absMode{
    private:
        // how many times of period we wait to gt Poicre map with PR
        int maxPoincrePeriodNum;
        // the phase which we deviede the poicre plane on
        double poincrePhase;
        // The leg number of the poincare phase
        int poincarePhaseLegNum;

        // The phase which is gotten as a result on poincre plane
        vec2dVector rPhaseVec;
        // fixed point
        osg::Vec2d fixedPhase;
        // delta as converge
        double delta;


        // how many simulation steps are in one period of oscillation
        int periodSteps;
        // the time length of 1 period oscillation
        double oscPeriod;

        // internal param to control
        enum STATUS_ {IDLE, INI, WAIT, EXE, FIN};
        enum STATUS_ STATUS;
        // count of period times
        int periodCnt;
        // time when we detect the system pass through the poincre plane
        double pTime;
        // poincre pass Flag
        bool poincreFlag;

    public:
        // constructor
        ExeMode(CalcPoincreMaps* _pCal);
        // set Function
        //  when we do this func, the simuation will starts
        bool set(osg::Vec2d fixedPoint_, double delta_, int maxPeriodNum_, double _poincrePhase, int _poincarePhaseLegNum);
        // this is the func which is called in every loop
        bool proc(HexabotController &tC, double time);
        // THis is called when you finish the smulation
        bool getPs(vec2dVector &phaseVec_);
        // This is called at the end of this state
        void terminate();
    };

    // abst class for main loop
    absMode* aMode;
    // state classes
    WaitMode waitSta;
    ExeMode exeSta;

public:
    // constrctor
    CalcPoincreMaps();
    // THis is called every step to calculate
    bool proc(HexabotController& tC, double time);
    // this is called to check whether the simuration is finished or not
    bool isFinished();
    // This is called to get (P1(x,y), P2(x,y))
    bool getPs(vec2dVector& _P_vec);
    // This is called to set the (x, y)
    //bool set(double _x, double _y);
    // set the (x_ini, y_ini) and configs  and start the calcu;lation
    bool set(osg::Vec2d x_ini_, osg::Vec2d x_fixed_, double delta_, int simNum_, CalcPoincreMapsConf conf_);
    // This should be called before we set the parameters
   // bool setConf(CalcPoincreMapConf _conf);
};


// The classes to calculate the dP_dx
 // itf class
class itfCalcJacobian{
public:
    typedef struct{
        // The phase When the Poincre plane is devided (phi = ??)
        double poincrePhase;
        // The leg number of the poincare phase
        int poincarePhaseLegNum;

        // How many steps of period with phase reset will we wait to calculate the stability on Poincre Map
        int poincrePeriodNum;
        // How many steps of period without PR will we wait to execute the phase reset
        int waitPeriodNum;
        // How many max steps of period with phase reset will we wait to calculate the stability on Poincre Map
        int maxPoincrePeriodNum;

        // calc diff stepp
        double delta;
    }CalcJacobianConf;
    typedef std::vector<osg::Vec2d> vec2dVector;
    typedef std::array<std::array<double, 2>,2> Matrix2_2;
    typedef std::vector<Matrix2_2> Matrix2_2Vector;

public:
    // constructor
    itfCalcJacobian(){}
    virtual ~itfCalcJacobian(){}

    // this is called to set the param
    //bool setConf(StabAnalyConf _conf);
    // this is called to start the simulation to calc
    virtual bool set(osg::Vec2d _fixedPhase, int simNum_, const CalcJacobianConf& conf_)=0;
    // this is called every step to calculate
    virtual bool proc(HexabotController& tC, double time)=0;
    // this is called to check the calculation is Finished or not
    virtual bool isFinished()=0;
    // This is called to get the Matrix
    virtual bool getDP(Matrix2_2& _dP_dx)=0;
};

 // The class to calcurate the dP_dx (this is the normal version )
class CalcJacobian_by_perturbation:public itfCalcJacobian{
private:
    // fixed point
    osg::Vec2d fixedPhase;
    // delta
    double delta;

    // parameters of this Poincre Map
    CalcPoincreMap::CalcPoincreMapConf pConf;

    // the class CalcPoincreMap
    CalcPoincreMap pCalc;

    // The phase dffs after Poincre map
    typedef struct{
        osg::Vec2d Phase_dx; // P (x + dx/2, y)
        osg::Vec2d Phase_dx_;// P (x - dx/2, y)
        osg::Vec2d Phase_dy;
        osg::Vec2d Phase_dy_;
    }pMapVal;
    // poincre map param
    pMapVal pmVal;
    // the dP matrix
    Matrix2_2 dP_dx;

    // finish Flag
    bool finishFlag;

    // the STATUS
    enum STATUS_ {IDLE, INI, DX, DX_, DY, DY_, FIN};
    enum STATUS_ STATUS;

    int simNum;

    // THis function changes the class depending on the situuation
    void changeStatus();

    // This function is called to calculate the Matrix dP/dx from pMapVal
    Matrix2_2 calc_dP_dx(const pMapVal& _pmVal, double _delta);

public:
    // constructor
    CalcJacobian_by_perturbation();
    // this is called to set the param
    //bool setConf(StabAnalyConf _conf);
    // this is called to start the simulation to calc
    bool set(osg::Vec2d _fixedPhase, int simNum_, const CalcJacobianConf& conf_);

    // this is called every step to calculate
    bool proc(HexabotController& tC, double time);
    // this is called to check the calculation is Finished or not
    bool isFinished();
    // This is called to get the Matrix
    bool getDP(Matrix2_2& _dP_dx);
};

 // The class to calculate the dP_dx (this is the improved version)
 //  calculate jacobian in the flow of the convergence
class CalcJacobian_by_flow:public itfCalcJacobian{
private:
    typedef std::vector<osg::Vec2d> vec2dVector;

    // fixed point
    osg::Vec2d fixedPhase;
    // accuracy
    double accuracy;
    // delta for initial value
    double delta_ini;

    // parameters of this Poincre Map
    CalcPoincreMaps::CalcPoincreMapsConf psConf;
    // the class CalcPoincreMap
    CalcPoincreMaps psCalc;

    // poincare maps phase moving
    vec2dVector d1_phaseVec;
    vec2dVector d2_phaseVec;

    // The phase dffs after Poincre map
    typedef struct{
        osg::Vec2d dPhase_bfr_1; // (dx_before, dy_before)
        osg::Vec2d dPhase_aft_1; // (dx_after, dy_after)
        osg::Vec2d dPhase_bfr_2;
        osg::Vec2d dPhase_aft_2;
    }pMapChangeVal;
    // poincre map param
    pMapChangeVal pmVal;

    // the dP matrix
    Matrix2_2 dP_dx;

    // finish Flag
    bool finishFlag;

    // the STATUS
    enum STATUS_ {IDLE, INI, D1, D2, FIN};
    enum STATUS_ STATUS;

    int simNum;

    // THis function changes the class depending on the situuation
    void changeStatus();

    // This function is called to calculate the Matrix dP/dx from pMapChangeVal
    Matrix2_2 calc_dP_dx(const pMapChangeVal& _pmVal);

public:
    // constructor
    CalcJacobian_by_flow();
    // this is called to set the param
    //bool setConf(StabAnalyConf _conf);
    // this is called to start the simulation to calc
    bool set(osg::Vec2d _fixedPhase, int simNum_, const CalcJacobianConf& conf_);

    // this is called every step to calculate
    bool proc(HexabotController& tC, double time);
    // this is called to check the calculation is Finished or not
    bool isFinished();
    // This is called to get the Matrix
    bool getDP(Matrix2_2& _dP_dx);

    // set delta ini
    void set_delta_ini(double val_);
};



// This is the class to do the stability analysis
//   I apply the status design pattern to distribute the responsible of the code
//   3 module version
class StabAnaly{
public:
    typedef struct{
        // The phase When the Poincre plane is devided (phi = ??)
        double poincrePhase;
        // The leg number of the poincare phase
        int poincarePhaseLegNum;

        // How many steps will we wait for calc poincre map without PR
        int waitPeriodNum;
        // How many steps of period will we wait to calculate the stability on Poincre Map
        int poincrePeriodNum;
        // How many max steps of period will we wait to calculate the stability on Poincre Map
        int maxPoincrePeriodNum;

        // Delta step which we used to calculate the diiferential
        double delta;

        // How to calculate Jacobian
        bool is_use_calcJacobian_by_flow;
        // initial value diff for calc J from flow
        double initialPhaseDiff;

        // The point where we analyse the stability
        //osg::Vec2d fixedPhase;
    }StabAnalyConf;

    // typedef
    typedef std::vector<osg::Vec2d> vec2dVector;
    typedef std::array<std::array<double, 2>,2> Matrix2_2;
    typedef std::vector<Matrix2_2> Matrix2_2Vector;

    // set default conf func
    static StabAnalyConf getDefaultStabAnalyConf(void){
        StabAnalyConf sConf;
        // delta phase to calculate defferential
        sConf.delta = 0.002; //
        // The phase which we use poincre map
        sConf.poincrePhase = 0;//39./ 20. *M_PI;
        sConf.poincarePhaseLegNum = 0;
        // poincre oeriod num
        sConf.poincrePeriodNum = 1;//1 change
        // waitPeriod NUm
        sConf.waitPeriodNum = 2;
        sConf.maxPoincrePeriodNum = 20;

        sConf.is_use_calcJacobian_by_flow = false;
        sConf.initialPhaseDiff = 0.1;

        return sConf;
    }

private:
    // Configuration params
    StabAnalyConf conf;
    // fixedPhase
    vec2dVector fPhaseVec;
    // absEigen value set
    vec2dVector absEigValVec;
    // Eigen value set
    Matrix2_2Vector eigValVec;
    // Eigen Vec set
    Matrix2_2Vector eigVecVec;

    // Matric dPdx
    Matrix2_2Vector dPdxVec;


    // Differentiation of Poincre map
    Matrix2_2  dP_dx;
    Matrix2_2  eigVal;
    Matrix2_2  eigVec;
    // Eigen Value of the Differentiation of Poincre Map
    std::array<double, 2> eVal;

    // data NUm
    int dataNum;
    // count
    int count;

    // finish Flag
    bool finishFlag;
    // status value
    enum STATUS_ {IDLE, CALC, FIN};
    enum STATUS_ STATUS;

    // sim Num
    int simNum;

private:
    // The dP_class
    itfCalcJacobian* itfCalcJ;
    CalcJacobian_by_perturbation calcJ_pertb;
    CalcJacobian_by_flow calcJ_flow;

private:
    // this is the method to use in private situation

    // Calc Eigen value of 2 by 2 matrix
    //  The result is contained as follows
    //    { real Value_1, im_value_1 }
    //    { real Value_2, im_value_2 }
    Matrix2_2 calcEigVal2_2(Matrix2_2 matrix);

    // Calc Eigen Vector of 2 by 2 matrix
    //  The result is contained as follows
    //    { eigvec_1x, eigVec_1y }
    //    { eigvec_2x, eigVec_2y }
    //
    //  In the case that the eig Val is imaginal, this return as follows
    //   EigVec = eigvec_1 +- i * eigvec_2
    //
    Matrix2_2 calcEigVec2_2(Matrix2_2 matrix);

    // change status
    void changeStatus();

public:
    // this is the interface to serve

    //default Const
    StabAnaly();

    // starts the simulation
    bool set(vec2dVector _fixedPhase, int simNum_, const StabAnalyConf& conf_);

    // CalcFunc ; it is called every time step
    bool calcStep(HexabotController& tC, double time);

    // isFinished calc??
    bool isFinished(void){return finishFlag;}

    // get the result!! absollute value of eigen value
    // bigger one is former one
    bool getResult(vec2dVector&  _eigAbsValVec, Matrix2_2Vector& _eigValVec, Matrix2_2Vector& _eigVecVec, Matrix2_2Vector& _dPdxVec);
};

#endif
