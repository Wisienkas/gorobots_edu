//
// stability analysis class CPP file
//

#include "stabilityAnalysis.h"
#include <osg/Matrix>
#include <array>


// StabAnaly ##########################################################
// constructor
StabAnaly::StabAnaly():calcJ_flow(), calcJ_pertb(), finishFlag(false), STATUS(IDLE), itfCalcJ(0)
{}

// starts simulation;
bool StabAnaly::set(vec2dVector _fixedPhase, int simNum_, const StabAnalyConf& conf_){
     conf = conf_;
     simNum = simNum_;
     // initialize th values
     for(int i = 0;i<2;i++){
         eVal.at(i) = 0.;
         for(int j = 0;j < 2;j++){
             dP_dx.at(i).at(j) = 0.;
         }
     }
     // fixed Phase add
     fPhaseVec = _fixedPhase;

     // set the simulation configulation
     itfCalcJacobian::CalcJacobianConf jconf;
     jconf.delta = conf.delta;
     jconf.poincrePeriodNum = conf.poincrePeriodNum;
     jconf.poincarePhaseLegNum = conf.poincarePhaseLegNum;
     jconf.poincrePhase = conf.poincrePhase;
     jconf.waitPeriodNum = conf.waitPeriodNum;
     jconf.maxPoincrePeriodNum = conf.maxPoincrePeriodNum;

     // Number of data
     dataNum = fPhaseVec.size();
     if(dataNum == 0){
         std::cout << " There are no data!!!! Check it" << std::endl;
         return false;
     }

     // reset the count
     count = 0;
     // reset the resulted data
     absEigValVec.clear();
     eigValVec.clear();
     dPdxVec.clear();
     eigVecVec.clear();

     // select the calcJacobian
     if(conf.is_use_calcJacobian_by_flow){
         itfCalcJ = &calcJ_flow;
         calcJ_flow.set_delta_ini(conf.initialPhaseDiff);
     }else{
         itfCalcJ = &calcJ_pertb;
     }

     // start the simulation
     itfCalcJ->set(fPhaseVec.at(count), simNum, jconf);

     // change the status and flag
     finishFlag = false;
     STATUS = CALC;

     return true;
}

// calc Func
bool StabAnaly::calcStep(HexabotController& tC, double time){
    // check the finish flag
    if(finishFlag){
        return false;
    }else{
        // proc the inner class
        itfCalcJ->proc(tC, time);

        // if the dP calc has finished
        if(itfCalcJ->isFinished()) changeStatus();

        return true;
    }
}

// get result
bool StabAnaly::getResult(vec2dVector&  _eigAbsValVec, Matrix2_2Vector& _eigValVec, Matrix2_2Vector& _eigVecVec, Matrix2_2Vector& _dPdxVec){
    if(finishFlag){
        _eigAbsValVec = absEigValVec;
        _eigValVec = eigValVec;
        _dPdxVec = dPdxVec;
        _eigVecVec = eigVecVec;

        // terminate this calc step
        finishFlag = false;
        STATUS = IDLE;

        return true;
    }else{
        return false;
    }
}

// change status
void StabAnaly::changeStatus(){
    //
    if(STATUS == CALC){
        // finish the calculation
        // get Data
        itfCalcJ->getDP(dP_dx);
        // calculate the eigen value
        eigVal = calcEigVal2_2(dP_dx);
        eigVec = calcEigVec2_2(dP_dx);

        // calculate abs value of eigen value
        osg::Vec2d absEigVal;
        double eig0 =  pow(eigVal.at(0).at(0), 2) + pow(eigVal.at(0).at(1), 2);
        eig0 = sqrt(eig0);
        double eig1 =  pow(eigVal.at(1).at(0), 2) + pow(eigVal.at(1).at(1), 2);
        eig1 = sqrt(eig1);

        // substitute
        if(eig0 > eig1){
            absEigVal.set(eig0, eig1);
        }else{
            absEigVal.set(eig1, eig0);
        }

        // add to the result
        absEigValVec.push_back(absEigVal);
        eigValVec.push_back(eigVal);
        eigVecVec.push_back(eigVec);
        dPdxVec.push_back(dP_dx);

        count++;

        // check there are data or not to calculate
        if(count < dataNum){
            // repeat the simulation
            // set the simulation configulation
            itfCalcJacobian::CalcJacobianConf jconf;
            jconf.delta = conf.delta;
            jconf.poincrePeriodNum = conf.poincrePeriodNum;
            jconf.poincrePhase = conf.poincrePhase;
            jconf.waitPeriodNum = conf.waitPeriodNum;
            jconf.maxPoincrePeriodNum = conf.maxPoincrePeriodNum;
            itfCalcJ->set(fPhaseVec.at(count), simNum, jconf);

            // cout
            std::cout <<">> sim" << simNum << "  " << count << "/"<< dataNum << " stepFinished >>  x"  << std::endl;

            // for debug
            #ifdef STABANALY_DEBUG
                std::cout << "@@ DEBUG @@ StabAnaly:: Restart the simulation, count:" << count <<  std::endl;
            #endif

        }else{
            // finish simulation
            STATUS = FIN;
            finishFlag = true;

            // for debug
            #ifdef STABANALY_DEBUG
                std::cout << "@@ DEBUG @@ StabAnaly:: Finish the simulation, count:" << count <<  std::endl;
            #endif
        }
    }else{
        std::cout << " status error occurs on StabAnaly " << std::endl;
    }
}

// calc EigVec
StabAnaly::Matrix2_2 StabAnaly::calcEigVec2_2(Matrix2_2 matrix){
    Matrix2_2 eigenVal = calcEigVal2_2(matrix);
    Matrix2_2 result;

    double e1x,e1y,e2x,e2y;
    // set
    double a = matrix.at(0).at(0);
    double b = matrix.at(0).at(1);
    double c = matrix.at(1).at(0);
    double d = matrix.at(1).at(1);

    // First Calculate the inner value of the sqrt
    double sqr = a*a + d*d - 2.*a*d + 4*b*c;

    // EigVal : real
    if(sqr > 0.){
        if(fabs(a - eigenVal.at(0).at(0)) > 0.001){
            e1y = 1.;
            e1x = (-b/(a-eigenVal.at(0).at(0))) * e1y;
        }else{
            e1y = 0.;
            e1x = 1.;
        }

        if(fabs(d- eigenVal.at(1).at(0)) > 0.001){
            e2x = 1.;
            e2y = (c /(eigenVal.at(1).at(0) - d)) * e2x;
        }else{
            e2y = 1.;
            e2x = 0.;
        }

        double mag = sqrt(e1x*e1x + e1y*e1y);
        result.at(0).at(0) = e1x / mag;
        result.at(0).at(1) = e1y / mag;

        mag = sqrt(e2x*e2x + e2y*e2y);
        result.at(1).at(0) = e2x / mag;
        result.at(1).at(1) = e2y / mag;
    }
    else{
        // do it later
        result.at(0).at(0) = 0.;
        result.at(0).at(1) = 0.;
        result.at(1).at(0) = 0.;
        result.at(1).at(1) = 0.;
    }

    return result;
}

// clac Eig val
StabAnaly::Matrix2_2 StabAnaly::calcEigVal2_2(Matrix2_2 matrix){
    Matrix2_2 result;
    // set
    double a = matrix.at(0).at(0);
    double b = matrix.at(0).at(1);
    double c = matrix.at(1).at(0);
    double d = matrix.at(1).at(1);

    // First Calculate the inner value of the sqrt
    double sqr = a*a + d*d - 2.*a*d + 4*b*c;

    // solution is Real number
    if(sqr >= 0){
        result.at(0).at(0) = 1/2. * (a + d - sqrt(sqr));
        result.at(0).at(1) = 0.;
        result.at(1).at(0) = 1/2. * (a + d + sqrt(sqr));
        result.at(1).at(1) = 0.;
    }
    // solution is Imaginary number
    else{
        result.at(0).at(0) = 1/2. * (a + d );
        result.at(0).at(1) = - 1/2. * sqrt(-sqr);
        result.at(1).at(0) = 1/2. * (a + d );
        result.at(1).at(1) = 1/2. * sqrt(-sqr);
    }

    return result;
}


// calcJacobian_by_perturbation ##############################################################
// this is the constructor
CalcJacobian_by_perturbation::CalcJacobian_by_perturbation()
    :itfCalcJacobian(), pCalc(), finishFlag(false), STATUS(IDLE)
{
}

// set function, with this function, we starts the simulation
bool CalcJacobian_by_perturbation::set(osg::Vec2d _fixedPhase, int simNum_, const CalcJacobianConf& conf_){
    // extract the parameters from conf
    delta = conf_.delta;

    // set the pConf
    pConf.poincrePeriodNum = conf_.poincrePeriodNum;
    pConf.poincrePhase = conf_.poincrePhase;
    pConf.poincarePhaseLegNum = conf_.poincarePhaseLegNum;
    pConf.waitPeriodNum = conf_.waitPeriodNum;
    simNum = simNum_;

    // change the status
    STATUS = INI;
    // change the flag
    finishFlag = false;

    // extract the parameters from conf
    fixedPhase = _fixedPhase;

    // start the simulation
    //  calculate P(x + 1/2.dx, y)
    double x = fixedPhase.x() + 1./2. * delta;
    HexabotController::checkPhase(x);
    double y = fixedPhase.y();
    HexabotController::checkPhase(y);

    // start the poincre Map
    pCalc.set(osg::Vec2d(x, y), simNum, pConf);

    STATUS = DX;
    return true;
}

// Proc function, it is called in every loop
bool CalcJacobian_by_perturbation::proc(HexabotController& tC, double time){
    // check the finishFlag
    if(!finishFlag){
        // Proc the inner class
        pCalc.proc(tC, time);

        // check the status change situation
        if(pCalc.isFinished()){changeStatus();}

        // finish
        return true;

    }else{
        return false;
    }
}

// change status function (it is private)
void CalcJacobian_by_perturbation::changeStatus(){
    double x,y;
    if(STATUS == DX){
    // finish to calculate P(x + dx/2, y);
        // get the data
        pCalc.getP(pmVal.Phase_dx);

        // do next simulation
        //  calculate P(x - 1/2.dx, y)
        x = fixedPhase.x() - 1./2. * delta;
        HexabotController::checkPhase(x);
        y = fixedPhase.y();
        HexabotController::checkPhase(y);

        // start the poincre Map
        pCalc.set(osg::Vec2d(x, y), simNum, pConf);

        // change the status
        STATUS = DX_;

        // for debug
        #ifdef CALCJACOBIAN_DEBUG
            std::cout << "@@ DEBUG @@@@ dP_Calc:: Finish the STATUS < DX > " <<  std::endl;
        #endif

    }else if(STATUS == DX_){
    // finish to calculate P(x - dx/2, y);
        // get the data
        pCalc.getP(pmVal.Phase_dx_);

        // do next simulation
        //  calculate P(x, y + dy/2)
        x = fixedPhase.x();
        HexabotController::checkPhase(x);
        y = fixedPhase.y() + 1./2. * delta ;
        HexabotController::checkPhase(y);

        // start the poincre Map
        pCalc.set(osg::Vec2d(x, y), simNum, pConf);

        // change the status
        STATUS = DY;

        // for debug
        #ifdef CALCJACOBIAN_DEBUG
            std::cout << "@@ DEBUG @@@@ dP_Calc:: Finish the STATUS < DX_ > " <<  std::endl;
        #endif

    }else if(STATUS == DY){
    // finish to calculate P(x, y + dy/2);
        // get the data
        pCalc.getP(pmVal.Phase_dy);

        // do next simulation
        //  calculate P(x, y - dy/2)
        x = fixedPhase.x();
        HexabotController::checkPhase(x);
        y = fixedPhase.y() - 1./2. * delta ;
        HexabotController::checkPhase(y);

        // start the poincre Map
        pCalc.set(osg::Vec2d(x, y), simNum, pConf);

        // change the status
        STATUS = DY_;

        // for debug
        #ifdef CALCJACOBIAN_DEBUG
            std::cout << "@@ DEBUG @@@@ dP_Calc:: Finish the STATUS < DY > " <<  std::endl;
        #endif

    }else if(STATUS == DY_){
    // finish to calculate P(x, y + dy/2);
        // get the data
        pCalc.getP(pmVal.Phase_dy_);

        // at this point we have finished the calculation
        STATUS = FIN;
        // calculate the Matrix dP/dx
        dP_dx = calc_dP_dx(pmVal, delta);
        // finish Flag
        finishFlag = true;

        // for debug
        #ifdef CALCJACOBIAN_DEBUG
            std::cout << "@@ DEBUG @@@@ dP_Calc:: Finish the STATUS < DY_ > " <<  std::endl;
        #endif

    }else{
        std::cout << "status error in dCalcPoincreMap" << std::endl;
    }
}

// calculate dP/dx function
StabAnaly::Matrix2_2 CalcJacobian_by_perturbation::calc_dP_dx(const pMapVal &_pmVal, double _delta){
    if(delta == 0.){
        StabAnaly::Matrix2_2 eMat;
        eMat.at(0).at(0) = 0.;
        eMat.at(0).at(1) = 0.;
        eMat.at(1).at(0) = 0.;
        eMat.at(1).at(1) = 0.;
        return eMat;
    }

    // calculate
    StabAnaly::Matrix2_2 rMat;
    rMat.at(0).at(0) = ( _pmVal.Phase_dx.x() - _pmVal.Phase_dx_.x() ) / _delta;
    rMat.at(1).at(0) = ( _pmVal.Phase_dx.y() - _pmVal.Phase_dx_.y() ) / _delta;
    rMat.at(0).at(1) = ( _pmVal.Phase_dy.x() - _pmVal.Phase_dy_.x() ) / _delta;
    rMat.at(1).at(1) = ( _pmVal.Phase_dy.y() - _pmVal.Phase_dy_.y() ) / _delta;
    // return
    return rMat;
}

// is Finished
bool CalcJacobian_by_perturbation::isFinished(){
    return finishFlag;
}

// this is called to get the Matrix
bool CalcJacobian_by_perturbation::getDP(StabAnaly::Matrix2_2& _dP_dx){
    if(!finishFlag){
        return false;
    }else{
        // substitute
        _dP_dx = dP_dx;
        // change the situation and flag
        finishFlag = false;
        STATUS = IDLE;

        return true;
    }
}


// calcJacobian_by_flow ########################################################################
// this is the constructor
CalcJacobian_by_flow::CalcJacobian_by_flow()
    :itfCalcJacobian(), psCalc(), finishFlag(false), STATUS(IDLE)
{
    // initial value
    delta_ini = 0.1;
}

void CalcJacobian_by_flow::set_delta_ini(double val_){
    delta_ini = val_;
}

// set function, with this function, we starts the simulation
bool CalcJacobian_by_flow::set(osg::Vec2d _fixedPhase, int simNum_, const CalcJacobianConf& conf_){
    // extract the parameters from conf
    accuracy = conf_.delta;

    // set the pConf
    psConf.maxPeriodNum = conf_.maxPoincrePeriodNum;
    psConf.poincrePhase = conf_.poincrePhase;
    psConf.poincarePhaseLegNum = conf_.poincarePhaseLegNum;
    psConf.waitPeriodNum = conf_.waitPeriodNum;
    simNum = simNum_;

    // change the status
    STATUS = INI;
    // change the flag
    finishFlag = false;

    // extract the parameters from conf
    fixedPhase = _fixedPhase;

    // start the simulation
    //  calculate P(x + dx, y)
    double x = fixedPhase.x() + delta_ini;
    HexabotController::checkPhase(x);
    double y = fixedPhase.y() - delta_ini;
    HexabotController::checkPhase(y);

    // start the poincre Map
    psCalc.set(osg::Vec2d(x, y), fixedPhase, accuracy, simNum, psConf);

    STATUS = D1;
    return true;
}

// Proc function, it is called in every loop
bool CalcJacobian_by_flow::proc(HexabotController& tC, double time){
    // check the finishFlag
    if(!finishFlag){
        // Proc the inner class
        psCalc.proc(tC, time);

        // check the status change situation
        if(psCalc.isFinished()){changeStatus();}

        // finish
        return true;

    }else{
        return false;
    }
}

// change status function (it is private)
void CalcJacobian_by_flow::changeStatus(){
    double x,y;
    if(STATUS == D1){
    // finish to calculate P(x + dx/2, y);
        // get the data
        psCalc.getPs(d1_phaseVec);

        // calc the evolution of the phase difference
        auto itr = d1_phaseVec.end();
        itr--;
        osg::Vec2d aft = (*itr);
        itr--;
        osg::Vec2d bfr = (*itr);

        pmVal.dPhase_aft_1 = osg::Vec2d(aft.x()-fixedPhase.x(), aft.y()-fixedPhase.y());
        pmVal.dPhase_bfr_1 = osg::Vec2d(bfr.x()-fixedPhase.x(), bfr.y()-fixedPhase.y());

        // do next simulation
        //  calculate P(x - 1/2.dx, y)
        x = fixedPhase.x();
        HexabotController::checkPhase(x);
        y = fixedPhase.y()+delta_ini;
        HexabotController::checkPhase(y);

        // start the poincre Map
        psCalc.set(osg::Vec2d(x, y), fixedPhase, accuracy, simNum, psConf);

        // change the status
        STATUS = D2;

        // for debug
        #ifdef CALCJACOBIAN_DEBUG
            std::cout << "@@ DEBUG @@@@ dP_Calc_flow:: Finish the STATUS < D1 > " <<  std::endl;
        #endif

    }else if(STATUS == D2){
    // finish to calculate P(x, y+dy);
        // get the data
        psCalc.getPs(d2_phaseVec);

        // calc the evolution of the phase difference
        auto itr = d2_phaseVec.end();
        itr--;
        osg::Vec2d aft = (*itr);
        itr--;
        osg::Vec2d bfr = (*itr);

        pmVal.dPhase_aft_2 = osg::Vec2d(aft.x()-fixedPhase.x(), aft.y()-fixedPhase.y());
        pmVal.dPhase_bfr_2 = osg::Vec2d(bfr.x()-fixedPhase.x(), bfr.y()-fixedPhase.y());

        // at this point we have finished the calculation
        STATUS = FIN;
        // calculate the Matrix dP/dx

        dP_dx = calc_dP_dx(pmVal);
        // finish Flag
        finishFlag = true;

        // for debug
        #ifdef CALCJACOBIAN_DEBUG
            std::cout << "@@ DEBUG @@@@ dP_Calc_flow:: Finish the STATUS < D2 > " <<  std::endl;
        #endif

    }else{
        std::cout << "status error in dCalcPoincreMap" << std::endl;
    }
}

// calculate dP/dx function
StabAnaly::Matrix2_2 CalcJacobian_by_flow::calc_dP_dx(const pMapChangeVal &_pmVal){
    //  Df = | x_1_a x_2_a | * | x_1_b x_2_b |-1
    //       | y_1_a y_2_a |   | y_1_b y_2_b |

    /*
    // confirm that the inverse matrix exists
    long double det = (long double)_pmVal.dPhase_bfr_1.x() * (long double)_pmVal.dPhase_bfr_2.y() - (long double)_pmVal.dPhase_bfr_2.x() * (long double)_pmVal.dPhase_bfr_1.y();
    if(fabs(det) < 0.00000000001){
        StabAnaly::Matrix2_2 rMat;
        rMat.at(0).at(0) = 0.;
        rMat.at(1).at(0) = 0.;
        rMat.at(0).at(1) = 0.;
        rMat.at(1).at(1) = 0.;
        // return
        return rMat;
    }

    // calculate inv matrix
    long double inv_a = (long double)_pmVal.dPhase_bfr_2.y() / det;
    long double inv_b = -(long double)_pmVal.dPhase_bfr_2.x() / det;
    long double inv_c = -(long double)_pmVal.dPhase_bfr_1.y() / det;
    long double inv_d = (long double)_pmVal.dPhase_bfr_1.x() / det;

    std::cout << "det, inva, invb, invc, invd" << det << ", "
              << inv_a << ", "  << inv_b << ", " << inv_c << ", " << inv_d <<std::endl;

    // calculate
    StabAnaly::Matrix2_2 rMat;
    rMat.at(0).at(0) = ((long double)_pmVal.dPhase_aft_1.x() * inv_a) + ((long double)_pmVal.dPhase_aft_2.x() * inv_c);
    rMat.at(0).at(1) = ((long double)_pmVal.dPhase_aft_1.x() * inv_b) + ((long double)_pmVal.dPhase_aft_2.x() * inv_d);
    rMat.at(1).at(0) = ((long double)_pmVal.dPhase_aft_1.y() * inv_a) + ((long double)_pmVal.dPhase_aft_2.y() * inv_c);
    rMat.at(1).at(1) = ((long double)_pmVal.dPhase_aft_1.y() * inv_b) + ((long double)_pmVal.dPhase_aft_2.y() * inv_d);
    // return

    std::cout << "a, b, c, d"
              << rMat.at(0).at(0) << ", "  << rMat.at(0).at(1) << ", " << rMat.at(1).at(0) << ", " << rMat.at(1).at(1) <<std::endl;

    */

    // for debug, I wrote again
    double dx_a1 = _pmVal.dPhase_aft_1.x();
    double dx_a2 = _pmVal.dPhase_aft_2.x();
    double dx_b1 = _pmVal.dPhase_bfr_1.x();
    double dx_b2 = _pmVal.dPhase_bfr_2.x();

    double dy_a1 = _pmVal.dPhase_aft_1.y();
    double dy_a2 = _pmVal.dPhase_aft_2.y();
    double dy_b1 = _pmVal.dPhase_bfr_1.y();
    double dy_b2 = _pmVal.dPhase_bfr_2.y();

    double det = dx_b1*dy_b2 - dx_b2*dy_b1;

    StabAnaly::Matrix2_2 rMat;
    rMat.at(0).at(0) = (dx_a1*dy_b2 - dx_a2*dy_b1) / det;
    rMat.at(0).at(1) = (-dx_a1*dx_b2 + dx_a2*dx_b1) / det;
    rMat.at(1).at(0) = (dy_a1*dy_b2 - dy_a2*dy_b1) / det;
    rMat.at(1).at(1) = (-dy_a1*dx_b2 + dy_a2*dx_b1) / det;

    std::cout << "a, b, c, d"
              << rMat.at(0).at(0) << ", "  << rMat.at(0).at(1) << ", " << rMat.at(1).at(0) << ", " << rMat.at(1).at(1) <<std::endl;

    return rMat;
}

// is Finished
bool CalcJacobian_by_flow::isFinished(){
    return finishFlag;
}

// this is called to get the Matrix
bool CalcJacobian_by_flow::getDP(StabAnaly::Matrix2_2& _dP_dx){
    if(!finishFlag){
        return false;
    }else{
        // substitute
        _dP_dx = dP_dx;
        // change the situation and flag
        finishFlag = false;
        STATUS = IDLE;

        return true;
    }
}



// calcPoincreMap #################################################################

// this is the constructor
CalcPoincreMap::CalcPoincreMap()
    :waitSta(this), exeSta(this), aMode(NULL), finishFlag(false)
{
    rPhase = osg::Vec2d(0.,0.);
}

// Proc: it is called every time in loop
bool CalcPoincreMap::proc(HexabotController& tC, double time){
    // main loop
    if(!finishFlag){
        //proc of the object
        if(aMode != NULL){
            // execute proc
            aMode->proc(tC, time);

            // check the termination
            if(aMode->isFinished()){
                changeStatus();
            }
        }
        return true;
    }else return false;
}

// change Status function
void CalcPoincreMap::changeStatus(){
    // change the status depending on the situation;
    if(aMode == &waitSta){
        // this means that we will do EXE stiation at next
        waitSta.terminate();

        // prepare for next
        exeSta.set(conf.poincrePeriodNum, conf.poincrePhase, conf.poincarePhaseLegNum);
        aMode = &exeSta;

        // for debug
        #ifdef CALCPOINCREMAP_DEBUG
            std::cout << "@@ DEBUG @@@@@@ P_Calc:: Finish the STATUS < wait Mode > " <<  std::endl;
        #endif

    }else if(aMode == &exeSta){
        // this means that the P() calc simulation has finished
        exeSta.getP(rPhase);
        exeSta.terminate();

        // terminate
        aMode = NULL;
        finishFlag = true;

        // for debug
        #ifdef CALCPOINCREMAP_DEBUG
            std::cout << "@@ DEBUG @@@@@@ P_Calc:: Finish the STATUS < calc Mode > " <<  std::endl;
        #endif

    }else{
        std::cout << " The status error occurs on CalcPoincreMap " << std::endl;
    }
}

// set the (x_ini, y_ini) and configs  and start the calcu;lation
bool CalcPoincreMap::set(osg::Vec2d x_ini_, int simNum_, CalcPoincreMapConf conf_){
    // conf updates
    conf = conf_;
    simNum  = simNum_;

    // initialize the state which we use
    waitSta.set(x_ini_, conf.waitPeriodNum);
    // set the mode
    aMode = &waitSta;
    // start
    finishFlag = false;

    return true;
}

// this is called to detect whther status transition has finished or not
bool CalcPoincreMap::isFinished(){
    return finishFlag;
}

// this is called to get the data of P(x, y)
bool CalcPoincreMap::getP(osg::Vec2d &_P){
    if(finishFlag){
        _P = rPhase;
        finishFlag = false;
        return true;
    }else{
        return false;
    }
}


// ### THis is the code of WaitMode class ###########################
// this is the constructor
CalcPoincreMap::WaitMode::WaitMode(CalcPoincreMap* _pCal):absMode(_pCal){
    periodTimes = 0;
    waitTime = 0.;
    iniDPhase = osg::Vec2d(0.,0.);
    finishFlag = false;
    STATUS = IDLE;
}

// The function to set
bool CalcPoincreMap::WaitMode::set(osg::Vec2d iniDP, int _periodTimes){
    iniDPhase = iniDP;
    periodTimes = _periodTimes;
    finishFlag = false;
    STATUS = INI;
    return true;
}

// Proc of this class
bool CalcPoincreMap::WaitMode::proc(HexabotController& tC, double time){
    if(STATUS == INI)
    // this is the situation that we set the parameters
    {
        // calc Wait time
        waitTime = ( (double)periodTimes - 0.5  ) * (1. / tC.eGetHexabotCtrConf().oscConf[0].freq );
        // memorize start time
        startTime = time;

        // deactivate the phase Reset and set the oscillation diff
        double phase1 = 0.; // the phase of the Leg 1
        double phase2 = M_PI; // the phase of the Leg 2

        for(int i=0;i<3;i++){
            // stop the phaseReset
            tC.eActivatePhaseReset_all(false);

            // set the Phase of CPG
            tC.eSetPhase(phase1,i);
            tC.eSetPhase(phase2,i+3);

            // update the phase
            if( i < 2){
                phase1 = phase1 + iniDPhase[i];
                phase2 = phase2 + iniDPhase[i];
            }
            // check phase
            HexabotController::checkPhase(phase1);
            HexabotController::checkPhase(phase2);
        }

        // change the status
        STATUS = WAIT;

        // for debug
        #ifdef CALCPOINCREMAP_DEBUG
            std::cout << "@@ DEBUG @@@@@@@@ WAIT:: Finish the STATUS < INI > "  << std::endl;
            std::cout << "@@ DEBUG @@@@@@@@ WAIT::   waitTime:" << waitTime << "[s], ini Dphase:" << iniDPhase[0]
                      << ", " << iniDPhase[1] << "[rad]" << std::endl;
        #endif

    }else if(STATUS == WAIT){
    // this is the situation that we wait the time out
        // Check how much time has passed
        if( (time - startTime) > waitTime ){
            STATUS = FIN;
        }else{};
    }else if(STATUS == FIN){
    //  THis is the situation that WAIT phase is finished
        finishFlag = true;

        // for debug
        #ifdef CALCPOINCREMAP_DEBUG
            std::cout << "@@ DEBUG @@@@@@@@ WAIT:: Finish the STATUS < FIN > "  << std::endl;
        #endif

    }else if(STATUS == IDLE){
        // nothing to do;
    }else{
        std::cout << "status error in WaitMode" << std::endl;
    }
    return true;
}

// Terminate function of this class
void CalcPoincreMap::WaitMode::terminate(){
    STATUS = IDLE;
    finishFlag = false;
}


// ### THis is the code of ExeMode class ###########################
// THis is the constructor
CalcPoincreMap::ExeMode::ExeMode(CalcPoincreMap *_pCal):absMode(_pCal){
    poincrePeriodNum = 0;
    poincrePhase = 0.;
    poincarePhaseLegNum = 0;
    rPhase = osg::Vec2d(0.,0.);
    finishFlag = true;
    STATUS = IDLE;
    poincreFlag = false;
}

// The function to set
bool CalcPoincreMap::ExeMode::set(int _poincrePeriodNum, double _poincrePhase, int _poincarePhaseLegNum){
    poincrePeriodNum = _poincrePeriodNum;
    poincrePhase = _poincrePhase;
    poincarePhaseLegNum = _poincarePhaseLegNum;
    finishFlag = false;
    periodCnt = 0;
    poincreFlag = false;

    STATUS = INI;
    return true;
}

// The Proc of this class
bool CalcPoincreMap::ExeMode::proc(HexabotController &tC, double time){
    // The transition depending on the status
    if(STATUS == INI){
        // we calc the simulation step nums of a period
        double contFreq = (double)tC.eGetHexabotCtrConf().controlFreq;
        double oscFreq = (double)tC.eGetHexabotCtrConf().oscConf[0].freq;
        periodSteps = (int)(contFreq / oscFreq);
        // oscillation period [s]
        oscPeriod = 1. / oscFreq;//[s]

        // we change the status
        STATUS = WAIT;

        // for debug
        #ifdef CALCPOINCREMAP_DEBUG
            std::cout << "@@ DEBUG @@@@@@@@ EXE:: Finish the STATUS < INI > "  << std::endl;
        #endif
    }
    // this status can be startjust after we initialize the parameters
    if(STATUS == WAIT){
        // we wait till the phase1 reaches the poincre plane (phi = poincrePhase)

        bool detectPoincreSection = false;
        double dPhase;
        // Poincre section checking
        if(fabs(poincrePhase) < 0.001){
            // detect by ground contact of Leg 1
            if( !tC.eIsLegContact(poincarePhaseLegNum) ){
                poincreFlag = true;
            }else if( tC.eIsLegContact(poincarePhaseLegNum) && poincreFlag){
                dPhase = tC.eGetPhase(poincarePhaseLegNum);
                poincreFlag = false;
                detectPoincreSection = true;
            }
        }else{
            // detect by only phase
            dPhase = tC.eGetPhase(poincarePhaseLegNum) - poincrePhase;
            if( fabs(dPhase) < 2. * (2. * M_PI / (double)periodSteps) ){
                if(dPhase < 0.){
                    poincreFlag = true;
                }
                // Poincre section is determined the place where the +- has changed
                else if(dPhase > 0. && poincreFlag){
                    poincreFlag = false;
                    detectPoincreSection = true;
                }
            }
        }

        // If detect poincre section
        if(detectPoincreSection){
                // we starts the phase reset
                tC.eActivatePhaseReset_all(true);

                // memorize the time
                pTime = time;
                // Counting the period starts
                periodCnt = 0;
                // change the status
                STATUS = EXE;

                // for debug
                #ifdef CALCPOINCREMAP_DEBUG
                    std::cout << "@@ DEBUG @@@@@@@@ EXE:: Finish the STATUS < WAIT > "  << std::endl;
                    std::cout << "@@ DEBUG @@@@@@@@ EXE::   nowTime:" << time << "[s], poincre section dPhase:" << dPhase
                              << "[rad]" << std::endl;
                #endif

                detectPoincreSection = false;
        }
    }else if(STATUS == EXE){
        // we execute the phase resetting mechanism till the cnt reaches the objective one
        // when reaches poincre plane

        bool detectPoincreSection2 = false;
        double dPhase2;
        // Poincre section checking
        if(fabs(poincrePhase) < 0.001){
            // detect by ground contact of Leg 1
            if( !tC.eIsLegContact(poincarePhaseLegNum) ){
                poincreFlag = true;
            }else if( tC.eIsLegContact(poincarePhaseLegNum) && poincreFlag){
                dPhase2 = tC.eGetPhase(poincarePhaseLegNum);
                poincreFlag = false;
                detectPoincreSection2 = true;
            }
        }else{
            // detect by only phase
            dPhase2 = tC.eGetPhase(poincarePhaseLegNum) - poincrePhase;
            if( fabs(dPhase2) < 2. * (2. * M_PI / (double)periodSteps) && ( (time - pTime) > 1./2. * oscPeriod )){
                if(dPhase2 < 0.){
                    poincreFlag = true;
                }
                // Poincre section is determined the place where the +- has changed
                else if(dPhase2 > 0. && poincreFlag){
                    poincreFlag = false;
                    detectPoincreSection2 = true;
                }
            }
        }

        // If detect poincre section
        if(detectPoincreSection2){
                periodCnt++;
                pTime = time;

                // for debug
                #ifdef CALCPOINCREMAP_DEBUG
                    std::cout << "@@ DEBUG @@@@@@@@ EXE:: in the STATUS < EXE > detect poincre plane"  << std::endl;
                    std::cout << "@@ DEBUG @@@@@@@@ EXE::   nowTime:" << time << "[s], poincre section dPhase:" << dPhase2
                          << "[rad], cnt:" << periodCnt << std::endl;
                #endif

                detectPoincreSection2 = false;

        }

        // when the count goes over the desired PeriodNum
        if(periodCnt == poincrePeriodNum){
            // stop the phase reset mechanism
            tC.eActivatePhaseReset_all(false);

            // get the phase diff data
            rPhase[0] = tC.eGetPhase(1) - tC.eGetPhase(0);
            rPhase[1] = tC.eGetPhase(2) - tC.eGetPhase(1);
            HexabotController::checkPhase(rPhase[0]);
            HexabotController::checkPhase(rPhase[1]);

            // change the status
            STATUS = FIN;

            // for debug
            #ifdef CALCPOINCREMAP_DEBUG
                std::cout << "@@ DEBUG @@@@@@@@ EXE:: Finish the STATUS < FIN > "  << std::endl;
                std::cout << "@@ DEBUG @@@@@@@@ EXE::   cnt:" << periodCnt << ", rdPhase:" << rPhase[0] << ", " << rPhase[1]
                          << "[rad]" << std::endl;
            #endif
        }
    }else if(STATUS == FIN){
        // finalize the simulation
        finishFlag = true;
    }else if(STATUS == IDLE){
        // the system is idle;
    }else{
        std::cout << "status error in ExeMode" << std::endl;
    }

    return true;
}

// The function to getP
bool CalcPoincreMap::ExeMode::getP(osg::Vec2d &_P){
    // check the flag
    if(finishFlag){
        _P[0] = rPhase[0];
        _P[1] = rPhase[1];
        return true;
    }else{
        return false;
    }
}

// Terminate function of this class
void CalcPoincreMap::ExeMode::terminate(){
    STATUS = IDLE;
    finishFlag = false;
}

// ### THis is the code of absMode class ###########################
 // constructor
CalcPoincreMap::absMode::absMode(CalcPoincreMap* _pCal):pCal(_pCal), finishFlag(false){}
 // destructor
CalcPoincreMap::absMode::~absMode(){
    pCal = 0;
}
 // real func is Finihsed
bool CalcPoincreMap::absMode::isFinished(){
    return finishFlag;
}

// ### THis is the code of itfMode class ###########################
 // virtual func "proc"
bool CalcPoincreMap::itfMode::proc(HexabotController &tC, double time){
    return false;
}
 // virtual func "isFinished"
bool CalcPoincreMap::itfMode::isFinished(){
    return false;
}


// calcPoincreMaps #################################################################

// this is the constructor
CalcPoincreMaps::CalcPoincreMaps()
    :waitSta(this), exeSta(this), aMode(NULL), finishFlag(false)
{
    rPhaseVec.clear();
}

// Proc: it is called every time in loop
bool CalcPoincreMaps::proc(HexabotController& tC, double time){
    // main loop
    if(!finishFlag){
        //proc of the object
        if(aMode != NULL){
            // execute proc
            aMode->proc(tC, time);

            // check the termination
            if(aMode->isFinished()){
                changeStatus();
            }
        }
        return true;
    }else return false;
}

// change Status function
void CalcPoincreMaps::changeStatus(){
    // change the status depending on the situation;
    if(aMode == &waitSta){
        // this means that we will do EXE stiation at next
        waitSta.terminate();

        // prepare for next
        exeSta.set(fixedPhase, delta, conf.maxPeriodNum, conf.poincrePhase, conf.poincarePhaseLegNum);
        aMode = &exeSta;

        // for debug
        #ifdef CALCPOINCREMAP_DEBUG
            std::cout << "@@ DEBUG @@@@@@ Ps_Calc:: Finish the STATUS < wait Mode > " <<  std::endl;
        #endif

    }else if(aMode == &exeSta){
        // this means that the P() calc simulation has finished
        exeSta.getPs(rPhaseVec);
        exeSta.terminate();

        // terminate
        aMode = NULL;
        finishFlag = true;

        // for debug
        #ifdef CALCPOINCREMAP_DEBUG
            std::cout << "@@ DEBUG @@@@@@ Ps_Calc:: Finish the STATUS < calc Mode > " <<  std::endl;
        #endif

    }else{
        std::cout << " The status error occurs on CalcPoincreMaps " << std::endl;
    }
}

// set the (x_ini, y_ini) and configs  and start the calcu;lation
bool CalcPoincreMaps::set(osg::Vec2d x_ini_, osg::Vec2d x_fixed_, double delta_, int simNum_, CalcPoincreMapsConf conf_){
    // conf updates
    conf = conf_;
    simNum  = simNum_;
    delta = delta_;
    fixedPhase = x_fixed_;
    rPhaseVec.clear();

    // initialize the state which we use
    waitSta.set(x_ini_, conf.waitPeriodNum);
    // set the mode
    aMode = &waitSta;
    // start
    finishFlag = false;

    return true;
}

// this is called to detect whther status transition has finished or not
bool CalcPoincreMaps::isFinished(){
    return finishFlag;
}

// this is called to get the data of P(x, y)
bool CalcPoincreMaps::getPs(vec2dVector &_P_vec){
    if(finishFlag){
        _P_vec = rPhaseVec;
        finishFlag = false;
        return true;
    }else{
        return false;
    }
}


// ### THis is the code of WaitMode class ###########################
// this is the constructor
CalcPoincreMaps::WaitMode::WaitMode(CalcPoincreMaps* _pCal):absMode(_pCal){
    periodTimes = 0;
    waitTime = 0.;
    iniDPhase = osg::Vec2d(0.,0.);
    finishFlag = false;
    STATUS = IDLE;
}

// The function to set
bool CalcPoincreMaps::WaitMode::set(osg::Vec2d iniDP, int _periodTimes){
    iniDPhase = iniDP;
    periodTimes = _periodTimes;
    finishFlag = false;
    STATUS = INI;
    return true;
}

// Proc of this class
bool CalcPoincreMaps::WaitMode::proc(HexabotController& tC, double time){
    if(STATUS == INI)
    // this is the situation that we set the parameters
    {
        // calc Wait time
        waitTime = ( (double)periodTimes - 0.5  ) * (1. / tC.eGetHexabotCtrConf().oscConf[0].freq );
        // memorize start time
        startTime = time;

        // deactivate the phase Reset and set the oscillation diff
        double phase1 = 0.; // the phase of the Leg 1
        double phase2 = M_PI; // the phase of the Leg 2

        for(int i=0;i<3;i++){
            // stop the phaseReset
            tC.eActivatePhaseReset_all(false);

            // set the Phase of CPG
            tC.eSetPhase(phase1,i);
            tC.eSetPhase(phase2,i+3);

            // update the phase
            if( i < 2){
                phase1 = phase1 + iniDPhase[i];
                phase2 = phase2 + iniDPhase[i];
            }
            // check phase
            HexabotController::checkPhase(phase1);
            HexabotController::checkPhase(phase2);
        }

        // change the status
        STATUS = WAIT;

        // for debug
        #ifdef CALCPOINCREMAP_DEBUG
            std::cout << "@@ DEBUG @@@@@@@@ Ps WAIT:: Finish the STATUS < INI > "  << std::endl;
            std::cout << "@@ DEBUG @@@@@@@@ Ps WAIT::   waitTime:" << waitTime << "[s], ini Dphase:" << iniDPhase[0]
                      << ", " << iniDPhase[1] << "[rad]" << std::endl;
        #endif

    }else if(STATUS == WAIT){
    // this is the situation that we wait the time out
        // Check how much time has passed
        if( (time - startTime) > waitTime ){
            STATUS = FIN;
        }else{};
    }else if(STATUS == FIN){
    //  THis is the situation that WAIT phase is finished
        finishFlag = true;

        // for debug
        #ifdef CALCPOINCREMAP_DEBUG
            std::cout << "@@ DEBUG @@@@@@@@ Ps WAIT:: Finish the STATUS < FIN > "  << std::endl;
        #endif

    }else if(STATUS == IDLE){
        // nothing to do;
    }else{
        std::cout << "status error in WaitMode" << std::endl;
    }
    return true;
}

// Terminate function of this class
void CalcPoincreMaps::WaitMode::terminate(){
    STATUS = IDLE;
    finishFlag = false;
}


// ### THis is the code of ExeMode class ###########################
// THis is the constructor
CalcPoincreMaps::ExeMode::ExeMode(CalcPoincreMaps *_pCal):absMode(_pCal){
    maxPoincrePeriodNum = 0;
    poincrePhase = 0.;
    poincarePhaseLegNum = 0;
    finishFlag = true;
    STATUS = IDLE;
    poincreFlag = false;
    delta = 0;
}

// The function to set
bool CalcPoincreMaps::ExeMode::set(osg::Vec2d fixedPoint_, double delta_, int maxPeriodNum_, double _poincrePhase, int _poincarePhaseLegNum){
    maxPoincrePeriodNum = maxPeriodNum_;
    fixedPhase = fixedPoint_;
    delta = delta_;
    poincrePhase = _poincrePhase;
    poincarePhaseLegNum = _poincarePhaseLegNum;
    finishFlag = false;
    periodCnt = 0;
    poincreFlag = false;
    rPhaseVec.clear();

    STATUS = INI;
    return true;
}

// The Proc of this class
bool CalcPoincreMaps::ExeMode::proc(HexabotController &tC, double time){
    // The transition depending on the status
    if(STATUS == INI){
        // we calc the simulation step nums of a period
        double contFreq = (double)tC.eGetHexabotCtrConf().controlFreq;
        double oscFreq = (double)tC.eGetHexabotCtrConf().oscConf[0].freq;
        periodSteps = (int)(contFreq / oscFreq);
        // oscillation period [s]
        oscPeriod = 1. / oscFreq;//[s]

        // we change the status
        STATUS = WAIT;

        // for debug
        #ifdef CALCPOINCREMAP_DEBUG
            std::cout << "@@ DEBUG @@@@@@@@ Ps EXE:: Finish the STATUS < INI > "  << std::endl;
        #endif
    }
    // this status can be startjust after we initialize the parameters
    if(STATUS == WAIT){
        // we wait till the phase1 reaches the poincre plane (phi = poincrePhase)

        bool detectPoincreSection = false;
        double dPhase;
        // Poincre section checking
        if(fabs(poincrePhase) < 0.001){
            // detect by ground contact of Leg 1
            if( !tC.eIsLegContact(poincarePhaseLegNum) ){
                poincreFlag = true;
            }else if( tC.eIsLegContact(poincarePhaseLegNum) && poincreFlag){
                dPhase = tC.eGetPhase(poincarePhaseLegNum);
                poincreFlag = false;
                detectPoincreSection = true;
            }
        }else{
            // detect by only phase
            dPhase = tC.eGetPhase(poincarePhaseLegNum) - poincrePhase;
            if( fabs(dPhase) < 2. * (2. * M_PI / (double)periodSteps) ){
                if(dPhase < 0.){
                    poincreFlag = true;
                }
                // Poincre section is determined the place where the +- has changed
                else if(dPhase > 0. && poincreFlag){
                    poincreFlag = false;
                    detectPoincreSection = true;
                }
            }
        }

        // If detect poincre section
        if(detectPoincreSection){
                // we starts the phase reset
                tC.eActivatePhaseReset_all(true);

                // memorize the time
                pTime = time;
                // Counting the period starts
                periodCnt = 0;
                // change the status
                STATUS = EXE;

                // for debug
                #ifdef CALCPOINCREMAP_DEBUG
                    std::cout << "@@ DEBUG @@@@@@@@ Ps EXE:: Finish the STATUS < WAIT > "  << std::endl;
                    std::cout << "@@ DEBUG @@@@@@@@ Ps EXE::   nowTime:" << time << "[s], poincre section dPhase:" << dPhase
                              << "[rad]" << std::endl;
                #endif

                detectPoincreSection = false;
        }
    }else if(STATUS == EXE){
        // we execute the phase resetting mechanism till the cnt reaches the objective one
        // when reaches poincre plane

        bool detectPoincreSection2 = false;
        double dPhase2;
        // Poincre section checking
        if(fabs(poincrePhase) < 0.001){
            // detect by ground contact of Leg 1
            if( !tC.eIsLegContact(poincarePhaseLegNum) ){
                poincreFlag = true;
            }else if( tC.eIsLegContact(poincarePhaseLegNum) && poincreFlag){
                dPhase2 = tC.eGetPhase(poincarePhaseLegNum);
                poincreFlag = false;
                detectPoincreSection2 = true;
            }
        }else{
            // detect by only phase
            dPhase2 = tC.eGetPhase(poincarePhaseLegNum) - poincrePhase;
            if( fabs(dPhase2) < 2. * (2. * M_PI / (double)periodSteps) && ( (time - pTime) > 1./2. * oscPeriod )){
                if(dPhase2 < 0.){
                    poincreFlag = true;
                }
                // Poincre section is determined the place where the +- has changed
                else if(dPhase2 > 0. && poincreFlag){
                    poincreFlag = false;
                    detectPoincreSection2 = true;
                }
            }
        }

        // If detect poincre section
        if(detectPoincreSection2){
                periodCnt++;
                pTime = time;

                // get the phase diff data
                double rPhase0 = tC.eGetPhase(1) - tC.eGetPhase(0);
                double rPhase1 = tC.eGetPhase(2) - tC.eGetPhase(1);
                HexabotController::checkPhase(rPhase0);
                HexabotController::checkPhase(rPhase1);
                osg::Vec2d rPhase = osg::Vec2d(rPhase0, rPhase1);

                // add to the result
                rPhaseVec.push_back(rPhase);

                // check it reaches or not
                double dx = fabs(rPhase.x() - fixedPhase.x());
                double dy = fabs(rPhase.y() - fixedPhase.y());

                // for debug
                #ifdef CALCPOINCREMAP_DEBUG
                    std::cout << "@@ DEBUG @@@@@@@@ Ps EXE:: in the STATUS < EXE > detect poincre plane"  << std::endl;
                    std::cout << "@@ DEBUG @@@@@@@@ Ps EXE::   nowTime:" << time << "[s], poincre section dPhase:" << dPhase2
                          << "[rad], cnt:" << periodCnt << std::endl;
                    std::cout << "@@ DEBUG @@@@@@@@ Ps EXE::   Phase: (" << rPhase.x() << ", " << rPhase.y() << " ),   dx : ("
                              << rPhase.x() - fixedPhase.x() << ", " << rPhase.y() - fixedPhase.y() << " ) " << std::endl;
                #endif

                // it reaches desired pos
                if(dx < delta && dy < delta){
                    // stop phase reset
                    tC.eActivatePhaseReset_all(false);
                    // change the status
                    STATUS = FIN;

                    // for debug
                    #ifdef CALCPOINCREMAP_DEBUG
                    std::cout << "@@ DEBUG @@@@@@@@ Ps EXE:: Finish the STATUS (NORMAL) < FIN > "  << std::endl;
                        std::cout << "@@ DEBUG @@@@@@@@ Ps EXE::   cnt:" << periodCnt << ", rdPhase:" << rPhase[0] << ", " << rPhase[1]
                                  << "[rad]" << std::endl;
                    #endif
                }

                detectPoincreSection2 = false;
        }

        // when the count goes over the max PeriodNum
        if(periodCnt > maxPoincrePeriodNum){
            // stop the phase reset mechanism
            tC.eActivatePhaseReset_all(false);
            // change the status
            STATUS = FIN;

            // for debug
            #ifdef CALCPOINCREMAP_DEBUG
            std::cout << "@@ DEBUG @@@@@@@@ Ps EXE:: Finish the STATUS (MAX PERIOD) < FIN > "  << std::endl;
            //std::cout << "@@ DEBUG @@@@@@@@ Ps EXE::   cnt:" << periodCnt << ", rdPhase:" << rPhase[0] << ", " << rPhase[1]
            //              << "[rad]" << std::endl;
            #endif
        }
    }else if(STATUS == FIN){
        // finalize the simulation
        finishFlag = true;
    }else if(STATUS == IDLE){
        // the system is idle;
    }else{
        std::cout << "status error in ExeMode" << std::endl;
    }

    return true;
}

// The function to getP
bool CalcPoincreMaps::ExeMode::getPs(CalcPoincreMaps::vec2dVector &phaseVec_){
    // check the flag
    if(finishFlag){
        phaseVec_ = rPhaseVec;
        return true;
    }else{
        return false;
    }
}

// Terminate function of this class
void CalcPoincreMaps::ExeMode::terminate(){
    STATUS = IDLE;
    finishFlag = false;
}

// ### THis is the code of absMode class ###########################
 // constructor
CalcPoincreMaps::absMode::absMode(CalcPoincreMaps* _pCal):pCal(_pCal), finishFlag(false){}
 // destructor
CalcPoincreMaps::absMode::~absMode(){
    pCal = 0;
}
 // real func is Finihsed
bool CalcPoincreMaps::absMode::isFinished(){
    return finishFlag;
}

// ### THis is the code of itfMode class ###########################
 // virtual func "proc"
bool CalcPoincreMaps::itfMode::proc(HexabotController &tC, double time){
    return false;
}
 // virtual func "isFinished"
bool CalcPoincreMaps::itfMode::isFinished(){
    return false;
}





