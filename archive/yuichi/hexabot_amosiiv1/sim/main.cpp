/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 *   $Log$
 *   Revision 1.4  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.3  2009/08/05 23:25:57  martius
 *   adapted small things to compile with changed Plotoptions
 *
 *   Revision 1.2  2008/05/01 22:03:55  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.1  2007/03/05 10:18:24  fhesse
 *   nimm2_eating created
 *
 ***************************************************************************/

/*
1, hexapod simulation 07

THis is the version to calculate fixed point and analise stability

I changed the program so much to deal it as object.

20140610 Ver 1.21
 Add the poincarePhaseLegNum!

20140514 Ver 1.20
 Add the program to do simulation by chnging the parameter gradually depending on time

20140507 Ver 1.10
 Add the programs to search Basin of the solution.

20140423 Ver 1.03
 Change stab anal to add some other jacobian calc

20140409 Ver 1.02
 Add Surface to change stiffness

*/

//! Version Control
#define VERSION_MAIN "Main_hex_Ver 1.21_20140610"

#include <osg/Matrix>
#include "hxSimulation.h"


// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace HEXABOT;

// debug mode
#define DEBUG_MAIN 1

// about simulation
const int ThisSim::S_SIMULATION_FREQ = 800;//3200; // 3200 in thesis
const int ThisSim::S_CONTROL_FREQ = 800;//3200; // 3200 in thesis

// about file
#define DETAIL_FILE_POS  "//home//ambe//Dropbox//AshigaruShare//hexData//test//"
//#define FILE_NAME "H_leg_sw006_ST07_i05_1"
//#define FILE_NAME "H_leg_sw006_ST05_d06_inhibi_2"
#define FILE_NAME "L_leg_sw000_ST10_d06_test"

// high speed mode : The movie is stopped and it gains simulation speed!!
const bool ThisSim::S_HIGHSPEED_MODE = true; //true;// false;

// configs setting
void ThisSim::setConfigs(void){
    // set configuration of the parameters
    //double phase1 = 0.;
    //double phase2 = M_PI;

    //======= Robot ==============================================
    // get default conf
    hrConf = Hexabot::getDefaultConf();

    // PID controller setting
    hrConf.servoParam.power = 300; //100 //200;//300;//100.;//100000;
    hrConf.servoParam.damp =  0.0005;//0.0005;
    hrConf.servoParam.integ = 0.;

    // Robot configulation
    hrConf.body.length = 0.42;
    hrConf.body.width = 0.08;
    hrConf.foot.footRadius = 0.007;

    hrConf.body.mass = 3.0 ; // 4.6
    hrConf.dyna.mass = 0.27; //2 ; //0.2 0.01
    hrConf.foot.mass = 0.27; //2 ;
    hrConf.wholeMass = (hrConf.body.mass + 18. * hrConf.dyna.mass + 6. * hrConf.foot.mass);

    hrConf.jLength.length_y_TCJ_to_TCJ = 0.2;
    hrConf.jLength.length_x_center_to_TCJ = 0.05; //m
    hrConf.jLength.length_TCJ_to_CTJ = 0.065; //m
    hrConf.jLength.length_CTJ_to_FTJ = 0.065;
    hrConf.jLength.length_FTJ_to_Toe = 0.115;

    // difference between body center and joint center
    hrConf.x_trans_center = 0.0;
    hrConf.y_trans_center = 0.0; // if the value is positive, a body will shift back

    // ======= Controller ==================================================
    hcConf = HexabotController::getDefaultConf();

    // set the Freq, Duty rate and inhiibi Coef
    for(int i = 0; i < 6; i++){
        hcConf.oscConf[i].freq = 1./ (10. / 0.4);//1./ (3. / 0.4);
        hcConf.oscConf[i].dutyRate = 0.6;
        hcConf.oscConf[i].inhibiCoef = 1.; //0.5 1.5; //1.5; //0.9
    }
    hcConf.controlFreq = (double)this->S_CONTROL_FREQ;
    hcConf.twoCpgCoef = 10.; //10. //2.5; //0.3; //0.05; //0.1;

    // set initial Freq Difference
    hcConf.oscConf[0].initialPhase = 0;//3/2. * M_PI;
    hcConf.oscConf[1].initialPhase = M_PI;//3/2. * M_PI;
    hcConf.oscConf[2].initialPhase = 0;//3/2. * M_PI;
    hcConf.oscConf[3].initialPhase = M_PI;//3/2. * M_PI;
    hcConf.oscConf[4].initialPhase = 0;//3/2. * M_PI;
    hcConf.oscConf[5].initialPhase = M_PI;//3/2. * M_PI;

    // to use phase reset or not
    //hcConf.PC_MODE = PC_IN_IN; // PC_PR_PR
    hcConf.PC_MODE = PC_PR_LPASS; // Low pass filter reset
    hcConf.TFactor = 1.;

    // to use one side phase modulation or not
    hcConf.is_oneSidePhaseModulation = false;//true; //true;//false;

    // set for poincre analysis
    //  the time when we start the phase reset mechanism after start the simulation.
    hcConf.biasStartTime = 2.;

    // set the leg trajectory
    bool is_leg_upDown = true;
    if(is_leg_upDown){
        for(int i=0;i<6;i++){
            hcConf.trjConf[i].walkLength = 0.0; //0.06;//0.04;
            //hcConf.trjConf[i].swingHeight = 0.04;
        }
    }


    // ====== Modules =============================================================

    //***** Find solution and stab analy ***********************************:
    fsConf = findSolution_analyzeStability::getDefaultConf();
    // poincare phase leg num
    fsConf.stabAnalyConf.poincarePhaseLegNum = 1;
    fsConf.findFpConf.fConf.poincarePhaseLegNum = 1;
    // accuracy of the solution
    fsConf.findFpConf.fConf.phaseThreshold = 0.001;//0.001
    // delta to calc jacobian
    fsConf.stabAnalyConf.delta = 0.1;
    fsConf.stabAnalyConf.initialPhaseDiff = 0.1;
    fsConf.stabAnalyConf.is_use_calcJacobian_by_flow = false;

    fsConf.findFpConf.fConf.maxPeriodNum = 150;
    fsConf.findFpConf.fConf.waitPeriodNum = 20;//40; //50;

    //fsConf.findFpConf.fConf.poincrePhase = 30./20.*M_PI;
    //fsConf.stabAnalyConf.poincrePhase = 30./20.*M_PI;

    // initial value to search
    //fsConf.initialStateVec.push_back(osg::Vec2d(2.092, 1.967)); // lower : duty 0.67
    //fsConf.initialStateVec.push_back(osg::Vec2d(4.308, 4.326)); // higher : duty 0.67

    //fsConf.initialStateVec.push_back(osg::Vec2d(2.09, 2.1)); // lower : duty 0.66
    //fsConf.initialStateVec.push_back(osg::Vec2d(2.13, 2.15)); // lower : duty 0.65
    //fsConf.initialStateVec.push_back(osg::Vec2d(2.18, 2.21)); // lower : duty 0.64
    //fsConf.initialStateVec.push_back(osg::Vec2d(2.24, 2.26)); // lower : duty 0.63
    //fsConf.initialStateVec.push_back(osg::Vec2d(2.30, 2.32)); // lower : duty 0.62
    //fsConf.initialStateVec.push_back(osg::Vec2d(2.36, 2.38)); // lower : duty 0.61
    //fsConf.initialStateVec.push_back(osg::Vec2d(2.41, 2.44)); // lower : duty 0.60
    //fsConf.initialStateVec.push_back(osg::Vec2d(2.67, 2.61)); // lower : duty 0.56
    //fsConf.initialStateVec.push_back(osg::Vec2d(4.0, 4.0)); // lower : duty 0.62

    //fsConf.initialStateVec.push_back(osg::Vec2d(3.53, 3.54)); // hoigher : duty 0.52
    /*
    fsConf.initialStateVec.push_back(osg::Vec2d(2.50, 2.50)); // lower : duty 0.50
    fsConf.initialStateVec.push_back(osg::Vec2d(2.60, 2.60)); // lower : duty 0.50
    fsConf.initialStateVec.push_back(osg::Vec2d(2.70, 2.70)); // lower : duty 0.50
    fsConf.initialStateVec.push_back(osg::Vec2d(2.80, 2.80)); // lower : duty 0.50
    fsConf.initialStateVec.push_back(osg::Vec2d(2.90, 2.90)); // lower : duty 0.50
    */

    //fsConf.initialStateVec.push_back(osg::Vec2d(3.12117, 3.17914)); // lower : duty 0.50
    //fsConf.initialStateVec.push_back(osg::Vec2d(3., 3.)); // lower : duty 0.50
    //fsConf.initialStateVec.push_back(osg::Vec2d(2.41644, 2.43121)); // lower : duty 0.6
    //fsConf.initialStateVec.push_back(osg::Vec2d(2.28563, 2.20201)); // lower : duty 0.64
    //fsConf.initialStateVec.push_back(osg::Vec2d(2.08, 2.09)); // lower : duty 0.66

    //fsConf.initialStateVec.push_back(osg::Vec2d(3.20, 3.20)); // lower : duty 0.50
    fsConf.initialStateVec.push_back(osg::Vec2d(3.87902, 3.88851)); // higher : duty 0.6
    //fsConf.initialStateVec.push_back(osg::Vec2d(3.87902, 3.88851)); // higher : duty 0.6
    //fsConf.initialStateVec.push_back(osg::Vec2d(3.7, 3.7)); // higher 2nd sol?? : duty 0.6
    //fsConf.initialStateVec.push_back(osg::Vec2d(3.869, 4.14)); // higher 2nd sol?? : duty 0.6
    //fsConf.initialStateVec.push_back(osg::Vec2d(4.21, 4.187)); // lower : duty 0.66


    /*
    fsConf.initialStateVec.push_back(osg::Vec2d(2.51644, 2.43121)); // lower : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(2.51644, 2.33121)); // lower : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(2.51644, 2.23121)); // lower : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(2.51644, 2.53121)); // lower : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(2.51644, 2.63121)); // lower : duty 0.6

    fsConf.initialStateVec.push_back(osg::Vec2d(2.61644, 2.43121)); // lower : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(2.61644, 2.33121)); // lower : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(2.61644, 2.23121)); // lower : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(2.61644, 2.53121)); // lower : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(2.61644, 2.63121)); // lower : duty 0.6

    fsConf.initialStateVec.push_back(osg::Vec2d(2.41644, 2.43121)); // lower : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(2.41644, 2.33121)); // lower : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(2.41644, 2.23121)); // lower : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(2.41644, 2.53121)); // lower : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(2.41644, 2.63121)); // lower : duty 0.6

    fsConf.initialStateVec.push_back(osg::Vec2d(2.71644, 2.43121)); // lower : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(2.71644, 2.33121)); // lower : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(2.71644, 2.23121)); // lower : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(2.71644, 2.53121)); // lower : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(2.71644, 2.63121)); // lower : duty 0.6

    fsConf.initialStateVec.push_back(osg::Vec2d(2.31644, 2.43121)); // lower : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(2.31644, 2.33121)); // lower : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(2.31644, 2.23121)); // lower : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(2.31644, 2.53121)); // lower : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(2.31644, 2.63121)); // lower : duty 0.6
    */

    /*
    fsConf.initialStateVec.push_back(osg::Vec2d(3.84302, 3.8651)); // higher : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(3.84302, 3.7651)); // higher : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(3.84302, 3.6651)); // higher : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(3.84302, 3.9651)); // higher : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(3.84302, 4.0651)); // higher : duty 0.6

    fsConf.initialStateVec.push_back(osg::Vec2d(3.94302, 3.8651)); // higher : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(3.94302, 3.7651)); // higher : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(3.94302, 3.6651)); // higher : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(3.94302, 3.9651)); // higher : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(3.94302, 4.0651)); // higher : duty 0.6

    fsConf.initialStateVec.push_back(osg::Vec2d(3.74302, 3.8651)); // higher : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(3.74302, 3.7651)); // higher : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(3.74302, 3.6651)); // higher : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(3.74302, 3.9651)); // higher : duty 0.6
    //fsConf.initialStateVec.push_back(osg::Vec2d(3.74302, 4.0651)); // higher : duty 0.6

    fsConf.initialStateVec.push_back(osg::Vec2d(3.64302, 3.8651)); // higher : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(3.64302, 3.7651)); // higher : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(3.64302, 3.6651)); // higher : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(3.64302, 3.9651)); // higher : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(3.64302, 4.0651)); // higher : duty 0.6

    fsConf.initialStateVec.push_back(osg::Vec2d(4.04302, 3.8651)); // higher : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(4.04302, 3.7651)); // higher : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(4.04302, 3.6651)); // higher : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(4.04302, 3.9651)); // higher : duty 0.6
    fsConf.initialStateVec.push_back(osg::Vec2d(4.04302, 4.0651)); // higher : duty 0.6
    */

    //******* Find Solution *******************************
    fSolConf = findSolution::getDefaultConf();
    fSolConf.findFpConf.fConf.phaseThreshold = 0.001;
    fSolConf.findFpConf.fConf.poincarePhaseLegNum = 1;
    fSolConf.findFpConf.fConf.waitPeriodNum = 20;
    fSolConf.findFpConf.fConf.maxPeriodNum = 150;
    //fSolConf.findFpConf.fConf.maxSimulationTime = 100;

    fSolConf.initialState = osg::Vec2d(4., 4.);

    //******* RunLogSimulation **************************
    rlConf = runLogSimulation::getDefaultConf();
    rlConf.poincarePhase = 0.;
    rlConf.waitTime = 30;

    //rlConf.initialState = osg::Vec2d(2.9, 2.9);
    rlConf.initialState = osg::Vec2d(2.4, 2.4);


    // ====== Simulation ============================================================
    sParam.time_wait_in_flight = 20.; //wait in the flight to converge PD deviation (s)
    sParam.time_wait_on_ground = 20.; // wait on the ground to converge PD deviation before start
    sParam.stiff_of_ground = 100.; // stiffness of the ground
    sParam.damp_of_ground = 0.45; // damping ration of the ground
}



// Constructor
ThisSim::ThisSim()
    :Simulation(), itfSim(0), repAnalSim(0), find_and_stab_onePSim(0), findSol_onePsim(0),
      runLog_onePSim(0), pTCPA(0), pTCPA_sub1(0), pTCPA_sub2(0),
      hexabot(0), controller(0), tController(0), pRepMod(0), simNum(1), paramOfs()
{
    // determine configs
    setConfigs();

    // set the simulation program
    //   param changing class
    //pRepMod = new cOneArray6ParamRepMod<double>(hcConf.oscConf[0].inhibiCoef, hcConf.oscConf[1].inhibiCoef, hcConf.oscConf[2].inhibiCoef, hcConf.oscConf[3].inhibiCoef,
    //                                             hcConf.oscConf[4].inhibiCoef, hcConf.oscConf[5].inhibiCoef, 1., 1., 0.01);


    // param chaniging class
    cChangeDuty_KeepSwingDuration_RepMod::paramSet prm(hcConf.oscConf[0].dutyRate, hcConf.oscConf[1].dutyRate, hcConf.oscConf[2].dutyRate, hcConf.oscConf[3].dutyRate, hcConf.oscConf[4].dutyRate, hcConf.oscConf[5].dutyRate,
                                                       hcConf.oscConf[0].freq, hcConf.oscConf[1].freq, hcConf.oscConf[2].freq, hcConf.oscConf[3].freq, hcConf.oscConf[4].freq, hcConf.oscConf[5].freq);

    //pRepMod = new cChangeDuty_KeepSwingDuration_RepMod(prm, 10, 0.6, 0.7, 0.01);
    pRepMod = new cChangeDuty_KeepSwingDuration_RepMod(prm, 10, 0.6, 0.6, -0.01);

    //pRepMod = new cChangeIniPhase_RepMod(fSolConf.initialState, osg::Vec2d(0.627, 0.627), osg::Vec2d(6.28, 6.28), osg::Vec2d(0.628, 0.628));

    //pRepMod = new cOneParamRepMod<double>(fsConf.stabAnalyConf.delta, 0.001, 0.1, 0.01);
    //pRepMod = new cOneParamRepMod<double>(hrConf.servoParam.power, 300, 50, -50);
    //pRepMod = new cOneParamRepMod<double>(hrConf.y_trans_center, 0.05, 0.05, 0.01);

    /*
    itfRepeatModulator* rep1 = new cOneParamRepMod<double>(hrConf.dyna.mass, 0.05, 0.05, -0.05);
    itfRepeatModulator* rep2 = new cOneParamRepMod<double>(hrConf.foot.mass, 0.05, 0.05, -0.05);
    itfRepeatModulator* rep3 = new cOneParamRepMod<double>(hrConf.body.mass, 1, 1, -0.8);
    */

    /*
    itfRepeatModulator* rep1 = new cOneParamRepMod<double>(hrConf.dyna.mass, 0.2, 0.05, -0.05);
    itfRepeatModulator* rep2 = new cOneParamRepMod<double>(hrConf.foot.mass, 0.2, 0.05, -0.05);
    itfRepeatModulator* rep3 = new cOneParamRepMod<double>(hrConf.body.mass, 3.4, 1, -0.8);
    cMultiRepMod* mRep = new cMultiRepMod();
    mRep->regist(rep1);
    mRep->regist(rep2);
    mRep->regist(rep3);
    itfRepMod = mRep;
    */

    //pRepMod = new cNoParamRepMod(1);

    // timed param Changing Agent
    cLinearFunc_changeDuty_keepST_tpa::paramSet prm2(hcConf.oscConf[0].dutyRate, hcConf.oscConf[1].dutyRate, hcConf.oscConf[2].dutyRate, hcConf.oscConf[3].dutyRate, hcConf.oscConf[4].dutyRate, hcConf.oscConf[5].dutyRate,
                                                           hcConf.oscConf[0].freq, hcConf.oscConf[1].freq, hcConf.oscConf[2].freq, hcConf.oscConf[3].freq, hcConf.oscConf[4].freq, hcConf.oscConf[5].freq);
    //pTCPA_sub1 = new cLinearFunc_changeDuty_keepST_tpa(prm2, 10, 0.5, 0.0);
    //pTCPA_sub2 = new cLinearFunc_changeDuty_keepST_tpa(prm2, 10, 0.5, 0.00025);
    //pTCPA = new cCombineTwoAgents_tpa(pTCPA_sub1, pTCPA_sub2, 100.);

    pTCPA_sub1 = new cLinearFunc_1prm_tpa(hrConf.y_trans_center, 0., 0.001, "cogPos");

    // oneProcSimulation class
    //find_and_stab_onePSim = new findSolution_analyzeStability(fsConf, true, true, true, false, DETAIL_FILE_POS, FILE_NAME);
    //findSol_onePsim = new findSolution(fSolConf, false, true, true, true, true, DETAIL_FILE_POS, FILE_NAME);
    //runLog_onePSim = new runLogSimulation(rlConf, pTCPA, 900, true, true, DETAIL_FILE_POS, FILE_NAME, 10);
    //runLog_onePSim = new runLogSimulation(rlConf, pTCPA_sub1, 100, true, true, DETAIL_FILE_POS, FILE_NAME, 10);

    findSol_onePsim = new findSolution(fSolConf, true, true, true, true, false, DETAIL_FILE_POS, FILE_NAME);

    // simulation class
    //repAnalSim = new repeatAnalysis<HexabotController>( (itfOneProcSimulation<HexabotController>&)(*find_and_stab_onePSim), (itfRepeatModulator&)(*pRepMod) );
    repAnalSim = new repeatAnalysis<HexabotController>( (itfOneProcSimulation<HexabotController>&)(*findSol_onePsim), (itfRepeatModulator&)(*pRepMod) );
    //repAnalSim = new repeatAnalysis<HexabotController>( (itfOneProcSimulation<HexabotController>&)(*runLog_onePSim), (itfRepeatModulator&)(*pRepMod) );

    itfSim = repAnalSim;
    //itfSim = runLog_onePSim;

}

// Destructor
ThisSim::~ThisSim(){
    /*
    if(repAnalSim) delete repAnalSim;
    if(find_and_stab_onePSim) delete(find_and_stab_onePSim);
    if(findSol_onePsim) delete(findSol_onePsim);
    if(runLog_onePSim) delete(runLog_onePSim);
    if(pRepMod) delete(pRepMod);
    if(pTCPA) delete(pTCPA);
    if(pTCPA_sub1) delete(pTCPA_sub1);
    if(pTCPA_sub2) delete(pTCPA_sub2);
    */
}

// log configs
void ThisSim::logConfigs(bool logFlag_){
    // without log
    if(!logFlag_) return;

    // log mode
    std::stringstream str;
    str << DETAIL_FILE_POS << "Configs_" << FILE_NAME << ".dat";
    paramOfs.open(str.str());


        paramOfs << "# This is the file contains the all config parameters of the robot and controller" << std::endl;
        paramOfs << "# Main.Ver :: " << VERSION_MAIN << ",  Cntr.Ver :: " << VERSION_HCON << ",  Robot.Ver :: " << VERSION_HPA << std::endl;
        paramOfs << std::endl;

        paramOfs << "***** Ssimulation ****************************" << std::endl;
        paramOfs << " SimFreq : " << S_SIMULATION_FREQ << ",  CtrFreq : " << S_CONTROL_FREQ << std::endl;
        paramOfs << std::endl;

        paramOfs << "***** Hexabot robot **************************" << std::endl;
        paramOfs << " size : " << std::endl;
        paramOfs << "   body length : " << hrConf.body.length << ", body width : "<< hrConf.body.width << ", y_TC_TC : " << hrConf.jLength.length_y_TCJ_to_TCJ << ", x_centr_TC : " << hrConf.jLength.length_x_center_to_TCJ << std::endl;
        paramOfs << "   joint length (sholder)1-2-3(foot): " <<     hrConf.jLength.length_TCJ_to_CTJ << ",  " << hrConf.jLength.length_CTJ_to_FTJ <<",  "<< hrConf.jLength.length_FTJ_to_Toe << std::endl;
        paramOfs << " mass : " << std::endl;
        paramOfs << "   whole : "<< hrConf.wholeMass << ", body : " << hrConf.body.mass + hrConf.dyna.mass * 6. << ", joint 1-2-3 : " <<  hrConf.dyna.mass << ", " <<  hrConf.dyna.mass << ", "  << hrConf.foot.mass <<  std::endl;
        paramOfs << " COM : " << "center" << std::endl;
        paramOfs << std::endl;

        paramOfs << "***** Ground **************************" << std::endl;
        paramOfs << "   stiffness k,  damp ratio : " << sParam.stiff_of_ground << ", " << sParam.damp_of_ground  << std::endl;
        paramOfs << std::endl;

        paramOfs << "***** Hexabot controller **************************" << std::endl;
        paramOfs << " General Param " << std::endl;
        paramOfs << "   PD gain P-D-I : " << hrConf.servoParam.power << ", " << hrConf.servoParam.damp << ", " << hrConf.servoParam.integ << std::endl;
        paramOfs << "   Freq - Duty   : " << hcConf.oscConf[0].freq << ", " << hcConf.oscConf[0].dutyRate  << std::endl;
        paramOfs << "   Inhibi Coef   : " << hcConf.oscConf[0].inhibiCoef << std::endl;
        paramOfs << "   Left and Right CPGs Relation Coef : " << hcConf.twoCpgCoef << std::endl;

        paramOfs << " Phase Modulation  " << std::endl;
        paramOfs << "   Modulation Mode : " << hcConf.PC_MODE << "  (0:Phasereset + inhibi, 1:Phasereset only on swing phase, 2:PhaseReset+PhaseReset, 3:Inhibi+inhibi )" << std::endl;
        paramOfs << "   Modulation Use one side : " << hcConf.is_oneSidePhaseModulation << std::endl;

        paramOfs << " Leg Trajectory" << std::endl;
        paramOfs << "   stance length : " << hcConf.trjConf[0].walkLength <<
                    ", swing height : " << hcConf.trjConf[0].swingHeight << std::endl;

        paramOfs << std::endl;
        paramOfs << "***** One proc simulators **************************" << std::endl;
        paramOfs << "*****    Find and stabAnaly  **************************" << std::endl;
        paramOfs << " Poincare section :  Leg:" << fsConf.findFpConf.fConf.poincarePhaseLegNum << ",  Phase:" << fsConf.findFpConf.fConf.poincrePhase << std::endl;
        paramOfs << " Accuracy of solution : " <<   fsConf.findFpConf.fConf.phaseThreshold << std::endl;
        paramOfs << " Diff in Jacobian calc : " << fsConf.stabAnalyConf.delta << std::endl;
        paramOfs << "*****    Find Solutions  **************************" << std::endl;
        paramOfs << " Poincare section :  Leg:" << fSolConf.findFpConf.fConf.poincarePhaseLegNum << ",  Phase:" << fSolConf.findFpConf.fConf.poincrePhase << std::endl;
        paramOfs << " Accuracy of solution : " <<   fSolConf.findFpConf.fConf.phaseThreshold << std::endl;
        paramOfs << " Initial val : ( " << fSolConf.initialState.x() << ", " <<fSolConf.initialState.y() << ")" << std::endl;
        paramOfs << "*****    Run log simulation  **************************" << std::endl;
        paramOfs << " Poincare section :  Leg:" << rlConf.poincarePhaseLegNum << ",  Phase:" << rlConf.poincarePhase << std::endl;
        paramOfs << " Initial val : ( " << rlConf.initialState.x() << ", " << rlConf.initialState.y() << ")" << std::endl;

      if(paramOfs.is_open()) paramOfs.close();

    return;
}

// create robot and controller
void ThisSim::create_robot_and_controller(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global){
    // The definition of the values
    AbstractWiring* wiring;
    OdeAgent* agent;

    // make robot
    // make wireing
    wiring = new One2OneWiring(new ColorUniformNoise(0.0));

    // make controller
    std::ostringstream ost2;
    ost2 << "hexabot" ;
    std::string hexName = ost2.str();
    char hexName2[30];
    strcpy(hexName2,hexName.c_str());

    tController = new HexabotController(hexName2, hrConf, hcConf, false);//  TestController::TestController(confy); // debug
    controller = tController;

    // make robot
    // Put the robot on initial position
    //    this position is important because by this position, the connection relation is determined.
    hexabot = new Hexabot(odeHandle, osgHandle, hrConf, hexName2);
    hexabot->setColor(Color(1.0,1.0,0));
    hexabot->place( osg::Matrix::rotate(0., osg::Vec3(1, 0, 0)) * osg::Matrix::translate( 0., 0., 0.1));

    // make agent
    agent = new OdeAgent(global);
    agent->init(controller, hexabot, wiring);

    // Create Joint
    // create a fixed joint to hold the robot in the air at the beginning SUSPEND
    envFixator = new lpzrobots::FixedJoint(hexabot->getMainPrimitive(),global.environment);
    envFixator->init(odeHandle, osgHandle, false);

    // make connection and something
    global.configs.push_back(hexabot);
    global.configs.push_back(controller);
    global.agents.push_back(agent);
}


// starting function (executed once at the beginning of the simulation loop)
void ThisSim::start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
{
    // simulation start
    itfSim->start_simulation();

    // Setting of the simulation base ===========================================
    //setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));
    setCameraHomePos(
        lpzrobots::Pos(-0.0114359, 1.66848, 0.922832),
        lpzrobots::Pos(0.866, -0.43884, 0)
    );

    // initialization
    // - set noise to 0.
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)
    global.odeConfig.noise=0.0;
    global.odeConfig.setParam("controlinterval", (int)(S_SIMULATION_FREQ/S_CONTROL_FREQ)); // 20 Hz control step
    global.odeConfig.setParam("simstepsize", 1./(double)S_SIMULATION_FREQ); // 100Hz simulation step

    // make surface
    OdeHandle surfaceHandle(odeHandle);
    const Substance surfaceSubstance(3.0, 0., sParam.stiff_of_ground, sParam.damp_of_ground);//k100 0.5 // k40 d0.5 // k2000, d0.7 mu 3, slip 0, k 500, dump 0.1 set
    surfaceHandle.substance = surfaceSubstance;

    OctaPlayground* playground = new OctaPlayground(surfaceHandle, osgHandle, osg::Vec3(100, 0., 0.2), 12);
    playground->setPosition(osg::Vec3(0,0,0));
    global.obstacles.push_back(playground);

    // update configs before making robots
    itfSim->request_updateConfigs();

    // log configs
    logConfigs(true);

    // create robot and controller
    create_robot_and_controller(odeHandle, osgHandle, global);

    // High speed mode
    if(S_HIGHSPEED_MODE){
        // stop movie and speed up
        this->noGraphics = true;
        globalData.odeConfig.setParam("realtimefactor", 50);
    }

    STATUS = WAIT_INITIALIZE;

    // for debug
#ifdef DEBUG_MAIN
    std::cout << "@@ DEBUG @@ Start the simulation " << std::endl;
#endif

}

//Function die eingegebene Befehle/kommandos verarbeitet
bool ThisSim::command (const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
{
    if (!down) return false;
    bool handled = false;
    FILE* f;
    switch ( key )
    {
      case 's' :
          f=fopen("controller","wb");
          controller->store(f) && printf("Controller stored\n");
          fclose(f);
          handled = true; break;
      case 'l' :
          f=fopen("controller","rb");
          controller->restore(f) && printf("Controller loaded\n");
          handled = true; break;
      case 'x':
        if (envFixator) {
          std::cout << "dropping robot" << std::endl;
          delete envFixator;
          envFixator = NULL;
        }
        break;

        fclose(f);
    }

    fflush(stdout);
    return handled;
}

// it is called once when "simulation_time_reached" is true
bool ThisSim::restart(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global){

    // restart situtaion
    if( itfSim->isFinished_simulation(simNum)){

        // finish simulation
        itfSim->finish_simulation(simNum);
        std::cout << "Simulation has finished!!  Step:" << simNum <<  std::endl;

        // finish procedure
        // nothing
        return false;
    }

    // restart condition

    //  configs update situation
    itfSim->request_updateConfigs();
    simNum++;

    // COUT
    std::cout<<">>> RESTART > > > >  Step:" << simNum <<std::endl;

    // Restart Session ########################################################################################
    //////// DELETE >>>>>>>>>>>>>>>>>>>>>>
    // Now we must delete all robots, controllers and agents from the simulation and create new robots and agents.
    while (global.agents.size() > 0)
    {
        // search the object
        OdeAgent* agent = (*global.agents.begin());
        AbstractController* controller = agent->getController();

        // I could not understand what they did in this discription
        //  Maybe they delete the config param by serching which concerned to this controller
        global.configs.erase(std::find(global.configs.begin(),global.configs.end(), controller));

        // Delete each objects
        // if I call it, it deletes everything which concerns
        delete (agent);

        // also about global agent
        global.agents.erase(global.agents.begin());
    }

    ///// Recreate Robot >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // create robot and controller
    create_robot_and_controller(odeHandle, osgHandle, global);

    // start the simulation
    itfSim->start_oneProc(simNum, *tController);
    STATUS = WAIT_INITIALIZE;

    // for debug
#ifdef DEBUG_MAIN
    std::cout << "@@ DEBUG @@ Restart the simulation " << std::endl;
#endif

    // restart!
    return true;
}

// it is called in every simulation step
void ThisSim::addCallback(GlobalData& globalData, bool draw, bool pause, bool control)
{
    // for demonstration: set simsteps for one cycle to 60.000/currentCycle (10min/currentCycle)
    // if simulation_time_reached is set to true, the simulation cycle is finished
    if (globalData.sim_step % (int)(S_SIMULATION_FREQ/S_CONTROL_FREQ) == 1 || S_SIMULATION_FREQ == S_CONTROL_FREQ)// every control step + a little(this is for waiting for getting data)
    {
        // get time
        int nowStep = globalData.sim_step / (int)(S_SIMULATION_FREQ/S_CONTROL_FREQ);
        double time = (double)nowStep * (double)(1./ (double)S_CONTROL_FREQ);

        //Time for the simulation
        // do something depending on situation
        if(STATUS == IDLE){
            //nothing
        }
        else if(STATUS == WAIT_INITIALIZE){
            // wait for PD control converging
            if(time > sParam.time_wait_in_flight){
                // fall robot
                if (envFixator) {
                    std::cout << "dropping robot" << std::endl;
                    delete envFixator;
                    envFixator = NULL;
                 }
            }
            if(time > sParam.time_wait_in_flight + sParam.time_wait_on_ground){
                // change status to do simulation
                STATUS = PROC_SIMULATION;

                // simulation start
                itfSim->start_oneProc(simNum, *tController);
                startTime = time;
            }
        }
        else if(STATUS == PROC_SIMULATION){
            // proc the simulation
            itfSim->calcStep(*tController, time - startTime);

            // detect the finish
            if(itfSim->isFinished_oneProc(*tController, time - startTime)){
                STATUS = IDLE;
                // stop the simulation
                itfSim->finish_oneProc(*tController, time - startTime);
                simulation_time_reached = true;
                std::cout << " ----> finish one Proc simulation" << std::endl;
            }
        }

    }
    /***********************************************************************************/
}

void ThisSim::bindingDescription(osg::ApplicationUsage & au) const {
    au.addKeyboardMouseBinding("Teachung: t","toggle mode");
    au.addKeyboardMouseBinding("Teaching: u","forward");
    au.addKeyboardMouseBinding("Teaching: j","backward");
    au.addKeyboardMouseBinding("Simulation: s","store");
    au.addKeyboardMouseBinding("Simulation: l","load");
}

int main (int argc, char **argv)
{
  ThisSim sim;
  // run simulation
  return sim.run(argc, argv) ? 0 : 1;
}
