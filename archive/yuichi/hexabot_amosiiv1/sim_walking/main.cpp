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
1, hexapod simulation 09

THis is the version to calculate fixed point and analise stability

I changed the program so much to deal it as object.

20140517 ver 2.00
 This version is for the amos ii robot in the simulator

20140527 ver 2.01
 This version is for the amos ii robot controlled by PD in the simulator

20140605 ver 2.10
 This version is for the amos ii robot controlled by PD in the simulator
 and the robot configulation is different. (AmosIIv1)

20140623 ver 2.20
 HexabotController is changed to deal with phase resetting

+++++++++++++++++++++++++++++++++++++++++++++++
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
#define VERSION_MAIN "Main_amos_Ver 2.20_20140623"

#include <osg/Matrix>
#include "hxSimulation.h"


// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace HEXABOT;
//using namespace amosII_pid;

// debug mode
#define DEBUG_MAIN 1

// about simulation
const int ThisSim::S_SIMULATION_FREQ = 1600;//3200; // 3200 in thesis
const int ThisSim::S_CONTROL_FREQ = 25;//3200; // 3200 in thesis

// about file
#define DETAIL_FILE_POS  "//home//ambe//Dropbox//AshigaruShare//hexData//amos//amosPrm//"
//#define FILE_NAME "H_leg_sw006_ST07_i05_1"
//#define FILE_NAME "H_leg_sw006_ST05_d06_inhibi_2"
#define FILE_NAME "test2" //"leg_sw010_ST10_d06_PD10_all_ver2"

// high speed mode : The movie is stopped and it gains simulation speed!!
const bool ThisSim::S_HIGHSPEED_MODE = false; //true;// false;

// configs setting
void ThisSim::setConfigs(void){
    // set configuration of the parameters
    //double phase1 = 0.;
    //double phase2 = M_PI;

    //======= Robot ==============================================
    // using amosii version 1
    //amConf = AmosII::getDefaultConf(1., 1, 1, 1);
    amConf = AmosII::getAmosIIv1Conf();
    amConf.rubberFeet = true;

    amConf.backPower = 200;
    amConf.backDamping = 0.001;
    amConf.coxaPower = 100;//200
    amConf.coxaDamping = 0.008; // 0.001
    amConf.secondPower = 10; // 200
    amConf.secondDamping = 0.003; // 0.001
    amConf.tebiaPower = 10; //200
    amConf.tebiaDamping = 0.003; //0.001
    //amConf.footPower = 100;

    /*
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
    hrConf.x_trans_center = 0.;
    hrConf.y_trans_center = 0.; // if the value is positive, a body will shift back

    */

    // ======= Controller ==================================================
    hcConf = HexabotController::getDefaultConf();

    // set the Freq, Duty rate and inhiibi Coef
    for(int i = 0; i < 6; i++){
        hcConf.oscConf[i].freq = 1./ (5. / 0.4);//1./ (3. / 0.4);
        hcConf.oscConf[i].dutyRate = 0.6;
        hcConf.oscConf[i].inhibiCoef = 1;//1.; //0.5 1.5; //1.5; //0.9
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
    hcConf.PC_MODE = PC_PR_LPASS;//PC_PR_LPASS;//PC_IN_IN; // PC_PR_PR
    hcConf.TFactor = 1.;

    // to use one side phase modulation or not
    hcConf.is_oneSidePhaseModulation = false;//true; //true;//false;

    // set for poincre analysis
    //  the time when we start the phase reset mechanism after start the simulation.
    hcConf.biasStartTime = 3.;

    // set the leg trajectory
    bool is_leg_upDown = true;
    if(is_leg_upDown){
        for(int i=0;i<6;i++){
            hcConf.trjConf[i].walkLength = 0.06; //0.06;//0.04;
            hcConf.trjConf[i].centerTrj = osg::Vec3d( 0.11 , 0. , -0.10);
            hcConf.trjConf[i].swingHeight = 0.04;
        }
        hcConf.trjConf[1].centerTrj = osg::Vec3d( 0.11 , -0.02, -0.10);
        hcConf.trjConf[4].centerTrj = osg::Vec3d( 0.11 , 0.02, -0.10);
    }


    // ====== Modules =============================================================

    //***** Find solution and stab analy ***********************************:
    fsConf = findSolution_analyzeStability::getDefaultConf();
    // accuracy of the solution
    fsConf.findFpConf.fConf.phaseThreshold = 0.001;//0.001
    // delta to calc jacobian
    fsConf.stabAnalyConf.delta = 0.1;
    fsConf.stabAnalyConf.initialPhaseDiff = 0.1;
    fsConf.stabAnalyConf.is_use_calcJacobian_by_flow = false;

    fsConf.findFpConf.fConf.maxPeriodNum = 150;
    fsConf.findFpConf.fConf.waitPeriodNum = 40; //50;

    //fsConf.findFpConf.fConf.poincrePhase = 30./20.*M_PI;
    //fsConf.stabAnalyConf.poincrePhase = 30./20.*M_PI;

    // initial value to search
    //fsConf.initialStateVec.push_back(osg::Vec2d(3.12117, 3.17914)); // lower : duty 0.50
    //fsConf.initialStateVec.push_back(osg::Vec2d(3., 3.)); // lower : duty 0.50
    //fsConf.initialStateVec.push_back(osg::Vec2d(2.41644, 2.43121)); // lower : duty 0.6
    //fsConf.initialStateVec.push_back(osg::Vec2d(2.28563, 2.20201)); // lower : duty 0.64
    //fsConf.initialStateVec.push_back(osg::Vec2d(2.08, 2.09)); // lower : duty 0.66

    //fsConf.initialStateVec.push_back(osg::Vec2d(3.20, 3.20)); // lower : duty 0.50
    fsConf.initialStateVec.push_back(osg::Vec2d(3.87902, 3.88851)); // higher : duty 0.6
    //fsConf.initialStateVec.push_back(osg::Vec2d(3.7, 3.7)); // higher 2nd sol?? : duty 0.6
    //fsConf.initialStateVec.push_back(osg::Vec2d(3.869, 4.14)); // higher 2nd sol?? : duty 0.6
    //fsConf.initialStateVec.push_back(osg::Vec2d(4.21, 4.187)); // lower : duty 0.66




    //******* Find Solution *******************************
    fSolConf = findSolution::getDefaultConf();
    fSolConf.findFpConf.fConf.phaseThreshold = 0.001;
    fSolConf.findFpConf.fConf.waitPeriodNum = 40;
    fSolConf.findFpConf.fConf.maxPeriodNum = 120;
    //fSolConf.findFpConf.fConf.maxSimulationTime = 100;

    fSolConf.initialState = osg::Vec2d(2.4, 2.4);

    //******* RunLogSimulation **************************
    rlConf = runLogSimulation::getDefaultConf();
    rlConf.poincarePhase = 0.;
    rlConf.waitTime = 30;

    rlConf.initialState = osg::Vec2d(2.4, 2.4);

    // ====== Simulation ============================================================
    sParam.time_wait_in_flight = 20.; //wait in the flight to converge PD deviation (s)
    sParam.time_wait_on_ground = 20.; // wait on the ground to converge PD deviation before start
    sParam.stiff_of_ground = 1000.; //1000.; // stiffness of the ground
    sParam.damp_of_ground = 0.45; // damping ration of the ground
}



// Constructor
ThisSim::ThisSim()
    :Simulation(), itfSim(0), repAnalSim(0), find_and_stab_onePSim(0), findSol_onePsim(0),
      runLog_onePSim(0), pTCPA(0), pTCPA_sub1(0), pTCPA_sub2(0),
      amosii(0), controller(0), tController(0), aController(0), pRepMod(0), simNum(1), paramOfs()
{
    // determine configs
    setConfigs();

    // set the simulation program
    //   param changing class
    //pRepMod = new cOneArray6ParamRepMod<double>(hcConf.oscConf[0].inhibiCoef, hcConf.oscConf[1].inhibiCoef, hcConf.oscConf[2].inhibiCoef, hcConf.oscConf[3].inhibiCoef,
    //                                             hcConf.oscConf[4].inhibiCoef, hcConf.oscConf[5].inhibiCoef, 0.9, 0.1, -0.1);


    // param chaniging class
    cChangeDuty_KeepSwingDuration_RepMod::paramSet prm(hcConf.oscConf[0].dutyRate, hcConf.oscConf[1].dutyRate, hcConf.oscConf[2].dutyRate, hcConf.oscConf[3].dutyRate, hcConf.oscConf[4].dutyRate, hcConf.oscConf[5].dutyRate,
                                                       hcConf.oscConf[0].freq, hcConf.oscConf[1].freq, hcConf.oscConf[2].freq, hcConf.oscConf[3].freq, hcConf.oscConf[4].freq, hcConf.oscConf[5].freq);

    //pRepMod = new cChangeDuty_KeepSwingDuration_RepMod(prm, 5, 0.6, 0.7, 0.01);
    //pRepMod = new cChangeDuty_KeepSwingDuration_RepMod(prm, 5, 0.6, 0.5, -0.01);

    //pRepMod = new cChangeIniPhase_RepMod(fSolConf.initialState, osg::Vec2d(2.4, 2.4), osg::Vec2d(0.8, 0.8), osg::Vec2d(0.628, 0.628));

    //pRepMod = new cOneParamRepMod<double>(fsConf.stabAnalyConf.delta, 0.001, 0.1, 0.01);
    //pRepMod = new cOneParamRepMod<double>(hrConf.servoParam.power, 300, 50, -50);

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
    std::vector<osg::Vec2d> iniPhases;
    iniPhases.push_back(osg::Vec2d(4., 4.));
    iniPhases.push_back(osg::Vec2d(2.4, 2.4));
    iniPhases.push_back(osg::Vec2d(3.14, 3.14));
    //iniPhases.push_back(osg::Vec2d(2.4, 4.));
    //iniPhases.push_back(osg::Vec2d(4., 2.4));
    // iniPhases.push_back(osg::Vec2d(2.12805, 2.37908));
    // iniPhases.push_back(osg::Vec2d(2., 2.));
    // iniPhases.push_back(osg::Vec2d(1.6, 2.4));
    // iniPhases.push_back(osg::Vec2d(1.6, 1.6));
    // iniPhases.push_back(osg::Vec2d(2.4, 1.6));
    pRepMod = new cChangeIniPhase_arbitally_RepMod(rlConf.initialState, iniPhases);
    //pRepMod = new cChangeIniPhase_RepMod(rlConfinitialState, osg::Vec2d(0.627, 0.627), osg::Vec2d(6.28, 6.28), osg::Vec2d(0.628, 0.628));

    // timed param Changing Agent
    cLinearFunc_changeDuty_keepST_tpa::paramSet prm2(hcConf.oscConf[0].dutyRate, hcConf.oscConf[1].dutyRate, hcConf.oscConf[2].dutyRate, hcConf.oscConf[3].dutyRate, hcConf.oscConf[4].dutyRate, hcConf.oscConf[5].dutyRate,
                                                           hcConf.oscConf[0].freq, hcConf.oscConf[1].freq, hcConf.oscConf[2].freq, hcConf.oscConf[3].freq, hcConf.oscConf[4].freq, hcConf.oscConf[5].freq);
    pTCPA_sub1 = new cLinearFunc_changeDuty_keepST_tpa(prm2, 2, 0.6, 0.0);
    //pTCPA_sub2 = new cLinearFunc_changeDuty_keepST_tpa(prm2, 7, 0.6, 0.0002);
    //pTCPA = new cCombineTwoAgents_tpa(pTCPA_sub1, pTCPA_sub2, 400.);

    // oneProcSimulation class
    //find_and_stab_onePSim = new findSolution_analyzeStability(fsConf, true, true, true, true, DETAIL_FILE_POS, FILE_NAME);
    //findSol_onePsim = new findSolution(fSolConf, false, true, true, true, true, DETAIL_FILE_POS, FILE_NAME);
    runLog_onePSim = new runLogSimulation(rlConf, pTCPA_sub1, 500, true, false, DETAIL_FILE_POS, FILE_NAME, 10);

    // simulation class
    //repAnalSim = new repeatAnalysis<HexabotController>( (itfOneProcSimulation<HexabotController>&)(*find_and_stab_onePSim), (itfRepeatModulator&)(*itfRepMod) );
    //repAnalSim = new repeatAnalysis<HexabotController>( (itfOneProcSimulation<HexabotController>&)(*findSol_onePsim), (itfRepeatModulator&)(*itfRepMod) );
    repAnalSim = new repeatAnalysis<HexabotController>( (itfOneProcSimulation<HexabotController>&)(*runLog_onePSim), (itfRepeatModulator&)(*pRepMod) );

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

        paramOfs << "***** Amos robot **************************" << std::endl;
        HexabotConf hrConf = AmosController_adptHexabot::changeConf(amConf);
        paramOfs << "Amos Version :" << amConf.amos_version << std::endl;
        paramOfs << " size : " << std::endl;
        paramOfs << "   body length : " << amConf.size + amConf.frontLength << ", body width : "<< amConf.width << ", y_TC_TC : " << hrConf.jLength.length_y_TCJ_to_TCJ << ", x_centr_TC : " << hrConf.jLength.length_x_center_to_TCJ << std::endl;
        paramOfs << "   joint length (sholder)1-2-3(foot): " <<     hrConf.jLength.length_TCJ_to_CTJ << ",  " << hrConf.jLength.length_CTJ_to_FTJ <<",  "<< hrConf.jLength.length_FTJ_to_Toe << std::endl;
        paramOfs << " mass : " << std::endl;
        paramOfs << "   whole : "<< hrConf.wholeMass << ", body : " << amConf.trunkMass + amConf.frontMass << ", joint 1-2-3 : " <<  amConf.coxaMass << ", " <<  amConf.secondMass << ", "  << amConf.tebiaMass <<  std::endl;
        paramOfs << " COM : " << "center" << std::endl;
        paramOfs << std::endl;

        paramOfs << "***** Ground **************************" << std::endl;
        paramOfs << "   stiffness k,  damp ratio : " << sParam.stiff_of_ground << ", " << sParam.damp_of_ground  << std::endl;
        paramOfs << std::endl;

        paramOfs << "***** Hexabot controller **************************" << std::endl;
        paramOfs << " General Param " << std::endl;
        //paramOfs << "   PD gain P-D-I : " << hrConf.servoParam.power << ", " << hrConf.servoParam.damp << ", " << hrConf.servoParam.integ << std::endl;
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
        paramOfs << " Accuracy of solution : " <<   fsConf.findFpConf.fConf.phaseThreshold << std::endl;
        paramOfs << " Diff in Jacobian calc : " << fsConf.stabAnalyConf.delta << std::endl;
        paramOfs << "*****    Find Solutions  **************************" << std::endl;
        paramOfs << " Accuracy of solution : " <<   fSolConf.findFpConf.fConf.phaseThreshold << std::endl;
        paramOfs << " Initial val : ( " << fSolConf.initialState.x() << ", " <<fSolConf.initialState.y() << ")" << std::endl;
        paramOfs << "*****    Run log simulation  **************************" << std::endl;
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
    ost2 << "amosii" ;
    std::string hexName = ost2.str();
    char hexName2[30];
    strcpy(hexName2,hexName.c_str());

    aController = new AmosController_adptHexabot(hexName2, amConf, hcConf, false, DETAIL_FILE_POS, FILE_NAME, simNum, false);
    //tController = new HexabotController(hexName2, hrConf, hcConf, false);//  TestController::TestController(confy); // debug
    tController = aController;
    controller = aController;

    // make robot
    // Put the robot on initial position
    //    this position is important because by this position, the connection relation is determined.
    amosii = new AmosII(odeHandle, osgHandle, amConf, hexName2);
    amosii->setColor(Color(1.0,1.0,0));
    amosii->place( osg::Matrix::rotate(0., osg::Vec3(1, 0, 0)) * osg::Matrix::translate( 0., 0., 0.2));

    // make agent
    agent = new OdeAgent(global);
    agent->init(controller, amosii, wiring);

    // Create Joint
    // create a fixed joint to hold the robot in the air at the beginning SUSPEND
    envFixator = new lpzrobots::FixedJoint(amosii->getMainPrimitive(),global.environment);
    envFixator->init(odeHandle, osgHandle, false);

    // make connection and something
    global.configs.push_back(amosii);
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
