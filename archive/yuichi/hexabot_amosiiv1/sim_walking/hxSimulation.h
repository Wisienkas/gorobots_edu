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
1, hexapod simulation 04

THis is the version to calculate fixed point and analise stability

If we set the initial value, this simulation can easily find the fixed point and evaluate stability by using poincre section

This is the program for hexapod robot
 I apply simple oscillator and make a program to repeat simulation.

*/

#ifndef __HX_SIMULATION_H
#define __HX_SIMULATION_H


#include <sstream>
#include <iostream>

#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>
#include <ode_robots/octaplayground.h>
#include <ode_robots/passivesphere.h>

#include <selforg/invertmotornstep.h>
#include <selforg/invertmotorspace.h>
#include <selforg/invertnchannelcontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/stl_adds.h>
#include <array>

#include "ode_robots/hexabot.h"
#include "hexabotController.h"
#include "amosControllerAdptHexabot.hpp"
#include "hxPeriodAnalysis.h"
#include "itfHexaSimulation.h"

#include "findsolution_analyzestability.hpp"
#include "findSolution.hpp"

#include "runLogSimulation.hpp"
#include "timedChangePrmAgts.hpp"

#include "repeatAnalysis.hpp"
#include "repeatModulators.hpp"

#include <math.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace HEXABOT;
//using namespace amosII_pid;


class ThisSim : public Simulation {
public:
    // simulation parameter
    typedef struct{
		// parameter fo simulation 
		//wait in the flight to converge PD deviation (s)
		int time_wait_in_flight;
		// wait on the ground to converge PD deviation before start
		int time_wait_on_ground;

        // the stifness of the ground k
        double stiff_of_ground;
        // the damping ratio of the ground gamma
        double damp_of_ground;
				
    }SimulationParam;

protected:

    // Simulation and control frequency
    static const int S_SIMULATION_FREQ; //
    static const int S_CONTROL_FREQ; //
    static const bool S_HIGHSPEED_MODE;
    

    // base system
    itfSimulation<HexabotController> *itfSim; // just refarence

    // system which we use
    repeatAnalysis<HexabotController> *repAnalSim;

    //One proc simulators ***************************
     // find and stab analy
     findSolution_analyzeStability *find_and_stab_onePSim;
     // find solution and log
     findSolution *findSol_onePsim;
     // runLogSimulation
     runLogSimulation *runLog_onePSim;

    //Repeat modulators ****************************
     itfRepeatModulator *pRepMod;
     //itfRepeatModulator *itfRepMod; // just refarence

    //TimedChangePram ******************************
     itfTimedChangePrmAgent *pTCPA;
     itfTimedChangePrmAgent *pTCPA_sub1;
     itfTimedChangePrmAgent *pTCPA_sub2;

     //itfTimedChangePrmAgent *itfTCPA; // just refarence

    // robot
    //Hexabot* hexabot;
    AmosII* amosii;

    // controller
    AbstractController* controller;
    HexabotController* tController;
    AmosController_adptHexabot* aController;

    // joint to fix
    lpzrobots::Joint* envFixator;

    // configs
    HexabotControllerConf hcConf;
    //HexabotConf hrConf;
    AmosIIConf amConf;

    findSolution_analyzeStability::findSolAnalyStabConf fsConf;
    findSolution::findSolConf fSolConf;
    runLogSimulation::runLogSimConf rlConf;
    
    // simulation count
	int simNum;
	
	// simulatio start time
	double startTime;
	
    // simulation param
    SimulationParam sParam;

    // file out
    std::ofstream paramOfs;


    //STATUS
    enum STATUS_ {IDLE, WAIT_INITIALIZE, PROC_SIMULATION, ERROR};
    enum STATUS_ STATUS;

public:
  // Constructor
  ThisSim();
  // Destructor
  virtual ~ThisSim();

private:
  // set conf file (pDiff[Num - 1])
  void setConfigs(void);
  
  // create robot
  void create_robot_and_controller(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global);

  // log Configs
  void logConfigs(bool logFlag_);

public:
  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global);

  //Function die eingegebene Befehle/kommandos verarbeitet
  virtual bool command (const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down);

  // it is called once when "simulation_time_reached" is true
  virtual bool restart(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global);

  // it is called in every simulation step
  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control);

  virtual void bindingDescription(osg::ApplicationUsage & au)const ;
 };


#endif
