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
6, template ashigaru6
 This is the program for straight connected ashigaru
 I apply simple oscillator and make a program to repeat simulation.
*/

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

#include "ode_robots/ashigaru.h"
#include "ashigaruController.h"

#define Num 3


// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace ASHIGARU;


class ThisSim : public Simulation {
protected:
	// high speed mode : The movie is stopped and it gains simulation speed!!
	static constexpr bool HIGHSPEED_MODE = false;
	// each simulation time (s)
	static constexpr double SIMULATION_TIME = 1200.;
	// evaluate step numbers
	//  we will average each value for this period from the last logged data, and check convergence
	static constexpr int EVALUATE_STEP = 10;
	// detect Conv threshold
	//  If the difference between above averaged value and sensed value is smaller than this threshold,
	//  we regard that system converges!!
	static constexpr double CONVERGENCE_THRESHOLD = 0.05;

	// Duty rate change param
	static constexpr double S_DUTY_MAX = 0.6;
	static constexpr double S_DUTY_MIN = 0.6;
	static constexpr double S_DUTY_STEP = 0.1;

	// Freq change param
	static constexpr double S_FREQ_MAX = 0.3;
	static constexpr double S_FREQ_MIN = 0.3;
	static constexpr double S_FREQ_STEP = 0.05;

	// Initial Phase Difference
	static constexpr double S_PHASE_DIFF1_MIN =  M_PI;
	static constexpr double S_PHASE_DIFF1_MAX =  M_PI;
	static constexpr double S_PHASE_DIFF2_MIN =  M_PI;
	static constexpr double S_PHASE_DIFF2_MAX =  M_PI;
	static constexpr double S_PHASE_DIFF1_STEP = M_PI / 2.;
	static constexpr double S_PHASE_DIFF2_STEP = M_PI / 2.;

	static const int	S_START_NUM = 1;

	// simulation parameter
	typedef struct{
		// simulation step number
		int simNum;
		double freq;
		double duty;
		double phaseDiff[Num-1];
	}SimulationParam;
	SimulationParam sParam;

	Ashigaru* ashiUnit[Num];
//	Ashigaru* ashiUnit2;
//	Ashigaru* ashiUnit3;
//	Ashigaru* ashiUnit4;

	AbstractController* controller[Num];
	AshigaruController* tController[Num];

//	AbstractController* controller2;
//	AbstractController* controller3;
//	AbstractController* controller4;

	lpzrobots::Joint* envFixator;
	lpzrobots::Joint* robotFixator[ Num -1 ];

	AshigaruControllerConf acConf[Num];
	AshigaruConf aConf;


	// Data for phase
	typedef struct{
		// phase difference Buff (100 buffer is enough)
		// pdBuff CPG1phase[i+1] - CPG1phase[i] ... i:ashigaru num (notice: only CPG 1)
		CirBuff pdBuff;
		// meaning phase difference (it was meaned in 1 oscillation)(it is only for CPG 1)
		double meaningDPhase;
		// current phase
		double currentPhase[3];
		// OutPhase
		double outPhase[3];
		// Bias
		double bias[3];
	}phaseSet;


	// Data for analyzing
    typedef struct{
    	// oscillation frequency
    	double oscFreq[Num];

    	// sensed data
    	AshigaruController::sensedDataSet sData[Num];

    	// phase data
    	phaseSet pData[Num];

    	// whole COG pos
    	osg::Vec3d cogPos;
    	// NE-stability margin
    	double NEstability;

    	//convergence situation
    	bool localConvergence;

    }analyData;

    // class for each step analizing data storage
    analyData aData;

    // The parameter to analize the convergence and stability of resultant movement
    typedef struct{
    	// phase Difference data
    	double phaseDiff[Num-1];
    	// Ne stability data
    	double NEstability;
    	// yaw angle data
    	double Yaw;
    	// speed data
    	double speed[2];
    }evalData;

    // buffer to analize meaning data of eva
    std::list<evalData> evaList;

    // result Data
    typedef struct{
    	// averaged stability value
    	double aveStability;
    	// minimum stability value in evaluation term
    	double minStability;
    	// averaged phase difference(rad)
    	double avePhaseDiff[Num-1];
    	// maximum deviation from averaged phase difference in evaluation term(rad)
    	double convDeviation;
    	// The time which was taken to converge
    	double convTime;
    	// The resultant period
    	int period;
    	// The speed of Yaw direction(rad / s)
    	double yawSpeed;
    	// The speed to move forward(m / s)(translation motion)
    	double transSpeed;
    }resultData;

    resultData rData;


    // the value for period calculation
    typedef struct{
    	// The step number which is memorized before
    	int beforeStep;
    	// expected oscillation period
    	int oscPeriod;
    	// memorized flag
    	bool mFlag;
    }periodSet;

    // period set
    periodSet peSet;

    // logging file set
    class FileSet{
    private:
    	// phase logging
        std::ofstream phaseOfs;
        // sensor logging
        std::ofstream sensorOfs;
        // foot contact graph
        std::ofstream fContactOfs;
        // Ashigaru Parameter logging
        std::ofstream paramOfs;

    public:
        // constructor
        FileSet(){};
        // destructor
        ~FileSet(){
        	if(phaseOfs.is_open())phaseOfs.close();
        	if(sensorOfs.is_open())sensorOfs.close();
        	if(fContactOfs.is_open())fContactOfs.close();
        	if(paramOfs.is_open())paramOfs.close();
        }
        // make a file
        bool makeFile(std::string name){
        	std::string fileName =  "//home//ambe//workspace//lpzrobots//simulation//";
        	fileName += name;

        	//make file
        	// phase logging
        	std::string phaseFileName = "_phaseLog.txt";
        	phaseFileName = fileName + phaseFileName;
        	char phaseFileNameChar[150];
        	strcpy(phaseFileNameChar,phaseFileName.c_str());

        	phaseOfs.open(phaseFileNameChar);
        	std::cout << "make a file Main phase"<< std::endl;
        	phaseOfs << "# This is the Phase difference data to analyze" << std::endl;
        	phaseOfs << "# 1 time(s), "
      			  "2 Ash0 cpgPhase[1], 3 Ash1 cpgPhase[1].... "
        		//<< Num + 2 <<" Ash0 MeanCpgPhase[1]," << Num + 3 << " Ash1 MeanCpgPhase[1]..... "
				<< Num + 2 <<" Ash 1-0 MphaseDiff," << Num + 3 << " Ash 2-1 MphaseDiff..... "
				<< 2 * Num + 1 <<" Ash 1-0 phaseDiff," << 2 * Num + 2 << " Ash 2-1 phaseDiff..... " << std::endl;
        	phaseOfs << "# " << 3 * Num <<" Convergence situation(1 Yes or 0 Not), " << std::endl;
        	phaseOfs << "# " << 3 * Num + 1 <<" outPhase Ash0-0, " << 3 * Num + 2 <<" outPhase Ash0-1, " << 3 * Num + 3 <<" outPhase Ash0-3, " << 3 * Num + 4 <<" outPhase Ash1-0,..... " << std::endl;
        	phaseOfs << "# " << 6 * Num + 1 <<" bias Ash0-0, " << 6 * Num + 2 <<" bias Ash0-1, " << 6 * Num + 3 <<" bias Ash0-3, " << 6 * Num + 4 <<" bias Ash1-0,..... " << std::endl;

        	// sensor file
        	std::string sensorFileName = "_sensorLog.txt";
        	sensorFileName = fileName + sensorFileName;
        	char sensorFileNameChar[150];
        	strcpy(sensorFileNameChar,sensorFileName.c_str());

        	sensorOfs.open(sensorFileNameChar);
        	std::cout << "make a file Main sensor"<< std::endl;
        	sensorOfs << "# This is the Sensor data to analyze" << std::endl;
        	sensorOfs << "# 1 time(s), "
        				"2 Ash0 Pose-r, 3 Ash0 Pose-p, 4 Ash0 Pose-y.... " << std::endl;
        	sensorOfs << "#  " << 3*Num + 2 <<" Ash0 Vel-x,"  << 3*Num + 3 <<" Ash0 Vel-y," << 3*Num + 4 << " Ash0 Vel-z,..... "<< std::endl;
        	sensorOfs << "#  " << 6*Num + 2 <<" Ash0 footSensor 0,"  << 6*Num + 3 <<" Ash0 footSensor 1," << 6*Num + 4 << " Ash0 footSensor 2,..... "<< std::endl;
        	sensorOfs << "#  " << 9*Num + 2 <<" Ash0 gPos-x,"  << 9*Num + 3 <<" Ash0 gPos-y," << 9*Num + 4 << " Ash0 gPos-z,..... "<< std::endl;
        	sensorOfs << "#  " << 12*Num + 2 <<" Whole CogPos-x,"  << 12*Num + 3 <<" Whole CogPos-y," << 12*Num + 4 << " Whole CogPos-z,"<< std::endl;
        	sensorOfs << "#  " << 12*Num + 5 <<" Whole NEstability," << std::endl;

        	//fContact file
        	std::string fContactFileName = "_fContactLog.txt";
			fContactFileName = fileName + fContactFileName;
			char fContactFileNameChar[150];
			strcpy(fContactFileNameChar,fContactFileName.c_str());

			fContactOfs.open(fContactFileNameChar);
			std::cout << "make a file Main foot contact information"<< std::endl;
			fContactOfs << "# This is the foot contact data to analyze walking pattern(1:contact, 0:not contact)" << std::endl;
			fContactOfs << "# 1 time(s), "
						  "2 Ash0-L0, 3 Ash0-L1, 4 Ash0-L2, 5 Ash1-L0.... " << std::endl;

			//parameter file
			std::string paramFileName = "_paramtLog.txt";
			paramFileName = fileName + paramFileName;
			char paramFileNameChar[150];
			strcpy(paramFileNameChar, paramFileName.c_str());

			paramOfs.open(paramFileNameChar);
			std::cout << "make a file about parameter"<< std::endl;
			fContactOfs << "# This is the parameter settings information" << std::endl;
			fContactOfs << "#   " << std::endl;
        	return true;
        }

        // initial Logging
        bool initialLogging(AshigaruControllerConf* aConf){
			paramOfs << "This mode is straight connected mode and number of Ashigaru is " << Num <<  std::endl;
			paramOfs << "  " << std::endl;
			paramOfs << "<<<<<<<<<<<<<<< The important information >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << std::endl;
			paramOfs << " Frequency : " << aConf[0].oscConf[1].freq<<  std::endl;
			paramOfs << " Duty Rate : " << aConf[0].oscConf[1].dutyRate << std::endl;
			paramOfs << " Initial Phase Diff : ";
			for(int j =0; j<Num-1; j++){
				paramOfs << aConf[j+1].oscConf[1].initialPhase - aConf[j].oscConf[1].initialPhase << " ";
			}
			paramOfs << std::endl;
			paramOfs << std::endl;

			paramOfs << "<<<<<<<<<<<<<<< The parameter of each CPG >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << std::endl;
			for(int i = 0; i< Num;i++){
				  paramOfs << "<<<<<<<<<<<<<<< Information of Controller "<< i << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"<< std::endl;
				  paramOfs << " Type    : " << "Simplified CPG" << std::endl;
				  paramOfs << " CnctType: " << "Straight Connect" << std::endl;
				  paramOfs << " ControllFrequency(Hz): " << aConf[i].controlFreq << std::endl;
				  paramOfs << " Bias start time(s): " << aConf[i].biasStartTime << std::endl;
				  paramOfs << " TwoCPGcnctCoef: " << aConf[i].twoCpgCoef << ", TwoCPGPhaseDiff: " << aConf[i].twoCPGAngleDiff <<  std::endl;

				  paramOfs << "" << std::endl;
				  paramOfs << "Parameter for CPG " << std::endl;

				  paramOfs << "Elements           : " << "CPG0 value,  CPG1 Value,  CPG2 Value ############################" << std::endl;
				  paramOfs << " RobotFrequency(Hz): " << aConf[i].oscConf[0].freq <<",  " << aConf[i].oscConf[1].freq <<",  " << aConf[i].oscConf[2].freq << std::endl;
				  paramOfs << " StanceRatio       : " << aConf[i].oscConf[0].dutyRate << ",  " <<aConf[i].oscConf[1].dutyRate << ",  " <<aConf[i].oscConf[2].dutyRate <<  std::endl;
				  paramOfs << " initialPhase      : " << aConf[i].oscConf[0].initialPhase << ",  "<< aConf[i].oscConf[1].initialPhase << ",  " <<  aConf[i].oscConf[2].initialPhase << std::endl;
				  paramOfs << " inhibiCoef        : " << aConf[i].oscConf[0].inhibiCoef << ",  " << aConf[i].oscConf[1].inhibiCoef << ",  " << aConf[i].oscConf[2].inhibiCoef << std::endl;
				  paramOfs <<  std::endl;
				  paramOfs <<  std::endl;

				  paramOfs << "Parameter for Trajectory making " << std::endl;
				  paramOfs << " In this simulation CPG0 is ignored " << std::endl;
				  paramOfs << "Elements           : " << "CPG0 value,  CPG1 Value,  CPG2 Value ############################" << std::endl;
				  paramOfs << " CenterPos(m)      : " << "("<< aConf[i].trjConf[0].centerTrj[0] <<"," << aConf[i].trjConf[0].centerTrj[1] <<"," << aConf[i].trjConf[0].centerTrj[2] <<")"
						  << "("<< aConf[i].trjConf[1].centerTrj[0] <<"," << aConf[i].trjConf[1].centerTrj[1] <<"," << aConf[i].trjConf[1].centerTrj[2] <<")"
						  << "("<< aConf[i].trjConf[2].centerTrj[0] <<"," << aConf[i].trjConf[2].centerTrj[1] <<"," << aConf[i].trjConf[2].centerTrj[2] <<")" << std::endl;
				  paramOfs << " Direction Vector  : " << "("<< aConf[i].trjConf[0].moveDirection[0] <<"," << aConf[i].trjConf[0].moveDirection[1] <<"," << aConf[i].trjConf[0].moveDirection[2] <<")"
						  << "("<< aConf[i].trjConf[1].moveDirection[0] <<"," << aConf[i].trjConf[1].moveDirection[1] <<"," << aConf[i].trjConf[1].moveDirection[2] <<")"
						  << "("<< aConf[i].trjConf[2].moveDirection[0] <<"," << aConf[i].trjConf[2].moveDirection[1] <<"," << aConf[i].trjConf[2].moveDirection[2] <<")" << std::endl;
				  paramOfs << " Swing Height      : " << aConf[i].trjConf[0].swingHeight <<",  " << aConf[i].trjConf[1].swingHeight <<",  " << aConf[i].trjConf[2].swingHeight  << std::endl;
				  paramOfs << " Stance Height     : " << aConf[i].trjConf[0].stanceHeight <<",  " << aConf[i].trjConf[1].stanceHeight <<",  " << aConf[i].trjConf[2].stanceHeight  << std::endl;
				  paramOfs << " walk Length       : " << aConf[i].trjConf[0].walkLength <<",  " << aConf[i].trjConf[1].walkLength <<",  " << aConf[i].trjConf[2].walkLength  << std::endl;
				  paramOfs << " " << std::endl;
			}
			return true;
        }

        // step Logging
        bool stepLogging(double time, const analyData& aDat){
		// make a log in Phase file
			phaseOfs << time << " ";
			// output phase
			for(int i = 0; i< Num; i++){
				phaseOfs << aDat.pData[i].currentPhase[1] << " ";
			}
			phaseOfs << "   " ;
			// meaning phase Diff
			for(int i = 0; i< Num-1; i++){
				phaseOfs << aDat.pData[i].meaningDPhase << " ";
			}
			phaseOfs << "   " ;
			// orginal phase Diff
			for(int i = 0; i< Num-1; i++){
				phaseOfs << aDat.pData[i].pdBuff.getCurBuff() << " ";
			}
			phaseOfs << "   " ;
			// convergence situation
			int cSitu = 0;
			if(aDat.localConvergence)cSitu = 1;
			phaseOfs << cSitu << "     " ;
			// outPhase
			for(int i = 0; i< Num; i++){
				for(int j = 0; j< 3;j++){
					phaseOfs << aDat.pData[i].outPhase[j] << " ";
				}
			}
			phaseOfs << "   " ;
			// bias
			for(int i = 0; i< Num; i++){
				for(int j = 0; j< 3;j++){
					phaseOfs << aDat.pData[i].bias[j] << " ";
				}
			}
			phaseOfs << std::endl;

		// make a log in sensor
			sensorOfs << time << " ";
			// output pose
			for(int i = 0; i< Num; i++){
				sensorOfs << aDat.sData[i].pose[0] << " "<< aDat.sData[i].pose[1] << " "<< aDat.sData[i].pose[2] << " ";
			}
			sensorOfs << "   " ;
			// output Velocity
			for(int i = 0; i< Num; i++){
				sensorOfs << aDat.sData[i].g_roboSpd[0] << " "<< aDat.sData[i].g_roboSpd[1] << " "<< aDat.sData[i].g_roboSpd[2] << " ";
			}
			sensorOfs << "   " ;
			// foot Sensor
			for(int i = 0; i< Num; i++){
				for(int j = 0; j < 3; j++){
					sensorOfs << aDat.sData[i].forceData[j] << " ";
				}
			}
			sensorOfs << "   " ;
			// grobal Position
			for(int i = 0; i< Num; i++){
				for(int j = 0; j < 3; j++){
					sensorOfs << aDat.sData[i].g_roboPos[j] << " ";
				}
			}
			sensorOfs << "   " ;
			// whole COG Position
			for(int i = 0; i< 3; i++){
				sensorOfs << aDat.cogPos[i] << " ";
			}
			sensorOfs << "   " ;
			// NE stability
			sensorOfs << aDat.NEstability << " ";
			sensorOfs << std::endl;

		// make a ,log in foot contact
			fContactOfs << time << "  ";
			// out put fContact data
			for(int i = 0; i < Num; i++){
				for(int j = 0; j < 3;j++){
					int a = 0;
					if(aDat.sData[i].forceData[j] > 0.01) a = 1;
					fContactOfs << a << " ";
				}
			}
			fContactOfs << std::endl;

			return true;
        }

        // close file
		bool close(void){
			if(phaseOfs.is_open())phaseOfs.close();
			if(sensorOfs.is_open())sensorOfs.close();
			if(fContactOfs.is_open())fContactOfs.close();
        	if(paramOfs.is_open())paramOfs.close();
			return true;
		}
    };

	// logging file
    //  This is the file for each simulation
    FileSet fSet;

    // this is the file for whole simulation
    std::ofstream wholeOfs;

    //analize for local conv
    typedef struct{
	    // previous phase difference
		double pDiff[Num-1];
		// phase difference detect period(times of oscillation period)
		static constexpr double C_TIME = 5;
		// phase diff threshold for convergence
		static constexpr double C_THRESHOLD = 0.01;
    }localConvParam;
    // The parameter to analyze for local convergence
    localConvParam lParam;


public:
  // Constructor
  ThisSim():Simulation(),fSet() //ashiUnit1(0),controller(0),robotFixator(0)
  {
	  wholeOfs.open("WholeData.txt");
	  wholeOfs << "# THis is the file which contains all of this simulation data" << std::endl;
	  wholeOfs << "# 1 simNum, 2 Freq, 3 Duty, 4 aveStability, 5 minStability, 6 period, 7 yawSpeed, 8 transSpeed"
			  << ", 9 convDeviation, 10 convTime, 11 avePhaseDiff 1-0, 12 avePhaseDiff 2-1..... "<<std::endl;

	  sParam.simNum = S_START_NUM;
  }

  // Destructor
  virtual ~ThisSim(){
	  fSet.close();
	  wholeOfs.close();
  }


private:
  // set conf file (pDiff[Num - 1])
  void setConf(double freq, double duty, double* pDiff){
	  // set configuration of the parameters
	  double phase1 = 0.;
	  double phase2 = M_PI;
	  for(int k = 0; k< Num; k++){
	  //AshigaruControllerConf confy;
		acConf[k] = AshigaruController::getStraightCaseConf();
		aConf = Ashigaru::getDefaultConf();

		// set the Freq and Duty rate
		for(int i = 0; i < 3; i++){
			acConf[k].oscConf[i].freq = freq;
			acConf[k].oscConf[i].dutyRate = duty;
		}

		// set initial Freq Difference
		acConf[k].oscConf[1].initialPhase = phase1;//3/2. * M_PI;
		acConf[k].oscConf[2].initialPhase = phase2;//1/2. * M_PI
		if(k < Num-1){
			phase1 += pDiff[k];
			phase2 += pDiff[k];
			AshigaruController::checkPhase(phase1);
			AshigaruController::checkPhase(phase2);
		}
	  }
  }

  // calculating meaning Phase
  void calcMeanDPhase(phaseSet& pDat, int meanPeriod){
	  // period is 0, stop
	  if(meanPeriod <= 0) return;
	  //set param
	  double temp = 0.;
	  bool tFlag = false;
	  // calculate meaning
	  //  In this situation, I have to take care that the Phi changes like this
	  //   0.1 -> 0. -> 6.27 -> 6.25.. In this case, if I calculate meaning, the result becomes bad
	  // so, to compensate this situation, I just detect the situation that the phi is close to 0, and In this situation,
	  // I change the phi by adding pi and do as same
	  for(int j = 0; j< meanPeriod; j++){
		temp += pDat.pdBuff.getPrvBuff(j);
		// in bad situation
		if(pDat.pdBuff.getPrvBuff(j) < 0.1){
			tFlag = true;
			break;
		}
	  }
	  // If it is not bad situation -> processing is end
	  if(!tFlag){
		pDat.meaningDPhase = temp / (double)meanPeriod;
		return;
	  }
	  // If it is bad situation
	  else{
		// compensate the bad situation simply, it is not optimal I notice
		temp = 0.;
		for(int j = 0; j< meanPeriod; j++){
			// compensate the bad boundary effects
			double a = pDat.pdBuff.getPrvBuff(j) + M_PI;
			AshigaruController::checkPhase(a);
			temp += a;
		}
		double meaning = temp / (double)meanPeriod - M_PI;
		AshigaruController::checkPhase(meaning);
		// this is the result whivh is compensated
		pDat.meaningDPhase = meaning;
		return;
	  }
  }

  // calculating Distance between point to Line in 3D
  bool calcDis_P_to_L(osg::Vec3d point, osg::Vec3d linePoint1, osg::Vec3d linePoint2, double& length, osg::Vec3d& cnctPoint){
	  // calculate vector from line point 1
	  osg::Vec3d lVec = osg::Vec3d(linePoint2[0] - linePoint1[0], linePoint2[1] - linePoint1[1], linePoint2[2] - linePoint1[2]);
	  osg::Vec3d pVec = osg::Vec3d(point[0] - linePoint1[0], point[1] - linePoint1[1], point[2] - linePoint1[2]);

	  double lLength = lVec[0] * lVec[0] + lVec[1] * lVec[1] + lVec[2] * lVec[2];
	  if(fabs(lLength) < 0.001)return false;

	  // calc param t
	  double t = (lVec[0]*pVec[0] + lVec[1]*pVec[1] + lVec[2]*pVec[2]) / lLength;

	  // calc cnctPoint
	  cnctPoint = linePoint1 + lVec * t;

	  // calc length
	  osg::Vec3d sVec = osg::Vec3d(point[0] - cnctPoint[0], point[1] - cnctPoint[1], point[2] - cnctPoint[2]);
	  length = sqrt( sVec[0]*sVec[0] + sVec[1]*sVec[1] + sVec[2]*sVec[2] );

	  return true;
  }

  // detect whether COG is in contact region or not
  bool isCogStable(osg::Vec3d cogPoint, std::vector<osg::Vec3d> contactPoint){
	 // check the number of contact point
	  if( (int)contactPoint.size() < 3) return false;

	 // check whether the COG is in the contact shape or not
	  // we check it by comparing whether the cnctpoint is upper than the line or not]
	  // we check it about all of the line which connect cog to arbitaly contact point.
	  for(int i = 0; i < (int)contactPoint.size();i++){
		  // calc gradient of the line from cog to a contact point
		  double a = ( contactPoint.at(i)[1] - cogPoint[1] )/ ( contactPoint.at(i)[0] - cogPoint[0] );
		  double b = cogPoint[1] - a * cogPoint[0];

		  //std::cout <<" a = " << a << std::endl;

		  // detect cog is stable or not
		  //  we detect stability by compareing whether the cnctpoint is upper than the line or not.
		  //  if it contains both feature we can say that system is stable.
		  bool upFlag = false;
		  bool downFlag = false;
		  //std::cout <<"i " << i << ", "<<  std::endl;
		  // extract the value of the iterator
		  for(int j = 0; j < (int)contactPoint.size(); j++){
			  double val = a * (contactPoint.at(j)[0]) + b - contactPoint.at(j)[1];

			  if(val > 0.){
				  upFlag = true;
			  }else if(val < 0.){
				  downFlag = true;
			  }
			  //std::cout <<"  val = " << val;
			  //if(j == ((int)contactPoint.size() -1))std::cout << std::endl;
		  }
		  /*
		  std::vector<osg::Vec3d>::iterator it = contactPoint.begin();
		  while( it != contactPoint.end() )  // check about every contact point
		  {
			  double val = a * ((*it)[0]) + b - (*it)[1];

			  if(val > 0)upFlag = true;
			  else if(val < 0)downFlag = true;
			  it++;
		  }
		  */

		  if(!upFlag || !downFlag) return false;
	  }
	  // at this point, cog is stable
	  return true;
  }

  // calculate NE stability margin
  // contact point should be choosed in order to fullfill follow situation
  //  if I connect the contact point from first to final one, the resultant line makes contact shape.
  //  The order of vector contactPoint is important. all of the line which could be gotten connecting each next two points of vector
  //  should be the line of contact shape
  double calcNEstability(osg::Vec3d cogPoint, std::vector<osg::Vec3d> contactPoint){
	 // detect stable or not
	  if(!isCogStable(cogPoint, contactPoint))return -1;
	  //if(contactPoint.size() < 3) return -1;

	 // calcNEstability
	  double mStability = 100.;

	  std::vector<osg::Vec3d>::iterator it = contactPoint.begin();
	  while( it != (contactPoint.end()) )  // check about every contact point
	  {
		  // set Value
		  double cogLength; // minimum length from cpg to the line of contact shape
		  osg::Vec3d cPoint; // the point on the line  when the length from cpg to the line is minimun
		  std::vector<osg::Vec3d>::iterator it1 = it;
		  ++it;
		  std::vector<osg::Vec3d>::iterator it2 = it;
		  if(it2 == (contactPoint.end()))it2 = contactPoint.begin();

		  // calculate minimum length from cpg to the line of contact shape
		  calcDis_P_to_L(cogPoint, *it1, *it2, cogLength, cPoint);

		  // calculate NE stability
		  double xd = ((*it2)[0]) - ((*it1)[0]);
		  double yd = ((*it2)[1]) - ((*it1)[1]);
		  double zd = ((*it2)[2]) - ((*it1)[2]);
 		  double l = sqrt(xd*xd + yd*yd) ;
		  double theta = atan2(zd, l);

		  double stability = cogLength * cos(theta) + cPoint[2] - cogPoint[2];
		  if(stability < mStability) mStability = stability;
	  }
	  return mStability;
  }

  // calculate the averaged phase, maximum phase deviation in evaluation duration
  bool calcResultData(resultData& rDat, const periodSet& peSet, std::list<evalData>& eDat){
	// we set the resultant period from last period
	  rDat.period = peSet.oscPeriod;

	// calculate the averaged phase, maximum phase deviation, ave and mini stability in evaluation duration
	  // calculate evaluation duration period
	  int ePeriod = rDat.period * EVALUATE_STEP;
	  if((int)eDat.size() < ePeriod) return false;
	  double eTime = ePeriod * 0.05;

	  // calculate phase average and max and min in this duration
	   // phase
	  double average[Num-1] = {0.};
	  double max[Num-1] = {0.};
	  double min[Num-1] = {0.};
	  for(int k = 0; k < Num-1;k++){
		  min[k] = 6.;
	  }


	   // stability
	  double minS = 10.;
	  double aveS = 0.;
	   // yaw
	  double yaw1 = 0.;// averaged value in first period of evaluation duration
	  double yaw2 = 0.;// averaged value in final period of evaluation duration
	   // spd
	  double spd[2] = {0.};

	  std::list<evalData>::iterator it = eDat.end();
	  for(int j = 0; j < ePeriod; j++){
		  // change iterator
		  it--;
		  // calc ave and min max,  do same thing for each ashigaru deff
		  for(int k = 0; k< Num-1; k++){
			  double a = (*it).phaseDiff[k];
			  average[k] += a;
			  if(max[k] < a)max[k] = a;
			  if(min[k] > a)min[k] = a;
		  }

		  // calc min stability
		  double b = (*it).NEstability;
		  if(b < minS) minS = b;
		  if(b > 0)aveS += b;// in average, we ignore the unstable point by regarding it as 0

		  // calc yaw angle change
		  if(j < rDat.period){// final ave
			  yaw2 += (*it).Yaw;
		  }else if(j >= (ePeriod - rDat.period) ){
			  yaw1 += (*it).Yaw;
		  }

		  // calc spd
		  spd[0] += (*it).speed[0];
		  spd[1] += (*it).speed[1];
	  }

	  // finally calculate averaged phase value and max deviation
	  double deviation = 0.;
	  for(int k = 0; k< Num-1; k++){
		  rDat.avePhaseDiff[k] = average[k] / (double)ePeriod;
		  if(deviation < (max[k] - rDat.avePhaseDiff[k]) ) deviation = max[k] - rDat.avePhaseDiff[k];
		  if(deviation < (rDat.avePhaseDiff[k] - min[k]) ) deviation = rDat.avePhaseDiff[k] - min[k];
 	  }
	  rDat.convDeviation = deviation;
	  // calculate ave stability and mini stability
	  rDat.aveStability = aveS / (double)ePeriod;
	  rDat.minStability = minS;

	  // calc Yaw angle change speed in EVAL_STEP
	  yaw1 = yaw1 / (double)rDat.period;
	  yaw2 = yaw2 / (double)rDat.period;
	  rDat.yawSpeed = (yaw1 - yaw2)/eTime;

	  // calc ave speed
	  spd[0] = spd[0] / (double)ePeriod;
	  spd[1] = spd[1] / (double)ePeriod;
	  rDat.transSpeed = sqrt(spd[0]*spd[0] +  spd[1]*spd[1]);

	//Detect the time when this system seem to converge
	  int count = 0;
	  bool continueFlag = true;
	  std::list<evalData>::iterator it2;
	  it2 = eDat.end();
	  while(it2 != eDat.begin() && continueFlag){
		  it2--;
		  count++;
		  // detect where is the converge point
		  for(int k=0; k<Num-1; k++){
			  double dev = fabs( (*it2).phaseDiff[k] - rDat.avePhaseDiff[k] );
			  if(dev > CONVERGENCE_THRESHOLD){
				  continueFlag = false;
			  }
		  }
	  }
	  rDat.convTime = SIMULATION_TIME - (double)count * 0.05;


	  return true;
  }



  // calculate aData from
  bool calcAnalyData(analyData& aDat, periodSet& pSet, AshigaruController** ptCont, localConvParam& lPara, const AshigaruConf& aConf, int nowStep, double time){
		// get analysis Data
		for(int i=0; i < Num;i++){
			// get the data
			 //oscillation Frequency
			aDat.oscFreq[i] = ptCont[i]->eGetAshigaruCtrConf().oscConf[1].freq;

			 // sensor Data
			aDat.sData[i] = ptCont[i]->eGetSensedData();

			 // phase Data
			for(int j = 0; j< 3;j++){
				aDat.pData[i].currentPhase[j] = ptCont[i]->eGetPhase(j);
				aDat.pData[i].outPhase[j] = ptCont[i]->eGetOscSet(j).outPhase;
				aDat.pData[i].bias[j] = ptCont[i]->eGetOscSet(j).footBias;
			}
			 // phase difference
			if(i < Num-1){
				double dp = ptCont[i+1]->eGetPhase(1) - ptCont[i]->eGetPhase(1);
				AshigaruController::checkPhase(dp);
				aDat.pData[i].pdBuff.addBuff(dp);
			}

			// calculate meaningPhaseDiff F = 0.02(Hz) is limit of Buffer
			 // calculate 1 period length by counting osc situation
			 int idealPeriod = (int)(ptCont[i]->eGetAshigaruCtrConf().controlFreq / aDat.oscFreq[i]);
			 if(aDat.pData[0].outPhase[1] < (2. * M_PI)/(double)idealPeriod ){
				 if(!pSet.mFlag){
					 pSet.mFlag = true;
					 pSet.oscPeriod = nowStep - pSet.beforeStep;
					 pSet.beforeStep = nowStep;
				 }
			 }else{
				 pSet.mFlag = false;
			 }
			 // calculate meaning
			 if(i < Num-1){
				//int period = (int)(ptCont[i]->eGetAshigaruCtrConf().controlFreq / aDat.oscFreq[i]);
				 int period = idealPeriod;//pSet.oscPeriod;
				 if( period > aDat.pData[i].pdBuff.getMaxBuffNum()) period = aDat.pData[i].pdBuff.getMaxBuffNum();
				 this->calcMeanDPhase(aDat.pData[i], period);
			 }

		}

		// judge convergence (locally)
		int t = (int)(time * 20.);
		int th = (int)(lPara.C_TIME / aDat.oscFreq[0] * 20.);
		if(t % th == 0){
			bool cFlag = true;
			for(int i=0;i<Num-1;i++){
				if(fabs(aDat.pData[i].meaningDPhase - lPara.pDiff[i]) > lPara.C_THRESHOLD){
					cFlag = false;
				}
				lPara.pDiff[i] = aDat.pData[i].meaningDPhase;
			}
			aDat.localConvergence = cFlag;
		}

		// calculate COG stability
		 // calc whole COG position
		 osg::Vec3d cogPoint = osg::Vec3d(0.,0.,0.);
		 for(int i = 0; i< Num;i++){
			cogPoint += aDat.sData[i].g_cogPos * aConf.wholeMass;
		 }
		 cogPoint = cogPoint / (aConf.wholeMass * (double)Num);
		 aDat.cogPos = cogPoint;

		 // detect the leg touches(this is for straight connection)
		 std::vector<osg::Vec3d> contactPos;
		 contactPos.clear();
		 // from right side to left side (order is important)
		 for(int k = 0;k<Num;k++){
			 if(aDat.sData[k].forceData[1] > 0.01)contactPos.push_back(aDat.sData[k].g_legToePos[1]);
		 }
		 for(int k = Num-1;k >= 0;k--){
			 if(aDat.sData[k].forceData[2] > 0.01)contactPos.push_back(aDat.sData[k].g_legToePos[2]);
		 }

		 // calc Stability
		 aDat.NEstability = calcNEstability(aDat.cogPos, contactPos);
		 return true;
  }


public:
  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    //setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));
    setCameraHomePos(
        lpzrobots::Pos(-0.0114359, 1.66848, 0.922832),
        lpzrobots::Pos(0.866, -0.43884, 0)
    );

    // initialization
    // - set noise to 0.
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)
    global.odeConfig.noise=0.0;
    global.odeConfig.setParam("controlinterval", 5); // 20 Hz control step
    global.odeConfig.setParam("simstepsize", 0.01); // 100Hz simulation step

    // use Playground as boundary:
    OctaPlayground* playground = new OctaPlayground(odeHandle, osgHandle, osg::Vec3(1000, 0.2, 0.4), 4);
    playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
    global.obstacles.push_back(playground);

    // The definition of the values
    AbstractWiring* wiring[Num];
    OdeAgent* agent[Num];

    // simulation param setting
    for(int k = 0;k<Num-1;k++){
    	if(k == 0){
    		sParam.phaseDiff[k] = this->S_PHASE_DIFF1_MIN;
    	}else if(k == 1){
    		sParam.phaseDiff[k] = this->S_PHASE_DIFF2_MIN;
    	}else{
    		sParam.phaseDiff[k] = M_PI;
    	}
    }
    sParam.duty = S_DUTY_MIN;
    sParam.freq = S_FREQ_MIN;

	wholeOfs << "# Phase Diff1:" << sParam.phaseDiff[0] << ", Phase Diff2:" << sParam.phaseDiff[1] <<  std::endl; // Change the phase Difference

    // make file
    std::ostringstream ost;
    ost << sParam.simNum;
    fSet.makeFile(ost.str());

    // set Configuration of osc
    setConf(sParam.freq, sParam.duty, sParam.phaseDiff);

    // make robot
    for(int i = 0; i< Num; i++){
    	// make wireing
    	wiring[i] = new One2OneWiring(new ColorUniformNoise(0.0));

    	// make controller
        std::ostringstream ost;
        ost << "ashigaru" << i;
        std::string ashName = ost.str();
        char ashName2[30];
        strcpy(ashName2,ashName.c_str());

        tController[i] = new AshigaruController(ashName2, aConf, acConf[i]);//  TestController::TestController(confy);
        controller[i] = tController[i];

        // make robot

        // Put the robot on initial position
        //    this position is important because by this position, the connection relation is determined.
        ashiUnit[i] = new Ashigaru(odeHandle, osgHandle, aConf, ashName2);
        ashiUnit[i]->setColor(Color(1.0,1.0,0));
        ashiUnit[i]->place( osg::Matrix::rotate(0., osg::Vec3(1, 0, 0)) * osg::Matrix::translate( ((double)i) * aConf.rate * aConf.connectLength, .0, 0.2));

        // make agent
        agent[i] = new OdeAgent(global);
        agent[i]->init(controller[i], ashiUnit[i], wiring[i]);

        // Create Joint
        if(i == 0){
            // create a fixed joint to hold the robot in the air at the beginning SUSPEND
            /*
        	envFixator = new lpzrobots::FixedJoint(
                    ashiUnit[i]->getMainPrimitive(),
                    global.environment);
            envFixator->init(odeHandle, osgHandle, false);
            */
        }else if(i > 0){
        	// Create ASHIGARU connection
        	robotFixator[i-1] = new lpzrobots::FixedJoint(
        			ashiUnit[i-1]->getTibiaPrimitive(Ashigaru::L0),
        			ashiUnit[i]->getMainPrimitive());
            	//global.environment);
        	robotFixator[i-1]->init(odeHandle, osgHandle, false);
        }

        // make connection and something
        global.configs.push_back(ashiUnit[i]);
        global.configs.push_back(controller[i]);
        global.agents.push_back(agent[i]);
    }

    // Logging
    fSet.initialLogging(acConf);

    // High speed mode
    if(this->HIGHSPEED_MODE){
    	// stop movie and speed up
    	this->noGraphics = true;
    	globalData.odeConfig.setParam("realtimefactor", 50);
    }
  }

  //Function die eingegebene Befehle/kommandos verarbeitet
  virtual bool command (const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    if (!down) return false;
    bool handled = false;
    FILE* f;
    switch ( key )
    {
      case 's' :
    	  f=fopen("controller","wb");
    	  controller[0]->store(f) && printf("Controller stored\n");
    	  fclose(f);
    	  handled = true; break;
      case 'l' :
    	  f=fopen("controller","rb");
    	  controller[0]->restore(f) && printf("Controller loaded\n");
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

  // logging Phase , it was called every watching steop of the system
  void stepLogging(analyData* aDat){
	  // Logging of the phase difference
  }

  // it is called once when "simulation_time_reached" is true
  virtual bool restart(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global){
	  // COUT
	  //std::cout<< "Restart Phase" <<std::endl;

	  // change variable #################################################################################
	   // add simulation step
	   sParam.simNum++;
	   // add Duty rate and Freq
	   sParam.duty += S_DUTY_STEP;
	   if(sParam.duty > S_DUTY_MAX + 0.0001){//double accuracy
		   sParam.duty = S_DUTY_MIN;
		   sParam.freq += S_FREQ_STEP;
		   wholeOfs << std::endl; // make a space of file to plot it
		   // this situation means that simulation is end!!
		   if(sParam.freq > S_FREQ_MAX + 0.0001){

			   // this is the phase to add Phase difference
			   sParam.phaseDiff[0] += this->S_PHASE_DIFF1_STEP;
			   sParam.duty = S_DUTY_MIN;
			   sParam.freq = S_FREQ_MIN;

			   if(sParam.phaseDiff[0] > S_PHASE_DIFF1_MAX + 0.0001){
				   sParam.phaseDiff[0] = S_PHASE_DIFF1_MIN;
				   sParam.phaseDiff[1] += S_PHASE_DIFF2_STEP;

				   // this is the end of the simulation
				   if(sParam.phaseDiff[1] > S_PHASE_DIFF2_MAX + 0.0001){
					   std::cout << "Simulation has finished!!  Step:" << sParam.simNum <<  std::endl;
					   return false;
				   }

			   }

			   wholeOfs << std::endl; // make a space of file to plot it
			   wholeOfs << std::endl; // make a space of file to plot it
			   wholeOfs << "# Phase Diff1:" << sParam.phaseDiff[0] << ", Phase Diff2:" << sParam.phaseDiff[1] <<  std::endl; // Change the phase Difference

		   }
	   }

	  // COUT
	  std::cout<<">>> RESTART > > > >  Step:" << sParam.simNum << " Freq:" << sParam.freq << " Duty:" << sParam.duty <<std::endl;

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
	  // we also end the local file logging
	  fSet.close();


      ///// Recreate Robot >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	  // The definition of the values
	  AbstractWiring* wiring[Num];
	  OdeAgent* agent[Num];

	  // make local files again
	  std::ostringstream ost;
	  ost << sParam.simNum;
	  fSet.makeFile(ost.str());

	  // set Configuration of osc
	  setConf(sParam.freq, sParam.duty, sParam.phaseDiff);

	  //std::cout<<"remake phase has started" << std::endl;
	  // make robots again
	  for(int i = 0; i< Num; i++){
	    	// make wiring
	    	wiring[i] = new One2OneWiring(new ColorUniformNoise(0.0));

	    	// make controller
	        std::ostringstream ost;
	        ost << "ashigaru" << i;
	        std::string ashName = ost.str();
	        char ashName2[30];
	        strcpy(ashName2,ashName.c_str());

	        tController[i] = new AshigaruController(ashName2, aConf, acConf[i]);//  TestController::TestController(confy);
	        controller[i] = tController[i];

	        // make robot
	        // Put the robot on initial position
	        //    this position is important because by this position, the connection relation is determined.
	        ashiUnit[i] = new Ashigaru(odeHandle, osgHandle, aConf, ashName2);
	        ashiUnit[i]->setColor(Color(1.0,1.0,0));
	        ashiUnit[i]->place( osg::Matrix::rotate(0., osg::Vec3(1, 0, 0)) * osg::Matrix::translate( ((double)i) * aConf.rate * aConf.connectLength, .0, 0.2));

	        // make agent
	        agent[i] = new OdeAgent(global);
	        agent[i]->init(controller[i], ashiUnit[i], wiring[i]);

	        // Create Joint
	        if(i == 0){
	            // create a fixed joint to hold the robot in the air at the beginning SUSPEND
	            /*
	        	envFixator = new lpzrobots::FixedJoint(
	                    ashiUnit[i]->getMainPrimitive(),
	                    global.environment);
	            envFixator->init(odeHandle, osgHandle, false);
	            */
	        }else if(i > 0){
	        	// Create ASHIGARU connection
	        	robotFixator[i-1] = new lpzrobots::FixedJoint(
	        			ashiUnit[i-1]->getTibiaPrimitive(Ashigaru::L0),
	        			ashiUnit[i]->getMainPrimitive());
	            	//global.environment);
	        	robotFixator[i-1]->init(odeHandle, osgHandle, false);
	        }

	        // make connection and something
	        global.configs.push_back(ashiUnit[i]);
	        global.configs.push_back(controller[i]);
	        global.agents.push_back(agent[i]);
	  }

	  // Logging
	  fSet.initialLogging(acConf);

	  // restart!
	  return true;
	}

  // it is called in every simulation step
  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control)
  {
  		// for demonstration: set simsteps for one cycle to 60.000/currentCycle (10min/currentCycle)
  		// if simulation_time_reached is set to true, the simulation cycle is finished

		if (globalData.sim_step % 5 == 1)// every control step + a little(this is for waiting for getting data)
  		{
			// get time
			double time = (double)(globalData.sim_step / 5) * 0.05;
			int nowStep = globalData.sim_step / 5;

			//Time for the simulation
			// Get and calculate data and logged
			if(time < SIMULATION_TIME){
				// calculate analyData for each step
				this->calcAnalyData(aData, peSet, tController, lParam, aConf, nowStep, time);

				// logging to buffer
				 evalData eva;
				 eva.NEstability = aData.NEstability;
				 for(int k = 0; k< Num-1; k++){
					eva.phaseDiff[k] = aData.pData[k].meaningDPhase;
				 }
				 eva.Yaw = aData.sData[1].pose[2];
				 eva.speed[0] = aData.sData[1].g_roboSpd[0];
				 eva.speed[1] = aData.sData[1].g_roboSpd[1];
				evaList.push_back(eva);

				// logging to file
				fSet.stepLogging(time, aData);

				int t = (int)(time * 20.);
				int th = (int)(lParam.C_TIME / aData.oscFreq[0] * 20.);
				if(t % th == 0){
					if(aData.localConvergence){
						std::cout <<"sim:" << sParam.simNum << ", x" << this->truerealtimefactor
								<< " t:" << time
								<< " !!!!! the system locally seems to converge !!!!!!!!!!!! "
								<< "d0:" << aData.pData[0].meaningDPhase
								<< "  d1:" << aData.pData[1].meaningDPhase << std::endl;
					}else{
						std::cout <<"sim:" << sParam.simNum << ", x" << this->truerealtimefactor
								<< " t:" << time
								<< " >>>>> the system seems to be unstable >>>>>>>>>>>>>>>>> "
								<< "d0:" << aData.pData[0].meaningDPhase
								<< "  d1:" << aData.pData[1].meaningDPhase << std::endl;
					}
				}
			}

			// The simulation time has finished
			//   analize logged data and evaluate it
			else if(time == SIMULATION_TIME){
				// analize about the data
				//  calculate stability, convergence , and so on.
				this->calcResultData(rData, peSet, evaList);

				// logging the data
				wholeOfs << sParam.simNum << " " <<acConf[0].oscConf[1].freq << " "<<  acConf[0].oscConf[1].dutyRate << " "
						<< rData.aveStability << " " << rData.minStability << " " << rData.period << " "
						<< rData.yawSpeed << " " << rData.transSpeed << " " << rData.convDeviation << " " << rData.convTime << " ";
				for(int k = 0; k < Num-1; k++){
					wholeOfs << rData.avePhaseDiff[k] << " ";
				}
				wholeOfs << std::endl;

				// stop the simulation
				this->simulation_time_reached = true;
			}

  		}
  		/***********************************************************************************/


  	}


  virtual void bindingDescription(osg::ApplicationUsage & au) const {
    au.addKeyboardMouseBinding("Teachung: t","toggle mode");
    au.addKeyboardMouseBinding("Teaching: u","forward");
    au.addKeyboardMouseBinding("Teaching: j","backward");
    au.addKeyboardMouseBinding("Simulation: s","store");
    au.addKeyboardMouseBinding("Simulation: l","load");
  }

  
};

int main (int argc, char **argv)
{ 
  ThisSim sim;
  // run simulation
  return sim.run(argc, argv) ? 0 : 1;
}
 
