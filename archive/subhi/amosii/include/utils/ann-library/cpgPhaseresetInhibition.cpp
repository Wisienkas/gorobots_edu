/*
 * SO2CPGPhasereset.cpp
 *
 *  Created on: Jan 22, 2013
 *      Author: subhi
 */

#include "cpgPhaseresetInhibition.h"
#include <cmath>
#include <sstream>
#include "../ann-framework/synapse.h"


CPGPhaseresetInhibition::CPGPhaseresetInhibition():SO2CPG()
{
  setDefaultTransferFunction(tanhFunction());
     setNeuronNumber(2);

  //  setAlpha(1.01);

   //  setPhi(M_PI*0.05);//* M_PI*30./180.

   //  updateSO2Weights();

     enableFrequencyTable(false);
     updateFrequencyTable();
     setFootbias(0,0);
      setFootbias(1,0);
     prvTouchF=false;

}
CPGPhaseresetInhibition::~CPGPhaseresetInhibition() {}
  // TODO Auto-generated destructor stub

double CPGPhaseresetInhibition::getInhibiCoeff()
{
  return inhibiCoeff;
}
void CPGPhaseresetInhibition::setControlInput(double Control_input)
{
  ContM=Control_input;
}
  void CPGPhaseresetInhibition::setInhibiCoeff(double inhibiCoeffvalue)
  {
    inhibiCoeff=inhibiCoeffvalue;
  }

  double* CPGPhaseresetInhibition::getFootbias()
{
  return footbias;
}

void CPGPhaseresetInhibition::setFootbias(int footbias_id,double footbias_value)
{
  footbias[footbias_id]=footbias_value;
}

void CPGPhaseresetInhibition::calculate(const std::vector<double> x,AmosIISensorNames sensorname,bool footSensorWorking,std::vector<double> predictActivity,std::vector<double> predictOutput,std::vector<double> currentOutput)
{

  double ContactForce=x.at(sensorname);
 if (footSensorWorking==false)
   ContactForce=0.0;

  touchF = (ContactForce> 0.1);//sensorname=L0_fs, touchF: touch force, if touchF ==1 then it is stance, else if touchF ==o is swing
  if (count>0)
  {
    footbias[1]=-inhibiCoeff*ContactForce;//*ContactForce;//currentOutput.at(1)
   // footbias[0]=+inhibiCoeff*ContactForce;//*ContactForce;//currentOutput.at(1)
    //footbias[1]=-inhibiCoeff*predictActivity.at(1);
    /* footbias[1]=inhibiCoeff*predictOutput.at(1);
         footbias[0]=inhibiCoeff*predictOutput.at(0);*/
   // footbias[0]=0;
   // liftValue_change=-ContactForce*cos(predictActivity.at(0));
    count--;

    /*int p=count;
    count=p-1;*/
  }
  else if( touchF && !prvTouchF )// this condition indicates  starting of stance
  {
    if (predictOutput.at(1)>0. && predictOutput.at(1)-currentOutput.at(1)<0)//&& predictOutput.at(1)-currentOutput.at(1)<0
      {
     footbias[0]=1.-predictActivity.at(0);//tanh(1.24)
     footbias[1]=-predictActivity.at(1);
    // liftValue_change=ContactForce*cos(predictActivity.at(0));
      }
    else if(predictOutput.at(1)<0 )
      {
      footbias[1]=-inhibiCoeff*ContactForce;//ContactForce; //x_ is sensor is F currentOutput.at(1)
    // footbias[0]=+inhibiCoeff*ContactForce;//ContactForce; //x_ is sensor is F currentOutput.at(1)
    //  footbias[1]=-inhibiCoeff*predictActivity.at(1);
      /* footbias[1]=inhibiCoeff*predictOutput.at(1);
      footbias[0]=inhibiCoeff*predictOutput.at(0);*/
      // liftValue_change=-ContactForce*cos(predictActivity.at(0));
      //footbias[0]=0;
      count=(int)( 28 * 0.02 /ContM);
      //footbias[0]=0;
      }
    else
      {
      footbias[0] = 0.;
      footbias[1] = 0.;
      }

  }
  else
  {
    footbias[0]=0;
    footbias[1]=0;
  }
 // setFootbias(0,footbias[0]);
  //setFootbias(1,footbias[1]);

  prvTouchF=touchF;
}
void CPGPhaseresetInhibition::step()
{

}



