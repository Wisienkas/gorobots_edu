/***************************************************************************
 *   Copyright (C) 2012 by Dennis Goldschmidt                              *
 *    goldschmidt@physik3.gwdg.de                                          *
 *                                                                         *
 * motormapping.cpp                                                        *
 *                                                                         *
 **************************************************************************/

#include "motormapping.h"
#include <vector>
using namespace std;

Mapping::Mapping() {

  //TC resizing
  tc.resize(2);
  tc_walk_deg.resize(2);
  tc_walk.resize(2);

  for (unsigned int i = 0; i < tc_walk.size(); i++) {
    tc_walk_deg.at(i).resize(3);
    tc_walk.at(i).resize(3);
  }

  //CTr resizing
  ctr.resize(2);
  ctr_walk_deg.resize(2);
  ctr_walk.resize(2);

  //    for(unsigned int i=0; i < ctr_walk.size(); i++) {
  //      ctr_walk_deg.at(i).resize(3);
  //      ctr_walk.at(i).resize(3);
  //    }

  //FTi resizing
  fti.resize(2);
  fti_walk_deg.resize(2);
  fti_walk.resize(2);

  //    for(unsigned int i=0; i < fti_walk.size(); i++) {
  //      fti_walk_deg.at(i).resize(3);
  //      fti_walk.at(i).resize(3);
  //    }

  //TC parameters
  tc.at(min) = -0.91;
  tc.at(max) = 0.91;
  tc_walk_deg.at(min).at(front) = 0.0; //deg ** MIN -70 deg
  tc_walk_deg.at(max).at(front) = 50.0;//deg ** MAX +70 deg
  tc_walk_deg.at(min).at(middle) = -30.0; //deg ** MIN -60 deg
  tc_walk_deg.at(max).at(middle) = 20.0;//deg ** MAX +60 deg
  tc_walk_deg.at(min).at(hind) = -60.0; //deg ** MIN -70 deg
  tc_walk_deg.at(max).at(hind) = -10.0;//deg ** MAX +70 deg
  for (unsigned int j = front; j < (hind + 1); j++) {
    for (unsigned int i = min; i < (max + 1); i++) {
      if (j == middle) {
        tc_walk.at(i).at(j) = 0.0167 * tc_walk_deg.at(i).at(j);
      } else {
        tc_walk.at(i).at(j) = 0.0143 * tc_walk_deg.at(i).at(j);
      }
    }
  }

  //CTr parameters
  max_ctr = 115.0;
  max_ctr_offset = 110.0;
  ctr.at(min) = -1.0;
  ctr.at(max) = 0.96;
  //for (unsigned int j = front; j < (hind + 1); j++) {
  ctr_walk_deg.at(min)/*.at(j)*/= 45.0; //deg ** MIN -70 deg
  ctr_walk_deg.at(max)/*.at(j)*/= 75.0;//deg ** MAX +70 deg
  for (unsigned int i = min; i < (max + 1); i++) {
    ctr_walk.at(i)/*.at(j)*/= 0.0133 * ctr_walk_deg.at(i)/*.at(j)*/;
  }
  //}

  //FTi parameters
  max_fti = 110.0;
  max_fti_offset = 110.0;
  fti.at(min) = -1.0;
  fti.at(max) = 1.0;
  //for (unsigned int j = front; j < (hind + 1); j++) {
  fti_walk_deg.at(min)/*.at(j)*/= -130.0; //deg ** MIN -130 deg
  fti_walk_deg.at(max)/*.at(j)*/= -120.0; //deg ** MAX -20 deg
  for (unsigned int i = min; i < (max + 1); i++) {
    fti_walk.at(i)/*.at(j)*/= 0.0182 * fti_walk_deg.at(i)/*.at(j)*/+ 1.3636;
  }
  //}

}

double Mapping::getDegrees(int motor, double reflex) {

  //*****Motor mapping

  //  //TC
  //  if (TC(motor)) {
  //    if ((motor % 3) == middle) //middle
  //      return 60 * getReflex(motor, motorvalue, fmodel_error); //convert from activation to deg
  //    else
  //      return 70 * getReflex(motor, motorvalue, fmodel_error); //convert from activation to deg
  //  }
  //  //CTr
  //  if (CTr(motor)) {
  //    return 75 * getReflex(motor, motorvalue, fmodel_error); //convert from activation to deg
  //  }
  //  //FTi
  //  if (FTi(motor)) {
  //    return 55 * getReflex(motor, motorvalue, fmodel_error) - 75; //convert from activation to deg
  //  }
  //  return 0;
  //  std::cout << endl << "out of motor number range (no return value)" << endl << endl; //warning when none of the if cases matches
  //TC
  if (TC(motor)) {
    if ((motor % 3) == middle) //middle
      return 60 * reflex; //convert from activation to deg
    else
      return 70 * reflex; //convert from activation to deg
  }
  //CTr
  if (CTr(motor)) {
    return 75 * reflex; //convert from activation to deg
  }
  //FTi
  if (FTi(motor)) {
    return 55 * reflex - 75; //convert from activation to deg
  }
  return 0;
  std::cout << endl << "out of motor number range (no return value)" << endl << endl; //warning when none of the if cases matches
}

double Mapping::getReflex(int motor, double motorvalue, double fmodel_error) {

  //*****Set new walking range with searching reflex offset

  //CTr
  if (CTr(motor)) {
    //*****Calculating searching reflex offset
    offset = fmodel_error * (max_ctr / max_ctr_offset);//0.6216;/*0...115 Linear or Exponential function!!**/
    if (((motor - CR0_m) % 3) == 2) {
      offset = fmodel_error * (max_ctr / (max_ctr_offset + 40));//0.5946;/*0...110 Linear or Exponential function!!**/
    }

    for (unsigned int i = min; i < (max + 1); i++) {
      ctr_walk.at(i) = 0.0133 * (ctr_walk_deg.at(i) - offset);
    }
  }
  //FTi
  if (FTi(motor)) {
    //*****Calculating searching reflex offset
    offset = fmodel_error * (max_fti / max_fti_offset);//0.5946;/*0...110 Linear or Exponential function!!**/
    if (((motor - FR0_m) % 3) == 2) {
      offset = fmodel_error * (max_fti / (max_fti_offset));//+40));//0.5946;/*0...110 Linear or Exponential function!!**/
    }
    for (unsigned int i = min; i < (max + 1); i++) {
      fti_walk.at(i) = 0.0182 * (fti_walk_deg.at(i) + offset) + 1.3636;
    }
  }

  //*****Return reflex signal

  //TC
  if (TC(motor)) {
    return (((motorvalue - tc.at(min)) / (tc.at(max) - tc.at(min))) * (tc_walk.at(max).at(motor % 3)
        - tc_walk.at(min).at(motor % 3))) + tc_walk.at(min).at(motor % 3);
  }
  //CTr
  if (CTr(motor)) {
    return (((motorvalue - ctr.at(min)) / (ctr.at(max) - ctr.at(min))) * (ctr_walk.at(max) - ctr_walk.at(min)))
        + ctr_walk.at(min);
  }
  //FTi
  if (FTi(motor)) {
    return (((motorvalue - fti.at(min)) / (fti.at(max) - fti.at(min))) * (fti_walk.at(max) - fti_walk.at(min)))
        + fti_walk.at(min);
  }
  return 0;
  std::cout << endl << "out of motor number range (no return value)" << endl << endl; //warning when none of the if cases matches
}

double Mapping::getElevator(int motor) {
  //TC
  if (TC(motor)) {
    return 0.8;
  }
  //CTr
  if (CTr(motor)) {
    return 1.0;//0.8
  }
  //FTi
  if (FTi(motor)) {
    return 0.4;//0.7
  }
  return 0;
  std::cout << endl << "out of motor number range (no return value)" << endl << endl; //warning when none of the if cases matches
}

int Mapping::getIRsensor(int motor) {
  //TC
  if (TC(motor)) {
    if(motor < TL0_m)
      return R0_irs - 2 * (motor - TR0_m);
    else
      return L0_irs - 2 * (motor - TL0_m);
  }
  //CTr
  if (CTr(motor)) {
    if(motor < CL0_m)
      return R0_irs - 2 * (motor - CR0_m);
    else
      return L0_irs - 2 * (motor - CL0_m);
  }
  //FTi
  if (FTi(motor)) {
    if(motor < FL0_m)
      return R0_irs - 2 * (motor - FR0_m);
    else
      return L0_irs - 2 * (motor - FL0_m);
  }
  return 0;
  std::cout << endl << "out of motor number range (no return value)" << endl << endl; //warning when none of the if cases matches
}

bool Mapping::TC(int motor) {
  if (motor < CR0_m) {
    return true;
  } else
    return false;
}

bool Mapping::CTr(int motor) {
  if (motor > TL2_m && motor < FR0_m) {
    return true;
  } else
    return false;
}

bool Mapping::FTi(int motor) {
  if (motor > CL2_m) {
    return true;
  } else
    return false;
}
