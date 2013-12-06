/***************************************************************************
 *   Copyright (C) 2012 by Dennis Goldschmidt                              *
 *    goldschmidt@physik3.gwdg.de                                          *
 *                                                                         *
 * motormapping.h                                                        *
 *                                                                         *
 **************************************************************************/

#ifndef MOTORMAPPING_H_
#define MOTORMAPPING_H_

#include <vector>
#include <iostream>
#include <cstdlib>
#include <ode_robots/amosiisensormotordefinition.h>

class Mapping{
  public:
    Mapping();
    double getDegrees(int motor, double reflex); //converts neural motor signals to degrees
    double getReflex(int motor, double motorvalue, double fmodel_error); //converts neural motor signals to motor signals used for reflexes
    double getElevator(int motor); //return leg lift for elevator reflex
    int getIRsensor(int motor);
    bool TC(int motor);
    bool CTr(int motor);
    bool FTi(int motor);

    //TC parameters
    std::vector<double> tc;
    std::vector< std::vector<double> > tc_walk_deg;
    std::vector< std::vector<double> > tc_walk;
    //CTr parameters
    std::vector<double> ctr;
    std::vector<double> ctr_walk_deg;
    std::vector<double> ctr_walk;
    //FTi parameters
    std::vector<double> fti;
    std::vector<double> fti_walk_deg;
    std::vector<double> fti_walk;

    //getReflex parameters
    double offset;
    double max_ctr;
    double max_ctr_offset;
    double max_fti;
    double max_fti_offset;


    enum legpair{
        front = 0,
        middle = 1,
        hind = 2,
    };

    enum minmax{
        min = 0,
        max = 1,
    };


};

#endif /* MOTORMAPPING_H_ */

