#ifndef ITFSIMULATION_H
#define ITFSIMULATION_H

#include <iostream>

/*
 THis is the abstruct class for a simulation

*/

template <class T_ctr>
class itfSimulation
{
public:
    // constructor
    itfSimulation(){}
    // destructor
    virtual ~itfSimulation(){
        return;
    }

    // interfaces
    //  this is called when simulation starts
    virtual void start_simulation(void)=0;

    // this is called when one periodic simulation starts
    virtual void start_oneProc(int simNum_, T_ctr& tC_)=0;

    // this is called every simulation step
    virtual void calcStep(T_ctr& tC_, double time_)=0;

    // this is called every simulation step
    //  if it returns true, this procedure of simulation will end
    virtual bool isFinished_oneProc(T_ctr& tC_, double time_)=0;

    // this is called when the procedure ends
    virtual void finish_oneProc(T_ctr& tC_, double time_)=0;

    // this is called when the procedure ends
    //  if it is true, the simulation will end
    virtual bool isFinished_simulation(int simNum_)=0;

    // this is called when the simulation ends
    virtual void finish_simulation(int simNum_)=0;

    // this is called before start simulation
    virtual void request_updateConfigs()=0;
};

#endif // BASEHEXASIMULATION_H
