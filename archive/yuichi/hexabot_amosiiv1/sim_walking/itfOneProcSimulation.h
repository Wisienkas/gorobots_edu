#ifndef ITFONEPROCSIMULATION_H
#define ITFONEPROCSIMULATION_H


/*
 THis is the interface class of one procedure on the simulation

*/
template <class T_ctr>
class itfOneProcSimulation{
public:
    // constructor
    itfOneProcSimulation(){}
    virtual ~itfOneProcSimulation(){}

    // interface

    // this is called when one periodic simulation starts
    virtual void start_oneProc(int simNum_, T_ctr& tC_)=0;

    // this is called every simulation step
    virtual void calcStep(T_ctr& tC_, double time_)=0;

    // this is called every simulation step
    //  if it returns true, this procedure of simulation will end
    virtual bool isFinished_oneProc(T_ctr& tC_, double time_)=0;

    // this is called when the procedure ends
    virtual void finish_oneProc(T_ctr& tC_, double time_)=0;

    // this is called when simulation restarts
    //  if it is true, the simulation will end
    virtual void inform_restart(int simNum_)=0;

    // this is called when the simulation ends
    virtual void finish_simulation(int simNum_)=0;
};

#endif // ITFONEPROCSIMULATION_H
