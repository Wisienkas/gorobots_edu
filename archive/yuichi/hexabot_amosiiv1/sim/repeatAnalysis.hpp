#ifndef REPEATANALYSIS_HPP
#define REPEATANALYSIS_HPP

#include "itfHexaSimulation.h"
#include "itfOneProcSimulation.h"
#include "itfRepeatModulator.h"

/*
 THis is the class to repeat simulation proc as commanded by repeatModulator class

*/
// THis class gives the functions to repeat the simulation again and again depending on the repeat class
template <class T_ctr>
class repeatAnalysis:public itfSimulation<T_ctr>
{
private:
    itfOneProcSimulation<T_ctr>& oneProcSim;
    itfRepeatModulator& repModulator;
    int simNum;

public:
    // constructors
    repeatAnalysis(itfOneProcSimulation<T_ctr>& oneProcSim_, itfRepeatModulator& rptModulator_)
        :itfSimulation<T_ctr>(), oneProcSim(oneProcSim_), repModulator(rptModulator_)
    {
        // initialize the value
        //oneProcSim = oneProcSim_;
        //repModulator = rptModulator_;
        simNum = 0;
    }

    // destructor
    ~repeatAnalysis(){
        // destructor
        return;
    }

    // interface
    //  this is called when simulation starts
    void start_simulation(void){
        // simulation start
        simNum = 0;
        return;
    }

    // this is called when one periodic simulation starts
    void start_oneProc(int simNum_, T_ctr& tC_){
        // start one proc simulation
        oneProcSim.start_oneProc(simNum_, tC_);
    }

    // this is called every simulation step
    void calcStep(T_ctr& tC_, double time_){
        // every simulationsteps
        oneProcSim.calcStep(tC_, time_);
    }

    // this is called every simulation step
    //  if it returns true, this procedure of simulation will end
    bool isFinished_oneProc(T_ctr& tC_, double time_){
        // is Finished
        return oneProcSim.isFinished_oneProc(tC_, time_);
    }

    // this is called when the procedure ends
    void finish_oneProc(T_ctr& tC_, double time_){
        oneProcSim.finish_oneProc(tC_, time_);
    }

    // this is called when the procedure ends
    //  if it is true, the simulation will end
    bool isFinished_simulation(int simNum_){
        return repModulator.isFinished();
    }

    // this is called when the simulation ends
    void finish_simulation(int simNum_){
        oneProcSim.finish_simulation(simNum);
    }

    // this is called before start simulation
    void request_updateConfigs(){
        // call change simulation
        simNum++;
        repModulator.changeConfigs(simNum);

        // inform restart
        if (simNum > 1){
            oneProcSim.inform_restart(simNum);
        }

        return;
    }
};

#endif // REPEATANALYSIS_HPP
