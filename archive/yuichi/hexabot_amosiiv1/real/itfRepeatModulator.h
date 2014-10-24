#ifndef ITFREPEATMODULATOR_H
#define ITFREPEATMODULATOR_H

// THis class manage the configs and other parameters for the simulation
class itfRepeatModulator{
public:
    itfRepeatModulator(){}
    virtual ~itfRepeatModulator(){}

    // interface
    virtual void changeConfigs(int simNum_)=0;
    virtual bool isFinished()=0;
};

#endif // ITFREPEATMODULATOR_H
