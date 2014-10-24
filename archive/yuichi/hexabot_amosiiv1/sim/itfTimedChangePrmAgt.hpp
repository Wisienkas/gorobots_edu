#ifndef ITFTIMEDCHANGEPRMAGT_HPP 100
#define ITFTIMEDCHANGEPRMAGT_HPP 

#include <iostream>
// THis class manage to change parameters depending on time for the simulation
//   this class just changes the parameter depending on time. They do not control whether the simulation is end or not
//

/*
Ver 1.00 on 20140515
THisi is the first version.

*/


class itfTimedChangePrmAgent{
public:
    itfTimedChangePrmAgent(){}
    virtual ~itfTimedChangePrmAgent(){}

    // interface
    virtual void update(double time){}
    virtual void start(double time){}
    virtual void end(double time){}
    virtual void ini(void){}

    // interface
    virtual std::string getPrmName(){return "";}
    virtual std::string getPrmValStr(){return "";}
};

#endif // ITFTIMEDCHANGEPRMAGT_HPP
