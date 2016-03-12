#ifndef SIMPLE_RANDOM_NUMBER_GENERTOR_H
#define SIMPLE_RANDOM_NUMBER_GENERTOR_H

#include <vector>
#include <random>
#include <iostream>

using namespace std;

class SimpleRNG {

public:

    SimpleRNG();

    static vector<int> getUniInt( int, int, unsigned int = 1 );
    static vector<double> getUniDouble( double, double, unsigned int = 1 );
    
    static vector<double> getNormalDouble( double, double, unsigned int, double = 0.0, double = 0.6 );

protected:


};

#endif
