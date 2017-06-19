#include "SimpleRNG.h"

using namespace std;

vector<int> SimpleRNG::getUniInt( int min, int max, unsigned int num ) {

    vector<int> numbers;

    uniform_int_distribution<int> unif( min, max );

    random_device rd;  
    default_random_engine re;
    re.seed( rd() );

    for( unsigned int i = 0; i < num; i++ ) {

        numbers.push_back( unif( re ) );

    }

    return numbers;

}

vector<double> SimpleRNG::getUniDouble( double min, double max, unsigned int num ) {

    vector<double> numbers;

    uniform_real_distribution<double> unif( min, max );
    
    random_device rd;
    default_random_engine re;
    re.seed( rd() );

    for( unsigned int i = 0; i < num; i++ ) {

        numbers.push_back( unif( re ) );

    }

    return numbers;

}

vector<double> SimpleRNG::getNormalDouble( double min, double max, unsigned int num, double mu, double std ) {

    vector<double> numbers;

    normal_distribution<double> normalf( mu, std );
    
    random_device rd;
    default_random_engine re;
    re.seed( rd() );

    for( unsigned int i = 0; i < num; i++ ) {
        
        double new_num = normalf( re );
        new_num = ( new_num < min ) ? min : new_num;
        new_num = ( new_num > max ) ? max : new_num;

        numbers.push_back( new_num );

    }

    return numbers;

}
