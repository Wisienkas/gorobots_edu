/*
 * transferfunction.h
 *
 *  Created on: Feb 28, 2012
 *      Author: timo
 */

#ifndef TRANSFERFUNCTION_H_
#define TRANSFERFUNCTION_H_

#include <cmath>

class TransferFunction {
public:
    TransferFunction() {};
    virtual ~TransferFunction() {};
    virtual double operator()(const double& ) const = 0;
};

class TanhFunction : public TransferFunction {
public:
    inline double operator()(const double& x) const {
        return std::tanh(x);
    }
};

class LogisticFunction : public TransferFunction {
public:
    inline double operator()(const double& x) const {
        return 1./(1+std::exp(-x));
    }
};

#endif /* TRANSFERFUNCTION_H_ */
