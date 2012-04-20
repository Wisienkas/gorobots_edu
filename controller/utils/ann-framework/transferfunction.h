/*****************************************************************************
 *  Copyright (C) 2012 by Timo Nachstedt                                     *
 *                                                                           *
 *  This program is free software: you can redistribute it and/or modify     *
 *  it under the terms of the GNU General Public License as published by     *
 *  the Free Software Foundation, either version 3 of the License, or        *
 *  (at your option) any later version.                                      *
 *                                                                           *
 *  This program is distributed in the hope that it will be useful,          *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            *
 *  GNU General Public License for more details.                             *
 *                                                                           *
 *  You should have received a copy of the GNU General Public License        *
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.    *
 *                                                                           *
 ****************************************************************************/


#ifndef TRANSFERFUNCTION_H_
#define TRANSFERFUNCTION_H_

#include <cmath>

/**
 * Transer Function Class
 *
 * This class represents the general transfer function interface that can be
 * used by a neuron to update its output value. Every type of transfer function
 * you want to use function has to subclass TransferFunction and to implement
 * the operator() method.
 */
class TransferFunction {
public:
    /**
     * The constructor
     */
    TransferFunction() {};

    /**
     * The destructor
     */
    virtual ~TransferFunction() {};

    /**
     * Method for function call
     *
     * This method overloads the () operator and represents the usage of
     * the transfer function. Every subclass has to implement it.
     *
     * @param a the activity value of the neuron
     * @return the new output value of the neuron
     */
    virtual double operator()(const double& a) const = 0;
};


/**
 * tanh transfer function (sigmoid)
 *
 * o(a) = tanh(a)
 */
class TanhFunction : public TransferFunction {
public:
    inline double operator()(const double& x) const {
        return std::tanh(x);
    }
};

/*
 * logistic transfer function (sigmoid)
 *
 * o(a) = 1./(1+exp(-x))
 */
class LogisticFunction : public TransferFunction {
public:
    inline double operator()(const double& x) const {
        return 1./(1+std::exp(-x));
    }
};

#endif /* TRANSFERFUNCTION_H_ */
