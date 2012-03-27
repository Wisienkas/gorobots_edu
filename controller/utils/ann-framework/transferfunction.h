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
