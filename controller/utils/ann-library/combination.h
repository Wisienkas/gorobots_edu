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


#ifndef COMBINATION_H_
#define COMBINATION_H_

#include "utils/ann-framework/ann.h"

// forward declarations
class SO2CPG;
class VRN;

/**
 * This is just an example of how to combine several networks into a parent
 * network. DON'T USE this class in your project as it will be removed in the
 * near future
 */
class Combination : public ANN
{
public:
    Combination();
    ~Combination();
    const double& getO1();
    const double& getO2();
private:
    SO2CPG * cpg;
    VRN * vrnLeft;
    VRN * vrnRight;
};


#endif /* COMBINATION_H_ */
