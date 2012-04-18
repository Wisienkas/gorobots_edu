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


#include "so2cpg.h"

#include <cmath>
#include <sstream>

SO2CPG::SO2CPG()
{
    setNeuronNumber(2);

    b(0, 0.01);
    b(1, 0.01);

    alpha = 1.01;
    phi   = 0.1*M_PI;
    updateSO2Weights();
    updateFrequencyTable();
}

const double& SO2CPG::getAlpha() const
{
    return alpha;
}

const double& SO2CPG::getPhi() const
{
    return phi;
}

void SO2CPG::setAlpha(const double& aalpha)
{
    alpha = aalpha;
    updateWeights();
    updateFrequencyTable();
}

void SO2CPG::setPhi(const double& aphi)
{
    phi = aphi;
    updateWeights();
}

void SO2CPG::updateFrequencyTable()
{
    std::stringstream filename;
    filename << "fVsPhi_a" << alpha << ".dat";
    frequencyTable.load(filename.str().c_str());
}

void SO2CPG::updateSO2Weights()
{
    w(0, 0,  alpha * cos(phi));
    w(0, 1,  alpha * sin(phi));
    w(1, 0, -alpha * sin(phi));
    w(1, 1,  alpha * cos(phi));
}
