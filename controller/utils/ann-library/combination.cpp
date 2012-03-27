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


#include "combination.h"

#include "so2cpg.h"
#include "vrn.h"
#include <iostream>

Combination::Combination()
{
    // this net has no own neurons
    setNeuronNumber(0);

    // create the subnets and add them
    cpg      = new SO2CPG();
    vrnLeft  = new VRN();
    vrnRight = new VRN();
    addSubnet(cpg);
    addSubnet(vrnLeft);
    addSubnet(vrnRight);

    // create connections from CPG to VRNs
    w(vrnLeft->getNeuronX(),  cpg->getNeuron(0), 3);
    w(vrnRight->getNeuronX(), cpg->getNeuron(0), 3);

    // set static input to second input of VRN
    setInput(vrnLeft ->getNeuronY(), 1);
    setInput(vrnRight->getNeuronY(), -1);

    // set frequency of the network
    cpg->setPhi(0.2);
}

Combination::~Combination()
{
}

const double& Combination::getO1()
{
    return getOutput(vrnLeft->getNeuronOutput());
}

const double& Combination::getO2()
{
    return getOutput(vrnRight->getNeuronOutput());
}
