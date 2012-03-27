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


#ifndef NEURON_H_
#define NEURON_H_

#include<list>
#include "transferfunction.h"

// forward declarations
class ANN;
class Synapse;
//class TransferFunction;

class Neuron {
public:
    Neuron();
    ~Neuron();
    void addSynapseIn(Synapse * synapse);
    void addSynapseOut(Synapse * synapse);
    const double& getActivity() const;
    const double& getBias() const;
    const double& getOutput() const;
    Synapse* getSynapseFrom(Neuron const * pre) const;
    void removeSynapseIn(Synapse const * synapse);
    void removeSynapseOut(Synapse const * synapse);
    void setActivity(const double& aactivity);
    void setBias(const double& abias);
    void setInput(const double& ainput);
    void setOutput(const double& aoutput);
    void setTransferFunction(TransferFunction const * const afunction);
    void updateActivity();
    void updateOutput();
private:
    typedef std::list<Synapse*> SynapseList;
    double output;
    double activity;
    double bias;
    double input;
    SynapseList synapsesOut;
    SynapseList synapsesIn;
    TransferFunction const * func;
    static TanhFunction const * const tanhFunction;
};

#endif /* NEURON_H_ */
