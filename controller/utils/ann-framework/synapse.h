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


#ifndef SYNAPSE_H_
#define SYNAPSE_H_

//forward declarations
class ANN;
class Neuron;

class Synapse {
public:
    Synapse(Neuron * const apost, Neuron * const apre);
    ~Synapse();
    Neuron* getPost();
    Neuron* getPre();
    const double& getWeight() const;
    void setWeight(const double& aweight);
private:
    Neuron * const pre;
    Neuron * const post;
    double weight;
};

#endif /* SYNAPSE_H_ */
