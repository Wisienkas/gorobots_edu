/*****************************************************************************
 *  Copyright (C) 2015 by Dennis Goldschmidt                                 *
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


#include "IRM.h"

IRM::IRM()
{
    setNeuronNumber(2);
    setTransferFunction(getNeuron(0), thresholdFunction());
    setTransferFunction(getNeuron(1), logisticFunction());

    // synaptic weights
    w( 1, 0, 6.0 );
    w( 1, 1, 9.0 );

    // neuron biases
    b( 1,  -7.0 );
}
