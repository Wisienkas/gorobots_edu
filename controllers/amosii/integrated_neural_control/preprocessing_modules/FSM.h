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


#ifndef FSM_H_
#define FSM_H_

#include "utils/ann-framework/ann.h"

/**
 * Foot Contact Sensor Mapping Class
 *
 *
 */
class FSM : public ANN {
public:
    /**
     * The constructor
     *
     * Sets synaptic weights and biases
     */
    FSM();
};


#endif /* FSM_H_ */
