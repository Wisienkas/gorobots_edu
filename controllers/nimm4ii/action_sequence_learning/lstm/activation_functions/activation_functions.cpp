/******************************************************************************
 * Copyright (c) 2013 Johannes Bergmann, Felix Weninger, Bjoern Schuller
 * Institute for Human-Machine Communication
 * Technische Universitaet Muenchen (TUM)
 * D-80290 Munich, Germany
 *
 * This file is part of CURRENNT.
 *
 * CURRENNT is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CURRENNT is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with CURRENNT.  If not, see <http://www.gnu.org/licenses/>.
 *****************************************************************************/

#ifndef ACTIVATION_FUNCTIONS_CPP
#define ACTIVATION_FUNCTIONS_CPP

#include "../helpers/NumericLimits.cpp"
#include <math.h>

namespace activation_functions {

    struct Identity
    {
        static float fn(float x)
        {
            return x;
        }

    };
    struct Logistic
    {
        static float fn(float x)
        {
            if (x < helpers::NumericLimits<float>::expLimit()) {
                if (x > -helpers::NumericLimits<float>::expLimit())
                    return (float)1.0 / ((float)1.0 + exp(-x));
                else
                    return 0;
            }
            
            return 1;
        }

    };
    struct Maxmin1
    {
        static float fn(float x)
        {
            return ((float)2.0 * Logistic::fn(x) - (float)1.0);
        }
    };    
    
    struct Maxmin2
    {
        static float fn(float x)
        {
            return ((float)4.0 * Logistic::fn(x) - (float)2.0);
        }
    };
    
    struct Max2min0
    {
        static float fn(float x)
        {
            return (float)2.0 * Logistic::fn(x);
        }

    };
    
    struct Tanh
    {
        static float fn(float x)
        {
            return Maxmin1::fn((float)2.0 * x);
        }
    };

}


#endif
