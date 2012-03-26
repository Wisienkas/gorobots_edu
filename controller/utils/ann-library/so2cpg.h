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


#ifndef SO2CPG_H_
#define SO2CPG_H_

#include "utils/ann-framework/ann.h"

class SO2CPG : public ANN
{
public:
    SO2CPG();
    const double& getAlpha() const;
    const double& getPhi() const;
    void setAlpha(const double aalpha);
    void setPhi(const double& aphi);
protected:
    void updateSO2Weights();
private:
    double phi;
    double alpha;

};


#endif /* SO2CPG_H_ */
