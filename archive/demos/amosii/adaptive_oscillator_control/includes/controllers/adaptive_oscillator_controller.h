/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Göttingen                           *
 *    timo.nachstedt@gmail.com                                             *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
  ***************************************************************************/

#ifndef ADAPTIVE_OSCILLATOR_CONTROLLER_H_
#define ADAPTIVE_OSCILLATOR_CONTROLLER_H_

#include <selforg/abstractcontroller.h>
#include <utils/ann-library/adaptiveso2cpgsynplas.h>

class AdaptiveOscillatorController : public AbstractController {

  public:
    AdaptiveOscillatorController();
    AdaptiveSO2CPGSynPlas* cpg();
    void init(int sensornumber, int motornumber, RandGen* randGen = 0);
    void enableFeedback(const bool enabled);
    int getSensorNumber() const;
    int getMotorNumber() const;
    void setShift(const double& ashift);

    void step(const sensor* sensors, int sensornumber,
        motor* motors, int motornumber);
    void stepNoLearning(const sensor* , int number_sensors,
        motor* , int number_motors);
    bool store(FILE* f) const;
    bool restore(FILE* f);
private:
    AdaptiveSO2CPGSynPlas cpg_;
    int t;
    bool feedback;
    double average, signal;
    double shift;
};


#endif /* ADAPTIVE_OSCILLATOR_CONTROLLER_H_ */
