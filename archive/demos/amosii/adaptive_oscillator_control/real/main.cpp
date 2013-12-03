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

// simple wiring
#include <selforg/one2onewiring.h>
// serial interface to AMOSII version 2
#include "amosiiserialv2.h"
// base class for real robot experiments
#include "realexperiment.h"
// adaptive controller
#include "adaptive_oscillator_controller.h"

class ThisExperiment : public lpzrobots::RealExperiment{
  public:
    void start(lpzrobots::RealExperimentGlobalData& globaldata, PlotOptionList plotoptions) {
      controller = new AdaptiveOscillatorController();

      One2OneWiring* wiring = new One2OneWiring(0, true);
      lpzrobots::AmosIISerialV2* robot =
          new lpzrobots::AmosIISerialV2("/dev/ttyUSB1"); // /dev/ttyS0
      Agent* agent = new Agent(plotoptions);
      agent->init(controller, robot, wiring);

      controller->enableFeedback(true);
      controller->cpg()->setPhi(0.02);
      controller->setShift(0.20);

      globaldata.agents.push_back(agent);
      globaldata.configs.push_back(robot);
      globaldata.configs.push_back(controller);

    }
  private:
    AdaptiveOscillatorController* controller;
};

int main(int argc, char **argv) {
  ThisExperiment exp;
  exp.setControlFrequency(50);
  return exp.run(argc, argv);
}

