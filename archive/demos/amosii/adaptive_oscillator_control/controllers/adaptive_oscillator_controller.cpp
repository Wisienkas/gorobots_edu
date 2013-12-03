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

#include "adaptive_oscillator_controller.h"
#include <ode_robots/amosiisensormotordefinition.h>

AdaptiveOscillatorController::AdaptiveOscillatorController() :
    AbstractController("adaptive amosII controller", "0.0") {
  t = 0;
  feedback = true;
  signal = 0.0;
  average = -0.25;
  shift = 0.1;
}

AdaptiveSO2CPGSynPlas* AdaptiveOscillatorController::cpg() {
  return &cpg_;
}

void AdaptiveOscillatorController::init(int sensornumber, int motornumber, RandGen* randGen) {
  cpg_.setPhi(0.20);
  cpg_.setEpsilon(0.01);
  //cpg.setEpsilon(0.01/5.0);
  cpg_.setGamma(1.0);
  cpg_.setBeta(0.0);
  cpg_.setMu(2.0);
  cpg_.setBetaDynamics(-1.0, 0.02, 0.00);
  cpg_.setGammaDynamics(-1.0, 0.02, 1.00);
  cpg_.setEpsilonDynamics(1.0, 0.02, 0.01);
  //cpg.setEpsilonDynamics( 2.0/25.0, 0.01, 0.01/5.0);
  cpg_.setOutput(0, 0.2);

  addInspectableValue("cpg.o0", &(cpg_.getOutput(0)), "cpg output 0");
  addInspectableValue("cpg.o1", &(cpg_.getOutput(1)), "cpg output 1");
  addInspectableValue("cpg.o2", &(cpg_.getOutput(2)), "cpg output 2");
  addInspectableValue("cpg.phi", &(cpg_.getPhi()), "cpg parameter phi");
  addInspectableValue("cpg.beta", &(cpg_.getBeta()), "cpg weight beta");
  addInspectableValue("cpg.epsilon", &(cpg_.getEpsilon()), "cpg weight epsilon");
  addInspectableValue("cpg.gamma", &(cpg_.getGamma()), "cpg weight gamma");
  addInspectableValue("cpg.p", &(cpg_.getPerturbation()), "cpg perturbation");
  addInspectableValue("avg", &(average), "average");
  addInspectableValue("sgn", &(signal), "signal");
}

void AdaptiveOscillatorController::enableFeedback(const bool enabled) {
  feedback = enabled;
}

int AdaptiveOscillatorController::getSensorNumber() const {
  return 0;
}

int AdaptiveOscillatorController::getMotorNumber() const {
  return 0;
}

void AdaptiveOscillatorController::setShift(const double& ashift) {
  shift = ashift;
}

void AdaptiveOscillatorController::step(const sensor* sensors, int sensornumber, motor* motors, int motornumber) {
  //assert(motornumber >= 19);

  const int inv = +1;

  signal = tanh(10.0 * (sensors[TR0_as] - average));
  average = 0.01 * signal + 1.0 * average;
  const double P = inv * feedback * 0.2 * signal; //tanh(200*signal);
  cpg_.setPerturbation(P);

  const double alpha = 6.0 * std::cos(shift * M_PI);
  const double beta = 6.0 * std::sin(shift * M_PI);

  const double ctr_min = 0.4;
  const double ctr_max = 0.9;
  const double fti_mid = -1.0;

  const double ctr_mid = 0.5 * (ctr_min + ctr_max);
  const double ctr_amp = 0.5 * (ctr_max - ctr_min);

  // generate motor commands
  const double s1 = tanh(+alpha * cpg_.getOutput(0) + beta * cpg_.getOutput(1));
  const double s2 = tanh(-beta * cpg_.getOutput(0) + alpha * cpg_.getOutput(1));
  motors[TR0_m] = 0.36 + inv * 0.36 * s1;
  motors[CR0_m] = ctr_mid + ctr_amp * s2;
  motors[FR0_m] = fti_mid;

  motors[TL0_m] = 0.36 - inv * 0.36 * s1;
  motors[CL0_m] = ctr_mid - ctr_amp * s2;
  motors[FL0_m] = fti_mid;

  motors[TR1_m] = -0.09 - inv * 0.42 * s1;
  motors[CR1_m] = ctr_mid - ctr_amp * s2;
  motors[FR1_m] = fti_mid;

  motors[TL1_m] = -0.09 + inv * 0.42 * s1;
  motors[CL1_m] = ctr_mid + ctr_amp * s2;
  motors[FL1_m] = fti_mid;

  motors[TR2_m] = -0.50 + inv * 0.36 * s1;
  motors[CR2_m] = ctr_mid + ctr_amp * s2;
  motors[FR2_m] = fti_mid;

  motors[TL2_m] = -0.50 - inv * 0.36 * s1;
  motors[CL2_m] = ctr_mid - ctr_amp * s2;
  motors[FL2_m] = fti_mid;

  motors[BJ_m] = 0;

  t++;

  cpg_.step();
}

void AdaptiveOscillatorController::stepNoLearning(const sensor*, int number_sensors, motor*, int number_motors) {

}

bool AdaptiveOscillatorController::store(FILE* f) const {
  return false;
}
;
bool AdaptiveOscillatorController::restore(FILE* f) {
  return false;
}
;

