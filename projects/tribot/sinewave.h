#ifndef __GOROBOTS_EDU_PROJECTS_TRIBOT_SINEWAVE_H
#define __GOROBOTS_EDU_PROJECTS_TRIBOT_SINEWAVE_H

#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>
#include "jitter.h"

namespace tribot {

  const double EAR_DISTANCE_MM = 13;
  const double SPEED_OF_SOUND = 340000;

  double generatePhaseshift(double angle, double frequency);

  class Sinewave {
  private:
    double frequency;
    double phaseshift;
    bool noise;
  public:
    static Jitter jitter;

    Sinewave(double frequency, double phaseshift, bool noise);

    std::vector<double> sample(std::vector<double> series) ;
    std::vector<double> sample(double from, double to, double stepSize) ;
    double sample(double t);
    double getNoise();
  };
}

#endif // __GOROBOTS_EDU_PROJECTS_TRIBOT_SINEWAVE_H
