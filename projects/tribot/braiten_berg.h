#ifndef __GOROBOTS_EDU_PROJECTS_TRIBOT_BRAITEN_BERG_H
#define __GOROBOTS_EDU_PROJECTS_TRIBOT_BRAITEN_BERG_H

#include "lizard_ear.h"

namespace tribot {

  struct Output {
    double left;
    double right;

    inline double getDifference() { return left - right; }
  };

  class BraitenBerg {
  private:

    double weight;
    lizard_ear lizardEar;

    Output normalizedEarOutput(std::vector<double> left, std::vector<double> right);
  public:
    const int SAMPLE_RATE = 50000; // 50 kHZ sampling for lizard ear
    const int SAMPLES = 500; // 500 samples per sampling session for lizard ear
    const int FREQUENCY = 1700; // sample frequency for lizard ear
    const int DB_SCALAR = 20; // log10(lizard_ear) * 20

    BraitenBerg(double weight);
    Output calculateOutput(double angle);
  };
}

#endif // end of __GOROBOTS_EDU_PROJECTS_TRIBOT_BRAITENBERG_H
