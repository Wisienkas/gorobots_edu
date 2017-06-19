#ifndef __GOROBOTS_EDU_PROJECTS_TRIBOT_BRAITEN_BERG_H
#define __GOROBOTS_EDU_PROJECTS_TRIBOT_BRAITEN_BERG_H

#include "lizard_ear.h"
#include "soundgenerator.h"

namespace tribot {

  struct Output {
    double left;
    double right;

  Output(double left, double right) : left(left), right(right) {}
    Output(){}
    inline double getDifference() { return left - right; }
  };

  class BraitenBerg {
  private:

    lizard_ear lizardEar;
    SoundGenerator soundGenerator;

    Output normalizedEarOutput(std::vector<double> left, std::vector<double> right);
  public:
    const int SAMPLE_RATE = 50000; // 50 kHZ sampling for lizard ear
    const int SAMPLES = 500; // 500 samples per sampling session for lizard ear
    const int FREQUENCY = 1700; // sample frequency for lizard ear
    const int DB_SCALAR = 20; // log10(lizard_ear) * 20

    BraitenBerg(SoundGenerator soundGenerator);
    Output calculateOutput(double angle);
  };
}

#endif // end of __GOROBOTS_EDU_PROJECTS_TRIBOT_BRAITENBERG_H
