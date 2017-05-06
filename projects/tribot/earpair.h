// HEADER GUARD
#ifndef __GOROBOTS_EDU_PROJECTS_TRIBOT_EAR_PAIR_H
#define __GOROBOTS_EDU_PROJECTS_TRIBOT_EAR_PAIR_H

#include <lizard_ear.h>

namespace tribot {

  class EarPair {
  private:
    lizard_ear ear;
    lpzrobots::Position& soundSource;
  public:
    EarPair(const Position& soundSource, const Position& microphones, double * orientation);

    tribot::Output()
  };
}

#endif // end of __GOROBOTS_EDU_PROJECTS_TRIBOT_EAR_PAIR_H
