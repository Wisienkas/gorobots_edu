#ifndef __GOROBOTS_EDU_PROJECTS_TRIBOT_JITTER_H
#define __GOROBOTS_EDU_PROJECTS_TRIBOT_JITTER_H

#include <random>

namespace tribot {
  class Jitter {
    std::uniform_real_distribution<double> unif;
    std::default_random_engine re;
  public:
    Jitter();
    virtual double get() ;
  };
}

#endif // end of __GOROBOTS_EDU_PROJECTS_TRIBOT_JITTER_H
