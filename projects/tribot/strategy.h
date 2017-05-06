// HEADER GUARD
#ifndef __GOROBOTS_EDU_PROJECTS_TRIBOT_STRATEGY_H
#define __GOROBOTS_EDU_PROJECTS_TRIBOT_STRATEGY_H

#include <selforg/abstractmeasure.h>

#include <string>
#include <braitenberg.h>

namespace tribot {

  class Strategy : public AbstractMeasure {

  private:

  public:
    Strategy(std::string name);

    virtual Output calcStep();
  };

}

#endif // end of __GOROBOTS_EDU_PROJECTS_TRIBOT_STRATEGY_H
