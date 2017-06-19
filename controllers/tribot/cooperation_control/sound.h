#ifndef __GOROBOTS_EDU_PROJECTS_TRIBOT_SOUND_H
#define __GOROBOTS_EDU_PROJECTS_TRIBOT_SOUND_H

#include <vector>

namespace tribot {
  struct Sound {
    std::vector<double> left;
    std::vector<double> right;
  };
}

#endif // end of __GOROBOTS_EDU_PROJECTS_TRIBOT_SOUND_H
