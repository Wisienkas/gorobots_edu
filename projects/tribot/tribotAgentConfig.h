// HEADER GUARD
#ifndef __GOROBOTS_EDU_PROJECTS_TRIBOT_AGENT_CONFIG_H
#define __GOROBOTS_EDU_PROJECTS_TRIBOT_AGENT_CONFIG_H

#include <ode_robots/pos.h>
#include <string>

namespace tribot {
  struct TribotAgentConfig {
    lpzrobots::Pos position;
    double orientation;
    std::string name;

    TribotAgentConfig(lpzrobots::Pos position,
                double orientation,
                std::string name)
    : position(position),
      orientation(orientation),
      name(name) {};
  };
}

#endif // end of __GOROBOTS_EDU_PROJECTS_TRIBOT_AGENT_CONFIG_H
