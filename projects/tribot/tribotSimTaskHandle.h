// HEADER GUARD
#ifndef __GOROBOTS_EDU_PROJECTS_TRIBOT_SIM_TASK_HANDLE_H
#define __GOROBOTS_EDU_PROJECTS_TRIBOT_SIM_TASK_HANDLE_H

#include "tribotAgentConfig.h"
#include "tribotGoal.h"
#include "soundgenerator.h"
#include <ode_robots/simulationtaskhandle.h>
#include <string>

namespace tribot {
  struct TribotSimTaskHandle : lpzrobots::SimulationTaskHandle {
    TribotAgentConfig agent1;
    TribotAgentConfig agent2;
    TribotGoal goal;
    std::string name;
    SoundGenerator soundGenerator;

    TribotSimTaskHandle(TribotAgentConfig agent1,
                        TribotAgentConfig agent2,
                        TribotGoal goal,
                        std::string name,
                        SoundGenerator soundGenerator)
      : agent1(agent1),
      agent2(agent2),
      goal(goal),
      name(name),
      soundGenerator(soundGenerator) {}
      };
}

#endif // end of __GOROBOTS_EDU_PROJECTS_TRIBOT_SIM_TASK_HANDLE_H
