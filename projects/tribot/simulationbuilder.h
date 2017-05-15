#ifndef __GOROBOTS_EDU_PROJECTS_TRIBOT_SIMULATION_BUILDER_H
#define __GOROBOTS_EDU_PROJECTS_TRIBOT_SIMULATION_BUILDER_H

#include <string>
#include "tribotGoal.h"
#include "tribotAgentConfig.h"
#include "tribotSimTaskHandle.h"
#include "soundgenerator.h"
#include <ode_robots/pos.h>
#include <experimental/optional>

namespace tribot {

  class SimulationBuilder {
  private:
    std::experimental::optional<TribotAgentConfig> agent1 =
      TribotAgentConfig(lpzrobots::Pos(0,0,0), .0, "Agent1");
    std::experimental::optional<TribotAgentConfig> agent2 =
      TribotAgentConfig(lpzrobots::Pos(5,0,0), .0, "Agent2");
    std::experimental::optional<TribotGoal> goal =
      TribotGoal(Position(2.5, 20,0), 5);
    std::experimental::optional<std::string> name = std::string("sim1");
    std::experimental::optional<SoundGenerator> soundGenerator
      = SoundGenerator();

  public:
    SimulationBuilder(){};
    SimulationBuilder setAgent1(double x, double y, double z, double orientation);
    SimulationBuilder setAgent2(double x, double y, double z, double orientation);
    SimulationBuilder setGoal(double x, double y, double z, double goalCondition);
    SimulationBuilder setSoundGenerator(SoundGenerator soundgenerator);
    SimulationBuilder setName(std::string name);
    TribotSimTaskHandle build();
  };

}

#endif // end of __GOROBOTS_EDU_PROJECTS_TRIBOT_SIMULATION_BUILDER_H
