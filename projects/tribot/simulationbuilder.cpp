#include "simulationbuilder.h"

namespace tribot {
  SimulationBuilder SimulationBuilder::setAgent1(double x, double y, double z, double orientation) {
    agent1 = TribotAgentConfig(lpzrobots::Pos(x,y,z), orientation, "agent1");
    return *this;
  }

  SimulationBuilder SimulationBuilder::setAgent2(double x, double y, double z, double orientation) {
    agent2 = TribotAgentConfig(lpzrobots::Pos(x,y,z), orientation, "agent2");
    return *this;
  }

  SimulationBuilder SimulationBuilder::setGoal(double x, double y, double z, double goalCondition) {
    goal = TribotGoal(Position(x,y,z), goalCondition);
    return *this;
  }

  SimulationBuilder SimulationBuilder::setSoundGenerator(SoundGenerator soundGenerator) {
    this->soundGenerator = soundGenerator;
    return *this;
  }

  SimulationBuilder SimulationBuilder::setName(std::string name) {
    this->name = name;
    return *this;
  }

  TribotSimTaskHandle SimulationBuilder::build() {
    return TribotSimTaskHandle(*agent1,
                               *agent2,
                               *goal,
                               *name,
                               *soundGenerator);
  }
}
