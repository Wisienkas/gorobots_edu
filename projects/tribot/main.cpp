#include "tribotSimulation.h"
#include "tribotSimTaskHandle.h"
#include "tribotAgentConfig.h"
#include "tribotGoal.h"

#include <ode_robots/pos.h>
#include <selforg/position.h>

tribot::TribotSimulation createSim(tribot::TribotSimTaskHandle context)
{
  tribot::TribotSimulation sim = tribot::TribotSimulation(context);
  sim.setSimTaskHandle(context);

  return sim;
}

int main(int argc, char **argv) {
  createSim(tribot::TribotSimTaskHandle(tribot::TribotAgentConfig(lpzrobots::Pos(0, 0, 0),
                                                                  0.0,
                                                                  "agent1"),
                                        tribot::TribotAgentConfig(lpzrobots::Pos(5, 0, 0),
                                                                  0.0,
                                                                  "agent2"),
                                        tribot::TribotGoal(Position(-10,20,0)),
                                        "Simulation 1")).run(argc, argv);
}
