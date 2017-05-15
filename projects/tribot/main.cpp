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


#include "soundgenerator.h"
#include <string>
#include <fstream>
#include <iostream>
#include "sound.h"
#include <cmath>

void writeSamplesToFile(std::string filename, double angle) {
  tribot::SoundGenerator sg;
  tribot::Sound sound = sg.sample(angle, 500, 50000, 1900);
  std::fstream stream;
  stream.open(filename, std::fstream::out);
  stream << "\"left\",\"right\"\n";
  std::vector<double> left = sound.left;
  std::vector<double> right = sound.right;

  for(std::pair<std::vector<double>::iterator, std::vector<double>::iterator>
        i(left.begin(), right.begin());
      i.first != left.end(); ++i.first, ++i.second) {
    stream << *i.first << "," << *i.second << "\n";
  }
  stream.close();
}

void runSim(int argc, char **argv);

#include "simulationbuilder.h"
int main(int argc, char **argv) {
  /*writeSamplesToFile("sound0.csv", 0);
  writeSamplesToFile("sound45.csv", M_PI / 4);
  writeSamplesToFile("sound90.csv", M_PI / 2);
  writeSamplesToFile("sound135.csv", 3 * M_PI / 4);
  writeSamplesToFile("sound-135.csv", -3 * M_PI / 4);
  writeSamplesToFile("sound-45.csv", -1 * M_PI / 4);
  writeSamplesToFile("sound-90.csv", M_PI / -2);*/
  createSim(tribot::SimulationBuilder()
            //.setAgent2(5,2,0, 0)
            .setGoal(-10, 20, 0, 3)
            .setName("test1")
            .build())
            .run(argc, argv);
  //runSim(argc, argv);
}

void runSim(int argc, char **argv) {
  // Robots start parallel, goal straight ahead
  createSim(tribot::SimulationBuilder()
            .setName("Simulation 1")
            .build()).run(argc, argv);
  // Robots start parallel, goal to the left of both robots
  createSim(tribot::SimulationBuilder()
            .setName("Simulation 2")
            .setGoal(-10, 20, 0, 4.0)
            .build()).run(argc, argv);
  // Robots start parallel, goal to the right of both robots
  createSim(tribot::SimulationBuilder()
            .setName("Simulation 3")
            .setGoal(15, 20, 0, 4.0)
            .build()).run(argc, argv);
  // Robots start shifted, goal straight ahead
  createSim(tribot::SimulationBuilder()
            .setName("Simulation 4")
            .setAgent1(0,-5,0, 0)
            .build()).run(argc, argv);
  // Robots start shifted, goal to the left of both robots
  createSim(tribot::SimulationBuilder()
            .setName("Simulation 5")
            .setAgent1(0,-5,0, 0)
            .setGoal(-10,20,0, 4.0)
            .build()).run(argc, argv);
  // Robots start shifted, goal to the right of both robots
  createSim(tribot::SimulationBuilder()
            .setName("Simulation 6")
            .setAgent1(0,-5,0, 0)
            .setGoal(15,20,0, 4.0)
            .build()).run(argc, argv);
  // Robots start with different orientation, goal straight ahead
  createSim(tribot::SimulationBuilder()
            .setName("Simulation 7")
            .setAgent1(0,0,0, M_PI / 4)
            .build()).run(argc, argv);
}
