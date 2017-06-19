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
#include "toolbox.h"
#include "jitter.h"
#include <cmath>
#include "sinewave.h"

/**
 * Used to extract data about the 2 sinewaves, one being noisy the other clean
 */
void writeSamplesToFile(std::string filename) {
  double frequency = 1900;
  double phaseshift = 0; // no phaseshift needed

  tribot::Sinewave cleanWave = tribot::Sinewave(frequency, phaseshift, false);
  tribot::Sinewave noisyWave = tribot::Sinewave(frequency, phaseshift, true);

  int samples = 500;
  int samplerate = 50000;

  double from = 0.0;
  double to = (double) samples / samplerate;
  double step = 1.0 / samplerate;

  std::vector<double> cleanData = cleanWave.sample(from, to, step);
  std::vector<double> noisyData = noisyWave.sample(from, to, step);

  std::fstream stream;
  stream.open(filename, std::fstream::out);
  // Adding CSV Header
  stream << "\"step\",\"clean\",\"noisy\"\n";
  int count = 1;
  for(std::pair<std::vector<double>::iterator, std::vector<double>::iterator>
        i(cleanData.begin(), noisyData.begin());
      i.first != cleanData.end(); ++i.first, ++i.second) {
    stream << ((double)count * step) << "," << (double)*i.first << "," << *i.second << "\n";
    count++;
  }
  stream.close();
}

void runSim(int argc, char **argv, bool, std::string);

#include "simulationbuilder.h"
int main(int argc, char **argv) {
  //writeSamplesToFile("shownoise.csv");

  // Will run a single example of the simulation with the parameters given here.
  tribot::TribotSimulation sim = createSim(tribot::SimulationBuilder()
                                           .setGoal(2.5, -20, 0, 3)
                                           .setName("test1")
                                           .setSoundGenerator(tribot::SoundGenerator(false))
                                           .build());
  sim.run(argc, argv);


  // Will run through all the different experiments
  /*for(int i = 1; i <= 5; i++) {
    runSim(argc, argv, false, "cleansim" + std::to_string(i) + "-");
    runSim(argc, argv, true, "noisysim" + std::to_string(i) + "-");
    }*/
}

/**
 * Will run all the different simulations
 */
void runSim(int argc, char **argv, bool noise, std::string prefix) {
  tribot::SoundGenerator sg(noise);
  // Robots start parallel, goal straight ahead
  createSim(tribot::SimulationBuilder()
            .setName(prefix + "1_1")
            .setSoundGenerator(sg)
            .build()).run(argc, argv);
  // Robots start parallel, goal to the left of both robots
  createSim(tribot::SimulationBuilder()
            .setName(prefix + "1_2")
            .setSoundGenerator(sg)
            .setGoal(-10, 20, 0, 4.0)
            .build()).run(argc, argv);
  // Robots start parallel, goal to the right of both robots
  createSim(tribot::SimulationBuilder()
            .setName(prefix + "1_3")
            .setGoal(15, 20, 0, 4.0)
            .setSoundGenerator(sg)
            .build()).run(argc, argv);
  // Robots start shifted, goal straight ahead
  createSim(tribot::SimulationBuilder()
            .setName(prefix + "2_1")
            .setAgent1(0,-5,0, 0)
            .setSoundGenerator(sg)
            .build()).run(argc, argv);
  // Robots start shifted, goal to the left of both robots
  createSim(tribot::SimulationBuilder()
            .setName(prefix + "2_2")
            .setAgent1(0,-5,0, 0)
            .setGoal(-10,20,0, 4.0)
            .setSoundGenerator(sg)
            .build()).run(argc, argv);
  // Robots start shifted, goal to the right of both robots
  createSim(tribot::SimulationBuilder()
            .setName(prefix + "2_3")
            .setAgent1(0,-5,0, 0)
            .setGoal(15,20,0, 4.0)
            .setSoundGenerator(sg)
            .build()).run(argc, argv);
}
