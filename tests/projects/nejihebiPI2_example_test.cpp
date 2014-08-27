/*
 * nejihebi_example_test.cpp
 *
 *  Created on: Jan 24, 2014
 *      Author: timo
 */

#include "gtest/gtest.h"

// mask names of objects in global scope
#define main projects_nejihebi_PI2_main
#define ThisSim ProjectNejihebiPI2MainSim

#include "projects/nejihebi/PI2_reinforcement_learning/sim/main.cpp"

TEST(NejihebiPI2Test, movement) {
  // increase stack size (Only necessary for Nejihebi robot)
  const rlim_t kStackSize = 32 * 1024 * 1024;   // min stack size = 16 MB
  struct rlimit rl;
  getrlimit(RLIMIT_STACK, &rl);
  if (rl.rlim_cur < kStackSize) {
      rl.rlim_cur = kStackSize;
      setrlimit(RLIMIT_STACK, &rl);
  }
  // end of increasing stack size
  ThisSim sim;
  sim.setStepLimit(640);
  //char *args[] = {"-nographics"};
  //sim.run(1, args);

  char *args[] = {"-r","10", "-rtf", "0"};
  sim.run(4, args);

  lpzrobots::Pos final_pos = sim.getFinalPosition();
  ASSERT_NEAR(-6.10691, final_pos.x(), 0.5);
  ASSERT_NEAR(4.75599, final_pos.y(), 0.5);
  ASSERT_NEAR( 0.846421, final_pos.z(), 0.01);






}

