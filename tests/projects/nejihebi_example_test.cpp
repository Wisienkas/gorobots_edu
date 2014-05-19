/*
 * nejihebi_example_test.cpp
 *
 *  Created on: Jan 24, 2014
 *      Author: timo
 */

#include "gtest/gtest.h"

// mask names of objects in global scope
#define main projects_nejihebi_example_main
#define ThisSim ProjectNejihebiExampleMainSim

#include "projects/nejihebi/example/sim/main.cpp"

TEST(NejihebiExampleTest, movement) {
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
  sim.setStepLimit(50);
  char *args[] = {"-nographics"};
  sim.run(1, args);
  lpzrobots::Pos final_pos = sim.getFinalPosition();
  ASSERT_NEAR(-3.817293, final_pos.x(), 1e-4);
  ASSERT_NEAR(-0.230475, final_pos.y(), 1e-4);
  ASSERT_NEAR( 0.844358, final_pos.z(), 1e-4);
}

