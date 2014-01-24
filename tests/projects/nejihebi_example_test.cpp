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

namespace {
  TEST(NejihebiExampleTest, movement) {
    // increase stack size
    const rlim_t kStackSize = 32 * 1024 * 1024;   // min stack size = 16 MB
    struct rlimit rl;
    getrlimit(RLIMIT_STACK, &rl);
    if (rl.rlim_cur < kStackSize) {
        rl.rlim_cur = kStackSize;
        setrlimit(RLIMIT_STACK, &rl);
    }
    ThisSim sim;
    ASSERT_TRUE(sim.test());
  }
}

