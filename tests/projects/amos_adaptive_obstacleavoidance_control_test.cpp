/*
 * amosii_example_test.cpp
 *
 *  Created on: May 22, 2014
 *      Author: Poramate
 *      "libtinyxml2-dev" needs to be installed!
 *      Installation can be done using synaptic package manager.
 *      To run unit test
 *      1) ~/workspace/pmanoonpong-gorobots-fork$ make test
 *      2) ~/workspace/pmanoonpong-gorobots-fork$ ./build/run_test
 *
 *      Trick = ~/workspace/pmanoonpong-gorobots-fork$ ./build/run_test --gtest_filter=Amosii*
 *      to test specific program, e.g., Amosii
 */


#include "gtest/gtest.h"

// mask names of objects in global scope
#define main projects_amosii_adaptive_obstacleavoidance_control_main
#define ThisSim ProjectAmosiiAdaptiveObstacleAvoidanceControlMainSim

#include "projects/amosii/adaptive_obstacleavoidance_control/sim/main.cpp"

TEST(AmosiiExampleTest, movement) {

  ThisSim sim;
  sim.setStepLimit(10000); // simulation time, e.g., 10000 time steps
  char *args[] = {"-r","10", "-rtf", "0"};
  sim.run(4, args);
  lpzrobots::Pos final_pos = sim.getFinalPosition(); // print final position
  std::cout<< final_pos.x() << std::endl;
  std::cout<< final_pos.y() << std::endl;
  std::cout<< final_pos.z() << std::endl;
  ASSERT_NEAR(3.89558, final_pos.x(), 5e-2);
  ASSERT_NEAR(-1.30729, final_pos.y(), 5e-2);
  ASSERT_NEAR(0.0439363, final_pos.z(), 5e-2);
}
