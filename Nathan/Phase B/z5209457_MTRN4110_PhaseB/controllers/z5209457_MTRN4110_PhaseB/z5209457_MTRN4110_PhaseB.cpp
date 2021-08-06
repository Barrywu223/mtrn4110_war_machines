// File:          z5209457_ZMTRN4110_PhaseB.cpp       
// Description:   Phase B code for MTRN4110
// Author:        Nathan Long

#include <webots/Robot.hpp>
#include <PathPlanner.h>

using namespace webots;

int main(int argc, char **argv) {
  
  // Create wrapper classes
  PathPlanner* planner = new PathPlanner();
  
  // Runs the path planner
  planner->run();
  delete planner;

  return 0;
}
