#include "CustomRobot.hpp"
#include "PathRobot.hpp"
#include "PathPlanner.hpp"

int main(int argc, char **argv) {
  
  PathPlanner* planner = new PathPlanner();
  // Runs the path planner
  planner->run();
  delete planner;
  
  // Runs the robot
  Robot* robot = new Robot();
  PathRobot* pr = new PathRobot(robot);

  // Main function
  pr->followMP();

  return 0;
}