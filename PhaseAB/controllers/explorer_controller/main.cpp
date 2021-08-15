#include "CustomRobot.hpp"
#include "ExplorerRobot.hpp"

int main(int argc, char **argv) {
  
  // Runs the robot
  Robot* robot = new Robot();
  ExplorerRobot* er = new ExplorerRobot(robot);

  robot->step(2*TIME_STEP);

  // Main function
  er->explore();

  return 0;
}