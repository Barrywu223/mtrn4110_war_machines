// File:          z5209457_main_controller.cpp       
// Description:   Phase A code for MTRN4110
// Author:        Nathan Long

#include <CustomRobot.h>
#include <PathRobot.h>

using namespace webots;

int main(int argc, char **argv) {
  
  // create the webots base Robot
  Robot* robot = new Robot();
  // Step world so data is available from sensors
  robot->step(2*robot->getBasicTimeStep()); 
  // Create wrapper classes
  PathRobot* pr = new PathRobot(robot);

  // Main function
  pr->followMP();

  return 0;
}
