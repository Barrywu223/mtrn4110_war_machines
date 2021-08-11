#include "CustomRobot.hpp"
#include "PathRobot.hpp"
#include "PathPlanner.hpp"

// unsigned stepIndex = 3;
// Message message;

int main(int argc, char **argv) {
  
  PathPlanner* planner = new PathPlanner();
  // Runs the path planner
  planner->run();
  delete planner;
  
  // create the webots base Robot
  Robot* robot = new Robot();
  // Step world so data is available from sensors
  // robot->step(2*robot->getBasicTimeStep()); 
  // Create wrapper classes
  PathRobot* pr = new PathRobot(robot);

  // Main function
  pr->followMP();


  return 0;

  // // create the Robot instance.  
  // Robot *robot = new Robot();
  
  // // initialize motor, compass and distance sensors
  // Motor *leftMotor = robot->getMotor("left wheel motor");
  // Motor *rightMotor = robot->getMotor("right wheel motor");
  // Compass *compass = robot->getCompass("compass");
  // compass->enable(TIME_STEP);
  // DistanceSensor *ds[3];
  // char dsNames[3][10] = {"ds_right","ds_front","ds_left"};
  // for(int i = 0; i < 3; i++){
  //   ds[i] = robot->getDistanceSensor(dsNames[i]);
  //   ds[i]->enable(TIME_STEP);
  // }  
  
  // // read the content form MotionPlan.txt
  // string textContent = readMotionPlan();
  // message.row = textContent[0] - 48;
  // message.column = textContent[1] - 48;
  // message.heading = textContent[2];
  
  // while (robot->step(TIME_STEP) != -1) {
  //   leftMotor->setPosition(INFINITY);
  //   rightMotor->setPosition(INFINITY);
  //   auto totalStep = textContent.length();
  //   for(stepIndex = 3; stepIndex < totalStep; ++stepIndex){;
  //     char command = textContent.at(stepIndex);
  //     obstacleDetection(ds,message);
  //     wirteMotionExectuion(myFile, message);
  //     printMessage(message);
  //     updateMessage(message, command);
  //     if (command == 'F'){
  //       go_forward(leftMotor, rightMotor);
  //       robot->step(1700);
  //     }
  //     else if (command == 'L'){
  //       turn_left(leftMotor, rightMotor);
  //       robot->step(1400);
  //     }
  //     else if (command == 'R'){
  //       turn_right(leftMotor, rightMotor);
  //       robot->step(1400);
  //     }
  //     // rotational correctoin by compass      
  //     angleCorrection(compass, message.heading, leftMotor, rightMotor, robot);
  //     // linear correction if a wall is ahead
  //     if (ds[1]->getValue() < distanceThreshold){
  //       linearCorrection(ds[1], leftMotor, rightMotor, robot);
  //     }
  //   }
  //   obstacleDetection(ds, message);
  //   wirteMotionExectuion(myFile, message);
  //   printMessage(message);
  //   break;
  // };


  // // Enter here exit cleanup code.
  // cout << PRELIX << "Motion plan executed!" << endl;
  // turn_off(leftMotor, rightMotor);
  // delete robot;
  // myFile.close();
  // return 0;
}