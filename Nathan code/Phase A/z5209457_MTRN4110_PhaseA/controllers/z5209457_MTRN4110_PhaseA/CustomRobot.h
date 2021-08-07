
#ifndef CUSTOMROBOT_H
#define CUSTOMROBOT_H

#define TIMESTEP 64
#define PI 3.14159
#define CELL_SIZE 165
#define WALL_THICK 15
#define WHEEL_R 20
#define MAX_ROT 6.28

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Compass.hpp>

using namespace webots;

class CustomRobot {
  
  // Define class variables for ease of use between functions
  Robot* r;
  Motor* lmotor;
  Motor* rmotor;
  PositionSensor* lpos;
  PositionSensor* rpos;
  DistanceSensor* ldist;
  DistanceSensor* rdist;
  DistanceSensor* fdist;
  Compass* compass;
  
private:

  // Defines methods for use within the class itself
  double getLPos();
  double getRPos();
  double getBearing();
  
public:

  CustomRobot(Robot*);
  ~CustomRobot(); 
  
  // Defines methods to interact with robot
  void forward();
  void turnLeft();  
  void turnRight();  
  void stop();
  bool leftWall();
  bool rightWall();
  bool forwardWall();  

};

#endif