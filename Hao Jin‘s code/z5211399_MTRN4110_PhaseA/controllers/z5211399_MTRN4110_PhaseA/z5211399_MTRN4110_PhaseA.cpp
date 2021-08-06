// File: z5211399_MTRN4110_PhaseA.cpp
// Date: 18/6/2021          
// Description: Controller of E-puck for phase A - Driving and Perception
// Author: Hao Jin
// Modifications:
// Platform: Windows 10
// Notes: The motion sequence is read form MotionPlan.txt

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes

#include <fstream>
#include <string>
#include <iomanip>
#include <map>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Compass.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;
const double Pi = 3.1415926;
const double distanceThreshold = 850;
const double MaxRotaionSpeed = 6.28 * 0.75; // Max. rotation speed = 6.28 rad/s
const string MOTION_PLAN_FILE_NAME = "../../MotionPlan2.txt";
const string MOTION_EXUCUTION_FILE_NAME = "../../MotionExecution.csv";
const string PRELIX = "[z5211399_MTRN4110_PhaseA] ";
unsigned stepIndex = 3;
const int TIME_STEP = 64;

struct ds_result
{
    bool r;
    bool l;
    bool f;
};
struct Message
{
  int step = 0;
  int row;
  int column;
  char heading;
  char leftWall;
  char frontWall;
  char rightWall;
};
Message message;

// created function
string readMotionPlan();
void go_forward(Motor *leftMotor, Motor *rightMotor);  
void obstacleDetection(DistanceSensor **ps, Message &m);
void turn_right(Motor *leftMotor, Motor *rightMotor);
void turn_left(Motor *leftMotor, Motor *rightMotor);
void turn_off(Motor *leftMotor, Motor *rightMotor);
void updateMessage(char command);
void printMessage();
void angleCorrection(Compass *compass, char heading, Motor *leftMotor, Motor *rightMotor, Robot *robot);
void linearCorrection(DistanceSensor *f, Motor *leftMotor, Motor *rightMotor, Robot *robot);
void wirteMotionExectuion(ofstream &myFile);


int main(int argc, char **argv) {
  // creat scv file
  ofstream myFile(MOTION_EXUCUTION_FILE_NAME);
  myFile << "Step,Row,Column,Heading,Left Wall,Front Wall,Right Wall" 
  << endl;
  
  // create the Robot instance.  
  Robot *robot = new Robot();
  
  // initialize motor, compass and distance sensors
  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");
  Compass *compass = robot->getCompass("compass");
  compass->enable(TIME_STEP);
  DistanceSensor *ds[3];
  char dsNames[3][10] = {"ds_right","ds_front","ds_left"};
  for(int i = 0; i < 3; i++){
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);
  }  
  
  // read the content form MotionPlan.txt
  string textContent = readMotionPlan();
  message.row = textContent[0] - 48;
  message.column = textContent[1] - 48;
  message.heading = textContent[2];
  
  while (robot->step(TIME_STEP) != -1) {
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    auto totalStep = textContent.length();
    for(stepIndex = 3; stepIndex < totalStep; ++stepIndex){;
      char command = textContent.at(stepIndex);
      obstacleDetection(ds,message);
      wirteMotionExectuion(myFile);
      printMessage();
      updateMessage(command);
      if (command == 'F'){
        go_forward(leftMotor, rightMotor);
        robot->step(1700);
      }
      else if (command == 'L'){
        turn_left(leftMotor, rightMotor);
        robot->step(1400);
      }
      else if (command == 'R'){
        turn_right(leftMotor, rightMotor);
        robot->step(1400);
      }
      // rotational correctoin by compass      
      angleCorrection(compass, message.heading, leftMotor, rightMotor, robot);
      // linear correction if a wall is ahead
      if (ds[1]->getValue() < distanceThreshold){
        linearCorrection(ds[1], leftMotor, rightMotor, robot);
      }
    }
    obstacleDetection(ds, message);
    wirteMotionExectuion(myFile);
    printMessage();
    break;
  };


  // Enter here exit cleanup code.
  cout << PRELIX << "Motion plan executed!" << endl;
  turn_off(leftMotor, rightMotor);
  delete robot;
  myFile.close();
  return 0;
}

string readMotionPlan(){
  ifstream textFile(MOTION_PLAN_FILE_NAME); 
  string textContent; 
  getline(textFile, textContent);  
  cout << PRELIX << "Reading in motion plan from " << MOTION_PLAN_FILE_NAME << "..." << endl;
  cout << PRELIX << "Motion plan: " << textContent << endl;
  cout << PRELIX << "Motion plan read in!" << endl;
  cout << PRELIX << "Executing motion plan..." << endl;
  return textContent; 
}

void turn_left(Motor *leftMotor, Motor *rightMotor){
  leftMotor->setVelocity(-MaxRotaionSpeed);
  rightMotor->setVelocity(MaxRotaionSpeed);
}

void go_forward(Motor *leftMotor, Motor *rightMotor){
  leftMotor->setVelocity(MaxRotaionSpeed);
  rightMotor->setVelocity(MaxRotaionSpeed);
}

void turn_right(Motor *leftMotor, Motor *rightMotor){
  leftMotor->setVelocity(MaxRotaionSpeed);
  rightMotor->setVelocity(-MaxRotaionSpeed);
}

void turn_off(Motor *leftMotor, Motor *rightMotor){
  leftMotor->setVelocity(0);
  rightMotor->setVelocity(0);
}

void obstacleDetection(DistanceSensor **ds, Message &m) {
  double dsValues[8];
  for (int i = 0; i < 3; i++){
    dsValues[i] = ds[i]->getValue();
  }
  ds_result dsResult;
  dsResult.r =
    dsValues[0] < distanceThreshold;
  dsResult.f =
    dsValues[1] < distanceThreshold;
  dsResult.l =
    dsValues[2] < distanceThreshold;

  if(dsResult.r){
    m.rightWall = 'Y';
  }
  else{
    m.rightWall = 'N';
  }
  if(dsResult.l){
    m.leftWall = 'Y';
  }
  else{
    m.leftWall = 'N';
  }
  if(dsResult.f){
    m.frontWall = 'Y';
  }
  else{
    m.frontWall = 'N';
  }
  /*
  cout << "right sensor: " << dsValues[0]
  << " front sensor: " << dsValues[1]
  << " left sensor: " << dsValues[2] << endl;
  */
  
}

void printMessage(){
  cout << PRELIX
  << "Step: " << setw(3) << setfill('0') << message.step
  << ", Row: " << message.row
  << ", Column: " << message.column
  << ", Heading: " << message.heading
  << ", Left Wall: " << message.leftWall
  << ", Front Wall: " << message.frontWall
  << ", Right Wall: " << message.rightWall
  << endl;
  message.step += 1;
}

void updateMessage(char command){
  if(command == 'L'){
    switch (message.heading) {
      case 'N':
        message.heading = 'W';
        break;
      case 'S':
        message.heading = 'E';
        break;
      case 'W':
        message.heading = 'S';
        break;
      case 'E':
        message.heading = 'N';
        break;
    }
  }
  else if (command == 'R') {
    switch (message.heading) {
    case 'N':
      message.heading = 'E';
      break;
    case 'S':
      message.heading = 'W';
      break;
    case 'W':
      message.heading = 'N';
      break;
    case 'E':
      message.heading = 'S';
      break;
    }
  }
  else if (command == 'F') {
    switch (message.heading) {
    case 'N':
      message.row -= 1;
      break;
    case 'S':
      message.row += 1;
      break;
    case 'W':
      message.column -= 1;
      break;
    case 'E':
      message.column += 1 ;
      break;
    }
  }
}

void angleCorrection(Compass *compass, char heading, Motor *leftMotor, Motor *rightMotor, Robot *robot){
  int bias = 1;
  int duration = 50;
  switch(heading){
    case 'N':
      while(auto compassResults = compass->getValues()){
        double ns = compassResults[0] * 10000;
        double we = compassResults[2] * 10000;
        if(ns < 10000-bias || ns > 10000+bias){
          if (we > 0){  // go right
            leftMotor->setVelocity(0.05 * MaxRotaionSpeed);
            rightMotor->setVelocity(-0.05 * MaxRotaionSpeed);
            robot->step(duration);
            }
          else{  // go left
            leftMotor->setVelocity(-0.05 * MaxRotaionSpeed);
            rightMotor->setVelocity(0.05 * MaxRotaionSpeed);
            robot->step(duration);
          }
        }
        else{
          break;
        }
      }
      break;
    case 'S':
      while(auto compassResults = compass->getValues()){
        double ns = compassResults[0] * 10000;
        double we = compassResults[2] * 10000;
        if(ns < -10000-bias || ns > -10000+bias){
          if (we < 0){
            leftMotor->setVelocity(0.05 * MaxRotaionSpeed);
            rightMotor->setVelocity(-0.05 * MaxRotaionSpeed);
            robot->step(duration);
            }
          else{
            leftMotor->setVelocity(-0.05 * MaxRotaionSpeed);
            rightMotor->setVelocity(0.05 * MaxRotaionSpeed);
            robot->step(duration);
          }
        }
        else{
          break;
        }
      }
      break;
    case 'W':
      while(auto compassResults = compass->getValues()){
        double ns = compassResults[0] * 10000;
        double we = compassResults[2] * 10000;
        if(we < 10000-bias || we > 10000+bias){
          if (ns < 0){
            leftMotor->setVelocity(0.05 * MaxRotaionSpeed);
            rightMotor->setVelocity(-0.05 * MaxRotaionSpeed);
            robot->step(duration);
            }
          else{
            leftMotor->setVelocity(-0.05 * MaxRotaionSpeed);
            rightMotor->setVelocity(0.05 * MaxRotaionSpeed);
            robot->step(duration);
          }
        }
        else{
          break;
        }
      }
      break;
    case 'E':
      while(auto compassResults = compass->getValues()){
        double ns = compassResults[0] * 10000;
        double we = compassResults[2] * 10000;
        if(we < -10000-bias || we > -10000+bias){
          if (ns > 0){
            leftMotor->setVelocity(0.05 * MaxRotaionSpeed);
            rightMotor->setVelocity(-0.05 * MaxRotaionSpeed);
            robot->step(duration);
            }
          else{
            leftMotor->setVelocity(-0.05 * MaxRotaionSpeed);
            rightMotor->setVelocity(0.05 * MaxRotaionSpeed);
            robot->step(duration);
          }
        }
        else{
          break;
        }
      }
      break;   
  }
}

void linearCorrection(DistanceSensor *f, Motor *leftMotor, Motor *rightMotor, Robot *robot){
  double bias = 5.0;
  double setValue = 390.0;
  int duration = 50;
  while(auto frontV = f->getValue()){
    // if too close to wall 
    if (frontV < setValue - bias){
      leftMotor->setVelocity(-0.1 * MaxRotaionSpeed);
      rightMotor->setVelocity(-0.1 * MaxRotaionSpeed);
      robot->step(duration);
    }
    // if too far fromt wall
    else if (frontV > setValue + bias){
      leftMotor->setVelocity(0.1 * MaxRotaionSpeed);
      rightMotor->setVelocity(0.1 * MaxRotaionSpeed);
      robot->step(duration);
    }
    // ok, let's leave the loop
    else{
      break;
    }  
  }
}

void wirteMotionExectuion(ofstream &myFile) {
  myFile << message.step << ","
  << message.row << ","
  << message.column << ","
  << message.heading << ","
  << message.leftWall << ","
  << message.frontWall << ","
  << message.rightWall << ","
  << endl;
}



