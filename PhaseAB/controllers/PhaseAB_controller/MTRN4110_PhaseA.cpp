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

#include "MTRN4110_PhaseA.hpp"

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

void printMessage(Message &message){
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

void updateMessage(Message &message, char command){
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

void wirteMotionExectuion(ofstream &myFile, Message &message) {
  myFile << message.step << ","
  << message.row << ","
  << message.column << ","
  << message.heading << ","
  << message.leftWall << ","
  << message.frontWall << ","
  << message.rightWall << ","
  << endl;
}



