// File:          z5207984_MTRN4110_PhaseA.cpp
// Date:          10/06/2021
// Description:   Controller of E-puck for Phase A - Driving and Perception
// Author:        BARRY WU
// Modifications:
// Platform:      Windows
// Notes:         
//

// webots headers
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/DistanceSensor.hpp>

// standard libraries
#include <iostream>
#include <array>
#include <algorithm>
#include <math.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

// All the webots classes are defined in the "webots" namespace
using namespace webots;


#define PI 3.14159

class ePuck {
public:
    int row;
    int col;
    char heading;
    int step = 0;
    ePuck(int x, int y, char z) {
      row = x;
      col = y;
      heading = z;
    }
    char frontWall = 'N';
    char leftWall = 'N';
    char rightWall = 'N';
    void currentHeading(double w);
    void updateRowCol();
    void updateWall(double f, double l, double r);
};

// Determines the heading of the epuck in a cardinal direction based on IMU data
void ePuck::currentHeading(double w) {
    double error = 0.5;
    if (-error < w && w < error) {
        heading = 'N';
    }
    else if (-PI/2 - error < w && w < -PI/2 + error) {
        heading = 'E';
    }
    else if (PI/2 - error < w && w < PI/2 + error) {
        heading = 'W';
    }
    else {
        heading = 'S';
    }   
}

// Updates row and column of epuck when it moves forward
void ePuck::updateRowCol() {
    
    if (heading == 'N') {
        row = row - 1;
    }
    else if (heading == 'S') {
        row = row + 1;
    }
    else if (heading == 'E') {
        col = col + 1;
    }
    else if (heading == 'W') {
        col = col - 1;
    }
}

void ePuck::updateWall(double f, double l, double r) {
    double max = 990;
    if (f > max) {
        frontWall = 'N';
    }
    else if (f < max) {
        frontWall = 'Y';
    }
    if (l > max) {
        leftWall = 'N';
    }
    else if (l < max) {
        leftWall = 'Y';
    }
    if (r > max) {
        rightWall = 'N';
    }
    else if (r < max) {
        rightWall = 'Y';
    }
}

// Wraps IMU values from [-PI, PI]
double wrapToPi(double x) {
    double x_max = PI;
    double x_min = -PI;
    double offset = 0.05;
    if (x > x_max) {
        x = x - 2*PI;
        return x;
    }
    else if (x < x_min) {
        x = x + 2*PI;
        return x;
    }
    else if (x > -PI && x < -PI + offset) {
        return -PI + 0.025;
    }
    else {
        return x;
    }
}
void createCsvFile(int step, int row, int col, char heading, char leftWall, char frontWall, char rightWall) {
    const std::string fileName = "../../MotionExecution.csv";
    std::ofstream file(fileName, std::ios::out|std::ios::app);
    
    if (file.is_open()) {
      if (step != 0) {
        file << step << ',' << row << ',' << col << ',' << heading << ',' << leftWall << ',' << frontWall << ',' << rightWall;
      }
      else {
        file << "Step" << ',' << "Row" << ',' << "Column" << ',' << "Heading" << ',' << "Left Wall" << ',' << "Front Wall" << ',' << "Right Wall" << std::endl;
        file << step << ',' << row << ',' << col << ',' << heading << ',' << leftWall << ',' << frontWall << ',' << rightWall;
      }
    }
    
    file << std::endl;
}

int main() {
  
  // Open MotionPlan.txt
  // const std::string motionPlan = “../../MotionPlan.txt”;
  // Load MotionPlan.txt
  std::string motionPlan;
  std::ifstream motion ("../../MotionPlan.txt");
  std::cout << "[z5207984_MTRN4110_PhaseA] Reading in motion plan from ../../MotionPlan.txt" << std::endl;
  if (motion.is_open()) {
      while (std::getline(motion,motionPlan)) {
          std::cout << "[z5207984_MTRN4110_PhaseA] Motion Plan: " << motionPlan << std::endl;
          std::cout << "[z5207984_MTRN4110_PhaseA] Motion plan read in!" << std::endl;
      }
      motion.close();
  }
  // Error if file unable to open
  else {
      std::cout << "Unable to open file" << std::endl;
  }
  std::cout << "[z5207984_MTRN4110_PhaseA] Executing motion plan..." << std::endl;
  // Initial coordinates and heading
  int xPos = motionPlan[0] - '0';
  int yPos = motionPlan[1] - '0';
  char direction = motionPlan[2];
  motionPlan.erase(0,3);
  
  // Create ePuck instance
  ePuck ePuck(xPos, yPos, direction);
  
  // create the Robot instance.
  Robot *robot = new Robot();
  Motor* leftMotor = robot->getMotor("left wheel motor");
  Motor* rightMotor = robot->getMotor("right wheel motor");
  
  // Enable timestep
  int timeStep = (int)robot->getBasicTimeStep();
  
  // Set motor speeds
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0);
  rightMotor->setVelocity(0);
  
  // Enable Inertial Unit
  InertialUnit* iMU = robot->getInertialUnit("inertial unit");
  iMU->enable(timeStep);
  
  // Enable distance sensors
  DistanceSensor* fsensor = robot->getDistanceSensor("front sensor");
  fsensor->enable(timeStep);
  DistanceSensor* lsensor = robot->getDistanceSensor("left sensor");
  lsensor->enable(timeStep);
  DistanceSensor* rsensor = robot->getDistanceSensor("right sensor");
  rsensor->enable(timeStep);
  
  while (robot->step(timeStep) != -1) {
      
      double* values = (double*)iMU->getRollPitchYaw();
      double currentYaw = values[2];
      double temp = currentYaw;
      double front = fsensor->getValue();
      double left = lsensor->getValue();
      double right = rsensor->getValue();
      ePuck.updateWall(front, left, right);
      createCsvFile(ePuck.step, ePuck.row, ePuck.col, ePuck.heading, ePuck.leftWall, ePuck.frontWall, ePuck.rightWall);
      std::cout << "[z5207984_MTRN4110_PhaseA] Step: 000, " << "Row: " << ePuck.row << ", " << "Column: " << ePuck.col << ", " << "Heading: " << ePuck.heading << ", " << "Left Wall: " << ePuck.leftWall << ", " << "Front Wall: " << ePuck.frontWall << ", " << "Right Wall: " << ePuck.rightWall << std::endl;
      for (char & c : motionPlan) {
        
        if (c == 'L') {
          temp = wrapToPi(currentYaw + PI/2);
          while (values[2] < temp) {
            leftMotor->setPosition(INFINITY);
            leftMotor->setVelocity(-1);
            rightMotor->setPosition(INFINITY);
            rightMotor->setVelocity(1);
            robot->step(timeStep);
          }
        }
        else if (c == 'R') {
          temp = wrapToPi(currentYaw - PI/2);
          while (values[2] > temp) {
            leftMotor->setPosition(INFINITY);
            leftMotor->setVelocity(1);
            rightMotor->setPosition(INFINITY);
            rightMotor->setVelocity(-1);
            robot->step(timeStep);
          }
        }
        else if (c == 'F') {
          auto start = robot->getTime();
          while (robot->getTime() < start + 4.1) {
            leftMotor->setPosition(INFINITY);
            leftMotor->setVelocity(2);
            rightMotor->setPosition(INFINITY);
            rightMotor->setVelocity(2);
            robot->step(timeStep);
          }
          ePuck.updateRowCol();
        }
        else {
            //break;
        }
        
        ePuck.currentHeading(values[2]);
        front = fsensor->getValue();
        left = lsensor->getValue();
        right = rsensor->getValue();
        
        ePuck.updateWall(front, left, right);
        ePuck.step++;
        createCsvFile(ePuck.step, ePuck.row, ePuck.col, ePuck.heading, ePuck.leftWall, ePuck.frontWall, ePuck.rightWall);
        if (ePuck.step > 9) {
            std::cout << "[z5207984_MTRN4110_PhaseA] Step: 0" << ePuck.step << ", " << "Row: " << ePuck.row << ", " << "Column: " << ePuck.col << ", " << "Heading: " << ePuck.heading << ", " << "Left Wall: " << ePuck.leftWall << ", " << "Front Wall: " << ePuck.frontWall << ", " << "Right Wall: " << ePuck.rightWall << std::endl;
        }
        else {
            std::cout << "[z5207984_MTRN4110_PhaseA] Step: 00" << ePuck.step << ", " << "Row: " << ePuck.row << ", " << "Column: " << ePuck.col << ", " << "Heading: " << ePuck.heading << ", " << "Left Wall: " << ePuck.leftWall << ", " << "Front Wall: " << ePuck.frontWall << ", " << "Right Wall: " << ePuck.rightWall << std::endl;
        }
        currentYaw = values[2];
        
      }
  std::cout << "[z5207984_MTRN4110_PhaseA] Motion plan executed!" << std::endl;
  break;
  }
}
