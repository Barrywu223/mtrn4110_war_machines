#ifndef MTRN4110_PHASEA_HPP
#define MTRN4110_PHASEA_HPP

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
const string MOTION_PLAN_FILE_NAME = "../../MotionPlan.txt";
const string MOTION_EXUCUTION_FILE_NAME = "../../MotionExecution.csv";
const string PRELIX = "[z5211399_MTRN4110_PhaseA] ";
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

// created function
string readMotionPlan();
void go_forward(Motor *leftMotor, Motor *rightMotor);  
void obstacleDetection(DistanceSensor **ps, Message &m);
void turn_right(Motor *leftMotor, Motor *rightMotor);
void turn_left(Motor *leftMotor, Motor *rightMotor);
void turn_off(Motor *leftMotor, Motor *rightMotor);
void updateMessage(Message &message, char command);
void printMessage(Message &message);
void angleCorrection(Compass *compass, char heading, Motor *leftMotor, Motor *rightMotor, Robot *robot);
void linearCorrection(DistanceSensor *f, Motor *leftMotor, Motor *rightMotor, Robot *robot);
void wirteMotionExectuion(ofstream &myFile, Message &message);

#endif