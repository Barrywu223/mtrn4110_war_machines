
#ifndef CUSTOMROBOT_H
#define CUSTOMROBOT_H

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
const double MaxRotationSpeed = 6.28 * 0.75; // Max. rotation speed = 6.28 rad/s
const string MOTION_PLAN_FILE_NAME = "../../MotionPlan.txt";
const string MOTION_EXUCUTION_FILE_NAME = "../../MotionExecution.csv";
const string PRELIX = "[z5211399_MTRN4110_PhaseA] ";
const int TIME_STEP = 64;

using namespace webots;

struct DsResult {
    bool r, l, f;
};

struct Message {
    int step = 0;
    int row;
    int column;
    char heading;
    char leftWall;
    char frontWall;
    char rightWall;
};

class CustomRobot {

    Robot *robot;
    Motor *leftMotor;
    Motor *rightMotor;
    DistanceSensor *ds[3];
    Compass *compass;

    DsResult ds_result;
    Message message;

private:
    // Defines methods for use within the class
    void angleCorrection();
    void linearCorrection();

public:
    CustomRobot(Robot *);
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