
#ifndef CUSTOMROBOT_H
#define CUSTOMROBOT_H

#include <string>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Compass.hpp>
#include <webots/Keyboard.hpp>
const double Pi = 3.1415926;
const double distanceThreshold = 850;
const double MaxRotationSpeed = 6.28 * 0.75; // Max. rotation speed = 6.28 rad/s
const int TIME_STEP = 64;

using namespace webots;

class CustomRobot
{

    Motor *leftMotor;
    Motor *rightMotor;
    DistanceSensor *ds[3];
    Compass *compass;
    Robot *robot;

private:
    // Defines methods for use within the class
    void angleCorrection();
    void linearCorrection();

public:
    Keyboard kb;
    CustomRobot(Robot *);
    ~CustomRobot();

    // Defines methods to interact with robot
    void forward();
    void turnLeft();
    void turnRight();
    void stop();

    char getCurrentHeading();
    bool leftWall();
    bool rightWall();
    bool forwardWall();
};

#endif