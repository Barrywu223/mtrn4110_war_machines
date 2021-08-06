// File:            GPS.cpp
// Date:            3/6/2021
// Description:     Read from GPS sensor
// Author:          Ming Xuan Chua
// Modifications:
// Platform:        Windows

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/GPS.hpp>

#define PI 3.14

// All the webots classes are defined in the "webots" namespace
using namespace webots;

int main(int argc, char** argv) {
    // create the Robot instance.
    Robot* robot = new Robot();
    Motor* leftMotor = robot->getMotor("left wheel motor");
    Motor* rightMotor = robot->getMotor("right wheel motor");
    // get the time step of the current world.
    int timeStep = (int)robot->getBasicTimeStep();
    GPS* gps = robot->getGPS("gps");
    gps->enable(timeStep);
    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    while (robot->step(timeStep) != -1) {
        double* gpsV = (double*)gps->getValues();// gps(1) is the height
        std::cout << gpsV[0] << "  " << gpsV[1] << "  " << gpsV[2] << std::endl;

        leftMotor->setPosition(INFINITY);
        rightMotor->setPosition(INFINITY);
        leftMotor->setVelocity(1 * PI);
        rightMotor->setVelocity(1 * PI);

    };

    // Enter here exit cleanup code.

    delete robot;
    return 0;
}