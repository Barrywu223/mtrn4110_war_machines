// File:            PositionController.cpp
// Date:            3/6/2021
// Description:     Control position of motors
// Author:          Ming Xuan Chua
// Modifications:
// Platform:        Windows

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

#define PI 3.14

// All the webots classes are defined in the "webots" namespace
using namespace webots;

int main(int argc, char** argv) {
    // create the Robot instance.
    Robot* robot = new Robot();
    //Initialise the motors
    Motor* leftMotor = robot->getMotor("left wheel motor");
    Motor* rightMotor = robot->getMotor("right wheel motor");
    // get the time step of the current world.
    int timeStep = (int)robot->getBasicTimeStep();

    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    while (robot->step(timeStep) != -1) {

        //units: rad, rad/s
        //Position Controller
        leftMotor->setPosition(5 * PI);
        rightMotor->setPosition(10 * PI);
        leftMotor->setVelocity(PI);
        rightMotor->setVelocity(2 * PI);
    };

    // Enter here exit cleanup code.

    delete robot;
    return 0;
}