#include "CustomRobot.hpp"

using namespace webots;

// Class constructor/destructor

CustomRobot::CustomRobot(Robot *r) {
    robot = r;
    // Gets handles to the motors of the robot
    leftMotor = r->getMotor("left wheel motor");
    rightMotor = r->getMotor("right wheel motor");

    char dsNames[3][10] = {"ds_right", "ds_front", "ds_left"};
    for (int i = 0; i < 3; i++) {
        ds[i] = robot->getDistanceSensor(dsNames[i]);
        ds[i]->enable(TIME_STEP);
    }

    // fdist->enable(TIMESTEP);
    compass = r->getCompass("compass");
    compass->enable(TIME_STEP);
    // Run motors in velocity control
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
}

CustomRobot::~CustomRobot() {delete robot;}

void CustomRobot::forward() {
    leftMotor->setVelocity(MaxRotationSpeed);
    rightMotor->setVelocity(MaxRotationSpeed);
    robot->step(1700);
    angleCorrection();
    linearCorrection();
}

void CustomRobot::turnLeft() {
    leftMotor->setVelocity(-MaxRotationSpeed);
    rightMotor->setVelocity(MaxRotationSpeed);
    robot->step(1400);
    angleCorrection();
    linearCorrection();
}

void CustomRobot::turnRight() {
    leftMotor->setVelocity(MaxRotationSpeed);
    rightMotor->setVelocity(-MaxRotationSpeed);\
    robot->step(1400);
    angleCorrection();
    linearCorrection();
}

void CustomRobot::stop() {
    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
    robot->step(TIME_STEP);
}

bool CustomRobot::leftWall() {
  return (ds[2]->getValue() < distanceThreshold) ? true : false;
}

bool CustomRobot::rightWall() {
  return (ds[0]->getValue() < distanceThreshold) ? true : false;
}

bool CustomRobot::forwardWall() {
  return (ds[1]->getValue() < distanceThreshold) ? true : false;
}

void CustomRobot::angleCorrection() {
    int bias = 1;
    int duration = 50; 
    // Detect current heading to avoid passing through classes
    const double *headings = compass->getValues();
    double rad = atan2(headings[0], headings[2]);
    double bearing = (rad - 1.5708) / Pi * 180.0;
    if (bearing < 0.0) bearing = bearing + 360.0;
    char heading = 'N';
    if (bearing > 80 && bearing < 100) heading = 'E';
    else if (bearing > 170 && bearing < 190) heading = 'S';
    else if (bearing > 260 && bearing < 280) heading = 'W';

    // Do angle correction
    switch (heading)
    {
    case 'N':
        while (auto compassResults = compass->getValues())
        {
            double ns = compassResults[0] * 10000;
            double we = compassResults[2] * 10000;
            if (ns < 10000 - bias || ns > 10000 + bias)
            {
                if (we > 0)
                { // go right
                    leftMotor->setVelocity(0.05 * MaxRotationSpeed);
                    rightMotor->setVelocity(-0.05 * MaxRotationSpeed);
                    robot->step(duration);
                }
                else
                { // go left
                    leftMotor->setVelocity(-0.05 * MaxRotationSpeed);
                    rightMotor->setVelocity(0.05 * MaxRotationSpeed);
                    robot->step(duration);
                }
            }
            else
            {
                break;
            }
        }
        break;
    case 'S':
        while (auto compassResults = compass->getValues())
        {
            double ns = compassResults[0] * 10000;
            double we = compassResults[2] * 10000;
            if (ns < -10000 - bias || ns > -10000 + bias)
            {
                if (we < 0)
                {
                    leftMotor->setVelocity(0.05 * MaxRotationSpeed);
                    rightMotor->setVelocity(-0.05 * MaxRotationSpeed);
                    robot->step(duration);
                }
                else
                {
                    leftMotor->setVelocity(-0.05 * MaxRotationSpeed);
                    rightMotor->setVelocity(0.05 * MaxRotationSpeed);
                    robot->step(duration);
                }
            }
            else
            {
                break;
            }
        }
        break;
    case 'W':
        while (auto compassResults = compass->getValues())
        {
            double ns = compassResults[0] * 10000;
            double we = compassResults[2] * 10000;
            if (we < 10000 - bias || we > 10000 + bias)
            {
                if (ns < 0)
                {
                    leftMotor->setVelocity(0.05 * MaxRotationSpeed);
                    rightMotor->setVelocity(-0.05 * MaxRotationSpeed);
                    robot->step(duration);
                }
                else
                {
                    leftMotor->setVelocity(-0.05 * MaxRotationSpeed);
                    rightMotor->setVelocity(0.05 * MaxRotationSpeed);
                    robot->step(duration);
                }
            }
            else
            {
                break;
            }
        }
        break;
    case 'E':
        while (auto compassResults = compass->getValues())
        {
            double ns = compassResults[0] * 10000;
            double we = compassResults[2] * 10000;
            if (we < -10000 - bias || we > -10000 + bias)
            {
                if (ns > 0)
                {
                    leftMotor->setVelocity(0.05 * MaxRotationSpeed);
                    rightMotor->setVelocity(-0.05 * MaxRotationSpeed);
                    robot->step(duration);
                }
                else
                {
                    leftMotor->setVelocity(-0.05 * MaxRotationSpeed);
                    rightMotor->setVelocity(0.05 * MaxRotationSpeed);
                    robot->step(duration);
                }
            }
            else break;
        }
        break;
    }
}

void CustomRobot::linearCorrection() {
    if (ds[1]->getValue() >= distanceThreshold) return;
    double bias = 5.0, setValue = 390.0;
    while (auto frontV = ds[1]->getValue()) {
        if (frontV < setValue - bias) {
            leftMotor->setVelocity(-0.1 * MaxRotationSpeed);
            rightMotor->setVelocity(-0.1 * MaxRotationSpeed);
            robot->step(TIME_STEP);
        }
        else if (frontV > setValue + bias) {
            leftMotor->setVelocity(0.1 * MaxRotationSpeed);
            rightMotor->setVelocity(0.1 * MaxRotationSpeed);
            robot->step(TIME_STEP);
        }
        else break;
    }
}