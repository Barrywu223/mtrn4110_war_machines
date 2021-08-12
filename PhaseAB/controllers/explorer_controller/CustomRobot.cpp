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

    compass = r->getCompass("compass");
    compass->enable(TIME_STEP);
    // Run motors in velocity control
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    stop();
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
    rightMotor->setVelocity(-MaxRotationSpeed);
    robot->step(1400);
    angleCorrection();
    linearCorrection();
}

void CustomRobot::stop() {
    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
    robot->step(TIME_STEP);
}

bool CustomRobot::rightWall() {
  return (ds[0]->getValue() < distanceThreshold) ? true : false;
}

bool CustomRobot::forwardWall() {
  return (ds[1]->getValue() < distanceThreshold) ? true : false;
}

bool CustomRobot::leftWall() {
  return (ds[2]->getValue() < distanceThreshold) ? true : false;
}

char CustomRobot::getCurrentHeading() {
    // Detect current heading 
    const double *headings = compass->getValues();
    double rad = atan2(headings[0], headings[2]);
    double bearing = (rad - 1.5708) / Pi * 180.0;
    if (bearing < 0.0) bearing = bearing + 360.0;
    char heading = 'N';
    if (bearing > 60 && bearing < 120) heading = 'E';
    else if (bearing > 150 && bearing < 210) heading = 'S';
    else if (bearing > 240 && bearing < 300) heading = 'W';
    return heading;
}

void CustomRobot::angleCorrection() {
    int bias = 1;
    char heading = getCurrentHeading();

    // Apply correction
    while (auto compassResults = compass->getValues()) {
        // Select correct values for correction
        double ns = compassResults[0] * 10000, we = compassResults[2] * 10000;
        double check = ns, condition = we; int weight = 1; bool gt = false;
        if      (heading == 'N') {check = ns; condition = we; weight = 1; gt = true;}
        else if (heading == 'S') {check = ns; condition = we; weight = -1; gt = false;}
        else if (heading == 'W') {check = we; condition = ns; weight = 1; gt = false;}
        else if (heading == 'E') {check = we; condition = ns; weight = -1; gt = true;}
        // Move vehicle within acceptable tolerance
        if (check < weight * 10000 - bias || check > weight * 10000 + bias) {
            if ((gt && condition > 0) || (!gt && condition < 0)) { // go right
                leftMotor->setVelocity(0.05 * MaxRotationSpeed);
                rightMotor->setVelocity(-0.05 * MaxRotationSpeed);
                robot->step(TIME_STEP);
            }
            else { // go left
                leftMotor->setVelocity(-0.05 * MaxRotationSpeed);
                rightMotor->setVelocity(0.05 * MaxRotationSpeed);
                robot->step(TIME_STEP);
            }
        }
        else break;
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