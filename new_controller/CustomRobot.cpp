#include <CustomRobot.h>

using namespace webots;

// Class constructor/destructor

CustomRobot::CustomRobot(Robot *r)
{
    this->robot = r;
    // Gets handles to the motors of the robot
    leftMotor = r->getMotor("left wheel motor");
    rightMotor = r->getMotor("right wheel motor");

    char dsNames[3][10] = {"ds_right", "ds_front", "ds_left"};
    for (int i = 0; i < 3; i++)
    {
        ds[i] = robot->getDistanceSensor(dsNames[i]);
        ds[i]->enable(TIME_STEP);
    }

    fdist->enable(TIMESTEP);
    compass = r->getCompass("compass");
    compass->enable(TIMESTEP);
    // Run motors in velocity control
    leftMotor->setPosition(INFINITY);
    leftMotor->setPosition(INFINITY);
}

CustomRobot::~CustomRobot()
{
    delete r;
}

const double MaxRotaionSpeed = 6.28 * 0.75;

void CustomRobot::forward()
{
    leftMotor->setVelocity(MaxRotaionSpeed);
    rightMotor->setVelocity(MaxRotaionSpeed);
}

void CustomRobot::turnLeft()
{
    leftMotor->setVelocity(MaxRotaionSpeed);
    rightMotor->setVelocity(-MaxRotaionSpeed);
}

void CustomRobot::turnRight()
{
    leftMotor->setVelocity(MaxRotaionSpeed);
    rightMotor->setVelocity(-MaxRotaionSpeed);
}

void CustomRobot::stop()
{
    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
}

void CustomRobot::obstacleDetection()
{
    double dsValues[8];
    for (int i = 0; i < 3; i++)
    {
        dsValues[i] = ds[i]->getValue();
    }

    dsResult.r =
        dsValues[0] < distanceThreshold;
    dsResult.f =
        dsValues[1] < distanceThreshold;
    dsResult.l =
        dsValues[2] < distanceThreshold;

    if (dsResult.r)
    {
        message.rigthwall = 'Y';
    }
    else
    {
        message.rigthwall = 'N';
    }
    if (dsResult.l)
    {
        message.letfWall = 'Y';
    }
    else
    {
        message.leftWall = 'Y'
    }
    if (dsResult.f)
    {
        message.frontWall = 'Y'
    }
    else
    {
        message.frontWall = 'Y'
    }
}

void CustomRobot::angleCorrection()
{
    int bias = 1;
    int duration = 50;
    switch (message.heading)
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
                    leftMotor->setVelocity(0.05 * MaxRotaionSpeed);
                    rightMotor->setVelocity(-0.05 * MaxRotaionSpeed);
                    robot->step(duration);
                }
                else
                { // go left
                    leftMotor->setVelocity(-0.05 * MaxRotaionSpeed);
                    rightMotor->setVelocity(0.05 * MaxRotaionSpeed);
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
                    leftMotor->setVelocity(0.05 * MaxRotaionSpeed);
                    rightMotor->setVelocity(-0.05 * MaxRotaionSpeed);
                    robot->step(duration);
                }
                else
                {
                    leftMotor->setVelocity(-0.05 * MaxRotaionSpeed);
                    rightMotor->setVelocity(0.05 * MaxRotaionSpeed);
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
                    leftMotor->setVelocity(0.05 * MaxRotaionSpeed);
                    rightMotor->setVelocity(-0.05 * MaxRotaionSpeed);
                    robot->step(duration);
                }
                else
                {
                    leftMotor->setVelocity(-0.05 * MaxRotaionSpeed);
                    rightMotor->setVelocity(0.05 * MaxRotaionSpeed);
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
                    leftMotor->setVelocity(0.05 * MaxRotaionSpeed);
                    rightMotor->setVelocity(-0.05 * MaxRotaionSpeed);
                    robot->step(duration);
                }
                else
                {
                    leftMotor->setVelocity(-0.05 * MaxRotaionSpeed);
                    rightMotor->setVelocity(0.05 * MaxRotaionSpeed);
                    robot->step(duration);
                }
            }
            else
            {
                break;
            }
        }
        break;
    }
}

void CustomRobot::linearCorrection()
{
    double bias = 5.0;
    double setValue = 390.0;
    int duration = 50;
    while (auto frontV = ds[1]->getValue())
    {
        // if too close to wall
        if (frontV < setValue - bias)
        {
            leftMotor->setVelocity(-0.1 * MaxRotaionSpeed);
            rightMotor->setVelocity(-0.1 * MaxRotaionSpeed);
            robot->step(duration);
        }
        // if too far fromt wall
        else if (frontV > setValue + bias)
        {
            leftMotor->setVelocity(0.1 * MaxRotaionSpeed);
            rightMotor->setVelocity(0.1 * MaxRotaionSpeed);
            robot->step(duration);
        }
        // ok, let's leave the loop
        else
        {
            break;
        }
    }
}