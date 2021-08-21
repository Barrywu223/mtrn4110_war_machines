
#ifndef PATHROBOT_H
#define PATHROBOT_H

#include <CustomRobot.hpp>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>

class PathRobot : public CustomRobot {

    Robot *robot;
    std::ofstream file;
    int step = 0, x, y;
    std::string heading;

private:
    const std::string header = "[war_machines_PhaseA] ";
    const std::string CSVPath = "../../MotionExecution.csv";
    const std::string filePath = "../../MotionPlan.txt";
    std::string rawData;

public:
    PathRobot(Robot *);
    ~PathRobot();

    void readFile();
    void setupCSV();
    void CSVAppend();
    void followMP();
};

#endif