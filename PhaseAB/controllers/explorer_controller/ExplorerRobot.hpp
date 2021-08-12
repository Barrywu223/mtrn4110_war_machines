#ifndef EXPLORERROBOT_H
#define EXPLORERROBOT_H

#include <vector>
#include <sstream>
#include "CustomRobot.hpp"

const int MAPSIZE_X = 17;
const int MAPSIZE_Y = 11;

using namespace std;
using namespace webots;

struct _Cell {
    bool up = false, 
         down = false, 
         left = false, 
         right = false,
         explored = false;
};
typedef struct _Cell Cell;

class Map {

private:
    int robot_x, robot_y;
    char robot_heading;
    std::vector<std::vector<Cell>> map;

public:
    Map();
    ~Map();

    void updateMap(int x, int y, char heading, bool l, bool f, bool r);
    string fullMapString();
    void printFullMap();
};

class ExplorerRobot : public CustomRobot {

private:
    //Starts at middle of map large enough for any starting location
    int x = 8, y = 5; 
    Map* map;

public:
    ExplorerRobot(Robot *);
    ~ExplorerRobot();

    void updateMap();
    void explore();
};

#endif