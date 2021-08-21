#ifndef EXPLORERROBOT_H
#define EXPLORERROBOT_H

#include <vector>
#include <sstream>
#include "CustomRobot.hpp"

const int MAPSIZE_X = 17;
const int MAPSIZE_Y = 9;

using namespace std;
using namespace webots;

struct _Cell {
    bool up = false, 
         down = false, 
         left = false, 
         right = false,
         explored = false,
         oor = true; //out of range
};
typedef struct _Cell Cell;

class Map {

private:
    const std::string mapPath = "../../Map.txt";
    char initialH;
    int robot_x, robot_y;
    int target_x = 0, target_y = 0;
    char robot_heading;
    std::vector<std::vector<Cell>> map;

public:
    Map();
    ~Map();

    void setInitial(char heading);
    bool updateMap(int x, int y, char heading, bool l, bool f, bool r);
    bool findUnexplored();
    std::string fullMapString();
    std::string printPartialMap();
    void printFullMap();
    void saveMap();
};

class ExplorerRobot : public CustomRobot {

private:
    //Starts at middle of map large enough for any starting location
    int x = 8, y = 4; 
    Map* map;

public:
    ExplorerRobot(Robot *);
    ~ExplorerRobot();

    bool updateMap();
    void explore();
};

#endif