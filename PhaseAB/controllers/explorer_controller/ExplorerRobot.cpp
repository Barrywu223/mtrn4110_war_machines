
#include "ExplorerRobot.hpp"

Map::Map() {
    map = std::vector<std::vector<Cell>> (MAPSIZE_X);
    for (int i = 0; i < MAPSIZE_X; i++) map[i] = std::vector<Cell>(MAPSIZE_Y);
}

Map::~Map() {}

void Map::updateMap(int x, int y, char heading, bool l, bool f, bool r) {
    if      (heading == 'N') {map[x][y].left = l; map[x][y].up = f; map[x][y].right = r;}
    else if (heading == 'S') {map[x][y].right = l; map[x][y].down = f; map[x][y].left = r;}
    else if (heading == 'E') {map[x][y].up = l; map[x][y].right = f; map[x][y].down = r;}
    else if (heading == 'W') {map[x][y].down = l; map[x][y].left = f; map[x][y].up = r;}
    map[x][y].explored = true;
    robot_x = x; robot_y = y; robot_heading = heading;
}

std::string Map::fullMapString() {
    std::stringstream mapString;
    for (int x = 0; x < MAPSIZE_X; x++) mapString << ((map[x][0].up) ? " ---" : "    ");
    mapString << "\n";
    for (int y = 1; y < MAPSIZE_Y-1; y++) {
        for (int x = 0; x < MAPSIZE_X; x++) {
            if (x == 0) mapString << ((map[0][y].left) ? "|" : " ");
            if (x == robot_x && y == robot_y) {
                if      (robot_heading == 'N') mapString << " ^ ";
                else if (robot_heading == 'S') mapString << " v ";
                else if (robot_heading == 'E') mapString << " > ";
                else if (robot_heading == 'W') mapString << " < ";
            }
            else mapString << ((map[x][y].explored) ? " 1 " : " 0 ");
            if (x == MAPSIZE_X-1) mapString << ((map[MAPSIZE_X-1][y].right) ? "|" : " ");
            else mapString << ((map[x][y].right || map[x+1][y].left) ? "|" : " ");
        }       
        mapString << "\n";
        for (int x = 0; x < MAPSIZE_X; x++) mapString << ((map[x][y].down || map[x][y+1].up) ? " ---" : "    ");
        mapString << "\n";
    }
    return mapString.str();
}

void Map::printFullMap() {
    cout << fullMapString();
}

ExplorerRobot::ExplorerRobot(Robot* r) : CustomRobot(r) {
    map = new Map;
}

ExplorerRobot::~ExplorerRobot() {
    delete map;
}

void ExplorerRobot::updateMap() {
    map->updateMap(x, y, getCurrentHeading(), leftWall(), forwardWall(), rightWall());
}

void ExplorerRobot::explore() {
    for (int i = 0; i < 30; i++) {
        updateMap();
        printf("%d %d %d\n", leftWall(), forwardWall(), rightWall());
        printf("%c %d %d\n", getCurrentHeading(), x, y);
        map->printFullMap();
        printf(">>>>> NEW MAP <<<<<\n");
        bool updatePos = true;
        if (!forwardWall()) {
            forward();
        }
        else if (!rightWall()) {
            turnRight();
            forward();
        }
        else if (!leftWall()) {
            turnLeft();
            forward();
        }
        else {
            turnLeft(); 
            turnLeft();
            updatePos = false;
        }
        if (updatePos) {
            char heading = getCurrentHeading();
            if      (heading == 'N') y -= 1;
            else if (heading == 'S') y += 1;
            else if (heading == 'E') x += 1;
            else if (heading == 'W') x -= 1; 
        }
    }
    stop();
}