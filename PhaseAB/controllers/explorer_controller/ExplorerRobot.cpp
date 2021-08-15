
#include "ExplorerRobot.hpp"
#include "PathPlanner.hpp"

Map::Map() {
    map = std::vector<std::vector<Cell>> (MAPSIZE_X);
    for (int i = 0; i < MAPSIZE_X; i++) map[i] = std::vector<Cell>(MAPSIZE_Y);
    // Make edges
    for (int i = 0; i < MAPSIZE_X; i++) {
        map[i][0].up = true;
        map[i][MAPSIZE_Y-1].down = true;
    }
    for (int i = 0; i < MAPSIZE_Y; i++) {
        map[0][i].left = true;
        map[MAPSIZE_X-1][i].right = true;
    }
}

Map::~Map() {}

bool Map::findUnexploredr(int x, int y) {
    if (x < 1 || y < 1 || x >= MAPSIZE_X || y >= MAPSIZE_Y) return false;
    if (!map[x][y].explored && !map[x][y].oor) {
        target = true;
        target_x = x; target_y = y;
        return true;
    }
    return (findUnexploredr(x-1, y) || findUnexploredr(x+1, y) || findUnexploredr(x, y-1) || findUnexploredr(x, y+1));
}

bool Map::findUnexplored() {
    for (int y = 0; y < MAPSIZE_Y; y++) {
        for (int x = 0; x < MAPSIZE_X; x++) {
            if (!map[x][y].explored && !map[x][y].oor) {
                target = true;
                target_x = x; target_y = y;
                return true;
            }
        }
    }
    target_x = 0; target_y = 0;
    return false;
}

bool Map::updateMap(int x, int y, char heading, bool l, bool f, bool r) {
    if (heading == 'N') {
        map[x][y].left = l; map[x][y].up = f; map[x][y].right = r;
        if (!l && x != 0)         map[x-1][y].oor = false;
        if (!r && x != MAPSIZE_X) map[x+1][y].oor = false;
        if (!f && y != 0)         map[x][y-1].oor = false;
    }
    else if (heading == 'S') {
        map[x][y].right = l; map[x][y].down = f; map[x][y].left = r;
        if (!l && x != MAPSIZE_X) map[x+1][y].oor = false;
        if (!r && x != 0)         map[x-1][y].oor = false;
        if (!f && y != MAPSIZE_Y) map[x][y+1].oor = false;
    }
    else if (heading == 'E') {
        map[x][y].up = l; map[x][y].right = f; map[x][y].down = r;
        if (!l && y != 0)         map[x][y-1].oor = false;
        if (!r && y != MAPSIZE_Y) map[x][y+1].oor = false;
        if (!f && x != MAPSIZE_X) map[x+1][y].oor = false;
    }
    else if (heading == 'W') {
        map[x][y].down = l; map[x][y].left = f; map[x][y].up = r;
        if (!l && y != MAPSIZE_Y) map[x][y+1].oor = false;
        if (!r && y != 0)         map[x][y-1].oor = false;
        if (!f && x != 0)         map[x-1][y].oor = false;
    }
    map[x][y].explored = true;
    map[x][y].oor = false;
    robot_x = x; robot_y = y; robot_heading = heading;
    return findUnexplored();
}

std::string Map::fullMapString() {
    std::stringstream mapString; mapString.str("");
    for (int x = 0; x < MAPSIZE_X; x++) mapString << ((map[x][0].up) ? " ---" : "    ");
    mapString << "\n";
    for (int y = 0; y < MAPSIZE_Y; y++) {
        for (int x = 0; x < MAPSIZE_X; x++) {
            if (x == 0) mapString << ((map[0][y].left) ? "|" : " ");
            if (x == robot_x && y == robot_y) {
                if      (robot_heading == 'N') mapString << " ^ ";
                else if (robot_heading == 'S') mapString << " v ";
                else if (robot_heading == 'E') mapString << " > ";
                else if (robot_heading == 'W') mapString << " < ";
            } else if (x == target_x && y == target_y) mapString << " x ";
            // else mapString << ((map[x][y].oor) ? " I " : (map[x][y].explored) ? " 1 " : " 0 ");
            else mapString << ((map[x][y].oor) ? "   " : (map[x][y].explored) ? "   " : "   ");
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

bool ExplorerRobot::updateMap() {
    return map->updateMap(x, y, getCurrentHeading(), leftWall(), forwardWall(), rightWall());
}

void ExplorerRobot::explore() {

    while (updateMap()) {
        PathPlanner *pp = new PathPlanner(map->fullMapString());
        std::string path = pp->findBestPath(); delete pp;
        char move; int foundPos;
        if ((foundPos = (int)path.find_first_not_of("0123456789NSEW")) != -1) move = path[foundPos];
        else {stop(); return;}

        if (move == 'F') {
            forward();
            char heading = getCurrentHeading();
            if      (heading == 'N') y -= 1;
            else if (heading == 'S') y += 1;
            else if (heading == 'E') x += 1;
            else if (heading == 'W') x -= 1; 
        }
        else if (move == 'R') turnRight();
        else if (move == 'L') turnLeft();
    }
    stop();
}