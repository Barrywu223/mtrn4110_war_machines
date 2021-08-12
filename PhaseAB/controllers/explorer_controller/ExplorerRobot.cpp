
#include "ExplorerRobot.hpp"

Map::Map() {
    map = std::vector<std::vector<Cell>> (MAPSIZE_X);
    for (int i = 0; i < MAPSIZE_X; i++) map[i] = std::vector<Cell>(MAPSIZE_Y);
}

Map::~Map() {}

void Map::updateMap(int x, int y, char heading, bool l, bool f, bool r) {
    if      (heading == 'N') {map[x][y].left = l; map[x-1][y].up = f; map[x][y].right = r;}
    else if (heading == 'S') {map[x][y].right = l; map[x-1][y].down = f; map[x][y].left = r;}
    else if (heading == 'E') {map[x-1][y].up = l; map[x][y].right = f; map[x-1][y].down = r;}
    else if (heading == 'W') {map[x-1][y].down = l; map[x][y].left = f; map[x-1][y].up = r;}
    map[x][y].explored = true;
}

void Map::printFullMap() {
    for (int x = 0; x < MAPSIZE_X; x++) printf((map[x][0].up) ? " ---" : "    ");
    printf("\n");
    for (int y = 1; y < MAPSIZE_Y-1; y++) {
        printf((map[0][y].left) ? "|" : " ");
        for (int x = 1; x < MAPSIZE_X-1; x++) {
            printf((map[x][y].explored) ? " 1 " : " 0 ");
            printf((map[x][y].right || map[x+1][y].left) ? "|" : " ");
        }
        printf("\n");
        for (int x = 0; x < MAPSIZE_X; x++) printf((map[x][y].down || map[x][y+1].up) ? " ---" : "    ");
        printf("\n");
    }
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
    
    for (int i = 0; i < 10; i++) {
        updateMap();
        printf("%d %d %d\n", leftWall(), forwardWall(), rightWall());
        printf("%c %d %d\n", getCurrentHeading(), x, y);
        map->printFullMap();
        printf(">>>>> NEW MAP <<<<<\n");
        if (!forwardWall()) {
            forward();
        }
        else if (!leftWall()) {
            turnLeft();
            forward();
        }
        else if (!rightWall()) {
            turnRight();
            forward();
        }
        else stop();
        char heading = getCurrentHeading();
        if      (heading == 'N') y -= 1;
        else if (heading == 'S') y += 1;
        else if (heading == 'E') x += 1;
        else if (heading == 'W') x -= 1; 
    }
    stop();
}