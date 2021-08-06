// File:          z5207984_MTRN4110_PhaseB.cpp
// Date:          05/07/2021
// Description:   Controller of E-puck for Phase B - Path Planning
// Author:        BARRY WU
// Modifications:
// Platform:      Windows
// Notes:         
//


#include <webots/Robot.hpp>

#include <utility>
#include <iostream>
#include <array>
#include <algorithm>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>


#define WIDTH 37
#define HEIGHT 11
#define INF 2147483647
// All the webots classes are defined in the "webots" namespace
using namespace webots;

struct coordinates {
  int row = 0;
  int col = 0;
  char heading;
  std::string pathPlan;
  int stepNum = 0;
};

// Defines forward or turn left/right depending on prev and current heading
char pathPlanStep(char prev, char curr) {
  char step = '0';
  if (prev == curr) {
    step = 'F';
  }
  else if (prev == 'N' && curr == 'E') {
    step = 'R';
  }
  else if (prev == 'N' && curr == 'W') {
    step = 'L';
  }
  else if (prev == 'N' && curr == 'S') {
    step = 'L';
  }
  else if (prev == 'E' && curr == 'S') {
    step = 'R';
  }
  else if (prev == 'E' && curr == 'N') {
    step = 'L';
  }
  else if (prev == 'S' && curr == 'W') {
    step = 'R';
  }
  else if (prev == 'S' && curr == 'E') {
    step = 'L';
  }
  else if (prev == 'W' && curr == 'N') {
    step = 'R';
  }
  else if (prev == 'W' && curr == 'S') {
    step = 'L';
  }
  return step;
}

int main() {

  // Load map and display
  std::cout << "[z5207984_MTRN4110_PhaseB Reading in map from ../../Map.txt..." << std::endl;
  std::string mapPath;
  std::ifstream MAP_FILE_NAME ("../../Map.txt");
  char mapData[HEIGHT][WIDTH];
  int a = 0;
  int b = 0;
  if (MAP_FILE_NAME.is_open()) {
    while (std::getline(MAP_FILE_NAME,mapPath)) {
      std::cout << "[z5207984_MTRN4110_PhaseB] " << mapPath << std::endl;
      for (b = 0; b < WIDTH; b++) {
        mapData[a][b] = mapPath[b];
      }
      a++;
    }
    MAP_FILE_NAME.close();
    std::cout << "[z5207984_MTRN4110_PhaseB] Map read in!" << std::endl;
  }
  else {
      std::cout << "Unable to open file" << std::endl;
  }
  std::cout << "[z5207984_MTRN4110_PhaseB] Finding shortest paths..." << std::endl;
  
  // Initialise starting/finishing positions and wall arrays
  int temp2 = 0;
  coordinates start;
  coordinates finish;
  
  int hWall[6][9] = {0};
  int vWall[5][10] = {0};
  
  // Find coordinates of starting/finsh point
  for (a = 1; a < HEIGHT; a = a + 2) {
    int temp1 = 0;
    for (b = 2; b < WIDTH; b = b + 4) {
      if (mapData[a][b] == 'v' || mapData[a][b] == '>' || mapData[a][b] == '<' || mapData[a][b] == '^') {
        start.col = temp1;
        start.row = temp2;
      }
      if (mapData[a][b] == 'v') {
        start.heading = 'S';
      }
      if (mapData[a][b] == '>') {
        start.heading = 'E';
      }
      if (mapData[a][b] == '<') {
        start.heading = 'W';
      }
      if (mapData[a][b] == '^') {
        start.heading = 'N';
      }
      if (mapData[a][b] == 'x') {
        finish.col = temp1;
        finish.row = temp2;
      }
      temp1++;
    }
    temp2++;
  }

  // Horizontal Wall array initialisation
  for (a = 0; a < HEIGHT; a = a + 2) {
    for (b = 2; b < WIDTH; b = b + 4) {
      if (mapData[a][b] == '-') {
        hWall[a/2][(b/2 + 1)/2 - 1] = 1;
      }
    }
  }
  
  // Vertical Wall array initialisation
  for (a = 1; a < HEIGHT; a = a + 2) {
    for (b = 0; b < WIDTH; b = b + 4) {
      if (mapData[a][b] == '|') {
        vWall[(a+1)/2 - 1][b/4] = 1;
      }
    }
  }
  
  // Flood Fill array initialisation
  int cellNum[5][9];
  
  // Initialise array to 45
  for (a = 0; a < 5; a++) {
      for (b = 0; b < 9; b++) {
          cellNum[a][b] = 45;
      }
  }
  
  // Make finish point 0
  cellNum[finish.row][finish.col] = 0;

  // Floodfill loop
  int currentExplored = 0;
  int mazeChange = 1;
  while (mazeChange != 0) {
    mazeChange = 0;
    for (a = 0; a < 5; a++) {
      for (b = 0; b < 9; b++) {
        if (cellNum[a][b] == currentExplored) {
          if (hWall[a][b] == 0) {
            if (cellNum[a-1][b] == 45) {
              cellNum[a-1][b] = cellNum[a][b] + 1;
              mazeChange = 1;
            }
          }
          if (hWall[a+1][b] == 0) {
            if (cellNum[a+1][b] == 45) {
              cellNum[a+1][b] = cellNum[a][b] + 1;
              mazeChange = 1;
            }
          }
          if (vWall[a][b] == 0) {
            if (cellNum[a][b-1] == 45) {
              cellNum[a][b-1] = cellNum[a][b] + 1;
              mazeChange = 1;
            }
          }
          if (vWall[a][b+1] == 0) {
            if (cellNum[a][b+1] == 45) {
              cellNum[a][b+1] = cellNum[a][b] + 1;
              mazeChange = 1;
            }
          }
        }
      }
    }
    currentExplored = currentExplored + 1;
  }
  
  // Find shortest path(s) 
  int pathFound = 1;
  int step = cellNum[start.row][start.col];
  bool branch;
  std::vector<std::vector<coordinates>> pathArray;
  std::vector<coordinates> temp;
  temp.push_back(start);
  pathArray.push_back(temp);
  
  while (step != 0) {
    for (int pathNum = 0; pathNum < pathFound; pathNum++) {
      branch = false;
      std::vector<coordinates> currentPath = pathArray[pathNum];
      std::vector<coordinates> tempPath = currentPath;
      coordinates current = currentPath.back();
      coordinates tempCoor = current;
      
      // North
      if (cellNum[current.row-1][current.col] == step-1 && hWall[current.row][current.col] == 0) {
        tempCoor.col = current.col;
        tempCoor.row = current.row-1;
        tempCoor.heading = 'N';
        tempPath = currentPath;
        tempPath.push_back(tempCoor);
        pathArray[pathNum] = tempPath;
        branch = true;
      }
      
      // South
      if (cellNum[current.row+1][current.col] == step-1 && hWall[current.row+1][current.col] == 0) {
        tempCoor.col = current.col;
        tempCoor.row = current.row+1;
        tempCoor.heading = 'S';
        tempPath = currentPath;
        tempPath.push_back(tempCoor);
        if (branch) {
          pathArray.push_back(std::vector<coordinates>());
          pathArray.at(pathFound) = tempPath; 
          pathFound++;
        }
        else {
          pathArray[pathNum] = tempPath;
        }
        branch = true;
      }
      
      // West
      if (cellNum[current.row][current.col-1] == step-1 && vWall[current.row][current.col] == 0) {
        tempCoor.col = current.col-1;
        tempCoor.row = current.row;
        tempCoor.heading = 'W';
        tempPath = currentPath;
        tempPath.push_back(tempCoor);
        if (branch) {
          pathArray.push_back(std::vector<coordinates>());
          pathArray.at(pathFound) = tempPath; 
          pathFound++;
        }
        else {
          pathArray[pathNum] = tempPath;
        }
        branch = true;
      }
      
      // East
      if (cellNum[current.row][current.col+1] == step-1 && vWall[current.row][current.col+1] == 0) {
        tempCoor.col = current.col+1;
        tempCoor.row = current.row;
        tempCoor.heading = 'E';
        tempPath = currentPath;
        tempPath.push_back(tempCoor);
        if (branch) {
          pathArray.push_back(std::vector<coordinates>());
          pathArray.at(pathFound) = tempPath; 
          pathFound++;
        }
        else {
          pathArray[pathNum] = tempPath;
        }
      }
    }
    step = step-1;
  }

  // Print shortest paths  
  for (int pathNum = 0; pathNum < pathFound; pathNum++) {
    std::cout << "[z5207984_MTRN4110_PhaseB] Path - " << pathNum+1 << ":" << std::endl;
    std::vector<coordinates> tempArray = pathArray[pathNum];
    char pathData[HEIGHT][WIDTH];
    for (int i = 0; i < HEIGHT; i++) {
      for (int j = 0; j < WIDTH; j++) {
        pathData[i][j] = mapData[i][j];
      }
    }
    for (a = 1; a < cellNum[start.row][start.col]+1; a++) {
      // Check if number is 2 digit
      if (cellNum[tempArray[a].row][tempArray[a].col] > 9) {
        char first = cellNum[tempArray[a].row][tempArray[a].col] / 10 + '0';
        char second = cellNum[tempArray[a].row][tempArray[a].col] % 10 + '0';
        pathData[2*tempArray[a].row+1][4*tempArray[a].col+2] = first;
        pathData[2*tempArray[a].row+1][4*tempArray[a].col+3] = second;
        
      }
      else {
        char replace = cellNum[tempArray[a].row][tempArray[a].col] + '0';
        pathData[2*tempArray[a].row+1][4*tempArray[a].col+2] = replace;
      }
      
    }
    for (a = 0; a < HEIGHT; a++) {
    std::cout << "[z5207984_MTRN4110_PhaseB] ";
      for (b = 0; b < WIDTH; b++) {
        std::cout << pathData[a][b];
      }
    std::cout << std::endl;
    }
  }
  std::cout << "[z5207984_MTRN4110_PhaseB] " << pathFound << " shortest paths found!" << std::endl;
  std::vector<coordinates> currentPlan;
  char tempHeading = start.heading;
  int shortestStep = 0;
  int shortestIndex = 0;
  std::string shortestPlan;
  int prevStepNum = INF;
  
  // Create Path Plan for each possibility and obtain shortest steps + shortest plan
  std::cout << "[z5207984_MTRN4110_PhaseB] Finding shortest path with least turns..." << std::endl;
  for (a = 0; a < pathFound; a++) {
    currentPlan = pathArray[a];
    for (b = 0; b < cellNum[start.row][start.col]+1; b++) {
      if (b == 0) {
        char pathX = start.row + '0';
        char pathY = start.col + '0';
        currentPlan[a].pathPlan.push_back(pathY);
        currentPlan[a].pathPlan.push_back(pathX);
        currentPlan[a].pathPlan.push_back(start.heading);
      }
      else {
        char nextStep = pathPlanStep(tempHeading, currentPlan[b].heading);
        if (nextStep == 'F') {
          currentPlan[a].pathPlan.push_back(nextStep);
          currentPlan[a].stepNum = currentPlan[a].stepNum + 1;
        }
        else if (tempHeading == 'N' && currentPlan[b].heading == 'S') {
          currentPlan[a].pathPlan.push_back('L');
          currentPlan[a].pathPlan.push_back('L');
          currentPlan[a].pathPlan.push_back('F');
          currentPlan[a].stepNum = currentPlan[a].stepNum + 3;
        }
        else if (tempHeading == 'S' && currentPlan[b].heading == 'N') {
          currentPlan[a].pathPlan.push_back('L');
          currentPlan[a].pathPlan.push_back('L');
          currentPlan[a].pathPlan.push_back('F');
          currentPlan[a].stepNum = currentPlan[a].stepNum + 3;
        }
        else {
          currentPlan[a].pathPlan.push_back(nextStep);
          currentPlan[a].pathPlan.push_back('F');
          currentPlan[a].stepNum = currentPlan[a].stepNum + 2;
        }
      }
      tempHeading = currentPlan[b].heading;
    }
    if (currentPlan[a].stepNum < prevStepNum) {
      shortestStep = currentPlan[a].stepNum;
      shortestPlan = currentPlan[a].pathPlan;
      shortestIndex = a;
    }
    prevStepNum = currentPlan[a].stepNum;
  }
  std::vector<coordinates> tempArray = pathArray[shortestIndex];
  for (int i = 0; i < HEIGHT; i++) {
    for (int j = 0; j < WIDTH; j++) {
      for (a = 1; a < cellNum[start.row][start.col]+1; a++) {
        // Check if number is 2 digit
        if (cellNum[tempArray[a].row][tempArray[a].col] > 9) {
          char first = cellNum[tempArray[a].row][tempArray[a].col] / 10 + '0';
          char second = cellNum[tempArray[a].row][tempArray[a].col] % 10 + '0';
          mapData[2*tempArray[a].row+1][4*tempArray[a].col+2] = first;
          mapData[2*tempArray[a].row+1][4*tempArray[a].col+3] = second;
          
        }
        else {
          char replace = cellNum[tempArray[a].row][tempArray[a].col] + '0';
          mapData[2*tempArray[a].row+1][4*tempArray[a].col+2] = replace;
        }
      }
      std::cout << mapData[i][j];
    }
    std::cout << std::endl;
  }
  std::cout << "[z5207984_MTRN4110_PhaseB] Shortest path with least turns found!" << std::endl;
  std::cout << "[z5207984_MTRN4110_PhaseB] Path Plan (" << shortestStep << " steps): " << shortestPlan << std::endl;
  std::cout << "[z5207984_MTRN4110_PhaseB] Writing path plan to ../../PathPlan.txt..." << std::endl;
  
  // Create PathPlan.txt
  std::ofstream pathPlanFile ("../../PathPlan.txt");
  pathPlanFile << shortestPlan << std::endl;
  pathPlanFile.close();
  std::cout << "[z5207984_MTRN4110_PhaseB] Path plan written to ../../PathPlan.txt!" << std::endl;
  
}
