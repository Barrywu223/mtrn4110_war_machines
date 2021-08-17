#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#define LINE_LENGTH 10+9*3

#include <sstream>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <vector>
#include <utility>
#include <unistd.h>

class PathPlanner {

  const std::string header = "[z5209457_MTRN4110_PhaseB] ";
  const std::string mapPath = "../../Map.txt";
  const std::string planPath = "../../MotionPlan.txt";
  const std::string outputPath = "../../Output.txt";
  const int MAPSIZE_X = 17;
  const int MAPSIZE_Y = 9;
  std::vector<std::string> rawMap;
  std::string bestPath = "";
  std::ofstream outputFile;
  
  int initialX, initialY; std::string initialH;
  int targetX, targetY;
  
  std::vector<std::vector<int>> dist;
  std::vector<std::vector<std::pair<int,int>>> posPaths;
  std::vector<std::string> paths;
  
private:

  void findTargets();
  void findDistMap();
  void findPosPaths();
  std::string getPathString(int);
  int countTurns(std::string);
  void printPathMap(int);
  void printAllMaps();
  std::string findShortestPath();
  void writeToFile();
  
  // Functions to interpret the raw map string
  // Map interpreted from (0,0) in the top left corner
  std::pair<int, int> getPos(int, int);
  bool getU(int, int);
  bool getD(int, int);
  bool getL(int, int);
  bool getR(int, int);
  
public:

  PathPlanner(std::string map);
  ~PathPlanner();
 
  std::string findBestPath();
  void run();
 
};

#endif