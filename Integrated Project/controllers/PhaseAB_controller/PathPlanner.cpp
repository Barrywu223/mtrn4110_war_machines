
#include <PathPlanner.hpp>

PathPlanner::PathPlanner() {
    // Open file to read in map
  std::ifstream file;
  file.open(mapPath);
  if (!file.is_open()) {
    std::cout << "Error: could not find map file" << mapPath; 
    exit(1);
  }
  // Open file to output data
  outputFile.open(outputPath);
  if (!outputFile.is_open()) {
    std::cout << "Error: could not find map file" << outputPath; 
    exit(1);
  }
  std::cout << header << "Reading in map from "<< mapPath << "..." << std::endl;
  outputFile << header << "Reading in map from "<< mapPath << "..." << std::endl;
  std::string buffer;
  while (std::getline(file, buffer)) {
    std::cout << header << buffer << std::endl;
    rawMap.push_back(buffer);
  }
  file.close();
  std::cout << header << "Map read in!" << std::endl;
  outputFile << header << "Map read in!" << std::endl;
}

PathPlanner::~PathPlanner() {outputFile.close();}

void PathPlanner::run() {
  findBestPath();
}

// Runs all functions in correct order to get path
void PathPlanner::findBestPath() {
  findTargets();
  findDistMap();
  findPosPaths();
  printAllMaps();
  findShortestPath();
  writeToFile();
}

// PRIVATE FUNCTIONS

// Gets information from the raw map about start and end positions
void PathPlanner::findTargets() {
  int foundPos;
  for (std::size_t i = 0; i < rawMap.size(); i++) {
    if ((foundPos = (int)rawMap[i].find("x")) != -1) {
      targetX = (i-1)/2; targetY = (foundPos-2)/4;
    }
    if ((foundPos = (int)rawMap[i].find_first_not_of("- |x")) != -1) {
      initialX = (i-1)/2; initialY = (foundPos-2)/4;
      if (rawMap[i][foundPos] == '^') initialH = "N";
      else if (rawMap[i][foundPos] == 'v') initialH = "S";
      else if (rawMap[i][foundPos] == '<') initialH = "W";
      else if (rawMap[i][foundPos] == '>') initialH = "E";
    }
  }
}

// Gets the position in the string relative to the square
std::pair<int, int> PathPlanner::getPos(int x, int y) {
  return std::make_pair(1+x*2, 2+y*4);
}

// Functions find whether the indicated wall is located at
// a given square in the maze
bool PathPlanner::getU(int x, int y) {
  std::pair<int, int> pos = getPos(x,y);
  return rawMap[pos.first-1][pos.second] == '-';
}

bool PathPlanner::getD(int x, int y) {
  std::pair<int, int> pos = getPos(x,y);
  return rawMap[pos.first+1][pos.second] == '-';
}

bool PathPlanner::getL(int x, int y) {
  std::pair<int, int> pos = getPos(x,y);
  return rawMap[pos.first][pos.second-2] == '|';
}

bool PathPlanner::getR(int x, int y) {
  std::pair<int, int> pos = getPos(x,y);
  return rawMap[pos.first][pos.second+2] == '|';
}

// Computes a Dijkstra style distance map
#define INF 0x3FFFFFFF
void PathPlanner::findDistMap() {
  dist = std::vector<std::vector<int>> (5, std::vector<int> (9, INF));
  dist[targetX][targetY] = 0;
  
  // Generates the distance map
  // Values defined by <<x,y>, dist>
  std::vector<std::pair<std::pair<int, int>, int>> pq;
  pq.push_back(std::make_pair(std::make_pair(targetX, targetY), 0));
  while(!pq.empty()) {
    auto elem = pq[0]; pq.erase(pq.begin());
    int currX = elem.first.first, currY = elem.first.second, currDist = elem.second;
    if (!getU(currX, currY) && currDist < dist[currX-1][currY]) {
      dist[currX-1][currY] = currDist+1;
      pq.push_back(std::make_pair(std::make_pair(currX-1, currY), currDist+1));
    }
    if (!getD(currX, currY) && currDist < dist[currX+1][currY]) {
      dist[currX+1][currY] = currDist+1;
      pq.push_back(std::make_pair(std::make_pair(currX+1, currY), currDist+1));
    }
    if (!getL(currX, currY) && currDist < dist[currX][currY-1]) {
      dist[currX][currY-1] = currDist+1;
      pq.push_back(std::make_pair(std::make_pair(currX, currY-1), currDist+1));
    }
    if (!getR(currX, currY) && currDist < dist[currX][currY+1]) {
      dist[currX][currY+1] = currDist+1;
      pq.push_back(std::make_pair(std::make_pair(currX, currY+1), currDist+1));
    }
  }
}

// Finds the co-ordinate traversal for all paths from distance maps
void PathPlanner::findPosPaths() {
  posPaths = std::vector<std::vector<std::pair<int,int>>> (1); int currPath = 0;
  std::vector<std::pair<std::pair<int, int>, int>> pq;
  pq.push_back(std::make_pair(std::make_pair(initialX, initialY), 0));
  
  while (!pq.empty()) {
    auto elem = pq[0]; pq.erase(pq.begin());
    int currX = elem.first.first, currY = elem.first.second, whichPath = elem.second;
    bool branch = false;
    posPaths[whichPath].push_back(std::make_pair(currX, currY));
    if (!getD(currX, currY) && dist[currX+1][currY] == dist[currX][currY]-1) {
      if (branch) {posPaths.resize(posPaths.size()+1); posPaths[++currPath] = posPaths[whichPath];}
      pq.push_back(std::make_pair(std::make_pair(currX+1, currY), (branch) ? currPath : whichPath));
      branch = true;
    }
    if (!getU(currX, currY) && dist[currX-1][currY] == dist[currX][currY]-1) {
      if (branch) {posPaths.resize(posPaths.size()+1); posPaths[++currPath] = posPaths[whichPath];}
      pq.push_back(std::make_pair(std::make_pair(currX-1, currY), (branch) ? currPath : whichPath));
      branch = true;
    }
    if (!getR(currX, currY) && dist[currX][currY+1] == dist[currX][currY]-1) {
      if (branch) {posPaths.resize(posPaths.size()+1); posPaths[++currPath] = posPaths[whichPath];}
      pq.push_back(std::make_pair(std::make_pair(currX, currY+1), (branch) ? currPath : whichPath));
      branch = true;
    }
    if (!getL(currX, currY) && dist[currX][currY-1] == dist[currX][currY]-1) {
      if (branch) {posPaths.resize(posPaths.size()+1); posPaths[++currPath] = posPaths[whichPath];}
      pq.push_back(std::make_pair(std::make_pair(currX, currY-1), (branch) ? currPath : whichPath));
      branch = true;
    }
  }
}

// Transforms positional representation to move based representation
std::string PathPlanner::getPathString(int whichPath) {
  std::vector<std::pair<int, int>> path = posPaths[whichPath];
  std::string pathStr, currH = initialH;
  // Push initial pose to the string
  pathStr.append(std::to_string(initialX));
  pathStr.append(std::to_string(initialY));
  pathStr.append(initialH);
  // "RR" moves deal with when initial pose is facing opposite necessary direction
  for (std::size_t i = 1; i < path.size(); i++) {
    int Xmove = path[i].first - path[i-1].first;
    int Ymove = path[i].second - path[i-1].second;
    if (Xmove == 1) {
      if (currH == "N") pathStr.append("RR");
      else if (currH == "E") pathStr.append("R");
      else if (currH == "W") pathStr.append("L"); 
      currH = "S";
    } else if (Xmove == -1) {
      if (currH == "S") pathStr.append("RR");
      else if (currH == "E") pathStr.append("L");
      else if (currH == "W") pathStr.append("R"); 
      currH = "N";
    }
    if (Ymove == 1) {
      if (currH == "W") pathStr.append("RR");
      else if (currH == "N") pathStr.append("R");
      else if (currH == "S") pathStr.append("L"); 
      currH = "E";
    } else if (Ymove == -1) {
      if (currH == "E") pathStr.append("RR");
      else if (currH == "N") pathStr.append("L");
      else if (currH == "S") pathStr.append("R"); 
      currH = "W";
    }
    pathStr.append("F");
  }
  return pathStr;
}

// Counts number of turns in a path
int PathPlanner::countTurns(std::string path) {
  int total = 0;
  for (char c : path) if (c == 'L' || c == 'R') total++;
  return total;
}

// Prints a path in the assignment style
void PathPlanner::printPathMap(int whichPath) {
  std::vector<std::string> pathMap = rawMap;
  std::vector<std::pair<int, int>> path = posPaths[whichPath];
  path.erase(path.begin()); // dont replace the first cell
  for (std::pair<int,int> pos : path) {
    std::string num = std::to_string(dist[pos.first][pos.second]);
    int strX = getPos(pos.first, pos.second).first;
    int strY = getPos(pos.first, pos.second).second;
    pathMap[strX][strY] = num[0];
    if (num.length() > 1) pathMap[strX][strY+1] = num[1];
  }
  for (std::string s: pathMap) {
    std::cout << header << s << std::endl;
    outputFile << header << s << std::endl;
  }
}

// Prints all maps
void PathPlanner::printAllMaps() {
  std::cout << header << "Finding shortest paths..."<< std::endl;
  outputFile << header << "Finding shortest paths..."<< std::endl;
  for (std::size_t i = 0; i < posPaths.size(); i++) {
    std::cout << header << "Path - " << i+1 << ":" << std::endl;
    outputFile << header << "Path - " << i+1 << ":" << std::endl;
    printPathMap(i);
  }
  std::cout << header << posPaths.size() << " shortest paths found!" << std::endl;
  outputFile << header << posPaths.size() << " shortest paths found!" << std::endl;
}

// Finds the best path with least amount of turns
void PathPlanner::findShortestPath() {
  std::cout << header << "Finding shortest path with least turns..." << std::endl;
  outputFile << header << "Finding shortest path with least turns..." << std::endl;
  int bestI = 0, bestL = countTurns(getPathString(0));
  for (std::size_t i = 1; i < posPaths.size(); i++) {
    if (countTurns(getPathString(i)) < bestL) {
      bestI = (int)i; bestL = countTurns(getPathString(i));
    }
  }
  printPathMap(bestI);
  bestPath = getPathString(bestI);
  std::cout << header << "Shortest path with least turns found!" << std::endl;
  outputFile << header << "Shortest path with least turns found!" << std::endl;
  std::cout << header << "Path Plan (" << bestPath.length()-3 << " steps): " << bestPath << std::endl;
  outputFile << header << "Path Plan (" << bestPath.length()-3 << " steps): " << bestPath << std::endl;
}

// Writes the path plan to the file
void PathPlanner::writeToFile() {
  std::ofstream file;
  file.open(planPath);
  if (!file.is_open()) {
    std::cout << "Error: could not find path file" << planPath; 
    exit(1);
  }
  std::cout << header << "Writing path plan to "<< planPath << "..." << std::endl;
  outputFile << header << "Writing path plan to "<< planPath << "..." << std::endl;
  file << bestPath;
  file.close();
  std::cout << header << "Path plan written to "<< planPath << "!" << std::endl;
  outputFile << header << "Path plan written to "<< planPath << "!" << std::endl;
}
