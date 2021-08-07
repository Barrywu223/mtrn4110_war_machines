
#include <PathRobot.h>

PathRobot::PathRobot(Robot* r) : CustomRobot(r) {
  robot = r;
  file.open(CSVPath);
  if (!file.is_open()) {
    std::cout << "Error: could not set up csv file " << CSVPath; 
    exit(1);
  }
  readFile();
  setupCSV();
}

PathRobot::~PathRobot() {
  file.close();
}

void PathRobot::readFile() {
  std::cout << header << "Reading in motion plan from " << filePath << "..." << std::endl;
  std::ifstream file (filePath);
  if (file.is_open()) {getline(file, rawData); file.close();}
  else {std::cout << "Error: unable to find motion plan " << filePath; exit(1);}
  std::cout << header << "Motion Plan: " << rawData << std::endl;
  // Initialise variables from the raw data
  x = int(rawData[0]) - '0'; y = int(rawData[1]) - '0'; heading = rawData[2];
  std::cout << header << "Motion plan read in!" << std::endl;
}

void PathRobot::setupCSV() {
  file << "Step,Row,Column,Heading,Left Wall,Front Wall,Right Wall\n";
  CSVAppend();
}

void PathRobot::CSVAppend() {
  file << step << "," << x << "," << y << "," << heading << "," 
  << ((leftWall()) ? "Y" : "N") << "," << ((forwardWall()) ? "Y" : "N") 
  << "," << ((rightWall()) ? "Y" : "N") << std::endl;
}

#define MOVESLEEP 3000
#define TURNSLEEP 2500
// #define DIST (double)CELL_DIST/(WHEEL_R*PI)*PI
void PathRobot::followMP() {
  std::cout << header << "Executing motion plan..." << std::endl;
  std::cout << header << "Step: " << std::setfill('0') << std::setw(3) << step 
  << ", Row: " << x << ", Column: " << y  << ", Heading: " << heading << ", Left Wall: " 
  << ((leftWall()) ? "Y" : "N") << ", Front Wall: " << ((forwardWall()) ? "Y" : "N") 
  << ", Right Wall: " << ((rightWall()) ? "Y" : "N") << std::endl;
  for (int i = 3; i < (int)rawData.length(); i++) {
    step++;
    // Perform operation
    if (rawData[i] == 'F') {
      forward(); 
      if (heading == "N") x -= 1;
      else if (heading == "S") x += 1;
      else if (heading == "E") y += 1;
      else if (heading == "W") y -= 1; 
    }
    else if (rawData[i] == 'L') {
      turnLeft();
      if (heading == "N") heading = "W";
      else if (heading == "S") heading = "E";
      else if (heading == "E") heading = "N";
      else if (heading == "W") heading = "S";
    }
    else if (rawData[i] == 'R') {
      turnRight();
      if (heading == "N") heading = "E";
      else if (heading == "S") heading = "W";
      else if (heading == "E") heading = "S";
      else if (heading == "W") heading = "N";
    }
    // Print to console and update CSV file
    std::cout << header << "Step: " << std::setfill('0') << std::setw(3) << step 
    << ", Row: " << x << ", Column: " << y  << ", Heading: " << heading << ", Left Wall: " 
    << ((leftWall()) ? "Y" : "N") << ", Front Wall: " << ((forwardWall()) ? "Y" : "N") 
    << ", Right Wall: " << ((rightWall()) ? "Y" : "N") << std::endl;
    CSVAppend();
  }
  stop();
  std::cout << header << "Motion plan executed!\n";
}
