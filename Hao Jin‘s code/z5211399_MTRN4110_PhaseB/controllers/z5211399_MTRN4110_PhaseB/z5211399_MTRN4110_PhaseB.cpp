// File: z5211399_MTRN4110_PhaseB.cpp
// Date: 2021/7/8
// Description: Controller of E-puck for Phase B - Path Planning
// Author: Hao Jin       
// Modifications: 
// Platform: Windows
// Notes: The map is read into the program.

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdio.h>
#include <limits.h>
#include <utility>
#include <queue>
#include <cmath>
#include <algorithm>
// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

// defined variables
#define EAST 0
#define SOUTH 1
#define WEST 2
#define NORTH 3
// global variables
const string MAP_FILE_NAME = "../../Map.txt";
const string PRELIX = "[z5211399_MTRN4110_PhaseB] ";
const string PathPlanFile = "../../PathPlan.txt";
const string OutputFile = "../../Output.txt";
const int numOfRows = 5;
const int numOfColumns = 9;
const int INF = INT_MAX;
int pathCounter = 0;
int initialDirection;
// P = <row number, column number>
typedef pair<int, int> P;
// starting point and ending point
P x, v;
// one single path
typedef vector<P> Path;
// a vector of paths
typedef vector<vector<P>> Pathlist;
Pathlist pathlist;

// create matrix
class Matrix {
public:
	Matrix() : data({}) {}

	Matrix(const int& rows, const int& cols) {
		Reset(rows, cols);
	}

	void Reset(const int& rows, const int& cols) {
		data.resize(rows);
		for (int i = 0; i < rows; ++i) {
			data.at(i).resize(cols);
		}
	}

	int At(const int& row, const int& col) const {
		return data.at(row).at(col);
	}

	int& At(const int& row, const int& col) {
		return data.at(row).at(col);
	}

	int GetNumRows() const {
		return data.size();
	}

	int GetNumColumns() const {
		if (GetNumRows() > 0) {
			return data[0].size();
		}

		return 0;
	}

private:
	vector<vector<int>> data;
};


Matrix verticalWall(numOfRows, numOfColumns + 1); // 5 10
Matrix horizontalWall(numOfRows + 1, numOfColumns); // 6 9
Matrix realMaze(numOfRows, numOfColumns); // 5 9


void MatrixInfo(const Matrix& m) {
	std::cout << "{ \"rows\": " << m.GetNumRows()
		<< ", \"cols\": " << m.GetNumColumns() << " }" << std::endl;
}

// created functions
bool readMap(string FileName, vector<string>& vecOfStrs);
void initializeMatrix(vector<string>& vecOfStrs);
void printmatrix(Matrix& matrix);
void searchPoints(char c, int row, int column);
void bfs();
void pathSearch(P currentPos, int currentStep, Path path);
bool cellSearch(P p, int dir, int step);
Path storePath(Path path, P cellPos);
void go2Pathlist(Path p);
void printPathlist(const Pathlist pathlist, vector<string> map);
int countTurns(Path path);
void printOptimalPath(const Pathlist pathlist, vector<string> map);
void pathPlan(Path path);

// oprn up outPut.txt
ofstream MyOutput(OutputFile);

int main(int argc, char** argv) {

	// read in Map.txt
	vector<string> map;
	bool readResult = readMap(MAP_FILE_NAME, map);
	if (readResult)
	{
		// Print the vector contents
		for (const auto& line : map) {
			cout << PRELIX << line << endl;
			MyOutput << PRELIX << line << endl;
		}
		cout << PRELIX << "Map read in!" << endl;
		MyOutput << PRELIX << "Map read in!" << endl;
	}

	initializeMatrix(map);
	bfs();
	Path path;
	path.push_back(x);
	pathSearch(x, 0, path);
	printPathlist(pathlist, map);
	cout << PRELIX << pathCounter << " shortest paths found!" << endl;
	MyOutput << PRELIX << pathCounter << " shortest paths found!" << endl;
	printOptimalPath(pathlist, map);
	MyOutput.close();

	return 0;
}

bool readMap(string FileName, vector<string>& vecOfStrs) {
	ifstream in(MAP_FILE_NAME);
	if (!in) {
		cout << PRELIX << "Cannot open up file " << MAP_FILE_NAME << endl;
		return false;
	}
	cout << PRELIX << "Reading in map from " << MAP_FILE_NAME << "..." << endl;
	MyOutput << PRELIX << "Reading in map from " << MAP_FILE_NAME << "..." << endl;
	string str;
	while (getline(in, str))
	{
		if (str.size() > 0) {
			vecOfStrs.push_back(str);
		}
	}
	in.close();
	return true;
}

void initializeMatrix(vector<string>& map) {
	int stringCounter = 0;
	int horizontalRow = 0;
	int horizontalColumn = 0;
	int verticalRow = 0;
	int verticalColumn = 0;

	for (string line : map) {
		// search for horizontal wall
		if (stringCounter % 2 == 0) {
			int i = 1;
			while (i <= 1 + 4 * 8) {
				if (line[i] == '-') {
					horizontalWall.At(horizontalRow, horizontalColumn) = 1;
				}
				i += 4;
				horizontalColumn += 1;
			}
			horizontalRow += 1;
			horizontalColumn = 0;
		}
		// search for vertical wall and critical points
		else
		{
			int j = 0;
			while (j <= 4 * 9) {
				if (line[j] == '|') {
					verticalWall.At(verticalRow, verticalColumn) = 1;
				}
				j += 4;
				verticalColumn += 1;
			}
			int k = 2;
			while (k <= 2 + 4 * 8) {
				searchPoints(line[k], verticalRow, (k - 2) / 4);
				k += 4;
			}
			verticalRow += 1;
			verticalColumn = 0;
		}
		stringCounter += 1;
	}

	//cout << "matrix for horizontal wall: " << endl;
	//printmatrix(horizontalWall);
	//cout << "matrix for vertical wall: " << endl;
	//printmatrix(verticalWall);
	//cout << "matrix for real maze: " << endl;
	//printmatrix(realMaze);

}
void printmatrix(Matrix& matrix) {
	for (int i = 0; i < matrix.GetNumRows(); i++) {
		for (int j = 0; j < matrix.GetNumColumns(); j++) {
			cout << matrix.At(i, j) << " ";
		}
		cout << endl;
	}
}

void bfs() {
	cout << PRELIX << "Finding shortest paths..." << endl;
	MyOutput << PRELIX << "Finding shortest paths..." << endl;
	queue<P> que;

	// initialize all cell values
	for (int i = 0; i < numOfRows; ++i)
		for (int j = 0; j < numOfColumns; ++j)
			realMaze.At(i, j) = INF;

	// push the goal point into queue
	que.push(x);

	// initialize current explored value
	realMaze.At(x.first, x.second) = 0;

	int ColumnShift[4] = { 1,0,-1,0 };
	int RowShift[4] = { 0,-1,0,1 };

	while (que.size()) {
		P p = que.front(); que.pop();

		if (p.first == v.first && p.second == v.second) break;

		for (int dir = 0; dir < 4; dir++) {
			int rowPos = p.first + RowShift[dir], columnPos = p.second + ColumnShift[dir];

			switch (dir) {
			case EAST:

				if (0 <= rowPos && rowPos < numOfRows && 0 <= columnPos && columnPos < numOfColumns && verticalWall.At(rowPos, columnPos) != 1 && realMaze.At(rowPos, columnPos) == INF) {
					que.push(P(rowPos, columnPos));
					realMaze.At(rowPos, columnPos) = realMaze.At(p.first, p.second) + 1;
				}
				break;

			case WEST:
				if (0 <= rowPos && rowPos < numOfRows && 0 <= columnPos && columnPos < numOfColumns && verticalWall.At(rowPos, columnPos + 1) != 1 && realMaze.At(rowPos, columnPos) == INF) {
					que.push(P(rowPos, columnPos));
					realMaze.At(rowPos, columnPos) = realMaze.At(p.first, p.second) + 1;
				}
				break;

			case NORTH:
				if (0 <= rowPos && rowPos < numOfRows && 0 <= columnPos && columnPos < numOfColumns && horizontalWall.At(rowPos, columnPos) != 1 && realMaze.At(rowPos, columnPos) == INF) {
					que.push(P(rowPos, columnPos));
					realMaze.At(rowPos, columnPos) = realMaze.At(p.first, p.second) + 1;
				}
				break;

			case SOUTH:
				if (0 <= rowPos && rowPos < numOfRows && 0 <= columnPos && columnPos < numOfColumns && horizontalWall.At(rowPos + 1, columnPos) != 1 && realMaze.At(rowPos, columnPos) == INF) {
					que.push(P(rowPos, columnPos));
					realMaze.At(rowPos, columnPos) = realMaze.At(p.first, p.second) + 1;
				}
				break;
			}
		}
	}
	// cout << "Finally..." << endl;
	// cout << "matrix for real maze: " << endl;
	// printmatrix(realMaze);
}



// search for the starting point and ending point
void searchPoints(char c, int row, int column) {
	if (c == 'v' || c == '^' || c == '<' || c == '>') {
		v.first = row;
		v.second = column;
		switch (c) {
		case 'v':
			initialDirection = SOUTH;
			break;
		case '^':
			initialDirection = NORTH;
			break;
		case '>':
			initialDirection = EAST;
			break;
		case '<':
			initialDirection = WEST;
			break;
		}
		//realMaze.At(v.first, v.second) = '8';
		// cout << "starting cell at: " << v.first << " " << v.second << endl;
	}
	if (c == 'x') {
		x.first = row;
		x.second = column;
		//At(v.first, v.second) = '9';
		// cout << "ending cell at: " << x.first << " " << x.second << endl;
	}
}

void pathSearch(P currentPos, int currentStep, Path path) {
	if (currentPos.first == v.first && currentPos.second == v.second) {
		pathCounter++;
		go2Pathlist(path);
		return;
	}
	// cout << "I am here..." << endl;
	if (cellSearch(currentPos, EAST, currentStep)) {
		P posHolder = currentPos;
		posHolder.second++;
		Path newpath = storePath(path, posHolder);
		//cout << currentStep + 1 << " go east to: " << posHolder.first << " " << posHolder.second << endl;
		pathSearch(posHolder, currentStep + 1, newpath);
	}
	if (cellSearch(currentPos, SOUTH, currentStep)) {
		P posHolder = currentPos;
		posHolder.first++;
		Path newpath = storePath(path, posHolder);
		//cout << currentStep + 1 << " go south to: " << posHolder.first << " " << posHolder.second << endl;
		pathSearch(posHolder, currentStep + 1, newpath);
	}
	if (cellSearch(currentPos, WEST, currentStep)) {
		P posHolder = currentPos;
		posHolder.second--;
		Path newpath = storePath(path, posHolder);
		//cout << currentStep + 1 << " go west to: " << posHolder.first << " " << posHolder.second << endl;
		pathSearch(posHolder, currentStep + 1, newpath);
	}
	if (cellSearch(currentPos, NORTH, currentStep)) {
		P posHolder = currentPos;
		posHolder.first--;
		Path newpath = storePath(path, posHolder);
		//cout << currentStep + 1 << " go north to: " << posHolder.first << " " << posHolder.second << endl;
		pathSearch(posHolder, currentStep + 1, newpath);
	}
	// cout << "No way out at step" << currentStep << " " << currentPos.first << " " << currentPos.second << endl;

}

bool cellSearch(P p, int dir, int step) {
	bool result = false;
	if (dir == EAST && p.second + 1 < numOfColumns && verticalWall.At(p.first, p.second + 1) != 1)
		result = (step + 1 == realMaze.At(p.first, p.second + 1)) ? true : false;

	else if (dir == SOUTH && p.first + 1 < numOfRows && horizontalWall.At(p.first + 1, p.second) != 1)
		result = (step + 1 == realMaze.At(p.first + 1, p.second)) ? true : false;

	else if (dir == WEST && p.second - 1 >= 0 && verticalWall.At(p.first, p.second) != 1)
		result = (step + 1 == realMaze.At(p.first, p.second - 1)) ? true : false;

	else if (dir == NORTH && p.first - 1 >= 0 && horizontalWall.At(p.first, p.second) != 1)
		result = (step + 1 == realMaze.At(p.first - 1, p.second)) ? true : false;

	return result;
}

Path storePath(Path path, P cellPos) {
	path.push_back(cellPos);
	return path;
}

void go2Pathlist(Path p) {
	pathlist.push_back(p);
}

void printPathlist(const Pathlist pathlist, vector<string> map) {
	int i = 1;
	for (const auto& path : pathlist) {
		cout << PRELIX << "Path - " << i << ":" << endl;
		MyOutput << PRELIX << "Path - " << i << ":" << endl;
		vector<string> mapHolder = map;
		for (const auto& p : path) {
			int rowNum = p.first;
			int colNum = p.second;
			int cellValue = realMaze.At(rowNum, colNum);
			// if at starting point
			if (rowNum == v.first && colNum == v.second) {
				int dir = initialDirection;
				string str;
				switch (dir) {
				case NORTH:
					str = "^";
					map.at(2 * rowNum + 1).replace(4 * colNum + 2, 1, str);
					break;
				case SOUTH:
					str = "v";
					map.at(2 * rowNum + 1).replace(4 * colNum + 2, 1, str);
					break;
				case EAST:
					str = ">";
					map.at(2 * rowNum + 1).replace(4 * colNum + 2, 1, str);
					break;
				case WEST:
					str = "<";
					map.at(2 * rowNum + 1).replace(4 * colNum + 2, 1, str);
					break;
				}
			}
			else if (cellValue <= 9) {
				string str1 = to_string(cellValue);
				mapHolder.at(2 * rowNum + 1).replace(4 * colNum + 2, 1, str1);
			}
			else {
				string str1 = to_string(cellValue / 10 % 10);
				string str2 = to_string(cellValue / 1 % 10);
				mapHolder.at(2 * rowNum + 1).replace(4 * colNum + 2, 1, str1);
				mapHolder.at(2 * rowNum + 1).replace(4 * colNum + 3, 1, str2);
			}
		}
		//cout << endl;
		//MyOutput << endl;
		for (const auto& line : mapHolder) {
			cout << PRELIX << line << endl;
			MyOutput << PRELIX << line << endl;
		}
		i += 1;
		// cout << "num of turns: " << countTurns(path) << endl;
	}
}

int countTurns(Path path) {
	reverse(path.begin(), path.end());
	//for (const auto& p : path) {
	//	cout << p.first << "," << p.second << " ";
	//}
	//cout << endl;
	int numOfTurn = 0;
	int preRow = v.first;
	int preCol = v.second;
	int dir = initialDirection;
	// cout << "initial direction is " << dir << endl;
	for (const auto& p : path) {
		bool atStart = (p.first == preRow && p.second == preCol);
		// cout << preRow << preCol << endl;
		if (!atStart) {
			// if robot is facing east
			if (p.first == preRow && p.second == preCol + 1) {
				if (dir == WEST) {
					//cout << "make 2 turns at" << preRow << "," << preCol << endl;
					numOfTurn += 2;
				}
				else if (dir == NORTH || dir == SOUTH) {
					//cout << "make 1 turn at" << preRow << "," << preCol << endl;
					numOfTurn += 1;
				}
				dir = EAST;
			}

			// if robot is facing west
			if (p.first == preRow && p.second == preCol - 1) {
				if (dir == EAST) {
					//cout << "make 2 turns at" << preRow << "," << preCol << endl;
					numOfTurn += 2;
				}
				else if (dir == NORTH || dir == SOUTH) {
					//cout << "make 1 turn at" << preRow << "," << preCol << endl;
					numOfTurn += 1;
				}
				dir = WEST;
			}

			// if robot is facing south
			if (p.first == preRow + 1 && p.second == preCol) {
				if (dir == NORTH) {
					//cout << "make 2 turns at" << preRow << "," << preCol << endl;
					numOfTurn += 2;
				}
				else if (dir == EAST || dir == WEST) {
					//cout << "make 1 turn at" << preRow << "," << preCol << endl;
					numOfTurn += 1;
				}
				dir = SOUTH;
			}

			// if robot is facing north
			if (p.first == preRow - 1 && p.second == preCol) {
				if (dir == SOUTH) {
					//cout << "make 2 turns at" << preRow << "," << preCol << endl;
					numOfTurn += 2;
				}
				else if (dir == WEST || dir == EAST) {
					//cout << "make 1 turn at" << preRow << "," << preCol << endl;
					numOfTurn += 1;
				}
				dir = NORTH;
			}
		}
		preRow = p.first;
		preCol = p.second;
	}
	return numOfTurn;
}

void printOptimalPath(const Pathlist pathlist, vector<string> map) {
	cout << PRELIX << "Finding shortest path with least turns..." << endl;
	MyOutput << PRELIX << "Finding shortest path with least turns..." << endl;

	int pathIndex = 0;
	int smallestStep = INF;
	int i = 0;
	for (const auto& path : pathlist) {
		int curStep = countTurns(path);
		if (curStep <= smallestStep) {
			smallestStep = curStep;
			pathIndex = i;
		}
		i += 1;
	}

	for (const auto& p : pathlist.at(pathIndex)) {
		int rowNum = p.first;
		int colNum = p.second;
		int cellValue = realMaze.At(rowNum, colNum);
		// if it is the starting point
		if (rowNum == v.first && colNum == v.second) {
			int dir = initialDirection;
			string str;
			switch (dir) {
			case NORTH:
				str = "^";
				map.at(2 * rowNum + 1).replace(4 * colNum + 2, 1, str);
				break;
			case SOUTH:
				str = "v";
				map.at(2 * rowNum + 1).replace(4 * colNum + 2, 1, str);
				break;
			case EAST:
				str = ">";
				map.at(2 * rowNum + 1).replace(4 * colNum + 2, 1, str);
				break;
			case WEST:
				str = "<";
				map.at(2 * rowNum + 1).replace(4 * colNum + 2, 1, str);
				break;
			}
		}
		else if (cellValue <= 9) {
			string str1 = to_string(cellValue);
			map.at(2 * rowNum + 1).replace(4 * colNum + 2, 1, str1);
		}
		else {
			string str1 = to_string(cellValue / 10 % 10);
			string str2 = to_string(cellValue / 1 % 10);
			map.at(2 * rowNum + 1).replace(4 * colNum + 2, 1, str1);
			map.at(2 * rowNum + 1).replace(4 * colNum + 3, 1, str2);
		}
	}
	for (const auto& line : map) {
		cout << PRELIX << line << endl;
		MyOutput << PRELIX << line << endl;
	}

	cout << PRELIX << "Shortest path with least turns found!" << endl;
	MyOutput << PRELIX << "Shortest path with least turns found!" << endl;
	pathPlan(pathlist.at(pathIndex));
}

void pathPlan(Path path) {
	reverse(path.begin(), path.end());
	vector<char> pathPlan;
	int numOfStep = 0;
	int preRow = v.first;
	int preCol = v.second;
	int dir = initialDirection;
	for (const auto& p : path) {
		bool atStart = (p.first == preRow && p.second == preCol);
		// not at starting pioint
		if (!atStart) {
			// if robot is facing east
			if (p.first == preRow && p.second == preCol + 1) {
				if (dir == WEST) {
					pathPlan.push_back('L');
					pathPlan.push_back('L');
					pathPlan.push_back('F');
					numOfStep += 3;
				}
				else if (dir == NORTH) {
					pathPlan.push_back('R');
					pathPlan.push_back('F');
					numOfStep += 2;
				}
				else if (dir == SOUTH) {
					pathPlan.push_back('L');
					pathPlan.push_back('F');
					numOfStep += 2;
				}
				else {
					pathPlan.push_back('F');
					numOfStep += 1;
				}
				dir = EAST;
			}

			// if robot is facing west
			if (p.first == preRow && p.second == preCol - 1) {
				if (dir == EAST) {
					pathPlan.push_back('L');
					pathPlan.push_back('L');
					pathPlan.push_back('F');
					numOfStep += 3;
				}
				else if (dir == SOUTH) {
					pathPlan.push_back('R');
					pathPlan.push_back('F');
					numOfStep += 2;
				}
				else if (dir == NORTH) {
					pathPlan.push_back('L');
					pathPlan.push_back('F');
					numOfStep += 2;
				}
				else {
					pathPlan.push_back('F');
					numOfStep += 1;
				}
				dir = WEST;
			}

			// if robot is facing south
			if (p.first == preRow + 1 && p.second == preCol) {
				if (dir == NORTH) {
					pathPlan.push_back('L');
					pathPlan.push_back('L');
					pathPlan.push_back('F');
					numOfStep += 3;
				}
				else if (dir == EAST) {
					pathPlan.push_back('R');
					pathPlan.push_back('F');
					numOfStep += 2;
				}
				else if (dir == WEST) {
					pathPlan.push_back('L');
					pathPlan.push_back('F');
					numOfStep += 2;
				}
				else {
					pathPlan.push_back('F');
					numOfStep += 1;
				}
				dir = SOUTH;
			}

			// if robot is facing north
			if (p.first == preRow - 1 && p.second == preCol) {
				if (dir == SOUTH) {
					pathPlan.push_back('L');
					pathPlan.push_back('L');
					pathPlan.push_back('F');
					numOfStep += 3;
				}
				else if (dir == WEST) {
					pathPlan.push_back('R');
					pathPlan.push_back('F');
					numOfStep += 2;
				}
				else if (dir == EAST) {
					pathPlan.push_back('L');
					pathPlan.push_back('F');
					numOfStep += 2;
				}
				else {
					pathPlan.push_back('F');
					numOfStep += 1;
				}
				dir = NORTH;
			}
		}
		// at starting point
		else {
			pathPlan.push_back(v.first + '0');
			pathPlan.push_back(v.second + '0');
			switch (dir) {
			case EAST:
				pathPlan.push_back('E');
				break;
			case WEST:
				pathPlan.push_back('W');
				break;
			case NORTH:
				pathPlan.push_back('N');
				break;
			case SOUTH:
				pathPlan.push_back('S');
				break;
			}
		}
		preRow = p.first;
		preCol = p.second;
	}
	ofstream MyPathPlan(PathPlanFile);
	cout << PRELIX << "Path Plan (" << numOfStep << " steps): ";
	MyOutput << PRELIX << "Path Plan (" << numOfStep << " steps): ";
	for (const auto& c : pathPlan) {
		cout << c;
		MyOutput << c;
		MyPathPlan << c;
	}
	cout << endl;
	MyOutput << endl;
	cout << PRELIX << "Writing path plan to " << PathPlanFile << "..." << endl;
	MyOutput << PRELIX << "Writing path plan to " << PathPlanFile << "..." << endl;
	MyPathPlan.close();
	cout << PRELIX << "Path plan written to " << PathPlanFile << "!" << endl;
	MyOutput << PRELIX << "Path plan written to " << PathPlanFile << "!" << endl;
}