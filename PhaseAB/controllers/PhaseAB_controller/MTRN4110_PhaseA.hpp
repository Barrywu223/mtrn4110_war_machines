#ifndef MTRN4110_PHASEA_HPP
#define MTRN4110_PHASEA_HPP



struct ds_result
{
    bool r;
    bool l;
    bool f;
};
struct Message
{
  int step = 0;
  int row;
  int column;
  char heading;
  char leftWall;
  char frontWall;
  char rightWall;
};

// created function
string readMotionPlan();
void go_forward(Motor *leftMotor, Motor *rightMotor);  
void obstacleDetection(DistanceSensor **ps, Message &m);
void turn_right(Motor *leftMotor, Motor *rightMotor);
void turn_left(Motor *leftMotor, Motor *rightMotor);
void turn_off(Motor *leftMotor, Motor *rightMotor);
void updateMessage(Message &message, char command);
void printMessage(Message &message);
void angleCorrection(Compass *compass, char heading, Motor *leftMotor, Motor *rightMotor, Robot *robot);
void linearCorrection(DistanceSensor *f, Motor *leftMotor, Motor *rightMotor, Robot *robot);
void wirteMotionExectuion(ofstream &myFile, Message &message);

#endif