
#include <CustomRobot.h>

using namespace webots;
  
// Class constructor/destructor

CustomRobot::CustomRobot(Robot* r) {
  this->r = r;
  // Gets handles to the motors of the robot
  lmotor = r->getMotor("left wheel motor");
  rmotor = r->getMotor("right wheel motor");
  lpos = r->getPositionSensor("left wheel sensor");
  lpos->enable(TIMESTEP);
  rpos = r->getPositionSensor("right wheel sensor");
  rpos->enable(TIMESTEP);
  // Gets handles to sensors
  ldist = r->getDistanceSensor("left sensor"); ldist->enable(TIMESTEP);
  rdist = r->getDistanceSensor("right sensor"); rdist->enable(TIMESTEP);
  fdist = r->getDistanceSensor("forward sensor"); fdist->enable(TIMESTEP);
  compass = r->getCompass("compass"); compass->enable(TIMESTEP);
  // Run motors in velocity control
  lmotor->setPosition(INFINITY);
  rmotor->setPosition(INFINITY);
}

CustomRobot::~CustomRobot() {
  delete r;
}

// Private class function

double CustomRobot::getLPos() {
  return lpos->getValue();
}

double CustomRobot::getRPos() {
  return rpos->getValue();
}

double CustomRobot::getBearing() {
  const double *heading = compass->getValues();
  double rad = atan2(heading[0], heading[2]);
  double bearing = (rad - 1.5708) / PI * 180.0;
  if (bearing < 0.0)
    bearing = bearing + 360.0;
  return bearing;
}

// Custom defined methods for robot interaction

#define MOVE_EPSILON 0.5
#define DIST (double)((CELL_SIZE-WALL_THICK/2)/(2*PI*WHEEL_R)*MAX_ROT)
void CustomRobot::forward() {
  double wallPos = fdist->getValue()/10;
  double currL = getLPos(); double currR = getRPos();
  while (getLPos() - (currL + DIST) < MOVE_EPSILON && getRPos() - (currR + DIST) < MOVE_EPSILON && wallPos > 50) {
    wallPos = fdist->getValue()/10;
    lmotor->setVelocity(MAX_ROT);
    rmotor->setVelocity(MAX_ROT);
    r->step(TIMESTEP);
  }
}


#define TURN_EPSILON 0.5
void CustomRobot::turnLeft() {
  double currBearing =  getBearing();
  double diff = abs(getBearing() - (currBearing - 90));
  if (diff < 0) diff += 360;
  while (diff > TURN_EPSILON) {
    diff = abs(getBearing() - (currBearing - 90));
    if (diff < 0) diff += 360;
    lmotor->setVelocity(-(MAX_ROT/4*diff/90 + MAX_ROT/32));
    rmotor->setVelocity(MAX_ROT/4*diff/90 + MAX_ROT/32);
    r->step(TIMESTEP);
  }
}

void CustomRobot::turnRight() {
  double currBearing =  getBearing();
  double diff = abs(getBearing() - (currBearing + 90));
  if (diff > 360) diff -= 360;
  while (diff > TURN_EPSILON) {
    diff = abs(getBearing() - (currBearing + 90));
    if (diff > 360) diff -= 360;
    lmotor->setVelocity((MAX_ROT/4*diff/90 + MAX_ROT/32));
    rmotor->setVelocity(-(MAX_ROT/4*diff/90 + MAX_ROT/32));
    r->step(TIMESTEP);
  }
}

void CustomRobot::stop() {
  lmotor->setVelocity(0);
  rmotor->setVelocity(0);
  r->step(TIMESTEP);
}

// Lookup table for all sensors constant 1000 = 0.1m linear
// Thus divide value by 10 for answer in mm
#define WALL_TOLERANCE 60
bool CustomRobot::leftWall() {
  return (ldist->getValue()/10 < WALL_TOLERANCE) ? true : false;
}

bool CustomRobot::rightWall() {
  return (rdist->getValue()/10 < WALL_TOLERANCE) ? true : false;
}

bool CustomRobot::forwardWall() {
  return (fdist->getValue()/10 < WALL_TOLERANCE) ? true : false;
}
