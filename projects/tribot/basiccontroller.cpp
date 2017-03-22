#include "basiccontroller.h"

#include <assert.h>
#include "toolbox.h"
#include <cmath>

using namespace std;
using namespace matrix;

BasicController::BasicController(lpzrobots::Tribot* robot, const Position& goal)
  : AbstractController("Basic Controller", "1.0"), robot(robot), goal(goal) {
  initialized = false;
}

void BasicController::stepNoLearning(const sensor* sensors, int number_sensors,
                                     motor* motors, int number_motors) {
  double angle = getAngle(goal);
  double maxTurnSpeed = 0.2;
  double turnAccel = 0.05;
  double maxSpeed = 1;
  double speedupAccel = 0.1;
  double speedupMargin = M_PI / 12;

  double left = motors[MIdx("left motor")];
  double right = motors[MIdx("right motor")];

  if(std::abs(angle - M_PI * 2) <= speedupMargin || angle <= speedupMargin) {
    std::cout << "SPEEDING UP\n";
    double diff = (left - right) / 5;
    motors[MIdx("left motor")] = std::max(left + speedupAccel - diff, maxSpeed);
    motors[MIdx("right motor")] = std::max(right + speedupAccel + diff, maxSpeed);
  } else {
    left = std::min(left, maxTurnSpeed);
    right = std::min(right, maxTurnSpeed);
    if(angle > M_PI) {
      std::cout << "TURNING RIGHT\n";
      motors[MIdx("left motor")] = std::max(left - turnAccel, .0);
      motors[MIdx("right motor")] = std::min(right + turnAccel, maxTurnSpeed);
    } else {
      cout << "TURNING LEFT\n";
      motors[MIdx("left motor")] = std::min(left + turnAccel, maxTurnSpeed);
      motors[MIdx("right motor")] = std::max(right - turnAccel, .0);
    }
  }
}

std::map<lpzrobots::Tribot*, double> BasicController::getMateAngles() {
  std::map<lpzrobots::Tribot*, double> map;
  lpzrobots::Tribot* mate = teammates.front();
  Position matePosition = mate->getPosition();
}

double BasicController::getAngle(Position point) {
  Position ownPosition = robot->getPosition();

  double x1 = ownPosition.x;
  double x2 = ownPosition.y;

  double y1 = point.x;
  double y2 = point.y;

  double theta = toolbox::trimRadian(atan2(y1 - x1, y2 - x2));
  double robotDirection = robot->getWheelToWorldAngle();

  return toolbox::trimRadian(robotDirection - theta);
}

void BasicController::step(const sensor* sensors, int sensornumber,
                           motor* motors, int motornumber) {
  stepNoLearning(sensors, sensornumber, motors, motornumber);
}

void BasicController::init(int sensornumber, int motornumber, RandGen* randGen) {
  assert(motornumber == 2);
  nSensors = sensornumber;
  nMotors = motornumber;
  initialized = true;
}

void BasicController::addTeammate(lpzrobots::Tribot* teammate)
{
  teammates.push_back(teammate);
}

void BasicController::printTeam()
{
  std::cout << "CONTROLLER: ";
  robot->printInfo();
  std::cout << "Orientation:\n" << robot->getOrientation() << "\n";
  std::cout << "Angle to Goal: " << getAngle(goal) << "\n";
  for (vector<lpzrobots::Tribot*>::iterator i = teammates.begin(); i != teammates.end(); ++i) {
    (*i)->printInfo();
  }
}

int BasicController::getSensorNumber() const {
  return nSensors;
}

int BasicController::getMotorNumber() const {
  return nMotors;
}

bool BasicController::store(FILE* f) const {
  return false;
}

bool BasicController::restore(FILE* f) {
  return false;
}
