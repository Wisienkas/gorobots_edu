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

  updateMotors(motors, calcActionForGoal(angle, robot->conf.speedupMargin));
}

void BasicController::updateMotors(motor* motors, const Action& action) {
  double left = motors[MIdx("left motor")];
  double right = motors[MIdx("right motor")];
  lpzrobots::Tribot::TribotConfig conf = robot->conf;

  if (action == FORWARD) {
    double diff = (left - right) / 5; // TODO include somehow in config
    motors[MIdx("left motor")] = left + conf.speedupAccelleration - diff;
    motors[MIdx("right motor")] = right + conf.speedupAccelleration + diff;
  } else if (action == LEFT || action == RIGHT) {
    turn(MIdx("left motor"),
         MIdx("right motor"),
         motors,
         conf.maxTurnSpeed,
         conf.maxTurnAccelleration,
         action);
  }
}

void BasicController::turn(int midxLeft,
                           int midxRight,
                           motor* motors,
                           const double& maxTurnSpeed,
                           const double& turnAccelleration,
                           const Action& action) {
  // Reduce turnspeed
  double left = std::min(motors[midxLeft], maxTurnSpeed);
  double right = std::min(motors[midxRight], maxTurnSpeed);
  // Inner switch instead of if case :O
  if (action == LEFT) {
    motors[midxLeft] = std::min(left + turnAccelleration, maxTurnSpeed);
    motors[midxRight] = std::max(right - turnAccelleration, .0);
  } else if (action == RIGHT) {
    motors[midxLeft] = std::max(left - turnAccelleration, .0);
    motors[midxRight] = std::min(right + turnAccelleration, maxTurnSpeed);
  }
}

Action BasicController::calcActionForGoal
(const double& angle, const double& margin) {
  if(std::abs(angle - M_PI * 2) <= margin || angle <= margin) {
    return FORWARD;
  } else if (angle > M_PI) {
    return RIGHT;
  } else if (angle <= M_PI) {
    return LEFT;
  } //TODO consider case of backwards

  // Grace case if something should cause the code to ever get here
  return LEFT;
}


std::map<lpzrobots::Tribot*, double> BasicController::getMateAngles() {
  std::map<lpzrobots::Tribot*, double> map;
  lpzrobots::Tribot* mate = teammates.front();
  Position matePosition = mate->getPosition();

  return map;
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
