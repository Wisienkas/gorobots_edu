#include "basiccontroller.h"

#include <assert.h>
#include "toolbox.h"
#include <cmath>

using namespace std;
using namespace matrix;

BasicController::BasicController(lpzrobots::Tribot* robot, const Position& goal)
  : AbstractController("Basic Controller", "1.0"),
    robot(robot),
    goal(goal),
    braitenBerg(1.0)
{
  initialized = false;
}

void BasicController::stepNoLearning(const sensor* sensors, int number_sensors,
                                     motor* motors, int number_motors) {
  tribot::Output output = getLizardEarOutput(goal);

  double leftPower = (output.left + 110.0) / 10.0;
  double rightPower = (output.right + 110.0) / 10.0;
  motors[MIdx("left motor")] = rightPower;
  motors[MIdx("right motor")] = leftPower;
  cout << "{left: " << leftPower << ", right: " << rightPower << "}\n";
}

double tribot::Output getLizardEarOutput(const Position& position) {
  double angle = getAngle(position);
  return braitenBerg.calculateOutput(angle);
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
