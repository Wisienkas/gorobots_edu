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
  tribot::Output goalOutput = getLizardEarOutput(goal);

  double offset = 110.0;
  double divideGoal = 5.0;
  double divideMate = 50.0;

  double leftPower = (goalOutput.left + offset) / divideGoal;
  double rightPower = (goalOutput.right + offset) / divideGoal;
  if(!mate) {
    setMotorPower(motors, leftPower, rightPower);
    return;
  }

  double turned90DegreeAngle = robot->getWheelToWorldAngle() + (M_PI / 2);
  tribot::Output mateOutput = getLizardEarOutput(mate->getPosition(), turned90DegreeAngle);

  leftPower = leftPower * ((mateOutput.left + offset) / divideMate);
  rightPower = rightPower * ((mateOutput.right + offset) / divideMate);
  setMotorPower(motors, leftPower, rightPower);
}

void BasicController::setMotorPower(motor* motors, double left, double right) {
  motors[MIdx("left motor")] = left;
  motors[MIdx("right motor")] = right;
}

void BasicController::stearParrallel(tribot::Output mate, motor * motors) {
  updateMateValue(mate.getDifference());
}

void BasicController::updateMateValue(double incoming) {
  if (std::fabs(incoming) > std::fabs(highestMateValue)) {
    highestMateValue = incoming;
  }
}

void BasicController::stearGoal(tribot::Output goal, tribot::Output mate, motor * motors) {
  double leftPower = (goal.left + 110.0) / 10.0;
  double rightPower = (goal.right + 110.0) / 10.0;
  motors[MIdx("left motor")] = rightPower;
  motors[MIdx("right motor")] = leftPower;
}

tribot::Output BasicController::getLizardEarOutput(const Position& position) {
  return getLizardEarOutput(position, robot->getWheelToWorldAngle());
}

tribot::Output BasicController::getLizardEarOutput(const Position& position,
                                                   double offsetDegree) {
  double angle = getAngle(position, offsetDegree);
  return braitenBerg.calculateOutput(angle);
}

void BasicController::setMatePositionReference(lpzrobots::Tribot * mate) {
  this->mate = mate;
}

double BasicController::getAngle(Position point) {
  return getAngle(point, robot->getWheelToWorldAngle());
}

double BasicController::getAngle(Position point, double robotDirection) {
  Position ownPosition = robot->getPosition();

  double x1 = ownPosition.x;
  double x2 = ownPosition.y;

  double y1 = point.x;
  double y2 = point.y;

  double theta = toolbox::trimRadian(atan2(y1 - x1, y2 - x2));

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
