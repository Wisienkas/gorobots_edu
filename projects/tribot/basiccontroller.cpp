#include "basiccontroller.h"

#include <assert.h>
#include "toolbox.h"
#include <cmath>

using namespace std;
using namespace matrix;

BasicController::BasicController(lpzrobots::Tribot* robot, const Position& goal,
                                 const std::string& name)
  : AbstractController(name, "1.0"),
    robot(robot),
    goal(goal),
    braitenBerg(1.0)
{
  initialized = false;
  calculateLizardEarSpectrum();
}

void BasicController::calculateLizardEarSpectrum() {
  double frontAngle = 0;
  double sideAngle = M_PI / 2;

  this->frontOutput = braitenBerg.calculateOutput(frontAngle);
  this->sideOutput = braitenBerg.calculateOutput(sideAngle);

  this->lowestEarOutput = std::min(
                                   std::min(std::fabs(frontOutput.left),
                                            std::fabs(frontOutput.right)),
                                   std::min(std::fabs(sideOutput.left),
                                            std::fabs(sideOutput.right))
                                   );
  this->highestEarOutput = std::max(
                                    std::max(std::fabs(frontOutput.left),
                                             std::fabs(frontOutput.right)),
                                    std::max(std::fabs(sideOutput.left),
                                             std::fabs(sideOutput.right))
                                    );
}

double BasicController::featureScaling(double x) {
  return
    (std::fabs(x) - this->lowestEarOutput) /
    (this->highestEarOutput - this->lowestEarOutput);
}

void BasicController::stepNoLearning(const sensor* sensors, int number_sensors,
                                     motor* motors, int number_motors) {
  tribot::Output goalOutput = getLizardEarOutput(goal);

  double offset = 110.0;
  double divideGoal = 5.0;
  double divideMate = 50.0;

  double leftPower = featureScaling(goalOutput.right);
  double rightPower = featureScaling(goalOutput.left);
  if(!mate) {
    setMotorPower(motors, leftPower, rightPower);
    return;
  }


  tribot::Output mateOutputFront = getLizardEarOutput(mate->getPosition());

  if (toolbox::isSameSign(mateOutputFront.getDifference(), goalOutput.getDifference())) {
    stearParrallel(mateOutputFront, motors);
  } else {
    stearGoal(goalOutput, mateOutputFront, motors);
  }
}

void BasicController::setMotorPower(motor* motors, double left, double right) {
  motors[MIdx("left motor")] = left;
  motors[MIdx("right motor")] = right;
}

void BasicController::stearParrallel(tribot::Output mate, motor * motors) {
  updateMateValue(mate.getDifference());
  // WHAT A HACK TODO FIX THIS LOL
  double turned90DegreeAngle = mate.getDifference() < 0 ?
                                                      robot->getWheelToWorldAngle() + (M_PI / 2) :
    robot->getWheelToWorldAngle() - (M_PI / 2);
  tribot::Output mateOutput = getLizardEarOutput(this->mate->getPosition(), turned90DegreeAngle);

  double baseSpeed = 0.5;
  setMotorPower(motors,
                baseSpeed + featureScaling(mateOutput.right),
                baseSpeed + featureScaling(mateOutput.left));

  std::cout << this->getName() << " power: " << featureScaling(mateOutput.right) << ", " <<
    featureScaling(mateOutput.left) << "\n";
}

void BasicController::updateMateValue(double incoming) {
  if (std::fabs(incoming) > std::fabs(highestMateValue)) {
    highestMateValue = incoming;
  }
}

void BasicController::stearGoal(tribot::Output goal, tribot::Output mate, motor * motors) {
  double baseSpeed = 0.5;
  double turned90DegreeAngle = robot->getWheelToWorldAngle() + (M_PI / 2);
  tribot::Output mateOutput = getLizardEarOutput(this->mate->getPosition(), turned90DegreeAngle);
  setMotorPower(motors,
                baseSpeed + featureScaling(goal.right) - featureScaling(mateOutput.right),
                baseSpeed + featureScaling(goal.left) - featureScaling(mateOutput.left));
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
