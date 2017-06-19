#include "basiccontroller.h"

#include <assert.h>
#include "toolbox.h"
#include <cmath>

using namespace std;
using namespace matrix;
using namespace tribot;

BasicController::BasicController(lpzrobots::Tribot* robot,
                                 const Position& goal,
                                 const std::string& name,
                                 SoundGenerator soundGenerator)
  : AbstractController(name, "1.0"),
    robot(robot),
    goal(goal),
    braitenBerg(soundGenerator)
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
  sim_step++;

  tribot::Output pair1Goal = getLizardEarOutput(goal);
  // Not used to commnented, due to compiler warning(I know its harmless)
  //tribot::Output pair2Goal = getLizardEarOutput(goal,
  //                                              robot->getWheelToWorldAngle() + M_PI / 2);

  // Determines rotation for to stay aligned
  tribot::Output pair1Mate = getLizardEarOutput(mate->getPosition());
  // Determines if speed should increase or fall using diff
  tribot::Output pair2Mate = getLizardEarOutput(mate->getPosition(),
                                                robot->getWheelToWorldAngle() + M_PI / 2);
  double k = 0.2;

  double goalMod = pair1Goal.getDifference() * pair1Mate.getDifference() < 0 ? 1 : 0;

  double power = featureScaling(pair1Goal.right) * goalMod + featureScaling(pair1Goal.left) * goalMod;
  double leftWheel = k
    + featureScaling(pair1Goal.right) * k
    + featureScaling(pair1Mate.left) * pair2Mate.getDifference() / 4
    - power / 20;
  double rightWheel = k
    + featureScaling(pair1Goal.left) * k // Using Cross -> seek mode
    + featureScaling(pair1Mate.right) * pair2Mate.getDifference() / 4 // love and fear
    - power / 20;

  setMotorPower(motors, leftWheel / 60, rightWheel / 60);
}

void BasicController::setMotorPower(motor* motors, double left, double right) {
  motors[MIdx("left motor")] = left;
  motors[MIdx("right motor")] = right;
}


double BasicController::rangeToGoal() {
  Position pos = robot->getPosition();
  return std::sqrt(std::pow(pos.x - goal.x, 2.0) +
                   std::pow(pos.y - goal.y, 2.0) +
                   std::pow(pos.z - goal.z, 2.0));
}

void BasicController::updateMateValue(double incoming) {
  if (std::fabs(incoming) > std::fabs(highestMateValue)) {
    highestMateValue = incoming;
  }
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
