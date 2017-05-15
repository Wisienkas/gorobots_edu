#include "basiccontroller.h"

#include <assert.h>
#include "toolbox.h"
#include <cmath>

using namespace std;
using namespace matrix;
using namespace tribot;

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
  sim_step++;

  tribot::Output pair1Goal = getLizardEarOutput(goal);
  // Determines rotation for to stay aligned
  tribot::Output pair1Mate = getLizardEarOutput(mate->getPosition());
  // Determines if speed should increase or fall using diff
  tribot::Output pair2Mate = getLizardEarOutput(mate->getPosition(),
                                                robot->getWheelToWorldAngle() + M_PI / 2);

  // Basespeed constant
  double k = 0.2;
  double speedup = 1;

  //double goalMod = 1;
  //double goalMod = std::pow(0.96, (pair1Goal.getDifference() * pair1Mate.getDifference()));
  double goalMod = pair1Goal.getDifference() * pair1Mate.getDifference() < 0 ? 1 : 0;

  double power = featureScaling(pair1Goal.right) * goalMod + featureScaling(pair1Goal.left) * goalMod;
  //std::cout << getName() << ":" << goalMod << "\n";
  double leftWheel = k
    + featureScaling(pair1Goal.right) * goalMod
    + featureScaling(pair1Mate.left) * pair2Mate.getDifference()
    - power / 2;
  double rightWheel = k
    + featureScaling(pair1Goal.left) * goalMod // Using Cross -> seek mode
    + featureScaling(pair1Mate.right) * pair2Mate.getDifference() // Using fear, sateliting
    - power / 2;

  setMotorPower(motors, softstep(leftWheel) * speedup, softstep(rightWheel) * speedup);
  /*
  double leftPower = featureScaling(goalOutput.right);
  double rightPower = featureScaling(goalOutput.left);
  if(!mate) {
    setMotorPower(motors, leftPower, rightPower);
    return;
  }

  tribot::Output mateOutputFront = getLizardEarOutput(mate->getPosition());

  //stearGoal(goalOutput, mateOutputFront, motors);
  if (toolbox::isSameSign(mateOutputFront.getDifference(), goalOutput.getDifference())) {
    stearParrallel(mateOutputFront, motors);
  } else {
    stearGoal(goalOutput, mateOutputFront, motors);
    }*/
}

double BasicController::softstep(double y) {
  double L = 1.0; // Max value of curve
  double x0 = 100.0; // mid point
  double k = 0.1; // Steepness of curve
  double x = sim_step; // time unit

  double a = L / (1.0 + std::pow(M_E, -k * (x - x0)));

  return y;
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

void BasicController::stearGoal(tribot::Output goal, tribot::Output mate, motor * motors) {
  double baseSpeed = 0.5;
  double turned90DegreeAngle = robot->getWheelToWorldAngle() + (M_PI / 2);
  tribot::Output mateOutput = getLizardEarOutput(this->mate->getPosition(), turned90DegreeAngle);

  setMotorPower(motors,
                baseSpeed + featureScaling(goal.right) - featureScaling(mateOutput.right),
                baseSpeed + featureScaling(goal.left)  - featureScaling(mateOutput.left));
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
