#include "legWheelBotDifferentialDriveController.h"
#include <assert.h>
#include <math.h>

using namespace std;
using namespace matrix;

LegWheelBotDifferentialDriveController::LegWheelBotDifferentialDriveController(const std::string& name, const std::string& revision)
  : AbstractController(name, revision) {
}

void LegWheelBotDifferentialDriveController::stepNoLearning(const sensor* sensors, int number_sensors,
                                     motor* motors, int number_motors) {
     
     float slowStartScale = std::fmax(1, stepNo/10000);
     float angle = atan2(sensors[3], sensors[2]);
     
     float PI = 3.14159265359;
     float correctionAngleMax = PI / 30;
     float swayFromMiddleToMaxCorrection = 0.60;
     
     float correction = std::fmin(correctionAngleMax, (sensors[5] / swayFromMiddleToMaxCorrection) * correctionAngleMax);
     
     angle = fmod(angle - correction, 2*PI);
     double angleSin = sin(angle);
     double angleCos = cos(angle);
     
  if(angleCos > 0) {
    motors[1] = std::fmax(0, angleSin);
    if(previousAngle >= angle)
      motors[0] = 1;
    else
      motors[0] = std::fmin(1, motors[1] + 0.3);
  } else {
    motors[0] = std::fmax(0, angleSin);
    if(previousAngle < angle)
      motors[1] = 1;
    else
      motors[1] = std::fmin(1, motors[0] + 0.3);
  }  

  motors[0] = motors[0] * slowStartScale;
  motors[1] = motors[1] * slowStartScale;
  
  previousAngle = angle;
  stepNo ++;
  travelledDist = sensors[6];
}

void LegWheelBotDifferentialDriveController::step(const sensor* sensors, int sensornumber,
                           motor* motors, int motornumber) {
  stepNoLearning(sensors,sensornumber, motors, motornumber);
}


void LegWheelBotDifferentialDriveController::init(int sensornumber, int motornumber, RandGen* randGen) {
  assert(motornumber >=2 && sensornumber >=8);
  // Set the number of sensors and motors
  nSensors = sensornumber;
  nMotors  = motornumber;
}

int LegWheelBotDifferentialDriveController::getSensorNumber() const {
  return nSensors;
}

int LegWheelBotDifferentialDriveController::getMotorNumber() const {
  return nMotors;
}

bool LegWheelBotDifferentialDriveController::store(FILE* f) const {
  return true;
}

bool LegWheelBotDifferentialDriveController::restore(FILE* f) {
  return true;
}