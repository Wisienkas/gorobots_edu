// HEADER GUARD
#ifndef __GOROBOTS_EDU_PROJECTS_TRIBOT_BASIC_CONTROLLER_H
#define __GOROBOTS_EDU_PROJECTS_TRIBOT_BASIC_CONTROLLER_H

#include <selforg/abstractcontroller.h>
#include <string>
// The Tribot robot header file
#include "tribot.h"

#include "braiten_berg.h"

const int SAMPLE_SIZE = 4;

namespace tribot {
class BasicController : public AbstractController{
 private:
  double nSensors;
  double nMotors;
  bool initialized;

  long sim_step = 0;

  lpzrobots::Tribot* robot;
  Position goal;
  lpzrobots::Tribot * mate;
  tribot::BraitenBerg braitenBerg;
  double highestMateValue = 0.0;

  tribot::Output frontOutput;
  tribot::Output sideOutput;
  double highestEarOutput;
  double lowestEarOutput;

  double featureScaling(double x);

  void calculateLizardEarSpectrum();

  void updateMateValue(double incoming);

  virtual double getAngle(Position point);
  virtual double getAngle(Position point, double robotDirection);

  void setMotorPower(motor* motor, double left, double right);

  tribot::Output getLizardEarOutput(const Position& position);
  tribot::Output getLizardEarOutput(const Position& position, double offsetAngle);

 public:
  BasicController(lpzrobots::Tribot* robot,
                  const Position& goal,
                  const std::string& name,
                  SoundGenerator soundGenerator);

  virtual void setMatePositionReference(lpzrobots::Tribot * mate);

  lpzrobots::Tribot * getRobot() {return robot;}
  Position getGoal() {return goal;}

  double rangeToGoal();

  /** initialisation of the controller with the given sensor/ motornumber
      Must be called before use. The random generator is optional.
  */
  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

  /** @return Number of sensors the controller
      was initialised with or 0 if not initialised */
  virtual int getSensorNumber() const;

    /** @return Number of motors the controller
        was initialised with or 0 if not initialised */
  virtual int getMotorNumber() const;

  /** performs one step.
      Calculates motor commands from sensor inputs.
      @param sensors sensors inputs scaled to [-1,1]
      @param sensornumber length of the sensor array
      @param motors motors outputs. MUST have enough space for motor values!
      @param motornumber length of the provided motor array
  */
  virtual void step(const sensor* sensors, int sensornumber,
                    motor* motors, int motornumber);


  /** performs one step without learning.
      @see step
  */
  virtual void stepNoLearning(const sensor* , int number_sensors,
                              motor* , int number_motors);

  /** stores the object to the given file stream (binary).
   */
  virtual bool store(FILE* f) const;

  /** loads the object from the given file stream (binary).
   */
  virtual bool restore(FILE* f);

};
}
#endif // end of __GOROBOTS_EDU_PROJECTS_TRIBOT_BASIC_CONTROLLER_H
