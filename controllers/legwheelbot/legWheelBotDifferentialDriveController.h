// Header guard
#ifndef __LEGWHEELDIFFERENTIALDRIVECONTROLLER_H
#define __LEGWHEELDIFFERENTIALDRIVECONTROLLER_H

#include <selforg/abstractcontroller.h>

class LegWheelBotDifferentialDriveController : public AbstractController{
  public:
    double travelledDist;
    

    LegWheelBotDifferentialDriveController(const std::string& name, const std::string& revision);

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

  private:
    double nSensors;
    double nMotors;
    double previousAngle = 0.;
    int stepNo = 0;
    
};

#endif // Header guard
