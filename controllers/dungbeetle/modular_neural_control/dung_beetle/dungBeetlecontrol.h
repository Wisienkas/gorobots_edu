/*
 * dungBeetlecontrol.h
 *
 *  Created on: Oct 24, 2015
 *      Author: giuliano
 */

#ifndef _DUNGBEETLECONTROL_H_
#define _DUNGBEETLECONTROL_H_

#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>
#include <selforg/configurable.h>
#include <selforg/types.h>

#include <assert.h>
#include <cmath>
#include <stdlib.h>
#include <string.h>
#include <vector>

#include <selforg/matrix.h>
#include "../plastic.h"
//#include <ode_robots/amosiisensormotordefinition.h>
#include <ode_robots/dungbeetlesensormotordefinition.h>




/**
 *
 * class for hexapod tripodgait using 19 DOF
 *
 */
class dungBeetlecontrol : public AbstractController {

  public:
    dungBeetlecontrol();


    dungBeetlecontrol(int dungBeetletype,bool mMCPGs,bool mMuscleModelisEnabled);
    void initialize(int dungBeetletype,bool mMCPGs,bool mMuscleModelisEnabled);
    virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);


    virtual ~dungBeetlecontrol();

    double sigmoid(double num) {
      return 1.0 / (1.0 + exp(-num));
    }

    /// returns the name of the object (with version number)
    virtual paramkey getName() const {
      return name;
    }
    /// returns the number of sensors the controller was initialised with or 0 if not initialised
    virtual int getSensorNumber() const {
      return numbersensors;
    }
    /// returns the mumber of motors the controller was initialised with or 0 if not initialised
    virtual int getMotorNumber() const {
      return numbermotors;
    }

    /// performs one step (includes learning).
    /// Calulates motor commands from sensor inputs.
    virtual void step(const sensor*, int number_sensors, motor*, int number_motors);

    /// performs one step without learning. Calulates motor commands from sensor inputs.
    virtual void stepNoLearning(const sensor*, int number_sensors, motor*, int number_motors) {
      // empty
    }
    ;

    /***** STOREABLE ****/
    /** stores the controller values to a given file. */
    virtual bool store(FILE* f) const;
    /** loads the controller values from a given file. */
    virtual bool restore(FILE* f);

  protected:
    unsigned short numbersensors, numbermotors;

    int t;
    paramkey name;

    //Begin ADD YOUR VARIABLE HERE//

    //0) Sensor inputs/scaling  ----------------

  public:
    //Angle sensors
    std::vector<sensor> x;
    std::vector<sensor> y;
    std::vector< std::vector<double> > y_MCPGs;
    //Adding more sensory inputs here
    plastic *cpg;


  private:
    bool MCPGs; //indicates whether the controller is based on Multiple CPGs or a single CPG.
    int dungBeetleType;//indicates whether the used robot AMOSv1 (amosType=1)  or AMOSv2(amosType=2).
    bool sensoryFeed;//indicates whether the used robot AMOSv1 (amosType=1)  or AMOSv2(amosType=2).
    bool muscleModel;//indicates whether the used robot AMOSv1 (amosType=1)  or AMOSv2(amosType=2).
    std::vector<double> inputDerivative;
    std::vector<double> inputDerivative2;
    std::ofstream plot;

    // bool mMuscleModelisEnabled;

};

#endif /* _DUNGBEETLECONTROL_H_ */
