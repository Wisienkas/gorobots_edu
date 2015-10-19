/***************************************************************************
 *   Copyright (C) 2010 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *                                                                         *
 *   ANY COMMERCIAL USE FORBIDDEN!                                         *
 *   LICENSE:                                                              *
 *   This work is licensed under the Creative Commons                      *
 *   Attribution-NonCommercial-ShareAlike 2.5 License. To view a copy of   *
 *   this license, visit http://creativecommons.org/licenses/by-nc-sa/2.5/ *
 *   or send a letter to Creative Commons, 543 Howard Street, 5th Floor,   *
 *   San Francisco, California, 94105, USA.                                *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  *
 *                                            * 
 *                                                                         *
 *   $Log: tripodgate18dof.h,v $
 *                                                                         *
 ***************************************************************************/
#ifndef __AMOSIICONTROL_H
#define __AMOSIICONTROL_H

#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>
#include <selforg/configurable.h>
#include <selforg/types.h>

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

#include <selforg/matrix.h>

#include <ode_robots/amosiisensormotordefinition.h>

//Include your control classes
#include "NeuralPreprocessingLearning.h"
#include "NeuralNavigationControl.h"
#include "NeuralLocomotionControl.h"


/**
 *
 * class for hexapod tripodgait using 19 DOF
 * 
 */
class AmosIIControl : public AbstractController {

  public:
    /*The new controller has numerous privileges:
    *1) selection between AMOSv1 (aAMOStype=1) and AMOSv2 (aAMOStype=2). [DEFAULT = 2]
    *2) selection between single CPG-based controller (mMCPGs=false) and Multiple CPGs-based control (mMCPGs=true). [DEFAULT = false]
    *3) possibility to utilize muscle model (mMuscleModelisEnabled=true). [DEFAULT = false]
    *4) selection between integrated navigation controller (mNNC=true) and pure locomotion controller (mNNC=false). [DEFAULT = false]
    * */
    AmosIIControl(int aAMOStype=2, bool mMCPGs=false, bool mMuscleModelisEnabled=false/*, bool mNNC=false*/);
    virtual ~AmosIIControl();
    virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);
    void enableContactForceMech();
    void disableContactForceMech();
    void increaseFrequency();
    void decreaseFrequency();
    void enableOscillatorCoupling();
    void disableOscillatorCoupling();
    void enableTripodGait();
    void enableTetrapodGait();
    void enableWaveGait();
    void enableIrregularGait();


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
    //Adding more sensory inputs here


    //1) Neural preprocessing and learning------------

    NeuralPreprocessingLearning preprocessing_learning;

    //2) Neural locomotion control------

    NeuralLocomotionControl* locomotion_control;

    /**********End ****************/

    //3) Motor postprocessing/scaling   ----------------

    //4) Any other parameters   ----------------

    //End ADD YOUR VARIABLE HERE//



  private:
    bool MCPGs; //indicates whether the controller is based on Multiple CPGs or a single CPG.
    int amosType;//indicates whether the used robot AMOSv1 (amosType=1)  or AMOSv2(amosType=2).
    bool sensoryFeed;//indicates whether the used robot AMOSv1 (amosType=1)  or AMOSv2(amosType=2).
    bool muscleModel;//indicates whether the used robot AMOSv1 (amosType=1)  or AMOSv2(amosType=2).
   // bool mMuscleModelisEnabled;
    unsigned int num_cpgs;

};

#endif

