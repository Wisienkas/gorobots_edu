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

#include <assert.h>
#include <cmath>
#include <stdlib.h>
#include <string.h>
#include <vector>

#include <selforg/matrix.h>

#include <ode_robots/amosiisensormotordefinition.h>

//Include your control classes
#include "NeuralPreprocessingLearning.h"
#include "NeuralLocomotionControlAdaptiveClimbing.h"


/**
 *
 * class for hexapod tripodgait using 19 DOF
 * 
 */
class AmosIIControl : public AbstractController {

  public:
    AmosIIControl();
    virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

    virtual ~AmosIIControl();

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

    NeuralLocomotionControlAdaptiveClimbing control_adaptiveclimbing;

    //3) Motor postprocessing/scaling   ----------------

    //4) Any other parameters   ----------------

    bool plot_irlearning;
    bool plot_preprofc;
    bool plot_preproirleg;
    bool plot_fmodel_ctr;
    bool plot_reflex_fs;
    bool plot_post_ctr;
    bool plot_fmodel_errors;
    bool plot_fmodel_w;
    bool plot_ussensor_obstacle_avoidance;
    bool plot_reversegear;
    bool plot_nlc;
    bool plot_testbjc;
    bool plot_cpg;
    bool testing;
    bool plot_fmodel_counter;
    bool plot_preproirs;

    //End ADD YOUR VARIABLE HERE//



  public:

};

#endif


