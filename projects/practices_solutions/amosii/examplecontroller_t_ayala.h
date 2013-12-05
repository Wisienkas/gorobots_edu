#ifndef __EMPTYCONTROLLER_H
#define __EMPTYCONTROLLER_H

#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>
#include <selforg/configurable.h>

#include <assert.h>
#include <cmath>
#include <stdlib.h>
#include <string.h>

#include <selforg/matrix.h>

/**
 * Empty robot controller.
 * The controller gets a number of input sensor values each timestep
 * and has to generate a number of output motor values.
 *
 * Go to the step() function and enter the control commands with respect to your task!
 *
 */
class EmptyController : public AbstractController {

  public:
    EmptyController()
    : AbstractController("EmptyController", "$Id: tripodgait18dof.cpp,v 0.1 $"){
      t = 0;

      outputH1 = 0.001;
      outputH2 = 0.001;
    };

    double mu;
    double TAU;

    double X;
    double Y;
    double X_dot;
    double Y_dot;

    virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0){
      //Tripodgait for 18 DOF Hexapod
      assert(motornumber>=18);
      mu = 0.2;
      TAU = 0.2;

      X = 1.5;
      Y = 0.0;
      X_dot = 0.0;
      Y_dot = 0.0;

    };

    virtual ~EmptyController(){};

    /// returns the name of the object (with version number)
    virtual paramkey getName() const {
      return name;
    }
    /// returns the number of sensors the controller was initialised with or 0
    /// if not initialised
    virtual int getSensorNumber() const {
      return number_channels;
    }
    /// returns the mumber of motors the controller was initialised with or 0 if
    // not initialised
    virtual int getMotorNumber() const {
      return number_channels;
    }

    /// performs one step (includes learning).
    /// Calulates motor commands from sensor inputs.
    virtual void step(const sensor* x_, int number_sensors,
        motor* y_, int number_motors){
      stepNoLearning(x_, number_sensors, y_, number_motors);
    };

    /// performs one step without learning. Calulates motor commands from sensor
    /// inputs.
    virtual void stepNoLearning(const sensor* x_, int number_sensors,
        motor* y_, int number_motors){
      //Tripodgait for 18 DOF Hexapod

      assert(number_sensors >= 18);
      assert(number_motors >= 18);

      X_dot = Y;
      Y_dot = (mu*(1-X*X)*Y-X);

      X += TAU*X_dot;
      Y += TAU*Y_dot;

      int u=1;
      int u2=-1;

      // generate motor commands
      // right rear coxa (knee) forward-backward joint (back is positive)
      y_[TR2_m] = u2*X/6;
      y_[CR2_m] = X_dot*0.1;
      y_[FR2_m] = 0;
      //left rear coxa (knee) forward-backward joint
      y_[TL2_m] = u*-X/6;
      y_[CL2_m] = -X_dot*0.1;
      y_[FL2_m] = 0;
      //right middle coxa (knee) forward-backward joint
      y_[TR1_m] = u2*-X/6;
      y_[CR1_m] = -X_dot*0.1;
      y_[FR1_m] = 0;
      //left middle coxa (knee) forward-backward joint
      y_[TL1_m] = u*X/6;
      y_[CL1_m] = X_dot*0.1;
      y_[FL1_m] = 0;
      //right front coxa (knee) forward-backward joint
      y_[TR0_m] = u2*X/6;
      y_[CR0_m] = X_dot*0.1;
      y_[FR0_m] = 0;
      //left front coxa (knee) forward-backward joint
      y_[TL0_m] = u*-X/6;
      y_[CL0_m] = -X_dot*0.1;
      y_[FL0_m] = 0;
      // backbone joint
      y_[BJ_m] = 0;

      // update step counter
      t++;
    };

    /***** STOREABLE ****/
    /** stores the controller values to a given file. */
    virtual bool store(FILE* f) const {
      return true;
    };
    /** loads the controller values from a given file. */
    virtual bool restore(FILE* f){
      return true;
    };


  protected:
    unsigned short number_channels;

    int t;
    paramkey name;
    double outputH1;
    double outputH2;

};

#endif


/*
 * List of available Sensors and their numbers/enum's to be used for example as x_[TR0_as] in the step function
 *
// Angle sensors (for actoric-sensor board (new board))
       TR0_as=0, //Thoracic joint of right front leg
       TR1_as=1, //Thoracic joint of right middle leg
       TR2_as=2, //Thoracic joint of right hind leg

       TL0_as=3, //Thoracic joint of left front leg
       TL1_as=4, //Thoracic joint of left middle leg
       TL2_as=5, //Thoracic joint of left hind leg

       CR0_as=6, //Coxa joint of right front leg
       CR1_as=7, //Coxa joint of right middle leg
       CR2_as=8, //Coxa joint of right hind leg

       CL0_as=9,  //Coxa joint of left hind leg
       CL1_as=10, //Coxa joint of left hind leg
       CL2_as=11, //Coxa joint of left hind leg

       FR0_as=12, //Fibula joint of right front leg
       FR1_as=13, //Fibula joint of right middle leg
       FR2_as=14, //Fibula joint of right hind leg

       FL0_as=15, //Fibula joint of left front leg
       FL1_as=16, //Fibula joint of left middle leg
       FL2_as=17, //Fibula joint of left hind leg

       BJ_as= 18, //Backbone joint angle

       //Foot contact sensors (AMOSII v1 and v2)
       R0_fs= 19, //Right front foot
       R1_fs= 20, //Right middle foot
       R2_fs= 21, //Right hind foot
       L0_fs= 22, //Left front foot
       L1_fs= 23, //Left middle foot
       L2_fs= 24, //Left hind foot

       // US sensors (AMOSII v1 and v2)
       FR_us=25, //Front Ultrasonic sensor (right)
       FL_us=26, //Front Ultrasonic sensor (left)

       // IR reflex sensors at legs (AMOSIIv2)
       R0_irs=31,
       R1_irs=29,
       R2_irs=27,
       L0_irs=32,
       L1_irs=30,
       L2_irs=28,

       // goal orientation sensors (relative position to reference object 1, e.g. camera)
       G0x_s=62,
       G0y_s=63,
       G0z_s=64,

       //Body speed sensors (only simulation)
       BX_spd= 66,
       BY_spd= 67,
       BZ_spd= 68,

       // goal orientation sensors (relative position to reference object 2, e.g. camera)
       G1x_s=72,
       G1y_s=73,
       G1z_s=74,

       // goal orientation sensors (relative position to reference object 3, e.g. camera)
       G2x_s=78,
       G2y_s=79,
       G2z_s=80,
 */