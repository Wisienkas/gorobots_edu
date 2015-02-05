#ifndef __EMPTYCONTROLLER_H
#define __EMPTYCONTROLLER_H

#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>
#include <selforg/configurable.h>

#include <assert.h>
#include <cmath>
#include <stdlib.h>
#include <string.h>
#include <cmath>


#include <selforg/matrix.h>

#define LEG_TYPE_FRONT 0
#define LEG_TYPE_MIDDLE 1
#define LEG_TYPE_REAR 2

#define LEG_FL_SUPPORT 0.01
#define LEG_FH_SUPPORT 0.4
#define LEG_DESIRED -0.8

#define LEG_MAX_SPEED 0.008
#define LEG_MIN_SPEED 0.0001

#define LEG_FAST_DEP_GAIN_FRONT 0.5
#define LEG_SLOW_DEP_GAIN_FRONT 0.3
#define LEG_SLOW_ELE_GAIN_FRONT 0.07

#define LEG_FAST_DEP_GAIN_MIDDLE 0.6
#define LEG_SLOW_DEP_GAIN_MIDDLE 0.45
#define LEG_SLOW_ELE_GAIN_MIDDLE 0.004

#define LEG_FAST_DEP_GAIN_REAR 0.5
#define LEG_SLOW_DEP_GAIN_REAR 0.4
#define LEG_SLOW_ELE_GAIN_REAR 0.07

#define FRONT_SENSOR_DIST_THRES_UP 0.5
#define FRONT_SENSOR_DIST_THRES_DOWN 0.1

#define BACK_JOINT_ANGLE 1;

typedef struct EmptyControllerConf {
    double WeightH1_H1;
    double WeightH2_H2;
    double WeightH1_H2;
    double WeightH2_H1;
    double fact;
    double direction;
    double bias;




} EmptyControllerConf;


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

          outputUS1 = 0;
          outputUS2 = 0;

      conf.WeightH1_H1 = 1.5;
      conf.WeightH2_H2 = 1.5;
      conf.WeightH1_H2 = 0.4;
      conf.WeightH2_H1 = -0.4;
      conf.fact = 0.4; //0.7;
      conf.direction = -1;
      conf.bias = 0.0; //negative is legs up

          lastR0 = 0;
          lastR1 = 0;
          lastR2 = 0;
          lastL0 = 0;
          lastL1 = 0;
          lastL2 = 0;


          addInspectableValue("FootDep_R0", &foot_dep_R0,"FootDep_R0");
          addInspectableValue("FootDep_R1", &foot_dep_R1,"FootDep_R1");
          addInspectableValue("FootDep_R2", &foot_dep_R2,"FootDep_R2");
          addInspectableValue("FootDep_L0", &foot_dep_L0,"FootDep_L0");
          addInspectableValue("FootDep_L1", &foot_dep_L1,"FootDep_L1");
          addInspectableValue("FootDep_L2", &foot_dep_L2,"FootDep_L2");
          addInspectableValue("FootContactSum", &FootContactSum,"FootContactSum");
          addInspectableValue("US_preprocessed", &outputUS2,"US_preprocessed");


    };

	


    virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0){
      //Tripodgait for 18 DOF Hexapod
      assert(motornumber>=18);
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

    virtual double getLegDepress(double angle1, double angle2)
    {
        double y_first = sin(angle1)*3.5;
        double y_sec = sin(angle2-angle1)*6;
        double y_comb = -y_first-y_sec;
        //std::cout << "y_first:\t"<< y_first << "\ty_sec:\t" << y_sec << "\ty_comb:\t" << y_comb << "\n";

        return y_comb;
    }

    virtual double getSign(double x)
    {
        if (x >= 0)
            return 1;
        else
            return -1;
    }
    virtual double getLegOutputSpeedGain(double angle1, double angle2, double foot_force, int leg_type)
    {

        double fast_dep = 0;
        double slow_dep = 0;
        double slow_ele = 0;
        double weak_support_force = 0;
        double full_support_force = 0;
        double leg_speed = 0;

        if (leg_type == LEG_TYPE_FRONT)
        {
            fast_dep = LEG_FAST_DEP_GAIN_FRONT;
            slow_dep = LEG_SLOW_DEP_GAIN_FRONT;
            slow_ele = LEG_SLOW_ELE_GAIN_FRONT;

            weak_support_force = LEG_FL_SUPPORT;
            full_support_force = LEG_FH_SUPPORT;
        }
        else if (leg_type == LEG_TYPE_MIDDLE)
        {
            fast_dep = LEG_FAST_DEP_GAIN_MIDDLE;
            slow_dep = LEG_SLOW_DEP_GAIN_MIDDLE;
            slow_ele = LEG_SLOW_ELE_GAIN_MIDDLE;

            weak_support_force = LEG_FL_SUPPORT*2;
            full_support_force = LEG_FH_SUPPORT*2;
        }
        else if (leg_type == LEG_TYPE_REAR)
        {
            fast_dep = LEG_FAST_DEP_GAIN_REAR;
            slow_dep = LEG_SLOW_DEP_GAIN_REAR;
            slow_ele = LEG_SLOW_ELE_GAIN_REAR;

            weak_support_force = LEG_FL_SUPPORT;
            full_support_force = LEG_FH_SUPPORT;
        }
        else
        {
            fast_dep = LEG_FAST_DEP_GAIN_FRONT;
            slow_dep = LEG_SLOW_DEP_GAIN_FRONT;
            slow_ele = LEG_SLOW_ELE_GAIN_FRONT;

            weak_support_force = LEG_FL_SUPPORT;
            full_support_force = LEG_FH_SUPPORT;
        }


        double leg_depress = getLegDepress(angle1, angle2);

        //positive when leg is too far down
        double dep_error = LEG_DESIRED-leg_depress;

        if (foot_force <= weak_support_force)
            leg_speed = fast_dep*dep_error;
        else if ((dep_error > 0) && (foot_force < full_support_force))
            return -100;
        else if (dep_error < 0)
            leg_speed = slow_dep*dep_error;
        else
            leg_speed = slow_ele*dep_error;


        if (abs(leg_speed) > LEG_MAX_SPEED)
            leg_speed = LEG_MAX_SPEED*getSign(leg_speed);

        return angle1 + leg_speed + LEG_MIN_SPEED*getSign(leg_speed);

    }

    /// performs one step without learning. Calulates motor commands from sensor
    /// inputs.
    virtual void stepNoLearning(const sensor* x_, int number_sensors,
        motor* y_, int number_motors){
      //Tripodgait for 18 DOF Hexapod

      assert(number_sensors >= 18);
      assert(number_motors >= 18);

      double activityH1 = conf.WeightH1_H1 * outputH1 + conf.WeightH1_H2 * outputH2
          + 0.01;
      double activityH2 = conf.WeightH2_H2 * outputH2 + conf.WeightH2_H1 * outputH1
          + 0.01;

      outputH1 = tanh(activityH1);
      outputH2 = tanh(activityH2);



        outputUS1 = outputUS1*0.95 + x_[FR_us]*0.05;
        outputUS2 = outputUS2*0.9 + outputUS1*0.1;


        if (((y_[BJ_m] > 0.5) && (outputUS2 > FRONT_SENSOR_DIST_THRES_DOWN)) || outputUS2 > FRONT_SENSOR_DIST_THRES_UP) {
            y_[BJ_m] = BACK_JOINT_ANGLE;
        }
        else
        {
            y_[BJ_m] = 0;
        }


        foot_dep_R0 = getLegDepress(x_[CR0_as], x_[FR0_as]);
        foot_dep_R1 = getLegDepress(x_[CR1_as], x_[FR1_as]);
        foot_dep_R2 = getLegDepress(x_[CR2_as], x_[FR2_as]);
        foot_dep_L0 = getLegDepress(x_[CL0_as], x_[FL0_as]);
        foot_dep_L1 = getLegDepress(x_[CL1_as], x_[FL1_as]);
        foot_dep_L2 = getLegDepress(x_[CL2_as], x_[FL2_as]);

       double devR0 = x_[TR0_as] - lastR0;
        double devR1 = x_[TR1_as] - lastR1;
        double devR2 = x_[TR2_as] - lastR2;
        double devL0 = x_[TL0_as] - lastL0;
        double devL1 = x_[TL1_as] - lastL1;
        double devL2 = x_[TL2_as] - lastL2;

        double tempOut;
        lastR0 = x_[TR0_as];
        lastR1 = x_[TR1_as];
        lastR2 = x_[TR2_as];
        lastL0 = x_[TL0_as];
        lastL1 = x_[TL1_as];
        lastL2 = x_[TL2_as];
        // generate motor commands
      // right rear coxa (knee) forward-backward joint (back is positive)
      y_[TR2_m] = outputH2 * conf.fact + conf.bias;


        if(devR2 > 0)
            y_[CR2_m] = outputH1 * conf.fact * conf.direction;
        else {
            tempOut = getLegOutputSpeedGain(x_[CR2_as], x_[FR2_as], x_[R2_fs], LEG_TYPE_REAR);
            if (tempOut != -100)
                y_[CR2_m] = tempOut;
        }


      //left rear coxa (knee) forward-backward joint
      y_[TL2_m] = -outputH2 * conf.fact + conf.bias;

        if(devL2 > 0)
            y_[CL2_m] = -outputH1 * conf.fact * conf.direction;
        else{
            tempOut = getLegOutputSpeedGain(x_[CL2_as], x_[FL2_as], x_[L2_fs], LEG_TYPE_REAR);
            if (tempOut != -100)
                y_[CL2_m] = tempOut;
        }



      //right middle coxa (knee) forward-backward joint
      y_[TR1_m] = -outputH2 * conf.fact + conf.bias;

        if(devR1 > 0)
            y_[CR1_m] = -outputH1 * conf.fact * conf.direction;
        else {
            tempOut =getLegOutputSpeedGain(x_[CR1_as], x_[FR1_as], x_[R1_fs], LEG_TYPE_MIDDLE);
            if (tempOut != -100)
                y_[CR1_m] = tempOut;
        }

      //left middle coxa (knee) forward-backward joint
      y_[TL1_m] = outputH2 * conf.fact + conf.bias;

        if(devL1 > 0)
            y_[CL1_m] = outputH1 * conf.fact * conf.direction;
        else {
            tempOut =getLegOutputSpeedGain(x_[CL1_as], x_[FL1_as], x_[L1_fs], LEG_TYPE_MIDDLE);
            if (tempOut != -100)
                y_[CL1_m] = tempOut;
        }


      //right front coxa (knee) forward-backward joint
      y_[TR0_m] = outputH2 * conf.fact + conf.bias;

        if(devR0 > 0)
            y_[CR0_m] = outputH1 * conf.fact * conf.direction;
        else {
            tempOut =getLegOutputSpeedGain(x_[CR0_as], x_[FR0_as], x_[R0_fs], LEG_TYPE_FRONT);
            if (tempOut != -100)
                y_[CR0_m] = tempOut;
        }

      //left front coxa (knee) forward-backward joint
      y_[TL0_m] = -outputH2 * conf.fact + conf.bias;

        if(devL0 > 0)
            y_[CL0_m] = -outputH1 * conf.fact * conf.direction;
        else {
            tempOut = getLegOutputSpeedGain(x_[CL0_as], x_[FL0_as], x_[L0_fs], LEG_TYPE_FRONT);
            if (tempOut != -100)
                y_[CL0_m] = tempOut;
        }

        FootContactSum = x_[L0_fs] + x_[L1_fs]+ x_[L2_fs] + x_[R0_fs] + x_[R1_fs]+ x_[R2_fs];

      // backbone joint
        /*
        y_[FR2_m] = -y_[1];
        y_[FL2_m] = -y_[4];
        y_[FR1_m] = -y_[7];
        y_[FL1_m] = -y_[10];
        y_[FR0_m] = -y_[13];
        */

        y_[FL0_m] = -y_[16];

        double f_joint = 0;
        y_[FR0_m] = f_joint;
        y_[FR1_m] = f_joint;
        y_[FR2_m] = f_joint;

        y_[FL0_m] = f_joint;
        y_[FL1_m] = f_joint;
        y_[FL2_m] = f_joint;



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

    double outputUS1;
    double outputUS2;


    double lastR0;
    double lastR1;
    double lastR2;
    double lastL0;
    double lastL1;
    double lastL2;

    double foot_dep_R0;
    double foot_dep_R1;
    double foot_dep_R2;
    double foot_dep_L0;
    double foot_dep_L1;
    double foot_dep_L2;



    double FootContactSum;

    EmptyControllerConf conf;

  public:
    virtual paramval getParam(const paramkey& key) const {
      if (key == "WeightH1_H1")
        return conf.WeightH1_H1;
      else if (key == "WeightH2_H2")
        return conf.WeightH2_H2;
      else if (key == "WeightH1_H2")
        return conf.WeightH1_H2;
      else if (key == "WeightH2_H1")
        return conf.WeightH2_H1;
      else if (key == "fact")
        return conf.fact;
      else if (key == "direction")
        return conf.direction;
      else if (key == "bias")
        return conf.bias;
      else
        return AbstractController::getParam(key);
    }

    virtual bool setParam(const paramkey& key, paramval val) {
      if (key == "WeightH1_H1")
        conf.WeightH1_H1 = val;
      else if (key == "WeightH2_H2")
        conf.WeightH2_H2 = val;
      else if (key == "WeightH1_H2")
        conf.WeightH1_H2 = val;
      else if (key == "WeightH2_H1")
        conf.WeightH2_H1 = val;
      else if (key == "fact")
        conf.fact = val;
      else if (key == "direction")
        conf.direction = val;
      else if (key == "bias")
        conf.bias = val;
      else
        return false;
      return true;
    }

    virtual paramlist getParamList() const {
      paramlist list;
      list.push_back(
          std::pair<paramkey, paramval>("WeightH1_H1", conf.WeightH1_H1));
      list.push_back(
          std::pair<paramkey, paramval>("WeightH2_H2", conf.WeightH2_H2));
      list.push_back(
          std::pair<paramkey, paramval>("WeightH1_H2", conf.WeightH1_H2));
      list.push_back(
          std::pair<paramkey, paramval>("WeightH2_H1", conf.WeightH2_H1));
      list.push_back(
          std::pair<paramkey, paramval>("fact", conf.fact));
      list.push_back(
          std::pair<paramkey, paramval>("direction", conf.direction));
      list.push_back(
          std::pair<paramkey, paramval>("bias", conf.bias));
      return list;
    }
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

       //Body speed sensors (only simulation)
        BX_spd= 66,
        BY_spd= 67,
        BZ_spd= 68,

        // goal orientation sensors (relative angle to reference object 1, e.g. camera)
        G0angleroll_s=69,
        G0anglepitch_s=70,
        G0angleyaw_s=71,

        // goal orientation sensors (relative position to reference object 2, e.g. camera)
        G1x_s=72,
        G1y_s=73,
        G1z_s=74,

        // goal orientation sensors (relative angle to reference object 2, e.g. camera)
        G1angleroll_s=75,
        G1anglepitch_s=76,
        G1angleyaw_s=77,

        // goal orientation sensors (relative position to reference object 3, e.g. camera)
        G2x_s=78,
        G2y_s=79,
        G2z_s=80,

        // goal orientation sensors (relative angle to reference object 3, e.g. camera)
        G2angleroll_s=81,
        G2anglepitch_s=82,
        G2angleyaw_s=83,


*/
