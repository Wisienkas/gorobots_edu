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

typedef struct EmptyControllerConf {
    double WeightH1_H1;
    double WeightH2_H2;
    double WeightH1_H2;
    double WeightH2_H1;
    double fact;
    double direction;
    double bias;

    double TR;
    double TL;
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

      conf.WeightH1_H1 = 1.5;
      conf.WeightH2_H2 = 1.5;
      conf.WeightH1_H2 = 0.4;
      conf.WeightH2_H1 = -0.4;
      conf.fact = 0.3; //0.7;
      conf.direction = 1;
      conf.bias = 0.0; //negative is legs up

      addInspectableValue("outputH1", &outputH1,"outputH1");
      addInspectableValue("outputH2", &outputH2,"outputH2");
      addInspectableValue("conf.TR", &conf.TR,"conf.TR");
      addInspectableValue("conf.TL", &conf.TL,"conf.TL");
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

      // generate motor commands
      // right rear coxa (knee) forward-backward joint (back is positive)
//      y_[TR2_m] = outputH2 * conf.fact + conf.bias;
//      y_[CR2_m] = outputH1 * conf.fact * conf.direction;
//      y_[FR2_m] = -y_[1];
//      //left rear coxa (knee) forward-backward joint
//      y_[TL2_m] = -outputH2 * conf.fact + conf.bias;
//      y_[CL2_m] = -outputH1 * conf.fact * conf.direction;
//      y_[FL2_m] = -y_[4];
//      //right middle coxa (knee) forward-backward joint
//      y_[TR1_m] = -outputH2 * conf.fact + conf.bias;
//      y_[CR1_m] = -outputH1 * conf.fact * conf.direction;
//      y_[FR1_m] = -y_[7];
//      //left middle coxa (knee) forward-backward joint
//      y_[TL1_m] = outputH2 * conf.fact + conf.bias;
//      y_[CL1_m] = outputH1 * conf.fact * conf.direction;
//      y_[FL1_m] = -y_[10];
//      //right front coxa (knee) forward-backward joint
//      y_[TR0_m] = outputH2 * conf.fact + conf.bias;
//      y_[CR0_m] = outputH1 * conf.fact * conf.direction;
//      y_[FR0_m] = -y_[13];
//      //left front coxa (knee) forward-backward joint
//      y_[TL0_m] = -outputH2 * conf.fact + conf.bias;
//      y_[CL0_m] = -outputH1 * conf.fact * conf.direction;
//      y_[FL0_m] = -y_[16];
//      // backbone joint
//      y_[BJ_m] = 0;

      conf.TL = 1.0;

      if(x_[FL_us]>0.35)
    	  conf.TR = -1.0;
      else
    	  conf.TR = 1.0;


      if(x_[FR_us]>0.35)
    	  conf.TR = -1.0;
      else
    	  conf.TR = 1.0;

      y_[TR2_m] = outputH1 * conf.fact * conf.direction * conf.TL+ conf.bias;
      y_[CR2_m] = outputH2 * conf.fact*0.7 ;
      y_[FR2_m] = 0;//-y_[1];//outputH2*0.1;//
      //left rear coxa (knee) forward-backward joint
      y_[TL2_m] = -outputH1 * conf.fact * conf.direction * conf.TR+ conf.bias;
      y_[CL2_m] = -outputH2 * conf.fact*0.7 ;
      y_[FL2_m] = 0;//-y_[4];//-outputH2*0.1;//
      //right middle coxa (knee) forward-backward joint
      y_[TR1_m] = -outputH1 * conf.fact * conf.direction * conf.TL+ conf.bias;
      y_[CR1_m] = -outputH2 * conf.fact*0.7 ;
      y_[FR1_m] = 0;//-y_[7];//-outputH2*0.1;//
      //left middle coxa (knee) forward-backward joint
      y_[TL1_m] = outputH1 * conf.fact * conf.direction * conf.TR+ conf.bias;
      y_[CL1_m] = outputH2 * conf.fact*0.7 ;
      y_[FL1_m] = 0;//-y_[10];//outputH2*0.1;//
      //right front coxa (knee) forward-backward joint
      y_[TR0_m] = outputH1 * conf.fact * conf.direction * conf.TL + conf.bias;
      y_[CR0_m] = outputH2 * conf.fact*0.7 ;
      y_[FR0_m] = 0;//-y_[13];//outputH2*0.1;//
      //left front coxa (knee) forward-backward joint
      y_[TL0_m] = -outputH1 * conf.fact * conf.direction * conf.TR+ conf.bias;
      y_[CL0_m] = -outputH2 * conf.fact*0.7 ;
      y_[FL0_m] = 0;//-y_[16];//-outputH2*0.1;//
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
