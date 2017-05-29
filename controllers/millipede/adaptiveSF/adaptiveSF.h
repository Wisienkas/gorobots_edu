#ifndef __adaptiveSF_H
#define __adaptiveSF_H

#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>
#include <selforg/configurable.h>

#include <dataCollection.hpp>

#include <assert.h>
#include <cmath>
#include <stdlib.h>
#include <string.h>
#include <millipede.h>

#include <localLegController.h>

#include <selforg/matrix.h>

//#include <millipedesensormotordefinition.h>
#define ERROR std::cout << "passed over here.. \n";

using namespace lpzrobots;
/**
 * Empty robot controller.
 * The controller gets a number of input sensor values each timestep
 * and has to generate a number of output motor values.
 *
 * Go to the step() function and enter the control commands with respect to your task!
 *
 */


class adaptiveSF : public AbstractController {

  public:
    double gl_dW = 0, gl_dslow = 0, gl_dfast = 0, gl_Af = 0, gl_As = 0, gl_Bf = 0, gl_Bs = 0, gl_SFweight = 0, total_error = 0;
    double posx = 0, posy = 0, posz = 0;
    double velx = 0, vely = 0, velz = 0;
    double accx = 0, accy = 0, accz = 0;
    std::vector<double> w;

    double liftamplitude, wideamplitude;
//    double sfeed;
    double S;
//    double followed;
    int simN = 0;
    bool saveData;
    bool active = false;
    MillipedeConf mconf;
    std::vector<localLegController> cpgs;
    std::vector<double> sensors, motors, odom;
    double followed = 0;


    adaptiveSF(MillipedeConf mconf_)
    : AbstractController("adaptiveSF", "$Id: adaptiveSF.cpp,v 0.1 $"){
        initialize();

      mconf = mconf_;

      // plot parameters using GUI "To display GUI, in terminal, type ./start -g 1 "
      addInspectableValue("outputH1", &outputH1,"outputH1");
      addInspectableValue("outputH2", &outputH2,"outputH2");

    };

    void initialize();
    void initializeCPGs(); // To be called right after selecting a mconf


    virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0){
      //Tripodgait for 18 DOF Hexapod
//      assert(motornumber>=18);
    };

    virtual ~adaptiveSF(){};

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
        motor* y_, int number_motors);

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

