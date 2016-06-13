#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

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
#include "utils/ann-framework/neuron.cpp"
#include "utils/ann-framework/synapse.cpp"
#include "utils/ann-framework/ann.cpp"
#include "utils/ann-library/vrn.cpp"
#include "ModularNeural.cpp"
#include <ode_robots/lildogsensormotordefinition.h>
#include <controllers/dogbot/adaptive_cpgs/lowPassfilter.cpp>




extern double I_l = 0.0;
extern double I_r = 0.0;
extern double I3 = 0.0;
extern double I2 = 0.0;


void getCommand(char key);



class modularNeuroController : public AbstractController {

  private:
    //utility function to draw outputs of the neurons
    void updateGui();


  public:
  //class constructor
    modularNeuroController();

    modularNeuroController(int lilDogtype,bool mCPGs,bool mMuscleModelisEnabled);
    void initialize(int lilDogtype,bool mCPGs,bool mMuscleModelisEnabled);
    
  //class destructor
    virtual ~modularNeuroController(){}

    virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

    double sigmoid(double num){
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
    //perform one step
    virtual void step(const sensor*, int number_sensors, motor*, int number_motors);

    /// performs one step without learning. Calulates motor commands from sensor inputs.
      virtual void stepNoLearning(const sensor*, int number_sensors, motor*, int number_motors) {
      // empty
      }

    /***** STOREABLE ****/
    /** stores the controller values to a given file. */
    virtual bool store(FILE* f) const;
    /** loads the controller values from a given file. */
    virtual bool restore(FILE* f);



  private:
  ModularNeural *cpg;
  bool mul_cpgs;

  double R0_H0;
  double R0_H1;
  double R0_H2;
  double R0_Per;
  //gui L0
  double L0_H0;
  double L0_H1;
  double L0_H2;
  double L0_Per;
  //gui L1
  double L1_H0;
  double L1_H1;
  double L1_H2;
  double L1_Per;
  //gui R1
  double R1_H0;
  double R1_H1;
  double R1_H2;
  double R1_Per;

  double timer;

  
  parameter omega0,omega1,omega2,omega3;
    //not relevant in the current lilDog
    int lilDogType;
    bool sensoryFeed;
    bool muscleModel;

  protected:
    
    unsigned short numbersensors, numbermotors;
    paramkey name;
    int t;

  public:
    
    std::vector<sensor> x;
    std::vector<sensor> y;
    std::vector< std::vector<double> > y_MCPGs;
    double feedback0,feedback1,feedback2,feedback3;
    //low pass filtering
      lowPass_filter *joint_R0,*joint_L0,*joint_R1,*joint_L1;
      
      

    /////////////////////////////////////////////

};

#endif
