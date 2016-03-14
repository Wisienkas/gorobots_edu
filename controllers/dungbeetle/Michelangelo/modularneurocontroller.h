#ifndef _MODULARNEUROCONTROLLER_H_
#define _MODULARNEUROCONTROLLER_H_

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
#include <ode_robots/dungbeetlesensormotordefinition.h>



extern double I_l = 0.0;
extern double I_r = 0.0;
extern double I3 = 0.0;
extern double I2 = 1.0;
extern bool multiple = false;

void getCommand(char key);



class modularNeuroController : public AbstractController {

	private:
		//utility function to draw outputs of the neurons
		void updateGui();


	public:
	//class constructor
		modularNeuroController();

		modularNeuroController(int dungBeetletype,bool mCPGs,bool mMuscleModelisEnabled);
		void initialize(int dungBeetletype,bool mCPGs,bool mMuscleModelisEnabled);
		
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
	ModularNeural * mod, *cpg_0,*cpg_1,*cpg_2,*cpg_3,*cpg_4,*cpg_5;

	//gui L0
	double input;
	double input1;
	double input2;
	double input3;
	//gui L1
	double L1_H0;
	double L1_H1;
	double L1_H2;
	double L1_Per;
	//gui L2
	double L2_H0;
	double L2_H1;
	double L2_H2;
	double L2_Per;
	//gui R1
	double R1_H0;
	double R1_H1;
	double R1_H2;
	double R1_Per;
	//gui R2
	double R2_H0;
	double R2_H1;
	double R2_H2;
	double R2_Per;
	double timer;

	
	parameter omega,omega1,omega2,omega3;
    int dungBeetleType;//indicates whether the used robot AMOSv1 (amosType=1)  or AMOSv2(amosType=2).
    bool sensoryFeed;//indicates whether the used robot AMOSv1 (amosType=1)  or AMOSv2(amosType=2).
    bool muscleModel;//indicates whether the used robot AMOSv1 (amosType=1)  or AMOSv2(amosType=2).

	protected:
		
    	unsigned short numbersensors, numbermotors;
		paramkey name;
		int t;

	public:
		
		double pattern1TC,pattern2TC,pattern1CT,pattern2CT,pattern3CT,pattern4CT,pattern1FT,pattern2FT;
		double o0,o1,o2;
		double perturbation;
		std::vector<sensor> x;
		std::vector<sensor> y;
		std::vector< std::vector<double> > y_MCPGs;
		std::vector<double> derivativeH0;
		std::vector<double> derivativeH1;
};

#endif
