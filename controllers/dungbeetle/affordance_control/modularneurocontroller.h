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
#include <algorithm>
#include <selforg/matrix.h>
#include "utils/ann-framework/neuron.cpp"
#include "utils/ann-framework/synapse.cpp"
#include "utils/ann-framework/ann.cpp"
#include "utils/ann-library/vrn.cpp"
#include "controllers/dungbeetle/multiple_adaptive_cpg/ModularNeural.cpp"
<<<<<<< HEAD
#include "utils/sim_robots/dungbeetle/amosiisensormotordefinition_laser.h"
#include <ode_robots/rangeFinder.h>
#include <controllers/dungbeetle/adaptivecpg/shiftregister.h>
#include <controllers/dungbeetle/adaptivecpg/lowPassfilter.cpp>
=======
#include "utils/sim_robots/dungbeetle/dungbeetlesensormotordefinition_laser.h"
#include <ode_robots/rangeFinder.h>
#include <controllers/dungbeetle/hind_leg_control/adaptivecpg/shiftregister.h>
#include <controllers/dungbeetle/hind_leg_control/adaptivecpg/lowPassfilter.cpp>
>>>>>>> d7e6b105acaa0fa52730375fd20873511d4c69c5
#include <opencv2/opencv.hpp>
#include "affordance.cpp"

#include <iostream>
#include <fstream>
<<<<<<< HEAD
=======
#include <map>
>>>>>>> d7e6b105acaa0fa52730375fd20873511d4c69c5

//Add Delay line////////////////////
#include "utils/delayline.h"
////////////////////////////////////

<<<<<<< HEAD

=======
extern int objectIndex = 0;
>>>>>>> d7e6b105acaa0fa52730375fd20873511d4c69c5
extern int sim_count = 0;
extern double I_l = 0.0;
extern double I_r = 0.0;
extern double I3 = 0.0;
extern double I2 = 1.0;
extern bool sim_flag = false;
extern bool pushed = false;
<<<<<<< HEAD
extern double distan = 0.90;
=======
extern double distan = 0.30;
>>>>>>> d7e6b105acaa0fa52730375fd20873511d4c69c5
extern int counter=0;
extern bool cylinder_object = false;
extern bool sphere_object = false;
extern bool box_object = false;
extern bool detected = false;
extern bool transporting = false;
extern bool turning = false;
extern double mode = 0.0;
<<<<<<< HEAD
=======
extern bool trainingFlag = true;
>>>>>>> d7e6b105acaa0fa52730375fd20873511d4c69c5

void getCommand(char key);




class modularNeuroController : public AbstractController {

	private:
		//utility function to draw outputs of the neurons
		void updateGui();

		void lineSegmentation();

		void saveOnFile();

<<<<<<< HEAD
=======
		void sampleEnvironment();

		

>>>>>>> d7e6b105acaa0fa52730375fd20873511d4c69c5

	public:
	//class constructor
		modularNeuroController();

		modularNeuroController(int dungBeetletype,bool mCPGs,bool mMuscleModelisEnabled);
		void initialize(int dungBeetletype,bool mCPGs,bool mMuscleModelisEnabled);
<<<<<<< HEAD
=======
		void setFilePath();
>>>>>>> d7e6b105acaa0fa52730375fd20873511d4c69c5


		
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
    	virtual cv::Mat getImg(){
    		return color_dst;
    	}

    /***** STOREABLE ****/
    /** stores the controller values to a given file. */
    virtual bool store(FILE* f) const;
    /** loads the controller values from a given file. */
    virtual bool restore(FILE* f);



	private:
	affordanceController *aff;
	ModularNeural *cpg;
	bool mul_cpgs;
<<<<<<< HEAD
=======
	std::map<std::string, std::string> config;
>>>>>>> d7e6b105acaa0fa52730375fd20873511d4c69c5

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

	
	parameter omega0,omega1,omega2,omega3,omega4,omega5,omega;
    int dungBeetleType;//indicates whether the used robot AMOSv1 (amosType=1)  or AMOSv2(amosType=2).
    bool sensoryFeed;//indicates whether the used robot AMOSv1 (amosType=1)  or AMOSv2(amosType=2).
    bool muscleModel;//indicates whether the used robot AMOSv1 (amosType=1)  or AMOSv2(amosType=2).

	protected:
		
    	unsigned short numbersensors, numbermotors;
		paramkey name;
		int t;
<<<<<<< HEAD
=======
		
>>>>>>> d7e6b105acaa0fa52730375fd20873511d4c69c5

	public:
		
		double pattern1TC,pattern2TC,pattern1CT,pattern2CT,pattern3CT,pattern4CT,pattern1FT,pattern2FT;
		double o0,o1,o2;
		double perturbation;
		std::vector<sensor> x;
		std::vector<sensor> y;
		std::vector< std::vector<double> > y_MCPGs;
		double feedback0,feedback1,feedback2,feedback3,feedback4,feedback5;
		//low pass filtering
  		lowPass_filter *joint_R0,*joint_L0,*joint_R1,*joint_L1,*joint_R2,*joint_L2;

  		cv::Mat img,img2;
  		double angleStep = (-M_PI/180*90-M_PI/180*90) / (180 - 1);
  		double sensorAngle;
    	vector<cv::Point2d> beams,dx,sx,cen;
		cv::Vec4f lines;
    	cv::Mat color_dst;    	
    	vector<vector<cv::Point> > contours;
  		vector<cv::Vec4i> hierarchy;
<<<<<<< HEAD
=======
  		vector<double> points_shape;
>>>>>>> d7e6b105acaa0fa52730375fd20873511d4c69c5
  		enum State {stop,zero,one,two,three,four,five};
  		double d0,tc0,size,d1,ds;
  		
		double dl,dr;
  		double theta;
  		double training_error;
  		fstream file,shape;
  		bool save;
		State previous,current;
		int timesteps=0;
<<<<<<< HEAD
=======
		std::string filePath;
>>>>>>> d7e6b105acaa0fa52730375fd20873511d4c69c5
    /////////////////////////////////////////////

};

#endif
