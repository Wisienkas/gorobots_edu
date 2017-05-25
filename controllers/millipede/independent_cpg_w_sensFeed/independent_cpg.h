#ifndef __IndependentCPGSF_H
#define __IndependentCPGSF_H

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
class neuralCPG{
public:
    neuralCPG(double weights[4]){
        for(int i = 0; i <4; i++)
            w[i] = weights[i];
        S1 = 1;
        S2 = 1;
        B1 = 0.01;
        B2 = 0.01;
        C1 = 0;
        C2 = 0;
    }
    ~neuralCPG(){}

    std::vector<double> calculateOutputs(){
        a1 = C1*w[0]+C2*w[2]+B1;
        a2 = C2*w[1]+C1*w[3]+B2;
        //*/
        C1 = tanh(a1);
        C2 = tanh(a2);
        std::vector<double> output;
        output.push_back(C1);
        output.push_back(C2);
        return output;
    }

    void reset(){
        S1 = 1;
        S2 = 1;
        B1 = 0.01;
        B2 = 0.01;
        C1 = 0;
        C2 = 0;
        return;
    }

    double w[4];
    double B1, B2;
    double a1, a2;

    double C1, C2, S1, S2;

};


class IndependentCPGSF : public AbstractController {

  public:

    double posx, posy, posz;
    double velx, vely, velz;
    double accx, accy, accz;

    double amplitude;
    double sfeed;
    double S;
    double followed;
    int simN;
    bool saveData;
    bool active;
    double C1, C2;
    double S1, S2;
    double B1, B2;
    std::vector<double> w;
    MillipedeConf mconf;
    std::vector<localLegController> cpgs;
    std::vector<double> sensors, motors, odom;

    IndependentCPGSF()
    : AbstractController("IndependentCPGSF", "$Id: tripodgait18dof.cpp,v 0.1 $"){
      t = 0;

      active = false;
      saveData = true;
      outputH1 = 0.001;
      outputH2 = 0.001;

      C1 = 0;
      C2 = 0;

      S1 = 1;
      S2 = 1;
      B1 = 0.01;
      B2 = 0.01;

      amplitude =0.4;
      sfeed = 0.03;
      S=0.1;
      followed = 0.005;
      simN = 0;

      w = std::vector<double>(4);
      w[0] = 1.4;
      w[1] = 1.4;
      w[2] = 0.38+S;
      w[3] = -0.38-S;

      posx = 0;
      posy = 0;
      posz = 0;
      velx = 0;
      vely = 0;
      velz = 0;
      accx = 0;
      accy = 0;
      accz = 0;

      mconf = lpzrobots::Millipede::getDefaultConf(1.0 /*_scale*/, 4 /*_legspersegment*/, 5 /*_nofsegments*/, 1 /*_useShoulder*/,1 /*_useFoot*/,1 /*_useBack*/);


      // plot parameters using GUI "To display GUI, in terminal, type ./start -g 1 "
      addInspectableValue("outputH1", &outputH1,"outputH1");
      addInspectableValue("outputH2", &outputH2,"outputH2");

    };

    void initializeCPGs(double sfWeight_){ // To be called right after selecting a mconf
        cpgs = std::vector<localLegController>( mconf.legsPerSegment*mconf.nOfSegments, localLegController( w ));
        for(int i = 0; i < cpgs.size(); i++){
            cpgs[i].learning_SF = false;
            cpgs[i].SFweight = sfWeight_;

        }
        return;
    }


    virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0){
      //Tripodgait for 18 DOF Hexapod
//      assert(motornumber>=18);
    };

    virtual ~IndependentCPGSF(){};

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

      //assert(number_sensors >= 18);
//      assert(number_motors >= 18);

      //----Students--------Adding your Neural Controller here------------------------------------------//

//      //Outputs of CPG

        if(t>=100 && !active)
            //std::cout << "activate now!" << std::endl;
            active = true;

        if(active){
            //Calculate velocity, acceleration and position of the robot

            double prevVelX = velx;
            double prevVelY = vely;
            double prevVelZ = velz;

            //comes in m/s (I guess)
            velx = x_[mconf.nOfSegments*mconf.legsPerSegment+1];
            vely = x_[mconf.nOfSegments*mconf.legsPerSegment+2];
            velz = x_[mconf.nOfSegments*mconf.legsPerSegment+3];

            posx += velx/100;
            posy += vely/100;
            posz += velz/100;

            accx = (velx-prevVelX)*100;
            accy = (vely-prevVelY)*100;
            accz = (velz-prevVelZ)*100;


            sensors.clear();
            motors.clear();
            odom.clear();

            odom.push_back(posx);
            odom.push_back(posy);
            odom.push_back(posz);
            odom.push_back(velx);
            odom.push_back(vely);
            odom.push_back(velz);
            odom.push_back(accx);
            odom.push_back(accy);
            odom.push_back(accz);

            for(int i = 0; i < mconf.nOfSegments; i++){

                for(int j = 0; j < mconf.legsPerSegment; j++){
                    int cpg_index = i*4+j;
//                    if (j%2 != 0)
//                        S = 0.12;
//                    else
//                        S = 0.1;

                    cpgs[cpg_index].w[2] = 0.18+S;
                    cpgs[cpg_index].w[3] = -0.18-S;

//                    cpgs[i*4+j].B1 = 0.01-sfeed*(x_[touchSensorIdentity(mconf, i, j)]+1)/2*cos(cpgs[i*3+j].a1);
//                    cpgs[i*4+j].B2 = 0.01-sfeed*(x_[touchSensorIdentity(mconf, i, j)]+1)/2*sin(cpgs[i*3+j].a2);

                    double input1 = 0, input2 = 0;
                    if((i==0 && j>1) || i>0){
                        input1 = followed*cpgs[i*4+j-2].C1;
//                        input2 = followed*cpgs[i*4+j-2].C2;
                    }

                    std::vector<double> motorCommands = cpgs[cpg_index].control_step(x_[touchSensorIdentity(mconf, i, j)], input1, input2);

                    y_[motorIdentity(mconf, i, j, 0)] = motorCommands[0];
                    y_[motorIdentity(mconf, i, j, 1)] = motorCommands[1];
                    y_[motorIdentity(mconf, i, j, 2)] = motorCommands[2];
//                    std::cout << cpgs[cpg_index].SFweight << std::endl;


                    sensors.push_back(x_[touchSensorIdentity(mconf, i, j)]);
                    motors.push_back(y_[motorIdentity(mconf,i,j,1)]);


                }

            }
            if(saveData)
              saveAllData(sensors, motors, odom, t-100, simN);
        }else{
            for(int i = 0; i < mconf.nOfSegments; i++){
                for(int j = 0; j < mconf.legsPerSegment; j++){
                    y_[motorIdentity(mconf, i, j, 0)] = 0;
                    y_[motorIdentity(mconf, i, j, 1)] = 0;
                    y_[motorIdentity(mconf, i, j, 2)] = 0;
                }
            }
        }

//      //Memory: 2 neuron recurrent net
//      double activeL=-(2*x_[FL_us]-0.7);
//      double activeR=-(2*x_[FR_us]-0.7);
//      //std::cout << activeL << "\t" << activeR << std::endl;
//      if(activeL<-1)
//        activeL=-1;
//      if(activeL>1)
//        activeL=1;

//      if(activeR<-1)
//        activeR=-1;
//      if(activeR>1)
//        activeR=1;
//      //std::cout << activeL << "\t" << activeR << std::endl;

//      double sums1, sums2;
//      sums1 = S1*5.4-S2*3.55+activeL*7;
//      sums2 = S2*5.4-S1*3.55+activeR*7;

//      S1 = tanh(sums1);
//      S2 = tanh(sums2);
//      //Antenna
//      double turnR = 1;
//      double turnL = 1;
      /*/
//      if(S1>0.95)
//        turnR=-1;

//      if(S2>0.95)
//        turnL=-1;
//    // */

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

       // goal orientation sensors (relative position to reference object 1 (red), e.g. camera)
       G0x_s=62,
       G0y_s=63,
       G0z_s=64,

       //Body speed sensors (only simulation)
       BX_spd= 66,
       BY_spd= 67,
       BZ_spd= 68,

       // goal orientation sensors (relative position to reference object 2 (green), e.g. camera)
       G1x_s=72,
       G1y_s=73,
       G1z_s=74,

       // goal orientation sensors (relative position to reference object 3 (blue), e.g. camera)
       G2x_s=78,
       G2y_s=79,
       G2z_s=80,

 */
