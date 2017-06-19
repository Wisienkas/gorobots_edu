#ifndef __NavigationCPGSF_H
#define __NavigationCPGSF_H

#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>
#include <selforg/configurable.h>

#include <dataCollection.hpp>

#include <assert.h>
#include <cmath>
#include <stdlib.h>
#include <string.h>
#include <millipede.h>

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
        double sumc1, sumc2;
        sumc1 = C1*w[0]+C2*w[2]+B1;
        sumc2 = C2*w[1]+C1*w[3]+B2;
        //*/
        C1 = tanh(sumc1);
        C2 = tanh(sumc2);
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

    double C1, C2, S1, S2;

};


class NavigationCPGSF : public AbstractController {

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
    double w[4];
    MillipedeConf mconf;
    std::vector<neuralCPG> cpgs;

    NavigationCPGSF()
    : AbstractController("NavigationCPGSF", "$Id: tripodgait18dof.cpp,v 0.1 $"){
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
      w[0] = 1.4;
      w[1] = 1.4;
      w[2] = 0.38;
      w[3] = -0.38;

      amplitude =0.4;
      sfeed = 0.03;
      S=0.1;
      followed = 0.005;
      simN = 0;

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

    void initializeCPGs(){ // To be called right after selecting a mconf
        for(int i = 0; i < 4*mconf.nOfSegments; i++){
            neuralCPG cpg(w);
            cpgs.push_back(cpg);
        }
    }


    virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0){
      //Tripodgait for 18 DOF Hexapod
//      assert(motornumber>=18);
    };

    virtual ~NavigationCPGSF(){};

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


            std::vector<double> sensors, motors, odom;

            odom.push_back(posx);
            odom.push_back(posy);
            odom.push_back(posz);
            odom.push_back(velx);
            odom.push_back(vely);
            odom.push_back(velz);
            odom.push_back(accx);
            odom.push_back(accy);
            odom.push_back(accz);

            bool left = false;
            bool right = true;
            for(int i = 0; i < mconf.nOfSegments; i++){

                if(left){
                    y_[39] = 1;
                    y_[38] = 0;
                    y_[37] = 1;
                    y_[36] = 0;
                }else if(right){
                    y_[39] = -1;
                    y_[38] = -0;
                    y_[37] = -1;
                    y_[36] = -0;
                }else{
                    y_[39] = 0;
                    y_[38] = 0;
                    y_[37] = 0;
                    y_[36] = 0;
                }
                for(int j = 0; j < mconf.legsPerSegment; j++){
                    if(right)
                        if (j%2 != 0){
                            S = 0.12;
                            amplitude = 0.45;
                        }
                        else{
                            S = 0.1;
                            amplitude = 0.35;
                        }
                    else if(left)
                        if (j%2 != 0){
                            S = 0.1;
                            amplitude = 0.35;
                        }
                        else{
                            S = 0.12;
                            amplitude = 0.45;
                        }

                    cpgs[i*4+j].w[2] = 0.18+S;
                    cpgs[i*4+j].w[3] = -0.18-S;

                    cpgs[i*4+j].B1 = 0.01-sfeed*(x_[touchSensorIdentity(mconf, i, j)]+1)/2*cos(cpgs[i*3+j].C1);
                    cpgs[i*4+j].B2 = 0.01-sfeed*(x_[touchSensorIdentity(mconf, i, j)]+1)/2*sin(cpgs[i*3+j].C2);

                    if((i==0 && j>1) || i>0){
                        cpgs[i*4+j].B1 += followed*cpgs[i*4+j-2].C1;
//                        cpgs[i*4+j].B2 -= followed*cpgs[i*4+j-2].C2;
                    }

                    std::vector<double> out;
                    out = cpgs[i*4+j].calculateOutputs();

                    y_[motorIdentity(mconf, i, j, 0)] = amplitude*out[0];
                    y_[motorIdentity(mconf, i, j, 1)] = amplitude*out[1];
                    y_[motorIdentity(mconf, i, j, 2)] = 0;

                    if((i == 0 && j>=2) || i == 1){
                      sensors.push_back(x_[touchSensorIdentity(mconf, i, j)]);
                      motors.push_back(y_[motorIdentity(mconf,i,j,0)]);
                    }

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

