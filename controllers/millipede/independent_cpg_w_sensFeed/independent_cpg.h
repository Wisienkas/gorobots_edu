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

            //Control step for all limbs
            for(int i = 0; i < mconf.nOfSegments; i++){

                for(int j = 0; j < mconf.legsPerSegment; j++){
                    int cpg_index = i*4+j;

                    //Update S (not used)
                    cpgs[cpg_index].w[2] = 0.18+S;
                    cpgs[cpg_index].w[3] = -0.18-S;


                    // Inputs to CPG
                    double input1 = 0, input2 = 0;
                    if((i==0 && j>1) || i>0){
                        input1 = followed*cpgs[i*4+j-2].C2-followed*cpgs[i*4+j-2].prevC2;
                        input2 = followed*cpgs[i*4+j-2].C1-followed*cpgs[i*4+j-2].prevC1;
//                        input1 = followed*cpgs[i*4+j-2].C1;
//                        input2 = followed*cpgs[i*4+j-2].C2;
                    }

                    //Control step for each leg
                    std::vector<double> motorCommands = cpgs[cpg_index].control_step(x_[touchSensorIdentity(mconf, i, j)], input1, input2);

                    y_[motorIdentity(mconf, i, j, 0)] = motorCommands[0];
                    y_[motorIdentity(mconf, i, j, 1)] = motorCommands[1];
                    y_[motorIdentity(mconf, i, j, 2)] = motorCommands[2];

                    //Data to be saved
                    sensors.push_back(cpgs[cpg_index].SFweight);
                    sensors.push_back(x_[touchSensorIdentity(mconf, i, j)]);
                    motors.push_back(cpgs[cpg_index].C1);
                    motors.push_back(cpgs[cpg_index].C2);

//                    sensors.push_back(x_[touchSensorIdentity(mconf, i, j)]);
//                    motors.push_back(y_[motorIdentity(mconf,i,j,1)]);


                }

            }
            if(saveData)
              saveAllData(sensors, motors, odom, t-100, simN);
        }else{
            //Waiting for robot initialisation (until robot stays steady on the ground after being created)
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
