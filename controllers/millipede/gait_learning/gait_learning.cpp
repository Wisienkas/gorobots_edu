#include "gait_learning.h"

/// Unfinished controller, without results!!!

using namespace std;

void gait_learning::initialize(){
    t = 0;

    active = false;
    saveData = true;
    outputH1 = 0.001;
    outputH2 = 0.001;

    S=0.1;

    w = std::vector<double>(4,0);
    w[0] = 1.4;
    w[1] = 1.4;
    w[2] = 0.38+S;
    w[3] = -0.38-S;

    liftamplitude = 0.4;
    wideamplitude = 0.7;

    //central sfw
//      gl_Af = 0.0125;
//      gl_As = 0.9;
//      gl_Bf = 0.0051;
//      gl_Bs = 0.0001;
    //distributed sfw
    gl_Af = 0.78;
    gl_As = 0.9972;
    gl_Bf = 0.001;
    gl_Bs = 0.0005;
    gl_SFweight = 0;
    total_error = 0;
    lastFwdMod = 0;


}

void gait_learning::initializeCPGs(){ // To be called right after selecting a mconf
    std::vector<double> learnParams;
    learnParams.push_back(gl_Af);
    learnParams.push_back(gl_As);
    learnParams.push_back(gl_Bf);
    learnParams.push_back(gl_Bs);
    nOfLegs = mconf.legsPerSegment*mconf.nOfSegments;

    CPGphases = std::vector<double>(nOfLegs*nOfLegs);
    phaseErrors = std::vector<double>(nOfLegs*nOfLegs);
    phaseErrors2 = std::vector<double>(nOfLegs*nOfLegs);
    input_synapses = std::vector<double>(nOfLegs*nOfLegs);
    phaseIntegralErrors = std::vector<double>(nOfLegs*nOfLegs);

    cpgs = std::vector<localLegController>( nOfLegs, localLegController( w, learnParams));

}

 void gait_learning::stepNoLearning(const sensor* x_, int number_sensors,
    motor* y_, int number_motors){
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


        double numberOfLegs = mconf.nOfSegments * mconf.legsPerSegment;

        total_error = 0;
        for(int i = 0; i < mconf.nOfSegments; i++){
            for(int j = 0; j < mconf.legsPerSegment; j++){
                int cpg_index = i*4+j;
                cpgs[cpg_index].w[2] = 0.18+S;
                cpgs[cpg_index].w[3] = -0.18-S;
                double sensor_data = x_[touchSensorIdentity(mconf, i, j)];


                double input1 = 0, input2 = 0;
                // Adding inter-limb coordination
                for(int k = 0; k < interNeuralConnections; k++){
                    //add to B1 the sum of all inputs from surrounding cpgs
                    int input_index;
                    if(cpg_index%2 == 0)
                        input_index = cpg_index-(interNeuralConnections/2-1)+k;
                    else
                        input_index = cpg_index-interNeuralConnections/2+k;

                    if(input_index >= 0 && input_index < numberOfLegs && input_index!=cpg_index){
                        double CPGphase_index = nOfLegs*input_index + cpg_index;
                        double phase = CPGphases[CPGphase_index];
                        double toCos1 = cpgs[cpg_index].C1-cpgs[input_index].C1 - phase;
                        double toCos2 = cpgs[cpg_index].C2-cpgs[input_index].C2 - phase;
                        double C = 0.100;
                        double signal1 = C*(1-cos(toCos1)) + sin(toCos1);
                        double signal2 = C*(1-cos(toCos2)) + sin(toCos2);

                        cpgs[cpg_index].interconnectionFilter[k] =cpgs[cpg_index].interconnectionFilter[k]*0.992 + signal1;//tanh(0.1 + signal1 + 1*cpgs[cpg_index].interconnectionFilter[k]);
//                        input1 += cpgs[cpg_index].input_synapses[k]*signal1;//cpgs[cpg_index].interconnectionFilter[k];
//                        input2 += cpgs[cpg_index].input_synapses[k]*signal2;//cpgs[cpg_index].interconnectionFilter[k];


                        double dInput1 = cpgs[input_index].C1 - cpgs[input_index].prevC1;
                        double dInput2 = cpgs[input_index].C2 - cpgs[input_index].prevC2;
                        input1 += cpgs[cpg_index].input_synapses[k]*dInput1 + cpgs[cpg_index].input_synapses[k+6]*dInput2;

//                        input1 += input_synapses[CPGphase_index]*signal1;
//                        input2 += input_synapses[CPGphase_index]*signal2;
                    }
                }


                std::vector<double> motorCommands = cpgs[cpg_index].control_step(sensor_data, input1, input2);
                y_[motorIdentity(mconf, i, j, 0)] = motorCommands[0];
                y_[motorIdentity(mconf, i, j, 1)] = motorCommands[1];
                y_[motorIdentity(mconf, i, j, 2)] = motorCommands[2];


                // gait learner:
                // difference between output from delayer and ouput of cpg
                double du0 = cpgs[cpg_index].C1-cpgs[cpg_index].prevC1; //fwdMod-lastFwdMod;

                double learning_rate = 0.00051;
                for(int k = 0; k < interNeuralConnections; k++){
                    int input_index;
                    if(cpg_index%2 == 0)
                        input_index = cpg_index-(interNeuralConnections/2-1)+k;
                    else
                        input_index = cpg_index-interNeuralConnections/2+k;

                    if(input_index >= 0 && input_index < numberOfLegs && input_index!=cpg_index){
                        double dInput1 = cpgs[input_index].C1 - cpgs[input_index].prevC1;
                        double dInput2 = cpgs[input_index].C2 - cpgs[input_index].prevC2;
                        //calculate error for prevC1
                        double prevC1err = cpgs[cpg_index].C1*dInput1;
                        double prevC2err = cpgs[cpg_index].C2*dInput2;
                        //backpropagate error with weights.
                        cpgs[cpg_index].input_synapses[k] = cpgs[cpg_index].input_synapses[k]*1 + learning_rate*prevC1err;
                        cpgs[cpg_index].input_synapses[k+6] = cpgs[cpg_index].input_synapses[k]*1 + learning_rate*prevC2err;



                        // filter error
                        phaseIntegralErrors[nOfLegs*input_index+cpg_index] = phaseIntegralErrors[nOfLegs*input_index+cpg_index] *0.99+ cpgs[cpg_index].interconnectionFilter[k];
                        phaseErrors[nOfLegs*input_index + cpg_index] = (du0*cpgs[cpg_index].interconnectionFilter[k]);//(cpgs[cpg_index].C1 - cpgs[cpg_index].interconnectionFilter[k]);
                        cpgs[cpg_index].gait_error[k] = (cpgs[cpg_index].C1 - cpgs[cpg_index].interconnectionFilter[k]); //cpgs[cpg_index].gait_error[k] *0.992 + (cpgs[cpg_index].C1 - cpgs[cpg_index].interconnectionFilter[k]);
                        phaseErrors2[nOfLegs*input_index + cpg_index] = (cpgs[cpg_index].C1 - cpgs[cpg_index].interconnectionFilter[k]);
                        total_error+=(cpgs[cpg_index].C1 - cpgs[cpg_index].interconnectionFilter[k]);
//                        if(t>800){
//                            cpgs[cpg_index].delays[k] = cpgs[cpg_index].delays[k]*0.992 + cpgs[cpg_index].gait_error[k]*learning_rate;//(du0*cpgs[cpg_index].interconnectionFilter[k])*learning_rate;

//                        if(t>2000)
//                           cpgs[cpg_index].input_synapses[k] =-0.01;//cpgs[cpg_index].input_synapses[k]*0.9995 - 0.0001/(1+cpgs[cpg_index].gait_error[k]*cpgs[cpg_index].gait_error[k]);
//                        }


                    }
                }

                if((i == 1 && j==2)){
                    for(int k = 0; k < 8; k++){
                        int input_index;
                        if(cpg_index%2 == 0)
                            input_index = cpg_index-(interNeuralConnections/2-1)+k;
                        else
                            input_index = cpg_index-interNeuralConnections/2+k;

                        sensors.push_back(cpgs[cpg_index].input_synapses[k]);//CPGphases[nOfLegs*input_index+cpg_index]);
//                        sensors.push_back(input_synapses[nOfLegs*input_index+cpg_index]);

//                        sensors.push_back(cpgs[cpg_index].input_synapses[k]);
                        sensors.push_back(cpgs[cpg_index].interconnectionFilter[k]);
                    }

                    motors.push_back(cpgs[cpg_index].SFweight);
                    motors.push_back(cpgs[cpg_index].C1);
                    motors.push_back(cpgs[cpg_index].C2);
                }

            }

        }


        //Inter limb coordination, dephase learning

        if(t>1000  ){
            double learning_rate = 0.01;
            for(int j = 0; j < nOfLegs; j++){
//                for(int i = 0; i<=j; i++)
//                    cout << "t \t \t";
                for(int i = j+1; i <nOfLegs; i++){
                    //learning_rate = 0.8+input_synapses[j*nOfLegs+i]/5*10;
                    double deltaPhase = sqrt(phaseErrors[nOfLegs*i+j]*phaseErrors[nOfLegs*i+j]+phaseErrors[nOfLegs*j+i]*phaseErrors[nOfLegs*j+i])/2;
                    double prev_phase = CPGphases[nOfLegs*i+j];
                    if (phaseIntegralErrors[nOfLegs*i+j]<0)
                        CPGphases[nOfLegs*i+j] -= deltaPhase*learning_rate;
                    else
                        CPGphases[nOfLegs*i+j] += deltaPhase*learning_rate;


//                    if(prev_phase>=0 && CPGphases[nOfLegs*i+j]>M_PI)
//                        CPGphases[nOfLegs*i+j] -= 2*M_PI;

//                    else if(prev_phase<0 && CPGphases[nOfLegs*i+j]<-M_PI)
//                        CPGphases[nOfLegs*i+j] += 2*M_PI;

                    CPGphases[nOfLegs*j+i] = -CPGphases[nOfLegs*i+j];
//                    double deltaPhase = (phaseErrors[nOfLegs*i+j]*phaseErrors[nOfLegs*i+j]+phaseErrors[nOfLegs*j+i]*phaseErrors[nOfLegs*j+i])/2;
//                    CPGphases[nOfLegs*i+j] = CPGphases[nOfLegs*i+j]*0.992+deltaPhase*learning_rate;
//                    CPGphases[nOfLegs*j+i] = CPGphases[nOfLegs*i+j]*0.992-deltaPhase*learning_rate;
//                    double toav1 = CPGphases[nOfLegs*i+j] + phaseErrors[nOfLegs*i+j]*learning_rate;
//                    double toav2 = CPGphases[nOfLegs*j+i] - phaseErrors[nOfLegs*j+i]*learning_rate;
//                    CPGphases[nOfLegs*i+j] = sqrt(toav1*toav1+toav2*toav2)/2;
//                    CPGphases[nOfLegs*j+i] = -CPGphases[nOfLegs*i+j];

//                    cout << CPGphases[nOfLegs*i+j] << "\t \t";
                }
//                cout << endl;

            }
        }

        //set inter limb coordination strength
        if(t<3000){
            double B_ilStr = 0.001;
            for(int j = 0; j < nOfLegs; j++){
                for(int i = 0; i < nOfLegs; i++){
                    input_synapses[j*nOfLegs+i] = -0.01;// input_synapses[j*nOfLegs+i]*0.0 - B_ilStr*(0.01/(0.01+phaseIntegralErrors[nOfLegs*i+j]*phaseIntegralErrors[nOfLegs*i+j]));//phaseErrors2[nOfLegs*j+i]*phaseErrors2[nOfLegs*j+i]))*B_ilStr;
                }
            }
        }


        //average all errors
        total_error = total_error/(nOfLegs);

//            std::cout << total_error << std::endl;
        //calculate dw
        gl_dslow = gl_dslow*gl_As + total_error*gl_Bs;
        gl_dfast = gl_dfast*gl_Af + total_error*gl_Bf;
        gl_dW = (gl_dslow + gl_dfast);
//        cout << gl_dW << endl;
        gl_SFweight -= gl_dW;
//            if(t>2000)
//                gl_SFweight = 0.10;

        if(saveData){
            saveAllData(sensors, motors, odom, t-100, simN);
        }

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
}



double gait_learning::access_CPGphases(int i, int j){
    if (i >= nOfLegs || j>= nOfLegs)
            throw 0;
    return CPGphases[nOfLegs*j + i];
}

