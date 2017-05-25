#include "adaptiveSF.h"


void adaptiveSF::initialize(){
    t = 0;

    active = false;
    saveData = true;
    outputH1 = 0.001;
    outputH2 = 0.001;

    S=0.1;

    w = std::vector<double>(4);
    w[0] = 1.4;
    w[1] = 1.4;
    w[2] = 0.38 + S;
    w[3] = -0.38 - S;

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
    gl_Bf = 0.00;
    gl_Bs = 0.0005;
    gl_SFweight = 0;
    total_error = 0;

}

void adaptiveSF::initializeCPGs(){ // To be called right after selecting a mconf
    std::vector<double> learnParams;
    learnParams.push_back(gl_Af);
    learnParams.push_back(gl_As);
    learnParams.push_back(gl_Bf);
    learnParams.push_back(gl_Bs);

    cpgs = std::vector<localLegController>( mconf.legsPerSegment*mconf.nOfSegments, localLegController( w, learnParams));

}

void adaptiveSF::stepNoLearning(const sensor* x_, int number_sensors,
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

        total_error = 0;
        for(int i = 0; i < mconf.nOfSegments; i++){
            for(int j = 0; j < mconf.legsPerSegment; j++){
                int cpg_index = i*4+j;
                cpgs[cpg_index].w[2] = 0.18+S;
                cpgs[cpg_index].w[3] = -0.18-S;
                double sensor_data = x_[touchSensorIdentity(mconf, i, j)];

                double input1 = 0, input2 = 0;
                if((i==0 && j>1) || i>0){
                    input1 = followed*cpgs[i*4+j-2].C1;
//                        input2 = followed*cpgs[i*4+j-2].C2;
                }


                std::vector<double> motorCommands = cpgs[cpg_index].control_step(sensor_data, input1, input2);
                //filter touch sensor with low pass filter (now mean filter for last n values)
//                double sumofvalues = 0;
//                for(int k = cpgs[cpg_index].lowpass.size()-1; k >= 0; k--){
//                    sumofvalues+= cpgs[cpg_index].lowpass[k];
//                    cpgs[cpg_index].lowpass[k+1] = cpgs[cpg_index].lowpass[k];
//                }
//                cpgs[cpg_index].lowpass[0] = x_[touchSensorIdentity(mconf, i, j)];
//                sumofvalues+= x_[touchSensorIdentity(mconf, i, j)];
//                 //average
//                sumofvalues = sumofvalues / cpgs[cpg_index].lowpass_n * 1.0;


                y_[motorIdentity(mconf, i, j, 0)] = motorCommands[0];
                y_[motorIdentity(mconf, i, j, 1)] = motorCommands[1];
                y_[motorIdentity(mconf, i, j, 2)] = motorCommands[2];

//                    if(i == 0 && j == 2)
//                        std::cout << cpgs[cpg_index].prevC1 << "\t" << cpgs[cpg_index].C1 << std::endl;

//                if (sumofvalues>1)
//                    sumofvalues = 1;
//                if(sumofvalues<0)
//                    sumofvalues = 0;
//                //compute error from forward model
//                 //1 calculate fwd model output
//                double G = sin(cpgs[cpg_index].C2) < sin(cpgs[cpg_index].prevC2) ? 0 : 1;

//                cpgs[cpg_index].fwdMod = 0.5*G + 0.5*cpgs[cpg_index].fwdMod;

//                 //2 calculate error
//                double error = -sumofvalues + cpgs[cpg_index].fwdMod;

//                // filter error
////                    for(int k = cpgs[cpg_index].filtererror_n-1; k >= 0; k--){
////                        error+= cpgs[cpg_index].filtererror[k];
////                        cpgs[cpg_index].filtererror[k+1] = cpgs[cpg_index].filtererror[k];
////                    }
////                    cpgs[cpg_index].filtererror[0] = sumofvalues-cpgs[cpg_index].fwdMod;
////                    error+= sumofvalues-cpgs[cpg_index].fwdMod;
////                    error= error/ cpgs[cpg_index].filtererror_n ;

//                if(t>100)
//{
//                 //3 calculate dw
//                cpgs[cpg_index].dslow = cpgs[cpg_index].dslow*cpgs[cpg_index].As + error*cpgs[cpg_index].Bs;
//                cpgs[cpg_index].dfast = cpgs[cpg_index].dfast*cpgs[cpg_index].Af + error*cpgs[cpg_index].Bf;
//                cpgs[cpg_index].dW = (cpgs[cpg_index].dslow + cpgs[cpg_index].dfast);

//                 //4 update weight
//                cpgs[cpg_index].SFweight = cpgs[cpg_index].dW; // > 0 ? cpgs[cpg_index].dW : 0;
//}
//                //calculate error for averaging

                total_error += cpgs[cpg_index].efferenceCopy_error;
//                    cpgs[cpg_index].SFweight = gl_SFweight; //cpgs[cpg_index].dW;
//                    cpgs[cpg_index].SFweight = gl_SFweight;


//                if((i == 0 && j>=2) || i == 1){
                    sensors.push_back(cpgs[cpg_index].SFweight);
                    sensors.push_back(sensor_data);
                    motors.push_back(y_[motorIdentity(mconf,i,j,1)]);
                    motors.push_back(cpgs[cpg_index].fwdMod);

//                }

            }

        }
        //average all errors
        total_error = total_error/(mconf.nOfSegments*mconf.legsPerSegment);

//            std::cout << total_error << std::endl;
        //calculate dw
        gl_dslow = gl_dslow*gl_As + total_error*gl_Bs;
        gl_dfast = gl_dfast*gl_Af + total_error*gl_Bf;
        gl_dW = (gl_dslow + gl_dfast);
        gl_SFweight += gl_dW;



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
}
