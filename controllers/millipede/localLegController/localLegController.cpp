#include"localLegController.h"

localLegController::localLegController(std::vector<double> w_){
    w = w_;
    lowpass_n = 5;
    filtererror_n = 2;
    lowpass = std::vector<double>(lowpass_n);
    gait_error = std::vector<double>(8);
    input_synapses = std::vector<double>(8);
    interconnectionFilter = std::vector<double>(8);
    delays = std::vector<double>(8);
//    filtererror = std::vector<double>(filtererror_n);

    liftamplitude = 0.4;
    wideamplitude = 0.7;

}

localLegController::localLegController(std::vector<double> w_, std::vector<double> learnParams_){
    w = w_;
    Af = learnParams_[0];
    As = learnParams_[1];
    Bf = learnParams_[2];
    Bs = learnParams_[3];
    lowpass_n = 5;
    filtererror_n = 2;
    lowpass = std::vector<double>(lowpass_n);
    gait_error = std::vector<double>(8);
    input_synapses = std::vector<double>(8);
    interconnectionFilter = std::vector<double>(8);
    delays = std::vector<double>(8);
//    filtererror = std::vector<double>(filtererror_n);

    liftamplitude = 0.4;
    wideamplitude = 0.7;
}

void localLegController::CPG_step(){
    a1 = C1*w[0]+C2*w[2]+B1;
    a2 = C2*w[1]+C1*w[3]+B2;
    //*/
    C1 = tanh(a1);
    C2 = tanh(a2);
    return;
}

void localLegController::CPG_step(double input1, double input2){
    a1 = C1*w[0]+C2*w[2]+B1+input1;
    a2 = C2*w[1]+C1*w[3]+B2+input2;
    //*/
    C1 = tanh(a1);
    C2 = tanh(a2);
    return;
}

std::vector<double> localLegController::control_step(double touchSensorData){

    double filteredData = sFeedFilter(touchSensorData);

    prevC1 = C1;
    prevC2 = C2;

    CPG_step(-SFweight*(filteredData+1)/2*cos(a1), -SFweight*(filteredData+1)/2*sin(a2));

    psn1 = C1*0.9+psn1*0.1;
    double y = C2 > 0 ? sin(C2) : -1;

    std::vector<double> motorOutput(3);
    motorOutput[0] = wideamplitude*sin(psn1*1.5);
    motorOutput[1] = liftamplitude*y;
    motorOutput[2] = 0.5*liftamplitude*y;

    if(learning_SF)
        sFeedbackLearning_step(filteredData);

    return motorOutput;
}

std::vector<double> localLegController::control_step(double touchSensorData, double CPGinput1, double CPGinput2){

    double filteredData = sFeedFilter(touchSensorData);

    prevC1 = C1;
    prevC2 = C2;

    CPGinput1 += -SFweight*(filteredData+1)/2*cos(a1);
    CPGinput2 += -SFweight*(filteredData+1)/2*sin(a2);

    CPG_step(CPGinput1, CPGinput2);

    psn1 = C1*0.9+psn1*0.1;
    double y = C2 > 0 ? sin(C2) : -1;

    std::vector<double> motorOutput(3);
    motorOutput[0] = wideamplitude*sin(psn1*1.5);
    motorOutput[1] = liftamplitude*y;
    motorOutput[2] = 0.5*liftamplitude*y;

    if(learning_SF)
        sFeedbackLearning_step(filteredData);

//    if(legPos_fixation){
//        motorOutput[0] = 0;
//        motorOutput[1] = liftamplitude;
//        motorOutput[2] = 0.5*liftamplitude;
//    }

    return motorOutput;
}

void localLegController::sFeedbackLearning_step(double touchSensorData){

    if (touchSensorData>1)
        touchSensorData = 1;
    if(touchSensorData<0)
        touchSensorData = 0;

    //1 Fwd Model
    double G = sin(C2) < sin(prevC2) ? 0 : 1;

    fwdMod = 0.5*G + 0.5*fwdMod;

    //2 calculate error
    efferenceCopy_error = -(fwdMod-touchSensorData);
    //3 calculate dw
    dslow = dslow*As + efferenceCopy_error*Bs;
    dfast = dfast*Af + efferenceCopy_error*Bf;
    dW = (dslow + dfast);

    if(efferenceCopy_error <= 0.5)
        legPos_fixation = true;
    else
        legPos_fixation = false;


    //4 update weight
    if(local_SFweight)
        SFweight = dW;

    return;
}

void localLegController::reset(){
    B1 = 0.01;
    B2 = 0.01;
    C1 = 0;
    C2 = 0;
    return;
}


double localLegController::sFeedFilter(double dataInput){

    if(learning_SF)
        dataInput = dataInput * 1.5 - 0.5;

    double sumofvalues = 0;
    lowpass[0] = dataInput;
    std::rotate(lowpass.begin(), lowpass.begin()+1, lowpass.end());
    for(int k = 0; k < lowpass.size(); k++){
        sumofvalues+= lowpass[k-1];
    }
     //average
//    sumofvalues = sumofvalues / lowpass_n * 1.0;
    if(!learning_SF)
        sumofvalues/= lowpass_n;

    return sumofvalues;
}
