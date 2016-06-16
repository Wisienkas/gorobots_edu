#include "ico_controller.h"

/// contructor (hint: use $ID$ for revision)
IcoController::IcoController(const std::string& name, const std::string& revision)
: AbstractController(name, revision){

    //For students, Initialization -begin//
    parameter.resize(2);
    input_angle_s.resize(2);
    predictive_signals_IR_R.resize(4);
    predictive_signals_IR_L.resize(4);
    orientation.resize(3);
    fitnessIR.resize(8);

    exploration_lowpass_g = 0;

    ico_controller.push_back(new ICO(1, 0.2));
    ico_controller.push_back(new ICO(1, 0.1));
    ico_controller.push_back(new ICO(1, 0.1));

    stop = false;

    //For students, Initialization -end//


    //plot values on GUI, ./start -g 1
    addInspectableValue("parameter1", &parameter.at(0),"parameter1");
    addInspectableValue("parameter2", &parameter.at(1),"parameter2");
    addInspectableValue("Dis2Green", &distance,"distance");
    addInspectableValue("NormalizedDistance2G", &input_distance_s,"NormalizedDistance2G");
    addInspectableValue("PredictOrientationG", &input_angle_s.at(0),"PredictOrientationG");
    addInspectableValue("ReflexOritetationG", &xt_reflex_angle,"ReflexOritetationG");

    addInspectableValue("XB", &xb, "XB");
    addInspectableValue("YB", &yb, "YB");
    addInspectableValue("ZB", &zb, "ZB");

    addInspectableValue("oriX", &orientation.at(0), "oriX");
    addInspectableValue("oriY", &orientation.at(1), "oriY");
    addInspectableValue("oriZ", &orientation.at(2), "oriZ");

    addInspectableValue("PredictFrontRight", &predictive_signals_IR_R.at(0), "PredictFrontRight");
    addInspectableValue("PredictFrontLeft", &predictive_signals_IR_L.at(0), "PredictFrontLeft");
    addInspectableValue("PredictMFLeft", &predictive_signals_IR_L.at(1), "PredictMFLeft");
    addInspectableValue("PredictMHLeft", &predictive_signals_IR_L.at(2), "PredictMHLeft");
    addInspectableValue("PredictHindLeft", &predictive_signals_IR_L.at(3), "PredictHindLeft");
    addInspectableValue("PredictHindRight", &predictive_signals_IR_R.at(3), "PredictHindRight");
    addInspectableValue("PredictMFRight", &predictive_signals_IR_R.at(1), "PredictMFRight");
    addInspectableValue("PredictMHRight", &predictive_signals_IR_R.at(2), "PredictMHRight");
    addInspectableValue("AR", &AR, "AR");
    addInspectableValue("AL", &AL, "AL");
    addInspectableValue("ARs", &ARs, "ARs");
    addInspectableValue("ALs", &ALs, "ALs");

    addInspectableValue("u_ico_out", &u_ico_out,"u_ico_out");

    //Save data
    outFileicolearning.open("ICOlearningcurve.txt");

}

void IcoController::init(int sensornumber, int motornumber, RandGen* randGen = 0){
    number_sensors = sensornumber;
    number_motors = motornumber;
}

int IcoController::getSensorNumber() const {
    return number_sensors;
}

int IcoController::getMotorNumber() const {
    return number_motors;
}

void IcoController::step(const sensor* sensors, int sensornumber,
        motor* motors, int motornumber){
    assert(number_sensors == sensornumber);
    assert(number_motors == motornumber);

    manual_control = false;
    // press keyboard "u" = move forward
    // press keyboard "j" = move backward
    // press keyboard "b" = move turn left
    // press keyboard "k" = move turn right
    // press keyboard " " = stop

    /*****************************************************************************************/
    // motors 0-4
    // motor 0 = left front motor
    // motor 1 = right front motor
    // motor 2 = left hind motor
    // motor 3 = right hind motor

    // sensors 0-3: wheel velocity of the corresponding wheel
    // sensor 0 = wheel velocity left front
    // sensor 1 = wheel velocity right front
    // sensor 2 = wheel velocity left hind
    // sensor 3 = wheel velocity right hind

    // sensors 4-11: IR Sensors
    // sensor 4 = front right IR
    // sensor 5 = front left IR
    // sensor 6 = middle hind left IR
    // sensor 7 = middle front left IR
    // sensor 8 = hind left IR
    // sensor 9 = hind right IR
    // sensor 10 = middle hind right IR
    // sensor 11 = middle front right IR

    // sensors 12-23: distance two objects in local coordinates (x,y,z)
    // sensor 12 = x direction to the green object (goal detection sensor)
    // sensor 13 = y direction to the green object (goal detection sensor)
    // sensor 14 = z direction to the green object (goal detection sensor)

    // sensor 15 = x orientation
    // sensor 16 = y orientation
    // sensor 17 = z orientation

    // sensor 18 = front left fitness IR
    // sensor 19 = front right fitness IR
    // sensor 20 = middle front right IR
    // sensor 21 = middle hind right IR
    // sensor 22 = hind right IR
    // sensor 23 = hind left IR
    // sensor 24 = middle hind left IR
    // sensor 25 = middle front left IR

    /*****************************************************************************************/

    parameter.at(0) = sensors[4];
    parameter.at(1) = sensors[5];

    predictive_signals_IR_L.at(0) = sensors[4];
    predictive_signals_IR_L.at(1) = sensors[11];
    predictive_signals_IR_L.at(2) = sensors[10];
    predictive_signals_IR_L.at(3) = sensors[9];
    predictive_signals_IR_R.at(0) = sensors[5];
    predictive_signals_IR_R.at(1) = sensors[6];
    predictive_signals_IR_R.at(2) = sensors[7];
    predictive_signals_IR_R.at(3) = sensors[8];

    orientation.at(0) = sensors[15];//*180.0/M_PI;
    orientation.at(1) = sensors[16];//*180.0/M_PI;
    orientation.at(2) = sensors[17];//*180.0/M_PI;

    xb = sensors[12];
    yb = sensors[13];
    zb = sensors[14];

    for(int i=0; i<8; i++){
        fitnessIR.at(i) = sensors[18+i];
    }

    //-----------------long distance----Calculating relative orientation to objects-----------------------------------------------//
    //Goal 1 // green

    if (sign(sensors[14 /*x*/]/*X axis*/)>0)
    {
        alpha = atan(sensors[13/*y*/]/sensors[14 /*x*/]) * 180 / M_PI; // angle in degrees
    }
    else
    { // behind robot angles "continue", directly behind robot is 180 degrees, left is negative, right is positive
        alpha_tmp = -1*atan (sensors[13/*y*/]/sensors[14 /*x*/]) * 180 / M_PI; // angle in degrees
        if (alpha_tmp<=0)
        { // left
            alpha = (-90 + (-90-alpha_tmp));
        }
        else
        { // right
            alpha = ( 90 + ( 90-alpha_tmp));
        }
    }
    input_angle_s.at(0) =  alpha/180.*M_PI; // map sensor input to +-pi
    if(abs(sensors[14])<1){
        input_angle_s.at(1) = sensors[13];
    }
    else{
        input_angle_s.at(1) = atan2(sensors[13], sensors[14]);
    }
    predictive_signal_green = input_angle_s.at(1);

    //Green
    distance = (sqrt(pow(sensors[14],2)+pow(sensors[13/*11 y*/],2)+pow(sensors[12/*10 x*/],2)));
    input_distance_s = distance;

    //-----------------short distance----Calculating relative orientation to objects-----------------------------------------------//

    //Angle reflex signals
    double range_reflex = 5;
    //1) goal 1 GREEN
    if(input_distance_s < range_reflex && abs(input_angle_s.at(1))>M_PI/4)
    {
        xt_reflex_angle = input_angle_s.at(1);
    }
    else
    {
        xt_reflex_angle = 0.0;
    }
    reflexive_signal_green = xt_reflex_angle;

    //---------------------Calculating means of IR sensors-----------------------------------------------//
    AR = 0;
    AL = 0;
    ARs = 0;
    ALs = 0;
    for(unsigned int i=0; i<predictive_signals_IR_R.size(); i++){
      AR += predictive_signals_IR_R[i];
    }
    AR /= predictive_signals_IR_R.size();
    for(unsigned int i=0; i<predictive_signals_IR_L.size(); i++){
      AL += predictive_signals_IR_L[i];
    }
    AL /= predictive_signals_IR_L.size();

    if(AR>0.4){
        ARs = AR;
    }
    else{
        ARs = 0;
    }

    if(AL>0.4){
        ALs = AL;
    }
    else{
        ALs = 0;
    }

    //---------------------Calculating relative distance to objects-----------------------------------------------//


    //input_angle_s.at(0) = Goal 1 red  [-1(-2),...,1(2)]

    //DISTANCE TO GOALS
    //input_distance_s  = normalized distance from Goal 1 [0,...,1]

    //2) Exploration noise

    double lp_gain =  0.99;
    double scale_exploration = 2.0;
    exploration_g = gauss();
    exploration_lowpass_old_g = exploration_lowpass_g;
    exploration_lowpass_g = exploration_lowpass_old_g*lp_gain+(1-lp_gain)*exploration_g;
    exp_output= scale_exploration*exploration_lowpass_g;

    //----Students--------Adding your ICO learning here------------------------------------------//

    //---List of input that you can use for ICO learning task-----//

    //ICO module1
    ico_controller[0]->setReflexiveNeuronInput(reflexive_signal_green);
    ico_controller[0]->setPredictiveNeuronInput(0, predictive_signal_green);

    //ICO module2
    ico_controller[1]->setReflexiveNeuronInput(ARs);
    ico_controller[1]->setPredictiveNeuronInput(0, AR);

    //ICO module3
    ico_controller[2]->setReflexiveNeuronInput(ALs);
    ico_controller[2]->setPredictiveNeuronInput(0, AL);

    if(learn){
        for(int i=0; i<3; i++){
            ico_controller[i]->step();
        }
    }
    else{
        for(int i=0; i<3; i++){
            ico_controller[i]->stepNoLearning();
        }
    }


//    std::cout << "----------------" << std::endl;
//    for(int i=0; i<3; i++){
//      std::cout << ico_controller[i]->dumpWeights();
//    }
//    std::cout << "----------------" << std::endl;

    //OUTPUT
    // Output to steer the robot at the moment, the robot is controlled by noise (as exploration or searching for an object)
    //u_ico_out = ico_controller[0]->getOutputNeuronOutput()+ico_controller[1]->getOutputNeuronOutput()+ico_controller[2]->getOutputNeuronOutput()+exp_output;
    u_ico_out = exp_output;//exp_output;

    double threshold = AR - AL;
    if(threshold<0.1 && threshold>-0.1){
        u_ico_out += ico_controller[0]->getOutputNeuronOutput();
    }

    u_ico_out += ico_controller[1]->getOutputNeuronOutput();
    u_ico_out -= ico_controller[2]->getOutputNeuronOutput();

    outFileicolearning<<predictive_signal_green<<' '<<reflexive_signal_green<<' '<<u_ico_out<<endl;

    //----Students--------Adding your ICO learning here------------------------------------------//


    double scale = 0.5;
    motors[0] = scale*1+u_ico_out; // left front wheel
    motors[1] = scale*1-u_ico_out; // right front wheel
    motors[2] = scale*1+u_ico_out; // left rear wheel
    motors[3] = scale*1-u_ico_out; // right rear wheel

    //-------------------------------------------------------------------------------------------------------------

    // manual steering
    if(manual_control)
    {
        ////std::cout<<"u = forward"<<" \n"<<"j = back"<<"\n"<<"b = TL "<<"\n"<<"k = TR"<<"\n"<<"space = stop \n"<<std::endl;
        for (int i=0;i<4;i++)
            motors[i]=mc[i];

    }

    motor1 = motors[0];
    motor2 = motors[1];
    motor3 = motors[2];
    motor4 = motors[3];

    // Example open loop controller:

    //    // turn right in place
    //    motors[0]=  1;
    //    motors[1]= -1;
    //    motors[2]=  1;
    //    motors[3]= -1;

    //    // turn left in place
    //    motors[0]= -1;
    //    motors[1]=  1;
    //    motors[2]= -1;
    //    motors[3]=  1;

}

void IcoController::stepNoLearning(const sensor* , int number_sensors,
        motor* , int number_motors){

}

bool IcoController::store(FILE* f) const {
    Configurable::print(f,"");
    return true;
}

bool IcoController::restore(FILE* f) {
    Configurable::parse(f);
    return true;
}

void IcoController::setMC(double left, double right){
    mc[0]=left;
    mc[1]=right;
    mc[2]=left;
    mc[3]=right;
}


double IcoController::gauss()
{
    double  sum;
    int   i;

    for( sum = i = 0; i < 12; i++) sum += (1.0*(double)rand()/(RAND_MAX));
    return(sum - 6.0);
}

