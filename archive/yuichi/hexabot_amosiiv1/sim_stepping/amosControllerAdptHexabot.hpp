#ifndef AMOSCONTROLLER_ADPT_HEXABOT_HPP 100
#define AMOSCONTROLLER_ADPT_HEXABOT_HPP

/*
  * This is the class which is the adapter of the HexabotController
  *
  *

20140520 Ver.1
    Start developing!!

20140527 Ver.1.01
    This is for amos pid controlled

20140605 Ver.1.02
    This is for amos v1 and v2
    we add the inverse kinematics to changecoordinates
*/

#include "hexabotController.h"
#include "ode_robots/amosII.h"

using namespace HEXABOT;
using namespace lpzrobots;
//using namespace amosII;

// Amos Controller
class AmosController_adptHexabot:public HexabotController {
protected:
    // amosII conf
    AmosIIConf &amConf;
    HexabotConf hxConf;

    // write all control command for Log
    std::ofstream ctrOfs;
    // write all sensed command for log
    std::ofstream snsOfs;

    // time (start time is 0)
    double amTime;
    unsigned long amCnt;

    // inverse kinematics-----------
    //  This is for amos version.
    //  We want to change the coordinates because the joint angle of the shoulder of amos2 can be changed
    //  In the hexabotController::stepNoLearning, this func is called in place of HexabotController::calcInvKinematics because this func is overloaded.
    //
    // get the angle data from leg toe position in Leg coordinate
    // toePos : in leg coordinate (m) , angle data : (rad)
    // the definition is written in another paper
    virtual bool clacInvKinematics(osg::Vec3d toePos, double& tAng, double& cAng, double& fAng){
        if(amConf.amos_version == 2){
            return HexabotController::clacInvKinematics(toePos, tAng, cAng, fAng);
        }else if(amConf.amos_version == 1){
            double angle = - amConf.mLegTrunkAngleH;

            //transfer coordinates
            osg::Vec3d toePosN;
            double x = toePos.x() * cos(angle) + toePos.z() * sin(angle);
            double y = toePos.y();
            double z = - toePos.x() * sin(angle) + toePos.z() * cos(angle);
            toePosN.set(x,y,z);
            return HexabotController::clacInvKinematics(toePosN, tAng, cAng, fAng);

        }
        else return false;
    }

private:
    // convert functions
     // Fore left T Joint
    sensor TfjointL_from_amos_to_hex(sensor amosVal){
        // I do not know so much but it seems that min is up(forward), max is down(backward)
        double angle = -amosVal * ( amConf.fcoxaJointLimitB - amConf.fcoxaJointLimitF ) /2. + (amConf.fcoxaJointLimitB + amConf.fcoxaJointLimitF)/2.;
        sensor hexVal = -2. * (angle / M_PI);
        return hexVal;
    }
     // Middle left T joint
    sensor TmjointL_from_amos_to_hex(sensor amosVal){
        double angle = -amosVal * ( amConf.mcoxaJointLimitB - amConf.mcoxaJointLimitF ) /2. + (amConf.mcoxaJointLimitB + amConf.mcoxaJointLimitF)/2.;
        sensor hexVal = -2. * (angle / M_PI);
        return hexVal;
    }
     // Rear left T joint
    sensor TrjointL_from_amos_to_hex(sensor amosVal){
        double angle = -amosVal * ( amConf.rcoxaJointLimitB - amConf.rcoxaJointLimitF ) /2. + (amConf.rcoxaJointLimitB + amConf.rcoxaJointLimitF)/2.;
        sensor hexVal = -2. * (angle / M_PI);
        return hexVal;
    }
    // Fore r T Joint
   sensor TfjointR_from_amos_to_hex(sensor amosVal){
       // I do not know so much but it seems that min is up(forward), max is down(backward)
       double angle = amosVal * ( amConf.fcoxaJointLimitB - amConf.fcoxaJointLimitF ) /2. + (amConf.fcoxaJointLimitB + amConf.fcoxaJointLimitF)/2.;
       sensor hexVal = -2. * (angle / M_PI);
       return hexVal;
   }
    // Middle r T joint
   sensor TmjointR_from_amos_to_hex(sensor amosVal){
       double angle = amosVal * ( amConf.mcoxaJointLimitB - amConf.mcoxaJointLimitF ) /2. + (amConf.mcoxaJointLimitB + amConf.mcoxaJointLimitF)/2.;
       sensor hexVal = -2. * (angle / M_PI);
       return hexVal;
   }
    // Rear r T joint
   sensor TrjointR_from_amos_to_hex(sensor amosVal){
       double angle = amosVal * ( amConf.rcoxaJointLimitB - amConf.rcoxaJointLimitF ) /2. + (amConf.rcoxaJointLimitB + amConf.rcoxaJointLimitF)/2.;
       sensor hexVal = -2. * (angle / M_PI);
       return hexVal;
   }

     // C joint
    sensor Cjoint_from_amos_to_hex(sensor amosVal){
        double angle = -amosVal * ( amConf.secondJointLimitD - amConf.secondJointLimitU ) /2. + (amConf.secondJointLimitD + amConf.secondJointLimitU)/2.;
        sensor hexVal = -2. * (angle / M_PI);
        return hexVal;
    }
     // F joint
    sensor Fjoint_from_amos_to_hex(sensor amosVal){
        double angle = -amosVal * ( amConf.tebiaJointLimitD - amConf.tebiaJointLimitU ) /2. + (amConf.tebiaJointLimitD + amConf.tebiaJointLimitU)/2.;
        sensor hexVal = (M_PI / 2. - angle) / (M_PI / 2.);
        return hexVal;
    }

    // invConv function
    motor TfjointL_from_hex_to_amos(motor hexVal){
        double angle = -hexVal * (M_PI / 2.);
        motor amosVal = -(angle - (amConf.fcoxaJointLimitB + amConf.fcoxaJointLimitF)/2.) / (( amConf.fcoxaJointLimitB - amConf.fcoxaJointLimitF )/2.);
        return amosVal;
    }
    motor TmjointL_from_hex_to_amos(motor hexVal){
        double angle = -hexVal * (M_PI / 2.);
        motor amosVal = -(angle - (amConf.mcoxaJointLimitB + amConf.mcoxaJointLimitF)/2.) / (( amConf.mcoxaJointLimitB - amConf.mcoxaJointLimitF )/2.);
        return amosVal;
    }
    motor TrjointL_from_hex_to_amos(motor hexVal){
        double angle = -hexVal * (M_PI / 2.);
        motor amosVal = -(angle - (amConf.rcoxaJointLimitB + amConf.rcoxaJointLimitF)/2.) / (( amConf.rcoxaJointLimitB - amConf.rcoxaJointLimitF )/2.);
        return amosVal;
    }
    motor TfjointR_from_hex_to_amos(motor hexVal){
        double angle = -hexVal * (M_PI / 2.);
        motor amosVal = (angle - (amConf.fcoxaJointLimitB + amConf.fcoxaJointLimitF)/2.) / (( amConf.fcoxaJointLimitB - amConf.fcoxaJointLimitF )/2.);
        return amosVal;
    }
    motor TmjointR_from_hex_to_amos(motor hexVal){
        double angle = -hexVal * (M_PI / 2.);
        motor amosVal = (angle - (amConf.mcoxaJointLimitB + amConf.mcoxaJointLimitF)/2.) / (( amConf.mcoxaJointLimitB - amConf.mcoxaJointLimitF )/2.);
        return amosVal;
    }
    motor TrjointR_from_hex_to_amos(motor hexVal){
        double angle = -hexVal * (M_PI / 2.);
        motor amosVal = (angle - (amConf.rcoxaJointLimitB + amConf.rcoxaJointLimitF)/2.) / (( amConf.rcoxaJointLimitB - amConf.rcoxaJointLimitF )/2.);
        return amosVal;
    }

    motor Cjoint_from_hex_to_amos(motor hexVal){
        double angle = -hexVal * (M_PI / 2.);
        motor amosVal = -(angle - (amConf.secondJointLimitD + amConf.secondJointLimitU)/2.) / (( amConf.secondJointLimitD - amConf.secondJointLimitU )/2.);
        return amosVal;
    }
    motor Fjoint_from_hex_to_amos(motor hexVal){
        double angle = - (hexVal * (M_PI / 2.) - (M_PI / 2.));
        motor amosVal = -(angle - (amConf.tebiaJointLimitD + amConf.tebiaJointLimitU)/2.) / (( amConf.tebiaJointLimitD - amConf.tebiaJointLimitU )/2.);
        return amosVal;
    }

    // convert sensor data
    void Sensor_from_amos_to_hex(const sensor* amos_, int amos_size_, sensor* hex_, int hex_size_){
        // initialize
        for(int i=0;i<hex_size_;i++){
            hex_[i] = 0.;
        }

        // convert T joint data
        hex_[HEXABOT::T1_as] = TfjointR_from_amos_to_hex(amos_[TR0_as]);
        hex_[HEXABOT::T2_as] = TmjointR_from_amos_to_hex(amos_[TR1_as]);
        hex_[HEXABOT::T3_as] = TrjointR_from_amos_to_hex(amos_[TR2_as]);
        hex_[HEXABOT::T4_as] = TfjointL_from_amos_to_hex(amos_[TL0_as]);
        hex_[HEXABOT::T5_as] = TmjointL_from_amos_to_hex(amos_[TL1_as]);
        hex_[HEXABOT::T6_as] = TrjointL_from_amos_to_hex(amos_[TL2_as]);

        // convert C F joint data
        for(int i=0;i<6;i++){
            hex_[HEXABOT::C1_as + i] = Cjoint_from_amos_to_hex(amos_[CR0_as + i]);
            hex_[HEXABOT::F1_as + i] = Fjoint_from_amos_to_hex(amos_[FR0_as + i]);
        }

        // convert Foot contact sensor
        for(int i=0;i<6;i++){
            hex_[HEXABOT::L1_fs + i] = amos_[R0_fs + i];
        }

        // convert torque
        for(int i=0; i<6;i++){
            hex_[HEXABOT::T1_ts + i] = amos_[TR0_ts + i];
            hex_[HEXABOT::C1_ts + i] = amos_[CR0_ts + i];
            hex_[HEXABOT::F1_ts + i] = amos_[FR0_ts + i];
        }

        // convert pose, angVel, gPos, gSpd
        for(int i=0;i<3;i++){
            hex_[HEXABOT::POSE_r + i] = amos_[G0angleroll_s+i];
            //hex_[W_x + i] = amos_[+i];
            hex_[HEXABOT::GSPD_Rx + i] = amos_[BX_spd+i];
            hex_[HEXABOT::GPOS_Rx + i] = amos_[BX_pos+i];
        }
    }

    // convert motor data
    void Motor_from_hex_to_amos(motor* hex_, int hex_size_, motor* amos_, int amos_size_){
        // initialize amos motor
        for(int i=0;i<amos_size_;i++){
            amos_[i] = 0.;
        }
        // convert joints
        amos_[TR0_m] = TfjointR_from_hex_to_amos(hex_[HEXABOT::T1_m]);
        amos_[TR1_m] = TmjointR_from_hex_to_amos(hex_[HEXABOT::T2_m]);
        amos_[TR2_m] = TrjointR_from_hex_to_amos(hex_[HEXABOT::T3_m]);
        amos_[TL0_m] = TfjointL_from_hex_to_amos(hex_[HEXABOT::T4_m]);
        amos_[TL1_m] = TmjointL_from_hex_to_amos(hex_[HEXABOT::T5_m]);
        amos_[TL2_m] = TrjointL_from_hex_to_amos(hex_[HEXABOT::T6_m]);

        // convert
        for(int i=0;i<6;i++){
            amos_[CR0_m +i] = Cjoint_from_hex_to_amos(hex_[HEXABOT::C1_m +i]);
            amos_[FR0_m +i] = Fjoint_from_hex_to_amos(hex_[HEXABOT::F1_m +i]);
        }

        // joint which is not used should be controlled as rigid body
        amos_[BJ_m] = 0.;

    }

public:
    // interfaces
     // change the configs to hexabot
     // Notice that we change the configs only about joint length, whole mass.
    static HexabotConf changeConf(AmosIIConf& conf_){
        HexabotConf hConf;
        //hConf = Hexabot::getDefaultConf();
        // configuration of rate
        hConf.rate = 1.; // we multiple 1 with the length
        // configuration of mass rate
        hConf.massRate = 1.;
        hConf.servoParam.power = 50;//20.;
        hConf.servoParam.damp = 0.;
        hConf.servoParam.integ = 0.;
        hConf.servoParam.maxVel = 1.7 * 1.961 * M_PI;

        // joint length
        double halfWidth;
        if(conf_.useShoulder) halfWidth = conf_.width/2. + conf_.shoulderLength;
        else halfWidth = conf_.width/2.;
        hConf.jLength.length_x_center_to_TCJ = halfWidth;
        hConf.jLength.length_y_TCJ_to_TCJ = (conf_.size + conf_.frontLength)/2.;
        hConf.jLength.length_TCJ_to_CTJ = conf_.coxaLength;
        hConf.jLength.length_CTJ_to_FTJ = conf_.secondLength;
        hConf.jLength.length_FTJ_to_Toe = conf_.tebiaLength;

        // Mass
        if(conf_.useShoulder){
            hConf.wholeMass = conf_.trunkMass + conf_.frontMass + 6. * (conf_.coxaMass + conf_.secondMass + conf_.tebiaMass + conf_.footMass + conf_.shoulderMass);
        }else hConf.wholeMass = conf_.trunkMass + conf_.frontMass + 6. * (conf_.coxaMass + conf_.secondMass + conf_.tebiaMass + conf_.footMass);

        // return
        return hConf;
    }

    // constructor
    AmosController_adptHexabot(std::string name_, AmosIIConf& aConf_, HexabotControllerConf& conf_, bool logCtrSnsFlag_ = false, std::string logLocation_ = "", std::string fileName_="", int logNum_ = 0, bool fileFlag_=false)
        :amConf(aConf_), hxConf(changeConf(amConf)), HexabotController(name_, hxConf, conf_, fileFlag_), amTime(0), amCnt(0)
    {
        // log setting
        if(logCtrSnsFlag_){
            std::stringstream ss;
            ss << logLocation_ << logNum_ << "_CtrLog_" << fileName_ << ".dat";
            ctrOfs.open(ss.str());

            std::stringstream ss2;
            ss2 << logLocation_ << logNum_ << "_SnsLog_" << fileName_ << ".dat";
            snsOfs.open(ss2.str());
        }

        // out put file
        if(ctrOfs.is_open()){
            ctrOfs << "# This is the file to log ctr parameter for amos ii " << std::endl;
            ctrOfs << "# 1:cnt 2:time, 3..... same as the amos enum motor " << std::endl;
        }
        if(snsOfs.is_open()){
            snsOfs << "# this is the file to log sns parameter for amos ii" << std::endl;
            snsOfs << "# 1:cnt 2:time, 3..... same as the amos enum sensor " << std::endl;
        }
    }

    // destructor
    virtual ~AmosController_adptHexabot(){
        if(ctrOfs.is_open()) ctrOfs.close();
        if(snsOfs.is_open()) snsOfs.close();
    }

   //*** Virtual functions ***************************************************:

    virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0){
        return;
    }

    /// returns the name of the object (with version number)
    //virtual paramkey getName() const {
    //  return name;
    //}
    /// returns the number of sensors the controller was initialised with or 0
    /// if not initialised
    virtual int getSensorNumber() const {
      return AMOSII_SENSOR_MAX;
    }
    /// returns the mumber of motors the controller was initialised with or 0 if
    // not initialised
    virtual int getMotorNumber() const {
      return AMOSII_MOTOR_MAX;
    }

    // performs one step (includes learning).
    // Calulates motor commands from sensor inputs.
    //! this is overloaded to adapt to amosII
    virtual void step(const sensor* x_, int number_sensors, motor* y_, int number_motors){
        // convert the sensor data and motor data to the hexabotController's parma
        sensor hex_s[HEXABOT_SENSOR_MAX];
        motor hex_m[HEXABOT_MOTOR_MAX];

        // control dt
        double dt = 1./conf.controlFreq;
        amTime += dt;
        amCnt++;

        // logging sensor
        if(snsOfs.is_open()){
            snsOfs << amCnt <<  " " << amTime << "   ";
            for(int i=0;i<number_sensors;i++){
                snsOfs << x_[i] << " ";
            }
            snsOfs << std::endl;
        }

        //convert senser data
        Sensor_from_amos_to_hex(x_, number_sensors, hex_s, HEXABOT_SENSOR_MAX);

        // call Hexabot step
        HexabotController::step(hex_s, HEXABOT_SENSOR_MAX, hex_m, HEXABOT_MOTOR_MAX);

        // convert motor data
        Motor_from_hex_to_amos(hex_m, HEXABOT_MOTOR_MAX, y_, number_motors);

        // logging motor
        if(ctrOfs.is_open()){
            ctrOfs << amCnt <<  " " << amTime << "   ";
            for(int i=0;i<number_motors;i++){
                ctrOfs << y_[i] << " ";
            }
            ctrOfs << std::endl;
        }
        //return;
    }

    // performs one step without learning. Calculates motor commands from sensor
    // inputs.
    //! this is overloaded to adapt to amosII
    virtual void stepNoLearning(const sensor* x_, int number_sensors, motor* y_, int number_motors){
        // convert the sensor data and motor data to the hexabotController's parma
        sensor hex_s[HEXABOT_SENSOR_MAX];
        motor hex_m[HEXABOT_MOTOR_MAX];

        //convert senser data
        Sensor_from_amos_to_hex(x_, number_sensors, hex_s, HEXABOT_SENSOR_MAX);

        // call Hexabot step
        HexabotController::stepNoLearning(hex_s, HEXABOT_SENSOR_MAX, hex_m, HEXABOT_MOTOR_MAX);

        // convert motor data
        Motor_from_hex_to_amos(hex_m, HEXABOT_MOTOR_MAX, y_, number_motors);

        //return;
    }

};

#endif // AMOSCONTROLLER_HPP
