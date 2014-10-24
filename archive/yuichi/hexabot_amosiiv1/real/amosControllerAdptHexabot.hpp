#ifndef AMOSCONTROLLER_ADPT_HEXABOT_HPP
#define AMOSCONTROLLER_ADPT_HEXABOT_HPP 200

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

20140814 Ver. 2.00
    This is for real amos ii robot

*/

#include <sstream>
#include "hexabotController.h"
#include <ode_robots/amosiisensormotordefinition.h>
#include <amosIIserialv1.h>
//#include "../../../lpzrobots/real_robots/robots/amosii/amosIIserialv1.h"
//#include "ode_robots/amosII.h"

// this is for robots exp

struct AmosIIConf {
    /**
     * @name flags
     *
     * Enable or disable different element and features
     */
    /**@{*/
    /** fix the shoulder element to the trunk. */
    bool useShoulder;
    /** whether to use joints at the knees or fix them */
    bool useTebiaJoints;
    /** use spring foot */
    bool useFoot;
    /** use the hinge joint in the back */
    bool useBack;
    /**
     * if true, rubber substance is used for feet instead of the substance
     * used for the rest of the robot
     */
    bool rubberFeet;
    /** decide whether you wand to use a local velocity sensors.
     *  If yes it gets velocity vector in local coordinates and pass it as
     *  sensorvalues */
    bool useLocalVelSensor;
    /** Use binary leg contact sensors. If false, a force sensor is used. */
    bool legContactSensorIsBinary;
    /**@}*/

    /** scaling factor for robot (length of body) */
    double size;
    /** trunk width */
    double width;
    /** trunk height */
    double height;
    /** length of the front of the body (if back joint is used) */
    double frontLength;
    /** radius of a wheel */
    double wheel_radius;
    /** width of a wheel */
    double wheel_width;
    /** mass of a wheel */
    double wheel_mass;

    /** trunk mass */
    double trunkMass;
    /** mass of the front part of the robot (if backboine joint is used) */
    double frontMass;
    /** mass of the shoulders (if used) */
    double shoulderMass;
    /** mass of the coxa limbs */
    double coxaMass;
    /** mass of the second limbs */
    double secondMass;
    /** mass of the tebia limbs */
    double tebiaMass;
    /** mass of the feet */
    double footMass;

    /** fix legs to trunk at this distance from bottom of trunk */
    double shoulderHeight;

    /** distance between hindlegs and middle legs */
    double legdist1;

    /** distance between middle legs and front legs */
    double legdist2;

    /** @name Leg extension from trunk
     *
     *  amosII has a fixed but adjustable joint that decides how the legs
     *  extend from the trunk.here you can adjust these joints, if
     *  shoulder = 0 this still influences how the legs extend (the coxa-trunk
     *  connection in that case)
     */
    /**@{*/
    /** angle in rad around vertical axis at leg-trunk fixation for front
     *  legs*/
    double fLegTrunkAngleV;
    /** angle in rad around horizontal axis at leg-trunk fixation for front
     *  legs */
    double fLegTrunkAngleH;
    /** rotation of front legs around own axis */
    double fLegRotAngle;
    /** angle in rad around vertical axis at leg-trunk fixation for middle
     *  legs */
    double mLegTrunkAngleV;
    /** angle in rad around horizontal axis at leg-trunk fixation for middle
     *  legs */
    double mLegTrunkAngleH;
    /** rotation of middle legs around own axis */
    double mLegRotAngle;
    /** angle in rad around vertical axis at leg-trunk fixation for rear legs
     * */
    double rLegTrunkAngleV;
    /** angle in rad around horizontal axis at leg-trunk fixation for rear
     *  legs */
    double rLegTrunkAngleH;
    /** rotation of rear legs around own axis */
    double rLegRotAngle;
    /**@}*/

    /**
     * @name leg part dimensions
     *
     * the lengths and radii of the individual leg parts
     */
    /**@{*/
    /** length of the shoulder limbs (if used) */
    double shoulderLength;
    /** radius of the shoulder limbs (if used) */
    double shoulderRadius;
    /** length of the coxa limbs */
    double coxaLength;
    /** radius of the coxa limbs */
    double coxaRadius;
    /** length of the second limbs */
    double secondLength;
    /** radius of the second limbs */
    double secondRadius;
    /** length of the tebia limbs including fully extended foot spring
     *  (if used) */
    double tebiaLength;
    /** radius of the tebia limbs */
    double tebiaRadius;
    /** range of the foot spring */
    double footRange;
    /** radius of the foot capsules, choose different from tebiaRadius */
    double footRadius;
    /**@}*/

    /**
     * @name Joint Limits
     *
     * set limits for each joint
     */
    /**{*/
    /** smaller limit of the backbone joint, positive is down */
    double backJointLimitD;
    /** upper limit of the backbone joint, positive is down */
    double backJointLimitU;
    /** forward limit of the front TC joints, negative is forward
     *  (zero specified by fcoxazero) */
    double fcoxaJointLimitF;
    /** backward limit of the front TC joints, negative is forward
     *  (zero specified by fcoxaZero) */
    double fcoxaJointLimitB;
    /** forward limit of the middle TC joints, negative is forward
     *  (zero specified by fcoxaZero) */
    double mcoxaJointLimitF;
    /** backward limit of the middle TC joints, negative is forward
     *  (zero specified by fcoxaZero) */
    double mcoxaJointLimitB;
    /** forward limit of the rear TC joints, negative is forward
     *  (zero specified by fcoxaZero) */
    double rcoxaJointLimitF;
    /** backward limit of the rear TC joints, negative is forward
     *  (zero specified by fcoxaZero) */
    double rcoxaJointLimitB;
    /** lower limit of the CTr joints, positive is down */
    double secondJointLimitD;
    /** upper limit of the CTr joints, positive is down */
    double secondJointLimitU;
    /** lower limit of the FTi joints, positive is down */
    double tebiaJointLimitD;
    /** upper limit of the FTi joints, positive is down */
    double tebiaJointLimitU;
    /**}*/

    /** preload of the foot spring */
    double footSpringPreload;
    /** upper limit of the foot spring = maximum value
     *  (negative is downwards (spring extends)) */
    double footSpringLimitU;
    /** lower limit of the foot spring = minimum value
     *  (negative is downwards (spring extends)) */
    double footSpringLimitD;

    /** maximal force of the backbone joint */
    double backPower;
    /** maximal force of the TC joint servos */
    double coxaPower;
    /** maximal force of the CTr joint servos */
    double secondPower;
    /** maximal force of the FTi joint servos */
    double tebiaPower;
    /** maximal force of the foot spring servos */
    double footPower;

    /** damping of the backbone joint servo */
    double backDamping;
    /** damping of the TC joint servos */
    double coxaDamping;
    /** damping of the CTr joint servo */
    double secondDamping;
    /** damping of the FTi joint servo */
    double tebiaDamping;
    /** damping of the foot spring servo */
    double footDamping;

    /** maximal velocity of the backbone joint servo */
    double backMaxVel;
    /** maximal velocity of the TC joint servo */
    double coxaMaxVel;
    /** maximal velocity of the CTr joint servo */
    double secondMaxVel;
    /** maximal velocity of the FTi joint servo */
    double tebiaMaxVel;
    /** maximal velocity of the foot spring servo */
    double footMaxVel;

    /**
     * @name front ultrasonic sensors
     *
     * configure the front ultrasonic sensors
     */
    /**{*/
    /** angle versus x axis */
    double usAngleX;
    /** angle versus y axis */
    double usAngleY;
    /** choose between parallel or antiparallel front ultrasonic sensors true
     *  means parallel */
    bool usParallel;
    /** range of the front ultrasonic sensors */
    double usRangeFront;
    /**}*/

    /** range of the infrared sensors at the legs */
    double irRangeLeg;

    /** path to texture for legs */
    std::string texture;
    /** path to texture for trunk */
    std::string bodyTexture;

    //-----------Add GoalSensor by Ren------------------------
    //std::vector<Primitive*> GoalSensor_references;
    //-----------Add GoalSensor by Ren------------------------

    // Internal variable storing the currently used version
    int amos_version;

};

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
    // It is for real robot.
    void Sensor_from_amos_to_hex(const sensor* amos_, int amos_size_, sensor* hex_, int hex_size_){
        // initialize
        for(int i=0;i<hex_size_;i++){
            hex_[i] = 0.;
        }

        // convert T joint data
        hex_[HEXABOT::T1_as] = 0;//TfjointR_from_amos_to_hex(amos_[TR0_as]);
        hex_[HEXABOT::T2_as] = 0;//TmjointR_from_amos_to_hex(amos_[TR1_as]);
        hex_[HEXABOT::T3_as] = 0;//TrjointR_from_amos_to_hex(amos_[TR2_as]);
        hex_[HEXABOT::T4_as] = 0;//TfjointL_from_amos_to_hex(amos_[TL0_as]);
        hex_[HEXABOT::T5_as] = 0;//TmjointL_from_amos_to_hex(amos_[TL1_as]);
        hex_[HEXABOT::T6_as] = 0;//TrjointL_from_amos_to_hex(amos_[TL2_as]);

        // convert C F joint data
        for(int i=0;i<6;i++){
            hex_[HEXABOT::C1_as + i] = 0;//Cjoint_from_amos_to_hex(amos_[CR0_as + i]);
            hex_[HEXABOT::F1_as + i] = 0;//Fjoint_from_amos_to_hex(amos_[FR0_as + i]);
        }

        // convert Foot contact sensor (should be normalized)
        //hex_[HEXABOT::L1_fs] = amos_[10];
        //hex_[HEXABOT::L2_fs] = amos_[15];
        //hex_[HEXABOT::L3_fs] = amos_[17];
        //hex_[HEXABOT::L4_fs] = amos_[13];
        //hex_[HEXABOT::L5_fs] = amos_[16];
        //hex_[HEXABOT::L6_fs] = amos_[9];
        for(int i=0;i<6;i++){
            hex_[HEXABOT::L1_fs + i] = amos_[R0_fs + i];
        }

        // convert torque
        for(int i=0; i<6;i++){
            hex_[HEXABOT::T1_ts + i] = 0;//amos_[TR0_ts + i];
            hex_[HEXABOT::C1_ts + i] = 0;//amos_[CR0_ts + i];
            hex_[HEXABOT::F1_ts + i] = 0;//amos_[FR0_ts + i];
        }

        // convert pose, angVel, gPos, gSpd
        for(int i=0;i<3;i++){
            //hex_[W_x + i] = amos_[+i];
            hex_[HEXABOT::GSPD_Rx + i] = 0;//amos_[BX_spd+i];
            hex_[HEXABOT::GPOS_Rx + i] = 0;//amos_[BX_pos+i];
        }

        // I insert the data of inclinometer and that of Current
        hex_[HEXABOT::POSE_r] = asin( ((amos_[BX_acs] -127.) * 5./255.) / 2. ) * 180. / M_PI; // inclinometer y [deg]
        hex_[HEXABOT::POSE_p] = asin( ((amos_[BY_acs] -127.) * 5./255.) / 2. ) * 180. / M_PI; // inclinometer x [deg]
        // I am not sure this translation is correct or not, but nobody is sure..... so I just use it!
        hex_[HEXABOT::POSE_y] = ((amos_[A_cs]/2. + 128) * 5. / 256. - 2.5) / 0.037; // current [A]
    }

    // convert motor data
    void Motor_from_hex_to_amos(motor* hex_, int hex_size_, motor* amos_, int amos_size_){
        // initialize amos motor
        for(int i=0;i<amos_size_;i++){
            amos_[i] = 0.;
        }
        // convert joints
        amos_[TR0_m] = TfjointR_from_hex_to_amos(hex_[HEXABOT::T1_m]) - 0.4;
        amos_[TR1_m] = TmjointR_from_hex_to_amos(hex_[HEXABOT::T2_m]) - 0.2;
        amos_[TR2_m] = TrjointR_from_hex_to_amos(hex_[HEXABOT::T3_m]);
        amos_[TL0_m] = TfjointL_from_hex_to_amos(hex_[HEXABOT::T4_m]) - 0.4;
        amos_[TL1_m] = TmjointL_from_hex_to_amos(hex_[HEXABOT::T5_m]) - 0.2;
        amos_[TL2_m] = TrjointL_from_hex_to_amos(hex_[HEXABOT::T6_m]);

        // convert
        for(int i=0;i<6;i++){
            amos_[CR0_m +i] = Cjoint_from_hex_to_amos(hex_[HEXABOT::C1_m +i]);
            amos_[FR0_m +i] = Fjoint_from_hex_to_amos(hex_[HEXABOT::F1_m +i]);
        }
        amos_[FR1_m] = Fjoint_from_hex_to_amos(hex_[HEXABOT::F2_m]) + 0.1;
        amos_[FL1_m] = Fjoint_from_hex_to_amos(hex_[HEXABOT::F5_m]) - 0.1;
        amos_[FL2_m] = Fjoint_from_hex_to_amos(hex_[HEXABOT::F6_m]) - 0.1;

        amos_[CR0_m] = Cjoint_from_hex_to_amos(hex_[HEXABOT::C1_m]) - 0.1;
        amos_[CR1_m] = Cjoint_from_hex_to_amos(hex_[HEXABOT::C2_m]) + 0.1;
        amos_[CR2_m] = Cjoint_from_hex_to_amos(hex_[HEXABOT::C3_m]) - 0.1;

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
