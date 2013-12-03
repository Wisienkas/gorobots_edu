/***************************************************************************
 *   Copyright (C) 2010 by
 *    Martin Biehl <mab@physik3.gwdg.de>                                   *
 *    Guillaume de Chambrier <s0672742@sms.ed.ac.uk>                       *
 *    martius@informatik.uni-leipzig.de                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 **************************************************************************/

//#define VERBOSE
#include <cmath>
#include <assert.h>

// #include <ode/ode.h>
#include <ode-dbl/ode.h>

// include primitives (box, spheres, cylinders ...)
#include <ode_robots/primitive.h>

// include sensors
//#include <ode_robots/speedsensor.h>
#include <ode_robots/irsensor.h>

// include joints
#include <ode_robots/joint.h>
#include <ode_robots/oneaxisservo.h>
#include <ode_robots/twoaxisservo.h>

#include <ode_robots/mathutils.h>

// include header file
#include "amosII.h"

using namespace osg;
using namespace std;

namespace lpzrobots {
  
  int t = 1;
  int c = 1;
  
  // constructor:
  // - give handle for ODE and OSG stuff
  // also initialize AmosII.conf with the configuration in the argument of the constructor
  AmosII::AmosII(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const AmosIIConf& c, const std::string& name) :
      OdeRobot(odeHandle, osgHandle, name, "AMOSII 0.1"), conf(c) {
    // robot is not created till now
    pos1d = new double[3];
    t = 0;
    massOfobject = new dMass();
    getPos1 = true;
    timeCounter = conf.T;
    energyOneStep = new double[1];
    energyOneStep[0] = 0.0;
    costOfTran = 0.0;
    created = false;
    check = false;
    recordGait = false;
    dones = new bool[6];
    conf.v[0] = 0;
    SpeedSensor::Dimensions speeddim = SpeedSensor::XY;
    SpeedSensor::Mode speedmode = SpeedSensor::TranslationalRel;
    speedsensor = new SpeedSensor(1.0, speedmode, speeddim);
    
    angles = new double[18]; //stores joint angles (see dointernalstuff)
    heights = new double[6];
    hcorrection = 0.20803;
    
    legContactArray = new Leg[6];
    
    addParameter("coxaPower", &conf.coxaPower);
    addParameter("secondPower", &conf.secondPower);
    addParameter("coxaDamp", &conf.coxaDamping);
    addParameter("fcoxaJointLimitF", &conf.fcoxaJointLimitF);
    addParameter("fcoxaJointLimitB", &conf.fcoxaJointLimitB);
    addParameter("mcoxaJointLimitF", &conf.mcoxaJointLimitF);
    addParameter("mcoxaJointLimitB", &conf.mcoxaJointLimitB);
    addParameter("rcoxaJointLimitF", &conf.rcoxaJointLimitF);
    addParameter("rcoxaJointLimitB", &conf.rcoxaJointLimitB);
    addParameter("secondJointLimitD", &conf.secondJointLimitD);
    addParameter("secondJointLimitU", &conf.secondJointLimitU);
    addParameter("coxaMaxVel", &conf.coxaMaxVel);
    
    if (conf.useTebiaJoints) {
      addParameter("tebiaPower", &conf.tebiaPower);
      addParameter("tebiaDamp", &conf.tebiaDamping);
      addParameter("tebiaJointLimitD", &conf.tebiaJointLimitD);
      addParameter("tebiaJointLimitU", &conf.tebiaJointLimitU);
    }
    int counter = 0;
    
//    // name the sensors
//    //todo add motors
//    for(int n=0; n<6; n++){
//      addInspectableDescription("x[" + itos (n*3) + "]",
//                                "leg pair " + itos(n/2) + (n%2==0 ? " right" : " left")
//                                + " forward/backward");
//      addInspectableDescription("x[" + itos (n*3+1) + "]",
//                                "leg pair " + itos(n/2) + (n%2==0 ? " right" : " left")
//                                + " up/down");
//      addInspectableDescription("x[" + itos (n*3+2) + "]",
//                                "leg pair " + itos(n/2) + (n%2==0 ? " right" : " left")
//                                + " outward/inward");
//    }
    
//---------------------------CHANG----KOH
    // name the sensors
    //todo add motors
    for (int i = 0; i < 3; i++) {
      addInspectableDescription("x[" + itos(i * 6) + "]", "TR" + itos(2 - i) + "_as" ": +1 Forward, -1 Backward");
      addInspectableDescription("x[" + itos(i * 6 + 1) + "]", "CR" + itos(2 - i) + "_as" ": +1 Upward, -1 Downward");
      addInspectableDescription("x[" + itos(i * 6 + 2) + "]", "FR" + itos(2 - i) + "_as" ": +1 Exten, -1 Flex");
      
      addInspectableDescription("x[" + itos(i * 6 + 3) + "]", "TL" + itos(2 - i) + "_as" ": +1 Forward, -1 Backward");
      addInspectableDescription("x[" + itos(i * 6 + 4) + "]", "CL" + itos(2 - i) + "_as" ": +1 Upward, -1 Downward");
      addInspectableDescription("x[" + itos(i * 6 + 5) + "]", "FL" + itos(2 - i) + "_as" ": +1 Exten, -1 Flex");
      
      addInspectableDescription("y[" + itos(i * 6) + "]", "TR" + itos(2 - i) + "_m" ": +1 Forward, -1 Backward");
      addInspectableDescription("y[" + itos(i * 6 + 1) + "]", "CR" + itos(2 - i) + "_m" ": +1 Upward, -1 Downward");
      addInspectableDescription("y[" + itos(i * 6 + 2) + "]", "FR" + itos(2 - i) + "_m" ": +1 Exten, -1 Flex");
      
      addInspectableDescription("y[" + itos(i * 6 + 3) + "]", "TL" + itos(2 - i) + "_m" ": +1 Forward, -1 Backward");
      addInspectableDescription("y[" + itos(i * 6 + 4) + "]", "CL" + itos(2 - i) + "_m" ": +1 Upward, -1 Downward");
      addInspectableDescription("y[" + itos(i * 6 + 5) + "]", "FL" + itos(2 - i) + "_m" ": +1 Exten, -1 Flex");
      
    }
    
    counter += 18;
    if (conf.useBack) {
      addInspectableDescription("x[" + itos(counter) + "]", " BJ_as" ": +1 Upward, -1 Downward");
      addInspectableDescription("y[" + itos(counter) + "]", " BJ_m" ": +1 Upward, -1 Downward");
      
      counter++;
    }
    if (conf.useLocalVelSensor) {
      addInspectableDescription("x[" + itos(counter) + "]", " local velocity forward");
      addInspectableDescription("x[" + itos(counter + 1) + "]", " local velocity sideward");
      counter += 2;
    }
    
    if (conf.useContactSensors) {
      
      for (int i = 0; i < 3; i++) {
        addInspectableDescription("x[" + itos(19 + i * 2) + "]",
            "R" + itos(2 - i) + "_fs : 0 = Off Ground, 1 = Touch down");
        addInspectableDescription("x[" + itos(19 + i * 2 + 1) + "]",
            "L" + itos(2 - i) + "_fs : 0 = Off Ground, 1 = Touch down");
      }
      counter += 6;
      
    }
//---------------------------CHANG----KOH
    
    // Georg: you can also add inspectables here to see them in the logfile/guilogger
    // e.g.
    // addInspectableValue("Energy", &E_t, "Energy over several timesteps");
    addInspectableValue("Power", &Power, "Power (work over time) used for one timesteps");
    
  }
  ;
  
  int AmosII::getMotorNumber() {
//    return  2*hipservos.size();
    return coxaservos.size() + secondservos.size() + /*conf.useTebiaJoints **/tebiaservos.size() + conf.useBack;
    std::cout << "motornumber:"
        << coxaservos.size() + secondservos.size() + /*conf.useTebiaJoints **/tebiaservos.size() + conf.useBack << endl;
  }
  ;
  
  /* sets actual motorcommands
   @param motors motors scaled to [-1,1]
   @param motornumber length of the motor array
   */
  void AmosII::setMotors(const motor* motors, int motornumber) {
#ifdef VERBOSE
    std::cout << "begin set motors \n";
#endif
    assert(created);
    // robot must exist
    
    int len = (min(motornumber, getMotorNumber()) - conf.useBack) / 3; //2;
        
//    std::cout << "motornumber:" <<  motornumber << endl;
    
//    tebiaservos[0]->set(motors[0]);
//    tebiaservos[1]->set(motors[1]);
//    tebiaservos[2]->set(motors[2]);
//    tebiaservos[3]->set(motors[3]);
//    tebiaservos[4]->set(motors[4]);
//    tebiaservos[5]->set(motors[5]);
    
//    tebiaservos[0]->set(0.0);
//    tebiaservos[1]->set(0.0);
//    tebiaservos[2]->set(0.0);
//    tebiaservos[3]->set(0.0);
//    tebiaservos[4]->set(0.0);
//    tebiaservos[5]->set(0.0);
    
//---------------------------CHANG----KOH
//We multiple with -1 to map to real hexapod
    for (int i = 0; i < len; i++) {
      coxaservos[i]->set(-1 * motors[3 * i]);
      secondservos[i]->set(-1 * motors[3 * i + 1]);
      tebiaservos[i]->set(-1 * motors[3 * i + 2]);
    }
    
    FOREACH(vector<OneAxisServo*>, footsprings, i) {
      if (*i)
        (*i)->set(0);
    }
    
//We multiple with -1 to map to real hexapod
    if (conf.useBack)
      backservo->set(-1 * motors[18]);
    
//---------------------------CHANG----KOH
    
#ifdef VERBOSE
    std::cout << "end set motors \n";
#endif
  }
  ;
  
  //%%%%%%%%%%%%%%%%%%%%%%%%%Default Sensors as Real robot%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
  
  int AmosII::getSensorNumber() {
    
    //-1)---------Add number of sensors
    return coxaservos.size() + secondservos.size() + tebiaservos.size() + conf.useBack + irSensorBank.size()
        + conf.useContactSensors * 6 + conf.useLocalVelSensor * 3;
    
    //return  coxaservos.size() + secondservos.size() + tebiaservos.size() + conf.useBack + irSensorBank.size()
    //  		+ conf.useContactSensors * 6 + conf.useLocalVelSensor * 3+ Other sensors;
    
#ifdef VERBOSE
    std::cout << "sensornumber:" << coxaservos.size() + secondservos.size() + tebiaservos.size() + conf.useBack + irSensorBank.size()
    + conf.useContactSensors * 6 + conf.useLocalVelSensor<< endl;
#endif 
    
  }
  ;
  
  /* returns actual sensorvalues
   @param sensors sensors scaled to [-1,1] (more or less)
   @param sensornumber length of the sensor array
   @return number of actually written sensors
   */
  int AmosII::getSensors(sensor* sensors, int sensornumber) {
    
#ifdef VERBOSE
    std::cout << "begin get sensors \n";
#endif
    
    assert(created);
    assert(
        sensornumber ==(int) (coxaservos.size() + secondservos.size() + tebiaservos.size() + conf.useBack + irSensorBank.size() + conf.useContactSensors * 6 + conf.useLocalVelSensor * 2));
//    int len = min(sensornumber, getSensorNumber() - irSensorBank.size())/2;
    
    /*
     *
     19x Angle sensors (as)
     6 x Foot sensors (fs)*+
     19 x Current sensors at each motor (cs)++
     18 x leg acce sensors / each leg has x,y,z acc sensors (las)*- may use only 12 sensors x,y axis

     4 x reflex ultrasonic sensors at front and middle legs (rus)*
     2 x Front Ultrasonic sensors (us)*+
     6 x IR sensors at legs (irs)*--

     1x 3D Accelerometer (x,y,z) at body (acs)++

     3 x Light sensors (ls)*+



     1x CMU camera at front body part (cmus)
     1x RS232-Laser scanner at rear body part (las)
     1X Compass sensor at rear body part (cs)
     3x Microphones at front legs and front part (ms)
     1x Speaker at body (sp)

     //--only amosII v1
     1 x rear IR sensor (rirs)*
     1 x ZAP 25 current sensor, average power sensor (aps)*-
     1 x mono microphone low freq at rear part (mms)*-
     2 x increnometer sensor at rear body part for x pointing forward and y axis pointing sideward (is)*
     1 x poti sensors (ps)*

     *32 x Analog inputs (~100 Hz low pass filters)

     */

///--------------Joint Angle sensors
//We multiple with -1 to map to real hexapod
    int len = 6;
    for (int i = 0; i < len; i++) {
      sensors[3 * i] = -1 * coxaservos[i]->get();
      sensors[3 * i + 1] = -1 * secondservos[i]->get();
      sensors[3 * i + 2] = -1 * tebiaservos[i]->get();
    }
    //increase sensor counter
    len = len * 3;
    
    if (conf.useBack) {
      sensors[len] = -1 * backservo->get();
      len++;
    }
    
///--------------Foot sensors
    if (conf.useContactSensors) {
      for (int i = 0; i < 6; i++) {
//        sensors[len + i] = conf.legContacts[i];
//----------- Add contact sensor ---- by Ren ------------------
        sensors[len + i] = ContactSens[i]->get();
//   		std::cout << "contact" << i << "  "<< conf.legContacts[i] << "\n";
      }
      len += 6;
    }
    
    /*
     ///--------------Speed sensor x,y,z
     if(conf.useLocalVelSensor)
     len+=speedsensor->get(&sensors[len],3);
     */

    /*
     ///--------------Foot sensors
     if(conf.useContactSensors) {
     for(int i = 0; i < 6; i++){
     sensors[len+i] = conf.legContacts[i];
     //   		std::cout << "contact" << i << "  "<< conf.legContacts[i] << "\n";
     }
     len+=6;
     }

     */

///--------------IR sensors
//    if (conf.irFront || conf.irBack){
//      len += irSensorBank.get(sensors+len, sensornumber-len);
//    }
//
    ///--------------IR sensors
    //todo!!
    // if (conf.ir){
    len += irSensorBank.get(sensors + len, 8 /*number of IR*/);
    //  }
    
///--------------Speed sensor x,y,z
    if (conf.useLocalVelSensor)
      len += speedsensor->get(&sensors[len], 3);
    
//2) ----ADD More sensors here//
    
//-------ADD More sensors here//
    
#ifdef VERBOSE
    std::cout << "end get sensors \n";
#endif
    return len;
  }
  ;
  
  //%%%%%%%%%%%%%%%%%%%%%%%%%Default Sensors as Real robot%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
  
  void AmosII::place(const osg::Matrix& pose) {
    // the position of the robot is the center of the body
    // to set the vehicle on the ground when the z component of the position is 0
    //    Matrix p2;
    //    p2 = pose * Matrix::translate(Vec3(0, 0, conf.legLength + conf.legLength/8)); 
    
    create(pose);
    
  }
  ;
  
  /**
   * updates the osg notes
   */
  void AmosII::update() {
#ifdef VERBOSE
    std::cout << "begin update \n";
#endif
    assert(created);
    // robot must exist
    
    for (vector<Primitive*>::iterator i = objects.begin(); i != objects.end(); i++) {
      if (*i)
        (*i)->update();
    }
    for (vector<Joint*>::iterator i = joints.begin(); i != joints.end(); i++) {
      if (*i)
        (*i)->update();
    }
    if (conf.useShoulder) { // if shoulder used update transform objects
      for (int i = 0; i < 6; i++) {
        shoulder[i].trans->update();
      }
    }
    
    //----------- Add contact sensor ---- by Ren --------------------
    for (int i = 0; i < 6; i++) {
      ContactSens[i]->update();
    }

    irSensorBank.update();
#ifdef VERBOSE
    std::cout << "end update \n";
#endif
  }
  ;
  
  double AmosII::outwardMechanicalPower(const dReal *torques, const dReal *angularV) {
    
    double mechanicalPower = 0.0;
    
    for (int i = 0; i < 3; i++) {
      mechanicalPower += torques[i] * angularV[i];
    }
    
    if (mechanicalPower <= 0.0) {
      mechanicalPower = 0.0;
    }
    
    return mechanicalPower;
  }
  
  double AmosII::energyConsumpThroughtHeatLoss(const dReal *torques) {
    // Georg: This should be sqr(torques[0]) + sqr(torques[1]) + ..
    // torque can be negative!
    // martin: when georg commented it used to be:
    // return pow(torques[0] + torques[1] + torques[2],2); 
    return (pow(torques[0], 2) + pow(torques[1], 2) + pow(torques[2], 2));
  }
  
  double AmosII::energyConsumption() {
    
    const dReal *torques;
    const dReal *angularV;
    double gamma = 0.005;
    double e = 0.0;
    
    for (unsigned int i = 0; i < legs.size(); i++) {
      torques = dBodyGetTorque(legs[i]->getBody());
      angularV = dBodyGetAngularVel(legs[i]->getBody());
      e += outwardMechanicalPower(torques, angularV) + gamma * energyConsumpThroughtHeatLoss(torques);
    }
    return e;
  }
  
  double AmosII::getMassOfRobot() {
    
    double totalMass = 0.0;
    
    for (unsigned int i = 0; i < objects.size(); i++) {
      dBodyGetMass(objects[i]->getBody(), massOfobject);
      totalMass += massOfobject->mass;
    }
    return totalMass;
  }
  
  double AmosII::costOfTransport(double E, double W, double V, double T) {
    return E / (W * V * T);
  }
  
  double AmosII::round(double num, int x) {
    
    return ceil((num * pow(10, x)) - 0.49) / pow(10, x);
    
  }
  /** this function is called in each timestep. It should perform robot-internal checks, 
   like space-internal collision detection, sensor resets/update etc.
   @param global structure that contains global data from the simulation environment
   */
  void AmosII::doInternalStuff(GlobalData& global) {
#ifdef VERBOSE
    std::cout << "begin do internal stuff \n";
#endif
//    irSensorBank.reset();
//
    Power = energyConsumption();
    speedsensor->sense(global);
//
//    t = global.time;
//
//    if(global.time <= timeCounter){
//      E_t += energyOneStep[0];
//    }
//
//    if(getPos1){
//#ifdef VERBOSE
//        std::cout << "hekko0 \n";
//#endif
//      pos1 = dBodyGetPosition(front->getBody());
//      pos1d[0] = pos1[0];
//      pos1d[1] = pos1[1];
//      pos1d[2] = pos1[2];
//#ifdef VERBOSE
//      std::cout << "hekko1a \n";
//#endif
//      getPos1 = false;
//    }
//
//    const dReal* velocity = dBodyGetLinearVel( front->getBody() );
//    const double v = abs(velocity[0]);
//    conf.v[0] = v;
//
////    cout << v << endl;
//
//
//    if(global.time >= timeCounter){
//
//      pos2 = dBodyGetPosition(front->getBody());
//      distance = sqrt(pow((pos2[0] - pos1d[0]),2) + pow((pos2[1] - pos1d[1]),2) + pow((pos2[2] - pos1d[2]),2));
//      conf.v[0] = distance/conf.T;
//      costOfTran = costOfTransport(E_t,getMassOfRobot(),conf.v[0],conf.T);
////      cout<< "cost of Transport: " << costOfTran << endl;
//      timeCounter += conf.T;
//      E_t = 0.0;
//      getPos1 = true;
//
//    }
//
//
//    for(unsigned int i = 0; i < 6; i++){
//
//      //at the moment bodyID is that of the tebia body
//      const dReal *position = dBodyGetPosition(legContactArray[i].bodyID);
//
//      // cout<< dJointGetUniversalAngle1(joints[0]->getJoint()) * 180/M_PI  << endl;
//      // cout<< dJointGetUniversalAngle2(joints[0]->getJoint())  * 180/M_PI<< endl;
//      //  cout << dJointGetUniversalAngle1(legContactArray[i].joint) * 180/M_PI << endl;
//      //  cout << dJointGetUniversalAngle2(legContactArray[i].joint) * 180/M_PI << endl;
//
//      heights[i] = abs(round(position[2] -  hcorrection,3));
//      //store angles of joints in degrees
//      angles[3*i]   = dJointGetHingeAngle(legContactArray[i].coxajoint) * 180/M_PI ;
//      angles[3*i+1] = dJointGetHingeAngle(legContactArray[i].secondjoint) * 180/M_PI ;
//      angles[3*i+2] = dJointGetHingeAngle(legContactArray[i].tebiajoint) * 180/M_PI ;
//    }

    //----------- Add contact sensor ---- by Ren-------------------
    for (int i = 0; i < 6; i++) {
      ContactSens[i]->reset();
    }
#ifdef VERBOSE
    std::cout << "end internal stuff \n";
#endif
  }
  
  /** checks for internal collisions and treats them. 
   *  In case of a treatment return true (collision will be ignored by other objects 
   *  and the default routine)  else false (collision is passed to other objects and 
   *  (if not treated) to the default routine).
   */
  bool AmosII::collisionCallback(void *data, dGeomID o1, dGeomID o2) {
//	/*new contsct sensor implementation, tries to look at feet only
//	 * as I am not shure what exactly is passed as o1 and o2, I check for
//	 * all collisions with each of them. Thus they may be spaces or normal geoms.
//	 */
//	//set all contacts to zero
//	for(int j = 0; j < 6; j++) {
//	  conf.legContacts[j] = 0;
//	}
//	for(int j = 0; j < 6; j++) {
//		const int NUM_CONTACTS = 4;//8;
//		dContact contacts[NUM_CONTACTS];
//		int numCollisions1 = dCollide(legContactArray[j].geomid, o1, NUM_CONTACTS, &contacts[0].geom, sizeof(dContact));
//		int numCollisions2 = dCollide(legContactArray[j].geomid, o2, NUM_CONTACTS, &contacts[0].geom, sizeof(dContact));
//		std::cout << "num collisions " << j << "  " << numCollisions1 << "  " << numCollisions2 << std::endl;
//		conf.legContacts[j] = (numCollisions1+numCollisions2) > 1 ? 1 : 0;
//	}
    
    //the following will tell you that the two passed geoms are different
//	  std::cout << "geomid1 " <<  o1 << "geomid2 " <<  o2 << std::endl;
    
    //old contact sensor, does not scale well because geoms are not treated equally
    // first checked geoms get all the attention!
    /*max number of contacts generated by dCollide if this is too low the contact sensors below fail
     * this is probably due to geomIDs of spaces being passed to collisioncallback
     * then contacts contains different pairs of geoms (not only o1 and o2 themselves)
     * so if more than NUM_CONTACTS pairs of geoms collide in the passed spaces
     * further collisions are not recorded in contacts */
    const int NUM_CONTACTS = 20; //8;
    dContact contacts[NUM_CONTACTS];
    int numCollisions = dCollide(o1, o2, NUM_CONTACTS, &contacts[0].geom, sizeof(dContact));
    // Georg: this would also be possible by a special substance with callback.
    //  I will maybe implement a contact sensor anyway...
//    std::cout << "num collisions " <<  numCollisions << std::endl;
    
    //set all contacts to zero
    for (int j = 0; j < 6; j++) {
      conf.legContacts[j] = 0;
    }
    
    for (int i = 0; i < numCollisions; i++) {
      //g1 is the geom that when moved along the depth vector reduces the depth of the contact
      dBodyID b1 = dGeomGetBody(contacts[i].geom.g1);
      dBodyID b2 = dGeomGetBody(contacts[i].geom.g2);
      // Georg: are you sure that b1 is always the leg?
      //we were not shure either so now it tests both for being the foot
      for (int j = 0; j < 6; j++) {
        if (legContactArray[j].bodyID == b1 || legContactArray[j].bodyID == b2) {
          conf.legContacts[j] = 1;
        }
      }
    }
//		if (legContactArray[1].bodyID == b1 || legContactArray[1].bodyID == b2) {
//			conf.legContacts[1] = 1;
//		}
//		if (legContactArray[2].bodyID == b1 || legContactArray[2].bodyID == b2) {
//			conf.legContacts[2] = 1;
//		}
//		if (legContactArray[3].bodyID == b1 || legContactArray[3].bodyID == b2) {
//			conf.legContacts[3] = 1;
//		}
//		if (legContactArray[4].bodyID == b1 || legContactArray[4].bodyID == b2) {
//			conf.legContacts[4] = 1;
//		}
//		if (legContactArray[5].bodyID == b1 || legContactArray[5].bodyID == b2) {
//			conf.legContacts[5] = 1;
//		}
    
    /*

     // cout<< "t: " << t << "   timeC: " << timeCounter << endl;
     if((t + 0.01) >= timeCounter){
     if(recordGait){
     cout<<"in here" << endl;
     fprintf(f,"%d,%d,%d,%d,%d,%d,%g;",conf.legContacts[0],conf.legContacts[2],conf.legContacts[4],conf.legContacts[1],conf.legContacts[3],conf.legContacts[5],t);
     fprintf(f,"\n");
     check = true;
     }else if(check == true && recordGait == false){
     fprintf(f,"]\n");
     fclose(f);
     check = false;
     }
     }*/

    return false;
  }
  
  /** creates vehicle at desired position 
   @param pos struct Position with desired position
   */
  void AmosII::create(const osg::Matrix& pose) {
    if (created) {
      destroy();
    }
#ifdef VERBOSE
    std::cout << "begin creation \n";
#endif
    //we want legs colliding with other legs, so we set internal collision flag to "false".
    odeHandle.createNewSimpleSpace(parentspace, false);
    // color of joint axis and whiskers
    OsgHandle osgHandleJ = osgHandle.changeColor(Color(72. / 255., 16. / 255., 16. / 255.));
    OneAxisServo* servo1;
    OneAxisServo* servo2;
    OneAxisServo* servo3;
    OneAxisServo* spring2;
    
    //get a representation of the origin
    Pos nullpos(0, 0, 0);
    
    /***************************************************************************
     *   CREATE BODY Metal and slip!
     *                                                                         *
     **************************************************************************/

    // create body
    double twidth = conf.width; // 1/1.5;
    double theight = conf.height; // 1/4;
    osg::Matrix trunkPos = TRANSM(0, 0, conf.legLength) * pose;
    
    std::string texture = "Images/toy_fur3.jpg";
    
    //**************Change Material substance*********//
    Substance bodySubstance(2.0, 0.1/*0.35 slip*/, 100.0, 0.4); //(roughness,slip,hardness,elasticity)
    OdeHandle bodyodeHandle = odeHandle;
    bodyodeHandle.substance.toMetal(3.0); // = bodySubstance;
    //**************Change Material substance*********//
    
    if (conf.useBack) {
      
      //**************Body Material substance*********//
      front = new Box(conf.frontLength, twidth, theight);
      front->setTexture(texture);
      front->init(bodyodeHandle, conf.frontMass, osgHandle); // Material substance!
      osg::Matrix frontPos = TRANSM(conf.size / 2 - conf.frontLength / 2, 0, 0) * trunkPos;
      front->setPose(frontPos);
      objects.push_back(front);
      //**************Body Material substance*********//
      
      center = new Box(conf.size - conf.frontLength, twidth, theight);
      center->setTexture(texture);
      center->init(bodyodeHandle, conf.trunkMass - conf.frontMass, osgHandle); // Material substance!
      osg::Matrix centerPos = TRANSM(-conf.size / 2 + (conf.size - conf.frontLength) / 2, 0, 0) * trunkPos;
      center->setPose(centerPos);
      objects.push_back(center);
      const Axis axis = Axis(0, 1, 0) * frontPos;
      /* create the joint from front to center part of trunk */
      HingeJoint* k = new HingeJoint(front, center, nullpos * TRANSM(-conf.frontLength / 2, 0, 0) * frontPos, axis);
      k->init(odeHandle, osgHandleJ, true, twidth * 1.05); // Material substance!
      joints.push_back(k);
      servo1 = new OneAxisServoVel(odeHandle, k, -1, 1, 1, 0.01, 0, 1.0); // parameters are set later
      backservo = servo1;
    } else {
      
      //In case no back joint!!
      trunk = new Box(conf.size, twidth, theight);
      trunk->setTexture(texture);
      trunk->init(bodyodeHandle, conf.trunkMass, osgHandle); // Material substance!
      trunk->setPose(trunkPos);
      objects.push_back(trunk);
      front = trunk;
      center = trunk;
    }
    
    // IR sensors at Front part
    irSensorBank.init(odeHandle, osgHandle);
    
    IRSensor* sensor1 = new IRSensor();
    irSensorBank.registerSensor(sensor1, front /*body part*/,
        Matrix::rotate(M_PI / 2, Vec3(conf.iranglex, conf.irangley, 0))
            * Matrix::translate(0.06/*0.06*/, -0.25 * twidth, -0.45 * theight), conf.irRangeFront * 0.05,
        RaySensor::drawRay);
    
    IRSensor* sensor2 = new IRSensor();
    irSensorBank.registerSensor(sensor2, front,
        Matrix::rotate(M_PI / 2, Vec3(conf.irparallel * conf.iranglex, conf.irangley, 0))
            * Matrix::translate(0.06, 0.25 * twidth, -0.45 * theight), conf.irRangeFront * 0.05, RaySensor::drawRay);
    
    //initialize the speedsensor
    if (conf.useLocalVelSensor) {
      speedsensor->init(front);
    }
    osg::Matrix m0 = pose;
    
    //    if(conf.irSensors == true){
    //      for(int i = -1; i < 2; i+=2){
    //
    //	irbox = new Box(0.1,0.1,0.1);
    //	irbox->setTexture(texture);
    //	irbox->init(odeHandle, 0.00001, osgHandle);
    //	irbox->setPose(ROTM(M_PI/4,0,0,1) * TRANSM(i/2,0,theight/2)*trunkPos);
    //	objects.push_back(irbox);
    //	fixedJoint = new FixedJoint(trunk,irbox);
    //	fixedJoint->init(odeHandle, osgHandleJ, true, 0.4);
    //	joints.push_back(fixedJoint);
    //      }
    //
    //      for(int i = -1; i < 2; i+=2){
    //
    //	irbox = new Box(0.1,0.1,0.15);
    //	irbox->setTexture(texture);
    //	irbox->init(odeHandle, 0.00001, osgHandle);
    //	irbox->setPose(TRANSM(0,i*twidth/2,theight/2 + 0.05)*trunkPos);
    //	objects.push_back(irbox);
    //	fixedJoint = new FixedJoint(trunk,irbox);
    //	fixedJoint->init(odeHandle, osgHandleJ, true, 0.4);
    //	joints.push_back(fixedJoint);
    //      }
    //
    //
    //      irSensorBank.init(odeHandle, osgHandle);
    //
    //      if (conf.irFront){ // add front left and front right infrared sensor to sensorbank if required
    //     	IRSensor* sensor = new IRSensor();
    //     	irSensorBank.registerSensor(sensor, objects[2],
    //     				    Matrix::rotate(-1*-M_PI/2, Vec3(0,1,0)) *
    //     				    Matrix::translate(1*0.05,0,0),
    //     				    conf.irRangeFront, RaySensor::drawAll);
    //       	IRSensor* sensor2 = new IRSensor();
    //       	irSensorBank.registerSensor(sensor2, objects[2],
    //				    Matrix::rotate(-1*-M_PI/2, Vec3(0,1,0)) *
    //				    Matrix::rotate(1*-M_PI/2, Vec3(0,0,1)) *
    //				    Matrix::translate(0,-0.05,0),
    //				    conf.irRangeFront, RaySensor::drawAll);
    //
    //      }
    //      if (conf.irBack){ // add front left and front right infrared sensor to sensorbank if required
    //
    //      	IRSensor* sensor = new IRSensor();
    //       	irSensorBank.registerSensor(sensor, objects[1],
    //       				    Matrix::rotate(1*-M_PI/2, Vec3(0,1,0)) *
    //       				    Matrix::translate(-1*0.05,0,0),
    //       				    conf.irRangeBack, RaySensor::drawAll);
    //
    //	IRSensor* sensor2 = new IRSensor();
    //	irSensorBank.registerSensor(sensor2, objects[1],
    //				    Matrix::rotate(1*-M_PI/2, Vec3(0,1,0)) *
    //				    Matrix::rotate(1*-M_PI/2, Vec3(0,0,1)) *
    //				    Matrix::translate(0,0.05,0),
    //				    conf.irRangeBack, RaySensor::drawAll);
    //      }
    //      if(conf.irLeft){
    //	IRSensor* sensor = new IRSensor();
    //	irSensorBank.registerSensor(sensor, objects[3],
    //				    Matrix::rotate(-1*-M_PI/2, Vec3(0,1,0)) *
    //				    Matrix::rotate(-M_PI/2, Vec3(0,0,1)) *
    //				    Matrix::translate(0,-0.05,0.05),
    //				    conf.irRangeLeft, RaySensor::drawAll);
    //
    //    	/* IRSensor* sensor2 = new IRSensor();
    //	   irSensorBank.registerSensor(sensor2, objects[3],
    //	   Matrix::rotate(-1*-M_PI/2, Vec3(0,1,0)) *
    //	   Matrix::rotate(1*-M_PI/2, Vec3(0,0,1)) *
    //	   Matrix::translate(0,-0.05,0),
    //	   conf.irRangeLeft, RaySensor::drawAll);*/
    //      }
    //      if(conf.irRight){
    //	IRSensor* sensor = new IRSensor();
    //	irSensorBank.registerSensor(sensor, objects[4],
    //				    Matrix::rotate(-1*-M_PI/2, Vec3(0,1,0)) *
    //				    Matrix::rotate(M_PI/2, Vec3(0,0,1)) *
    //				    Matrix::translate(0,0.05,0.05),
    //				    conf.irRangeLeft, RaySensor::drawAll);
    //
    //
    //	/* IRSensor* sensor2 = new IRSensor();
    //	   irSensorBank.registerSensor(sensor2, objects[4],
    //	   Matrix::rotate(1*-M_PI/2, Vec3(0,1,0)) *
    //	   Matrix::rotate(1*-M_PI/2, Vec3(0,0,1)) *
    //	   Matrix::translate(0,0.05,0),
    //	   conf.irRangeRight, RaySensor::drawAll);*/
    //      }
    //    }
    
    // legs  (counted from back to front)
    
    osg::Matrix legtrunkconnections[6];
    osg::Matrix shouldertrunkconn[6];
    
    double l0 = conf.shoulderLength;
    double t0 = conf.shoulderRadius;
    double l1 = conf.coxaLength;
    double t1 = conf.coxaRadius;
    double l2 = conf.secondLength;
    double t2 = conf.secondRadius;
    double l3 = conf.tebiaLength - conf.tebiaRadius - conf.useFoot * (conf.tebiaRadius + conf.footRange);
    double t3 = conf.tebiaRadius;
    double l4 = 2 * conf.tebiaRadius + conf.footRange - conf.footRadius;
    double t4 = conf.footRadius;
    
    for (int n = 0; n < 6; n++) {
      
      int v = n;
      
      //create 3d-coordinates for the leg-trunk connection:
      Pos pos = Pos(
          -conf.size * 16.5 / 43.0 + ((int) n / 2) * conf.legdist1 - ((int) n / 4) * (conf.legdist1 - conf.legdist2), //from (0,0,0) we go down x-axis, make two legs then up legdist1 and so on
          n % 2 == 0 ? -twidth / 2 : twidth / 2, //switch left or right side of trunk for each leg
          conf.legLength - theight / 2 + conf.shoulderHeight); // height of leg fixation to trunk (trunk bottom sits at totallegLength)
              
      /*get a coordinate system at the position pos by rotating such that z-axis points toward trunk,
       * pose is where the robot will be placed so we begin there.
       */
      legtrunkconnections[n] = ROTM(M_PI / 2, v % 2 == 0 ? -1 : 1, 0, 0) * TRANSM(pos) * pose;
      /*we create a transformation matrix that represents the transformation from the trunk center to
       * the trunk-shoulder connections. we need it because we need the coordinates
       * relative to the trunk to create one body including the shoulders*/
      shouldertrunkconn[n] = ROTM(M_PI / 2, v % 2 == 0 ? -1 : 1, 0, 0) * TRANSM(0, 0, -conf.legLength) * TRANSM(pos);
      
      if (conf.useBack) {
        if (n != 4 && n != 5) {
          shouldertrunkconn[n] = ROTM(M_PI / 2, v % 2 == 0 ? -1 : 1, 0, 0)
              * TRANSM(conf.frontLength / 2, 0, -conf.legLength) * TRANSM(pos);
        } else {
          shouldertrunkconn[n] = ROTM(M_PI / 2, v % 2 == 0 ? -1 : 1, 0, 0)
              * TRANSM(-(conf.size - conf.frontLength) / 2, 0, -conf.legLength) * TRANSM(pos);
        }
      }
      
    }
    
    //if wanted, leg trunk connections are rotated here:
    legtrunkconnections[0] = ROTM(conf.rLegRotAngle, 0, 0, 1) * ROTM(conf.rLegTrunkAngleH, 1, 0, 0)
        * ROTM(conf.rLegTrunkAngleV, 0, 1, 0) * legtrunkconnections[0];
    legtrunkconnections[1] = ROTM(conf.rLegRotAngle, 0, 0, -1) * ROTM(conf.rLegTrunkAngleH, -1, 0, 0)
        * ROTM(conf.rLegTrunkAngleV, 0, 1, 0) * legtrunkconnections[1];
    legtrunkconnections[2] = ROTM(conf.mLegRotAngle, 0, 0, 1) * ROTM(conf.mLegTrunkAngleH, 1, 0, 0)
        * ROTM(conf.mLegTrunkAngleV, 0, 1, 0) * legtrunkconnections[2];
    legtrunkconnections[3] = ROTM(conf.mLegRotAngle, 0, 0, -1) * ROTM(conf.mLegTrunkAngleH, -1, 0, 0)
        * ROTM(conf.mLegTrunkAngleV, 0, 1, 0) * legtrunkconnections[3];
    legtrunkconnections[4] = ROTM(conf.fLegRotAngle, 0, 0, 1) * ROTM(conf.fLegTrunkAngleH, 1, 0, 0)
        * ROTM(conf.fLegTrunkAngleV, 0, 1, 0) * legtrunkconnections[4];
    legtrunkconnections[5] = ROTM(conf.fLegRotAngle, 0, 0, -1) * ROTM(conf.fLegTrunkAngleH, -1, 0, 0)
        * ROTM(conf.fLegTrunkAngleV, 0, 1, 0) * legtrunkconnections[5];
    
    //also the relative coordinates for the shoulders
    shouldertrunkconn[0] = ROTM(conf.rLegRotAngle, 0, 0, 1) * ROTM(conf.rLegTrunkAngleH, 1, 0, 0)
        * ROTM(conf.rLegTrunkAngleV, 0, 1, 0) * shouldertrunkconn[0];
    shouldertrunkconn[1] = ROTM(conf.rLegRotAngle, 0, 0, -1) * ROTM(conf.rLegTrunkAngleH, -1, 0, 0)
        * ROTM(conf.rLegTrunkAngleV, 0, 1, 0) * shouldertrunkconn[1];
    shouldertrunkconn[2] = ROTM(conf.mLegRotAngle, 0, 0, 1) * ROTM(conf.mLegTrunkAngleH, 1, 0, 0)
        * ROTM(conf.mLegTrunkAngleV, 0, 1, 0) * shouldertrunkconn[2];
    shouldertrunkconn[3] = ROTM(conf.mLegRotAngle, 0, 0, -1) * ROTM(conf.mLegTrunkAngleH, -1, 0, 0)
        * ROTM(conf.mLegTrunkAngleV, 0, 1, 0) * shouldertrunkconn[3];
    shouldertrunkconn[4] = ROTM(conf.fLegRotAngle, 0, 0, 1) * ROTM(conf.fLegTrunkAngleH, 1, 0, 0)
        * ROTM(conf.fLegTrunkAngleV, 0, 1, 0) * shouldertrunkconn[4];
    shouldertrunkconn[5] = ROTM(conf.fLegRotAngle, 0, 0, -1) * ROTM(conf.fLegTrunkAngleH, -1, 0, 0)
        * ROTM(conf.fLegTrunkAngleV, 0, 1, 0) * shouldertrunkconn[5];
    
    /***************************************************************************
     *   CREATE LEGS
     *                                                                         *
     **************************************************************************/
    //create the legs
    for (int n = 0; n < 6; n++) {
      
      int v = n;
      
      //get a representation of the origin
      Pos nullpos(0, 0, 0);
      
      osg::Matrix c1;
      /*m0 is the position where the center of mass of the zeroth limb capsule is placed */
      osg::Matrix m0;
      
      if (conf.useShoulder) {
        //shift connection of coxa outward
        c1 = TRANSM(0, 0, -l0) * legtrunkconnections[n];
        //create shoulder
        shoulder[n].should = new Capsule(t0, l0);
        shoulder[n].should->setTexture(texture);
        //add shoulder to trunk body
        //the shoulder"s pose has to be given relative to the trunk's pose
        //add the first four shoulders to center the other two to front
        shoulder[n].trans = new Transform(v < 4 ? center : front, shoulder[n].should,
            TRANSM(0, 0, -l0 / 2) * shouldertrunkconn[n]);
        shoulder[n].trans->init(odeHandle, conf.shoulderMass, osgHandle);
        
      } else {
        //first limb data
        c1 = legtrunkconnections[n];
      }
      
      /*m1 is the position where the center of mass of the first limb capsule is placed */
      osg::Matrix m1 = TRANSM(0, 0, -l1 / 2) * ROTM(conf.fcoxaZero, 0, 1, 0) * c1;
      
      //calculate anchor of the first joint
      const osg::Vec3 anchor1 = nullpos * c1;
      //and it's axis (multiplication with c1 indicates in which (local) coordinate system it is)
      const Axis axis1 = Axis(0, 1, 0) * c1;
      
      //proceed along the leg (and the respective z-axis) for second limb
      osg::Matrix c2 = TRANSM(0, 0, -l1 / 2) * m1;
      osg::Matrix m2 = TRANSM(0, 0, -l2 / 2) * ROTM(conf.fsecondZero, v % 2 == 0 ? 1 : -1, 0, 0) * c2;
      const osg::Vec3 anchor2 = nullpos * c2;
      const Axis axis2 = Axis(v % 2 == 0 ? 1 : -1, 0, 0) * c2;
      
      //and third
      osg::Matrix c3 = TRANSM(0, 0, -l2 / 2) * m2;
      osg::Matrix m3 = TRANSM(0, 0, -l3 / 2) * ROTM(conf.ftebiaZero, v % 2 == 0 ? 1 : -1, 0, 0) * c3;
      const osg::Vec3 anchor3 = nullpos * c3;
      const Axis axis3 = Axis(v % 2 == 0 ? 1 : -1, 0, 0) * c3;
      
      //		  std::cout << "leg number (n): " << n << "leg number (v): " << v << endl << "pos: \n";
      //		  pos.print();
      //		  std::cout << "\n anchor1: \n";
      //		  Pos check = nullpos * c1;
      //		  check.print();
      //		  std::cout << "\n";
      //		  Pos check1 = nullpos * c2;
      //		  std::cout << "\n anchor2: \n";
      //		  check1.print();
      //		  std::cout << "\n";
      
      //now create first limp
      Primitive* coxaThorax;
      /*create upper limp with radius t1 and length l1 (length refers only to length of
       * the cylinder without the semispheres at both ends) */
      coxaThorax = new Capsule(t1, l1);
      coxaThorax->setTexture(texture);
      coxaThorax->init(odeHandle, conf.coxaMass, osgHandle);
      //put it at m1
      coxaThorax->setPose(m1);
      //		  coxaPos.push_back(coxaThorax->getPosition());
      coxas.push_back(coxaThorax);
      objects.push_back(coxaThorax);
      legs.push_back(coxaThorax);
      if (conf.useShoulder) {
        odeHandle.addIgnoredPair(shoulder[n].trans, coxaThorax);
        ignoredPairs.push_back(shoulder[n].trans);
        ignoredPairs.push_back(coxaThorax);
      }
      // powered hip joint of trunk to first limb
      HingeJoint* j = new HingeJoint(v < 4 ? center : front, coxaThorax, anchor1, -axis1);
      j->init(odeHandle, osgHandleJ, true, t1 * 2.1);
      joints.push_back(j);
      //angles of joints in legcontactarray are stored during doInternalStuff
      legContactArray[n].coxajoint = j->getJoint();
      //create motor, overwrite the jointLimit argument with 1.0 because
      //it is less obscure and setMinMax makes mistakes otherwise
      servo1 = new OneAxisServoVel(odeHandle, j, -1, 1, 1, 0.01, 0, 1.0); // parameters are set later
      //PUSH THIS STUFF BACK INTO STH!!!! not hipservos obviously
      coxaservos.push_back(servo1);
      
      // second limb
      Primitive* secondThorax;
      secondThorax = new Capsule(t2, l2);
      secondThorax->setTexture(texture);
      secondThorax->init(odeHandle, conf.secondMass, osgHandle);
      secondThorax->setPose(m2);
      //		  secondPos.push_back(secondThorax->getPosition());
      seconds.push_back(secondThorax);
      objects.push_back(secondThorax);
      legs.push_back(secondThorax);
      
      /* create the joint from first to second limb (coxa to second) */
      HingeJoint* k = new HingeJoint(coxaThorax, secondThorax, anchor2, -axis2);
      k->init(odeHandle, osgHandleJ, true, t1 * 2.1);
      joints.push_back(k);
      legContactArray[n].secondjoint = k->getJoint();
      servo2 = new OneAxisServoVel(odeHandle, k, -1, 1, 1, 0.01, 0, 1.0); // parameters are set later
      //PUSH THIS STUFF BACK INTO STH!!!! not hipservos obviously
      secondservos.push_back(servo2);
      /* set joint limits (via the servo) */

      // third limb
      Primitive* tebia;
      tebia = new Capsule(t3, l3);
      tebia->setTexture(texture);
      tebia->init(odeHandle, conf.tebiaMass, osgHandle);
      tebia->setPose(m3);
      //		  tebiaPos.push_back(tebia->getPosition());
      tebias.push_back(tebia);
      objects.push_back(tebia);
      legs.push_back(tebia);
      
      // IR sensor at each leg
      IRSensor* sensor = new IRSensor();
      irSensorBank.registerSensor(sensor, tebia,
          Matrix::rotate(-1 * -M_PI / 2, Vec3(0, 1, 0)) * Matrix::translate(0, 0, -0.2 * conf.tebiaLength /*Position*/),
          conf.irRangeFront * 0.035 /*ir range = 3*0.035*/, RaySensor::drawRay);
      
      //for the geom in legContactArray collisions are processed by collisioncallback above
      //and used for a contact sensor! for this sensor
      //also in doInternal Stuff, the Body's position is retrieved and used to calculate "heights"
      
      legContactArray[n].legID = n;
      if (!conf.useFoot) {
        legContactArray[n].geomid = tebia->getGeom();
        legContactArray[n].bodyID = tebia->getBody();
      }
      //		  if(conf.useTebiaJoints){
      // springy knee joint
      HingeJoint* l = new HingeJoint(secondThorax, tebia, anchor3, -axis3);
      l->init(odeHandle, osgHandleJ, true, t3 * 2.1);
      joints.push_back(l);
      legContactArray[n].tebiajoint = l->getJoint();
      // servo used as a spring
      
      //			servo3 = new OneAxisServoCentered(l, -1, 1, 1, 0.2,2,10.0,1.0); // parameters are set later
      servo3 = new OneAxisServoVel(odeHandle, l, -1, 1, 1, 0.01, 0, 1.0); // parameters are set later
      tebiaservos.push_back(servo3);
      
      //		  }else{
      //			// fixed knee joint
      //			FixedJoint* l = new FixedJoint(secondThorax, tebia);
      //			l->init(odeHandle, osgHandleJ, false, t3 * 2.1);
      //			joints.push_back(l);
      //			legContactArray[n].tebiajoint = l->getJoint();
      //		  }
      
      /***************************************************************************
       *   CREATE FOOT rubber foot and hard!
       *                                                                         *
       **************************************************************************/
      //spring foot at the end
      if (conf.useFoot) {
        osg::Matrix c4 = TRANSM(0, 0, -l3 / 2 - 2 * conf.tebiaRadius - conf.footRange + conf.footRadius) * m3;
        //			  osg::Matrix m4 = TRANSM(0,0,-l3/2-l4/2-conf.footSpringPreload) * m3;
        osg::Matrix m4 = TRANSM(0, 0, -conf.footSpringPreload) * c4;
        const osg::Vec3 anchor4 = nullpos * m4;
        const Axis axis4 = Axis(0, 0, -1) * c4;
        
        OdeHandle my_odeHandle = odeHandle;
        if (conf.rubberFeet) {
          /*				    roughness  = 3;
           * 				    slip       = 0.0;
           hardness   = _hardness;
           elasticity = 0.95;
           */
          /*				  Substance::Substance( float roughness, float slip, float hardness, float elasticity)*/
          Substance FootSubstance(3.0, 0.0, 500.0, 0.1);
          my_odeHandle.substance = FootSubstance; // 5
          //my_odeHandle.substance.toRubber(5); //KOH change (5)
        }
        
        Primitive* foot;
        foot = new Capsule(t4, l4);
        foot->setTexture(texture);
        foot->init(my_odeHandle, conf.footMass, osgHandle);
        foot->setPose(m4);
        //			  footPos.push_back(foot->getPosition());
        feet.push_back(foot);
        objects.push_back(foot);
        //the foot is not used for energy calculation
        //legs.push_back(foot);
        
        SliderJoint* m = new SliderJoint(tebia, foot, anchor4, axis4);
        m->init(odeHandle, osgHandleJ, true, t3, true);
        joints.push_back(m);
        
        //			  spring2 = new SliderServo(m, -1, 1, 3, 0.1,0,10,1.0); // parameters are set later
        spring2 = new Spring(m, -1, 1, 1);
        footsprings.push_back(spring2);
        odeHandle.addIgnoredPair(secondThorax, foot);
        ignoredPairs.push_back(secondThorax);
        ignoredPairs.push_back(foot);
        //			  odeHandle.addIgnoredPair(coxaThorax,foot);
        //			  ignoredPairs.push_back(coxaThorax);
        //			  ignoredPairs.push_back(foot);
        //if feet are used they should be used for collision detection
        //so overwrite the previous geom and body of the tebia
        legContactArray[n].geomid = foot->getGeom();
        legContactArray[n].bodyID = foot->getBody();

        //----------- Add contact sensor ---- by Ren ----------------------------------
        ContactSens[n] = new ContactSensor(false, 250, 1.01 * t4);
        ContactSens[n]->init(odeHandle, osgHandle, foot, true, TRANSM(0, 0, -0.5 * l4));
        odeHandle.addIgnoredPair(tebia, ContactSens[n]->getTransformObject());
      }
      
      //		  cout << "hello \n";
    }
    
    if (conf.useWhiskers) {
      // New: wiskers
      for (int n = -1; n < 2; n += 2) {
        double l1 = conf.whiskerLength * 0.5;
        double t1 = conf.whiskerLength / 30;
        
        Primitive* whisker;
        Pos pos = Pos(conf.size / (2) + t1, n * twidth / 4,
        /*2*conf.tibiaLength*/conf.legLength + theight / 5);
        
        osg::Matrix m = ROTM(M_PI / 10, n, 0, 0) * ROTM(M_PI / 2 + M_PI / 10, 0, -1, 0) * TRANSM(pos) * pose;
        whisker = new Capsule(t1, l1);
        whisker->init(odeHandle, conf.shoulderMass / 10, osgHandleJ);
        osg::Matrix m1 = TRANSM(0, 0, -l1 / 2) * m;
        whisker->setPose(m1);
        objects.push_back(whisker);
        
        //FixedJoint* k = new FixedJoint(trunk, whisker);
        //k->init(odeHandle, osgHandle, false, 0);
        HingeJoint* k = new HingeJoint(front, whisker, Pos(0, 0, 0) * m, Axis(1, 0, 0) * m);
        k->init(odeHandle, osgHandleJ, true, t1 * 2.1);
        // servo used as a spring
        servo3 = new HingeServo(k, -M_PI / 6, M_PI / 6, .1, 0.01, 0);
        whiskersprings.push_back(servo3);
        joints.push_back(k);
        
        Primitive* whisker2;
        whisker2 = new Capsule(t1 / 2, l1);
        whisker2->init(odeHandle, conf.shoulderMass / 10, osgHandleJ);
        osg::Matrix m2 = TRANSM(0, 0, -l1 / 2) * ROTM(M_PI / 10, n, 0, 0) * ROTM(M_PI / 10, 0, 1, 0)
            * TRANSM(0, 0, -l1 / 2) * m1;
        whisker2->setPose(m2);
        objects.push_back(whisker2);
        
        //      k = new FixedJoint(whisker, whisker2);
        //      k->init(odeHandle, osgHandleJ, false, 0);
        k = new HingeJoint(whisker, whisker2, Pos(0, 0, -l1 / 2) * m1, Axis(0, 1, 0) * m1);
        k->init(odeHandle, osgHandleJ, true, t1 * 2.1);
        // servo used as a spring
        servo3 = new HingeServo(k, -M_PI / 6, M_PI / 6, .05, 0.01, 0);
        whiskersprings.push_back(servo3);
        joints.push_back(k);
        
      }
    }
    
    setParam("dummy", 0); // apply all parameters.
        
    created = true;
#ifdef VERBOSE
    std::cout << "end of creation \n";
#endif
  }
  ;
  
  /** destroys vehicle and space
   */
  void AmosII::destroy() {
    if (created) {
#ifdef VERBOSE
      std::cout << "begin destruction \n";
#endif
      //  odeHandle.removeIgnoredPair(bigboxtransform);
//      odeHandle.removeIgnoredPair(front,headtrans);
//      odeHandle.removeIgnoredPair(center,headtrans);
//      these need to be removed probably, make a vector of the according geoms and then...
      for (unsigned int i = 0; i < (sizeof(ignoredPairs) / 2); i++) {
        int a = 2 * i;
        int b = 2 * i + 1;
        odeHandle.removeIgnoredPair(ignoredPairs[a], ignoredPairs[b]);
      }
      ignoredPairs.clear();
      
      //does it work?
      for (vector<Primitive*>::iterator i = objects.begin(); i != objects.end(); i++) {
        for (vector<Primitive*>::iterator j = objects.begin(); j != objects.end(); j++) {
          if (odeHandle.isIgnoredPair((*i)->getGeom(), (*j)->getGeom()))
            odeHandle.removeIgnoredPair((*i)->getGeom(), (*j)->getGeom());
        }
        
      }
      
      irSensorBank.clear();
      
//      FOREACH(vector<TwoAxisServo*>, hipservos, i){
//        if(*i) delete *i;
//      }
//      hipservos.clear();
      
      if (conf.useBack) {
        delete backservo;
//    	  backservo.clear();
      }
//      FOREACH(std::vector<Primitive*>, seconds, i){
//          FOREACH(std::vector<Primitive*>, feet, j){
//           if(*i) odeHandle.removeIgnoredPair(*i,*j);
//          }
//      }
      FOREACH(vector<OneAxisServo*>, coxaservos, i) {
        if (*i)
          delete *i;
      }
      coxaservos.clear();
      FOREACH(vector<OneAxisServo*>, secondservos, i) {
        if (*i)
          delete *i;
      }
      secondservos.clear();
      FOREACH(vector<OneAxisServo*>, tebiaservos, i) {
        if (*i)
          delete *i;
      }
      tebiaservos.clear();
      FOREACH(vector<OneAxisServo*>, footsprings, i) {
        if (*i)
          delete *i;
      }
      footsprings.clear();
      FOREACH(vector<OneAxisServo*>, whiskersprings, i) {
        if (*i)
          delete *i;
      }
      whiskersprings.clear();
      
      for (vector<Joint*>::iterator i = joints.begin(); i != joints.end(); i++) {
        if (*i)
          delete *i;
      }
      joints.clear();
      
      for (int i = 0; i < 6; i++) {
        //	if(shoulder[i].should) delete shoulder[i].should; is done by transform primitive
        if (shoulder[i].trans)
          delete shoulder[i].trans;
      }
      
      for (vector<Primitive*>::iterator i = objects.begin(); i != objects.end(); i++) {
        if (*i)
          delete *i;
      }
      objects.clear();
      //should all be empty as objects were cleared:
      legs.clear();
      coxas.clear();
      seconds.clear();
      tebias.clear();
      feet.clear();
      
      delete[] pos1d;
      delete massOfobject;
      delete speedsensor;
      delete[] energyOneStep;
      delete[] dones;
      delete[] angles;
      delete[] heights;
      delete[] legContactArray;
      
      //----------- Remove contact sensor ---- by Ren
      delete[] ContactSens;

      odeHandle.deleteSpace();
#ifdef VERBOSE
      std::cout << "end of destruction \n";
#endif
    }
    
    created = false;
  }
  
  bool AmosII::setParam(const paramkey& key, paramval val) {
    // the parameters are assigned here
    bool rv = Configurable::setParam(key, val);
    // we simply set all parameters here
    
    if (conf.useBack) {
      backservo->setPower(conf.backPower);
      backservo->setDamping(conf.backDamping);
      backservo->setMaxVel(conf.backMaxVel);
      backservo->setMinMax(conf.backJointLimitU, conf.backJointLimitD);
    }
    FOREACH(vector<OneAxisServo*>, coxaservos, i) {
      if (*i) {
        (*i)->setPower(conf.coxaPower);
        (*i)->setDamping(conf.coxaDamping);
        (*i)->setMaxVel(conf.coxaMaxVel);
      }
    }
    
    coxaservos[0]->setMinMax(conf.rcoxaJointLimitF, conf.rcoxaJointLimitB); //max is forward, forward is pos.
    coxaservos[1]->setMinMax(conf.rcoxaJointLimitF, conf.rcoxaJointLimitB);
    coxaservos[2]->setMinMax(conf.mcoxaJointLimitF, conf.mcoxaJointLimitB);
    coxaservos[3]->setMinMax(conf.mcoxaJointLimitF, conf.mcoxaJointLimitB);
    coxaservos[4]->setMinMax(conf.fcoxaJointLimitF, conf.fcoxaJointLimitB);
    coxaservos[5]->setMinMax(conf.fcoxaJointLimitF, conf.fcoxaJointLimitB);
    
    FOREACH(vector<OneAxisServo*>, secondservos, i) {
      if (*i) {
        (*i)->setPower(conf.secondPower);
        (*i)->setDamping(conf.secondDamping);
        (*i)->setMaxVel(conf.secondMaxVel);
        (*i)->setMinMax(conf.secondJointLimitU, conf.secondJointLimitD); //yes, min is up, up is negative
      }
    }
    FOREACH(vector<OneAxisServo*>, tebiaservos, i) {
      if (*i) {
        (*i)->setPower(conf.tebiaPower);
        (*i)->setDamping(conf.tebiaDamping);
        (*i)->setMaxVel(conf.tebiaMaxVel);
        (*i)->setMinMax(conf.tebiaJointLimitU, conf.tebiaJointLimitD); //yes, min is up, up is negative
      }
    }
    FOREACH(vector<OneAxisServo*>, footsprings, i) {
      if (*i) {
        (*i)->setPower(conf.footPower);
        (*i)->setDamping(conf.footDamping);
        (*i)->setMaxVel(conf.footMaxVel);
        (*i)->setMinMax(conf.footSpringLimitD, conf.footSpringLimitU); //yes, min is up, up is negative
      }
    }
    return rv;
    
  }

}
