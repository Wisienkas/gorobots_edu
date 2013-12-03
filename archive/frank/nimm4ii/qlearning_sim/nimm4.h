/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
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
 *   $Log: nimm4.h,v $
 *   Revision 1.10  2009/05/11 17:03:07  martius
 *   minor substance change
 *
 *   Revision 1.9  2008/05/07 16:45:52  martius
 *   code cosmetics and documentation
 *
 *   Revision 1.8  2007/11/07 13:21:16  martius
 *   doInternal stuff changed signature
 *
 *   Revision 1.7  2007/09/06 18:48:00  martius
 *   createNewSimpleSpace used
 *
 *   Revision 1.6  2006/07/14 12:23:41  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.5.4.13  2006/06/25 16:57:14  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.5.4.12  2006/04/04 14:13:24  fhesse
 *   documentation improved
 *
 *   Revision 1.5.4.11  2006/03/31 16:20:28  fhesse
 *   class Joint; changed to: class Hinge2Joint;
 *
 *   Revision 1.5.4.10  2006/03/30 12:34:56  martius
 *   documentation updated
 *
 *   Revision 1.5.4.9  2005/12/15 17:04:08  martius
 *   Primitives are not longer inherited from OSGPrimitive, moreover
 *   they aggregate them.
 *   Joint have better getter and setter
 *
 *   Revision 1.5.4.8  2005/12/14 15:37:09  martius
 *   robots are working with osg
 *
 *   Revision 1.5.4.7  2005/12/13 18:11:40  martius
 *   still trying to port robots
 *
 *   Revision 1.5.4.6  2005/12/12 23:41:19  martius
 *   added Joint wrapper
 *
 *   Revision 1.5.4.5  2005/12/11 23:35:08  martius
 *   *** empty log message ***
 *
 *   Revision 1.5.4.4  2005/12/06 10:13:25  martius
 *   openscenegraph integration started
 *
 *   Revision 1.5.4.3  2005/11/16 11:26:52  martius
 *   moved to selforg
 *
 *   Revision 1.5.4.2  2005/11/15 12:29:26  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.5.4.1  2005/11/14 17:37:18  martius
 *   moved to selforg
 *
 *   Revision 1.5  2005/10/27 16:10:41  fhesse
 *   nimm4 as example
 *
 *   Revision 1.4  2005/09/22 12:24:37  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.3  2005/08/31 11:14:06  martius
 *   removed unused vars
 *
 *   Revision 1.2  2005/08/03 20:38:56  martius
 *   added textures and correct placement
 *
 *   Revision 1.1  2005/07/29 15:13:11  martius
 *   a robot with 4 independent wheels
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __NIMM4_H
#define __NIMM4_H

#include <ode_robots/oderobot.h>
#include <ode_robots/relativepositionsensor.h>

namespace lpzrobots {

  class Primitive; 
  class Hinge2Joint; 


  struct Nimm4Conf{
  	std::vector<Primitive*> rpos_sensor_references;
  };

  /** Robot that looks like a Nimm 2 Bonbon :-)
      4 wheels and a capsule like body   
  */
  class Nimm4 : public OdeRobot{
  public:
  
    /**
     * constructor of nimm4 robot
     * @param odeHandle data structure for accessing ODE
     * @param osgHandle ata structure for accessing OSG
     * @param size scaling of robot
     * @param force maximal used force to realize motorcommand
     * @param speed factor for changing speed of robot
     * @param sphereWheels switches between spheres and  'normal' wheels 
     */
    Nimm4(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const std::string& name,
	  double size=1, double force=3, double speed=15, bool sphereWheels=true, Nimm4Conf conf = getDefaultConf());

    virtual ~Nimm4(){
      destroy();
    };

    /**
     * updates the OSG nodes of the vehicle
     */
    virtual void update();


    /** sets the pose of the vehicle
	@param pose desired pose matrix
    */
    virtual void place(const osg::Matrix& pose);

    /** returns actual sensorvalues
	@param sensors sensors scaled to [-1,1] 
	@param sensornumber length of the sensor array
	@return number of actually written sensors
    */
    virtual int getSensors(sensor* sensors, int sensornumber);

    /** sets actual motorcommands
	@param motors motors scaled to [-1,1] 
	@param motornumber length of the motor array
    */
    virtual void setMotors(const motor* motors, int motornumber);

    /** returns number of sensors
     */
    virtual int getSensorNumber(){
      return sensorno;
    };

    /** returns number of motors
     */
    virtual int getMotorNumber(){
      return motorno;
    };

    /** this function is called in each timestep. It should perform robot-internal checks, 
	like space-internal collision detection, sensor resets/update etc.
	@param globalData structure that contains global data from the simulation environment
    */
    virtual void doInternalStuff(GlobalData& globalData);

    /** setting the color of the body, must be called before place()/create() */
    // when plotting hte track insimulation it has the same color as the main body
    virtual void setBodyColor(Color c){ this->osgHandle.color=c;};
    /** setting the color of the wheels, must be called before place()/create() */
    virtual void setWheelColor(Color c){ wcolor = c; };
    /** setting the texture of the body, must be called before place()/create() */
    virtual void setBodyTexture(std::string t){ btexture = t; };
    /** setting the texture of the body, must be called before place()/create() */
    virtual void setWheelTexture(std::string t){ wtexture = t; };

/*
    // set reference primitive for Relative position sensor and activate Rel.Pos.Sensing
    // TODO: must be called before init; maybe put the activation in a Nimm4Conf to solve this
    virtual void setRelPosSensorReference(Primitive* ref){
    	assert (!created);
    	rpos_sensor->setReference(ref);
    	rpos_sensing_active = true;
    	sensorno+=rpos_sensor->getSensorNumber();
    };
*/

    static Nimm4Conf getDefaultConf(){
    	Nimm4Conf conf;
    	conf.rpos_sensor_references.clear(); //enforce empty vector -> no relative position sensing
    	return conf;
    };

  protected:
    /** the main object of the robot, which is used for position and speed tracking */
    virtual Primitive* getMainPrimitive() const { return object[0]; }

    /** creates vehicle at desired pose
	@param pose 4x4 pose matrix
    */
    virtual void create(const osg::Matrix& pose); 

    /** destroys vehicle and space
     */
    virtual void destroy();

    /** additional things for collision handling can be done here
     */
    static void mycallback(void *data, dGeomID o1, dGeomID o2);

    double length;     // chassis length
    double width;      // chassis width
    double height;     // chassis height
    double radius;     // wheel radius
    double wheelthickness; // thickness of the wheels  
    bool sphereWheels; // draw spherical wheels?
    double cmass;      // chassis mass
    double wmass;      // wheel mass
    int sensorno;      // number of sensors
    int motorno;       // number of motors
    int segmentsno;    // number of motorsvehicle segments
    double speed;      // factor for adjusting speed of robot

    double max_force;  // maximal force for motors

    bool created;      // true if robot was created

    Substance wheelsubstance; // material of wheel

    Primitive* object[5];  // 1 capsule, 4 wheels
    Hinge2Joint* joint[4]; // joints between cylinder and each wheel

    Color bcolor, wcolor; // body and wheel color
    std::string btexture, wtexture; // body and wheel texture


    std::vector<RelativePositionSensor> rpos_sensor;  // Relative position sensors
    bool rpos_sensing_active; //

  };

}

#endif
