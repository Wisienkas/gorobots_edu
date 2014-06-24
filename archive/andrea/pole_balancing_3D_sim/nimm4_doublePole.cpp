/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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
 ***************************************************************************/
#include <assert.h>
#include <ode-dbl/ode.h>

// include primitives (box, spheres, cylinders ...)
#include "ode_robots/primitive.h"
#include "ode_robots/osgprimitive.h"

// include joints
#include "ode_robots/joint.h"

// include header file
#include "nimm4_doublePole.h"
#include "newACICO.h"
#include "stdlib.h"
using namespace osg;


namespace lpzrobots {

// constructor:
// - give handle for ODE and OSG stuff
// - size of robot, maximal used force and speed factor are adjustable
// - sphereWheels switches between spheres or wheels as wheels
//   (wheels are only drawn, collision handling is always with spheres)
nimm4_doublePole::nimm4_doublePole(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
		const std::string& name,
		double size/*=1.0*/, double force /*=3*/, double speed/*=15*/,
		bool sphereWheels /*=true*/)
: // calling OdeRobots constructor with name of the actual robot
    						  OdeRobot(odeHandle, osgHandle, name, "$Id$")
{
	// robot is not created till now
	created=false;

	// choose color (here the color of the "Nimm Zwei" candy is used,
	// where the name of the Nimm2 and Nimm4 robots comes from ;-)
	this->osgHandle.color = Color(2, 156/255.0, 0, 1.0f);

	// maximal used force is calculated from the force factor and size given to the constructor
	max_force   = force*size*size/2;

	// speed and type of wheels are set
	this->speed = speed;
	this->sphereWheels = sphereWheels;

	height=size;
	length=size*4; // length of body
	width=size/2;  // diameter of body
	radius=size/6; // wheel radius
	wheelthickness=size/20; // thickness of the wheels (if wheels used, no spheres)
	cmass=8*size;  // mass of the body
	wmass=size;    // mass of the wheels
	sensorno=5;  //6;    // number of sensors
	motorno=4;     // number of motors
	segmentsno=6;//7;  // number of segments of the robot //change here
	//added
	firstPoleLength = 1.5;
	secondPoleLength = 3;
	wheelsubstance.toRubber(50);

};


/** sets actual motorcommands
      @param motors motors scaled to [-1,1] 
      @param motornumber length of the motor array
 */
void nimm4_doublePole::setMotorsIntern(const motor* motors, int motornumber){
	assert(created); // robot must exist
	// the number of controlled motors is minimum of
	// "number of motorcommands" (motornumber) and
	// "number of motors inside the robot" (motorno)
	int len = (motornumber < motorno)? motornumber : motorno;

	// for each motor the motorcommand (between -1 and 1) multiplied with speed
	// is set and the maximal force to realize this command are set
	for (int i=0; i<len; i++){
		joints[i]->setParam(dParamVel2, motors[0]*speed);
		joints[i]->setParam(dParamFMax2, max_force);
	}
};

/** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
 */
int nimm4_doublePole::getSensorsIntern(sensor* sensors, int sensornumber){
	assert(created); // robot must exist

	// the number of sensors to read is the minimum of
	// "number of sensors requested" (sensornumber) and
	// "number of sensors inside the robot" (sensorno)
	int len = (sensornumber < sensorno)? sensornumber : sensorno;
	//std::cout<<"Sensors, Technical:  ";
	// for each sensor the anglerate of the joint is red and scaled with 1/speed
	for (int i=0; i<len; i++){
		if (i <= 3)
		{
			sensors[i]=dynamic_cast<Hinge2Joint*>(joints[i])->getPosition2Rate();
			sensors[i]/=speed;  //scaling
		}
		else
			sensors[i]=dynamic_cast<HingeJoint*>(joints[i])->getPosition1();

		//std::cout << sensors[i]  << "\t";

	}
	Pos capsulelocation = objects[0]->getPosition();
	sensors[0] = capsulelocation.x();
	//std::cout  << "\n";

	// the number of red sensors is returned
	return len;
};


void nimm4_doublePole::placeIntern(const osg::Matrix& pose){
	// the position of the robot is the center of the body (without wheels)
	// to set the vehicle on the ground when the z component of the position is 0
	// width*0.6 is added (without this the wheels and half of the robot will be in the ground)
	Matrix p2;
	p2 = pose * Matrix::translate(Vec3(0, 0, width*0.6));
	create(p2);
};


/**
 * updates the osg notes
 */
void nimm4_doublePole::update(){

	OdeRobot::update();

	assert(created); // robot must exist

	/*for (unsigned int i=0; i<objects.size(); i++) { // update objects
		objects[i]->update();
	}

	for (unsigned int i=0; i < joints.size(); i++) { // update joints //change here
		joints[i]->update();
	}
*/


	if (inquireFailure())
	{
		//std::cout <<"Failure" << std::endl;
		double xDisplace;
		if (controller) xDisplace=((newACICO*)controller)->indexX; else xDisplace = 0;
		//((OdeRobot*)this)->place(Pos(xDisplace,1,0.2));
		((OdeRobot*)this)->place(osg::Matrix::translate(xDisplace,1,0.2) * osg::Matrix::rotate(0.0, 0, 0, 0));
		if (controller) ((newACICO*)controller)->failure_flag = 1;
		//((Box*)objects[5])->setPose();   // (Matrix::rotate(0, 0, 0, 0) * Matrix::translate(0,0,0));
	}
};

/** this function is called in each timestep. It should perform robot-internal checks,
      like space-internal collision detection, sensor resets/update etc.
      @param global structure that contains global data from the simulation environment
 */
void nimm4_doublePole::doInternalStuff(GlobalData& global){

	 OdeRobot::doInternalStuff(global);
}

/** creates vehicle at desired pose
      @param pose matrix with desired position and orientation
 */
void nimm4_doublePole::create( const osg::Matrix& pose ){
	if (created)
	{  // if robot exists destroy it
		destroy();
	}
	bool secondpole = false;
	if (controller)
	{
		if (((newACICO*)controller)->XDIM > 4) secondpole = true;
	}
//	if (secondpole)
		// create car space
		odeHandle.createNewSimpleSpace(parentspace, true);
	if (secondpole)
	{
		objects.resize(7);  // 1 capsule, 4 wheels //added 2 poles
		joints.resize(6); // joints between cylinder and each wheel //added also to poles
	}
	else
	{
		objects.resize(6);  // 1 capsule, 4 wheels //added pole
		joints.resize(5); // joints between cylinder and each wheel //added also to pole
	}
	OdeHandle wheelHandle(odeHandle);
	// make the material of the wheels a hard rubber
	wheelHandle.substance = wheelsubstance;
	// create cylinder for main body
	// initialize it with ode-, osghandle and mass
	// rotate and place body (here by -90� around the y-axis)
	// use texture 'wood' for capsule
	// put it into objects[0]
	Capsule* cap = new Capsule(width/2, length);
	cap->setTexture("Images/wood.rgb");
	cap->init(odeHandle, cmass, osgHandle);
	cap->setPose(Matrix::rotate(-M_PI/2, 0, 1, 0) * pose);
	objects[0]=cap;

	// added here: make poles
	Box* cap2 = new Box(0.1, 0.1, firstPoleLength);
	cap2->setTexture("Images/wood.rgb");
	cap2->init(odeHandle, cmass*0.1, osgHandle);

	float angle = 0;
	cap2->setPose(Matrix::rotate(angle, 0, 0.1, 0) * Matrix::translate(0.2+sin(angle)*firstPoleLength/2,0.2,0.2+cos(angle)*firstPoleLength/2 ) * pose);

	objects[5]=cap2;


	if(secondpole)
	{
		Box* cap3 = new Box(0.1, 0.1, secondPoleLength);
		cap3->setTexture("Images/wood.rgb");
		cap3->init(odeHandle, cmass*0.001, osgHandle);
		cap3->setPose(Matrix::rotate(4, 0, 0, 0) * Matrix::translate(-0.2,-0.2,0.2+secondPoleLength/2) * pose);
		objects[6]=cap3;
	}

	// create wheels
	/*   front
         -----
      1 |     | 2
        |     |
        |     |
      3 |     | 4
         -----
	 */
	for (int i=1; i<5; i++) {
		// create sphere with radius
		// and initialize it with odehandle, osghandle and mass
		// calculate position of wheels(must be at desired positions relative to the body)
		// rotate and place body (here by 90Deg around the x-axis)
		// set texture for wheels
		Sphere* sph = new Sphere(radius);
		sph->setTexture("Images/wood.rgb");
		sph->init(wheelHandle, wmass, osgHandle.changeColor(Color(0.8,0.8,0.8)));
		Vec3 wpos = Vec3( ((i-1)/2==0?-1:1)*length/2.0,
				((i-1)%2==0?-1:1)*(width*0.5+wheelthickness),
				-width*0.6+radius );
		sph->setPose(Matrix::rotate(M_PI/2, 0, 0, 1) * Matrix::translate(wpos) * pose);
		objects[i]=sph;
	}

	// generate 4 joints to connect the wheels to the body
	for (int i=0; i<4; i++) {
		Pos anchor(dBodyGetPosition (objects[i+1]->getBody()));
		//anchor.print();
		joints[i] = new Hinge2Joint(objects[0], objects[i+1], anchor, Axis(0,0,1)*pose, Axis(0,1,0)*pose);
		joints[i]->init(odeHandle, osgHandle, true, 2.01 * radius);
	}
	for (int i=0; i<4; i++) {
		// set stops to make sure wheels always stay in alignment
		joints[i]->setParam(dParamLoStop, 0);
		joints[i]->setParam(dParamHiStop, 0);
	}

	Pos anchor(0.2,0.1,0.2);		//anchor(0.2,1.1,0.7);
	joints[4] = new HingeJoint(objects[0], objects[5], anchor*pose, Axis(0,1,0)*pose);
	joints[4]->init(odeHandle, osgHandle, true, 2.01 * radius, false);
	if (secondpole)
	{
		//Pos anchor2(-0.2,0.9,0.7);
		Pos anchor2(-0.2,-0.1,0.2);
		joints[5] = new HingeJoint(objects[0], objects[6], anchor2*pose, Axis(0,1,0)*pose);
		joints[5]->init(odeHandle, osgHandle, true, 2.01 * radius, false);
	}

	//angle = ((float)rand())/RAND_MAX*0.3-0.15;
	if (controller) angle = ((newACICO*)controller)->indexT; else angle = 0;
	objects[5]->setPose(Matrix::rotate(angle, 0, 0.1, 0) * Matrix::translate(0.2+sin(angle)*firstPoleLength/2,0.2,0.2+cos(angle)*firstPoleLength/2 ) * pose);
	//angle = ((float)rand())/RAND_MAX*0.3-0.15;
	if (secondpole)
		objects[6]->setPose(Matrix::rotate(angle, 0, 0.1, 0) * Matrix::translate(-0.2+sin(angle)*secondPoleLength/2,-0.2,0.2+cos(angle)*secondPoleLength/2 ) * pose);



	created=true; // robot is created
};

/*Added:
 *
 */
bool nimm4_doublePole::inquireFailure()
{	Pos capsulelocation = objects[0]->getPosition();
//if (fabs(capsulelocation.x()) > 10) return false;
//if (!isfinite(capsulelocation.x()))
//{
//	((OdeRobot*)this)->place(Pos(0,1,0.2));
//	std::cout << "WARNING nan detected \n";
//	return false;
//}
HingeJoint *checkjoint = (HingeJoint*)joints[4];
if (fabs(checkjoint->getPosition1() ) > 0.205)  return true;
if (joints.size()>5) //this checks the second pole
{
	HingeJoint *checkjoint = (HingeJoint*)joints[5];
	if (fabs(checkjoint->getPosition1() ) > 0.205)  return true;
}
if (fabs(capsulelocation.x()) > 2.4 / 0.32)		return true;

//checkjoint = (HingeJoint*)joints[5];
//if (fabs(checkjoint->getPosition1() ) > 0.205)return true;

return false;
}
/** destroys vehicle and space
 */
void nimm4_doublePole::destroy(){
	if (created){
		cleanup();
		odeHandle.deleteSpace(); // destroy space
	}
	created=false; // robot does not exist (anymore)
}

}
