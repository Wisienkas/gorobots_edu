/***************************************************************************
 *   Copyright (C) 2005-2013 LpzRobots development team                    *
 *   Authors:                                                              *
 *    Simón Smith <artificialsimon at ed dot ac dot uk>                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
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

#include "differential.h"

// Using namespaces
using namespace osg;
using namespace std;
namespace lpzrobots{

Differential::Differential(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
		const DifferentialConf& conf, const string& name)
: OdeRobot(odeHandle, osgHandle, name, "2.0"), conf(conf){

}


Differential::~Differential(){
}


void Differential::placeIntern(const Matrix& pose){
	// Configuration check: wheels have to be bigger than the body
	assert(2. * conf.wheelRadius > conf.bodyHeight);
	// Moving robot upward such that the wheel are not stuck on the ground
	Matrix initialPose;
	initialPose = Matrix::translate(Vec3(0, 0, conf.wheelRadius) * pose);
	// Creating the robot
	create(initialPose);
}

int Differential::getSensorNumberIntern(){
	return 1;
}

int Differential::getSensorsIntern(double* sensors, int sensornumber){
	std::list<sensor> orientations =  this->orientationSensor->getList();
	double orientation1,orientation2;
	orientation1 = orientations.front();
	orientations.pop_front();
	orientation2 = orientations.front();
	//Negative sign, such that clockwise rotations are negative
	sensors[0] = -atan2(orientation2,orientation1);
	return 1;
}

void Differential::create(const Matrix& pose) {
	/* Creating body */
	// Cylinder geometry primitive as body
	auto body = new Cylinder(conf.bodyRadius, conf.bodyHeight);
	// Setting texture from Image library
	body->setTexture("Images/purple_velour.jpg");
	// Initializing the primitive
	body->init(odeHandle, conf.bodyMass, osgHandle);
	//Rotate body such that x direction in body frame is forward
	Matrix rotatedPose = Matrix::rotate(M_PI/2.,0,0,1) * pose;
	// Setting the pose of the primitive
	body->setPose(rotatedPose);
	// Adding the primitive to the list of objects
	objects.push_back(body);

	/* Creating the left wheel */
	auto lWheel = new Cylinder(conf.wheelRadius, conf.wheelHeight);
	// Setting texture from Images library
	lWheel->setTexture("Images/chess.rgb");
	lWheel->init(odeHandle, conf.wheelMass, osgHandle);
	// The cylinder is rotated 90º on the Y axis
	// then translated outside the radius of the body plus half of
	// its own height
	// -- All transformations have to be relative to the position so
	// the robot can be initialised in any position --
	Matrix lWheelPose =
			Matrix::rotate(M_PI / 2.0, 0, 1, 0) *
			Matrix::translate(conf.bodyRadius + conf.wheelHeight / 2.0, .0, .0) *
			pose;
	// Setting the pose of the wheel
	lWheel->setPose(lWheelPose);
	// adding the wheel to the list of objects
	objects.push_back(lWheel);
	// Joining the wheel to the body by a hingejoint
	// the anchor comes from the wheel and the axis of rotation
	// is relative to the pose of the left wheel
	auto bodyLeftWheelJoint = new HingeJoint(body, lWheel,
			lWheel->getPosition(),
			Axis(0, 0, 1) * lWheelPose);
	// Initializing the joint
	bodyLeftWheelJoint->init(odeHandle, osgHandle);
	// Adding the joint to the list of joints
	joints.push_back(bodyLeftWheelJoint);

	/* Creating the right wheel */
	// Analog to left wheel but changing translation direction
	auto rWheel = new Cylinder(conf.wheelRadius, conf.wheelHeight);
	rWheel->setTexture("Images/chess.rgb");
	rWheel->init(odeHandle, conf.wheelMass, osgHandle);
	Matrix rWheelPose = Matrix::rotate(M_PI / 2.0, 0, 1, 0) *
			Matrix::translate(-(conf.bodyRadius + conf.wheelHeight / 2.0), .0, .0) *
			pose;
	rWheel->setPose(rWheelPose);
	objects.push_back(rWheel);
	auto bodyRightWheelJoint = new HingeJoint(body, rWheel,
			rWheel->getPosition(),
			Axis(0, 0, 1) * rWheelPose);
	bodyRightWheelJoint->init(odeHandle, osgHandle);
	joints.push_back(bodyRightWheelJoint);

	/* Motors */
	// Left wheel motor, the OdeHandle, the joint and the maximun
	// power that motor will be used to achieve desired speed
	auto motor = std::make_shared<AngularMotor1Axis>(odeHandle, bodyLeftWheelJoint,
			conf.wheelMotorPower);
	motor->setBaseName("left motor");
	motor->setVelovityFactor(conf.wheelMotorMaxSpeed);
	addSensor(motor);
	addMotor(motor);

	// Right wheel motor
	motor = std::make_shared<AngularMotor1Axis>(odeHandle, bodyRightWheelJoint,
			conf.wheelMotorPower);
	motor->setBaseName("right motor");
	motor->setVelovityFactor(conf.wheelMotorMaxSpeed);
	addSensor(motor);
	addMotor(motor);

    //Create range finder
	auto rangeFinder = std::make_shared<RangeFinder>();
	rangeFinder->setInitData(this->odeHandle, this->osgHandle, TRANSM(0,0,0));
	rangeFinder->init(body);
	//Register beam range
	rangeFinder->registerSensorRange(15, M_PI/2., -M_PI/2., 15, 1, RaySensor::drawRay);

	rangeFinder->setBaseName("RF Beam ");
	std::vector<std::string> rfIndices(rangeFinder->getSensorNumber());
	for(int i = 1; i <= rangeFinder->getSensorNumber();i++)
		rfIndices[i-1] = to_string(i);
	rangeFinder->setNames(rfIndices);
	addSensor(rangeFinder);
	//Position sensor
	auto positionSensor = std::make_shared<RelativePositionSensor>(1, 1, RelativePositionSensor::XYZ, false);
	positionSensor->init(body);
	positionSensor->setBaseName("Position ");
	positionSensor->setNames({"X", "Y", "Z"});
	addSensor(positionSensor);
	//Create speed sensor (linear)
	auto speedSensor = std::make_shared<SpeedSensor>(1.0, SpeedSensor::TranslationalRel, SpeedSensor::XYZ);
	speedSensor->init(body);
	speedSensor->setBaseName("Body Speed ");
	speedSensor->setNames({"X","Y","Z"});
	addSensor(speedSensor);
	//Create speed sensor (angular)
	auto rotspeedSensor = std::make_shared<SpeedSensor>(1.0, SpeedSensor::RotationalRel, SpeedSensor::XYZ);
	rotspeedSensor->init(body);
	rotspeedSensor->setBaseName("Angular velocity ");
	rotspeedSensor->setNames({"roll", "pitch", "yaw"});
	addSensor(rotspeedSensor);
	//Create orientation sensor
	this->orientationSensor = new AxisOrientationSensor(AxisOrientationSensor::Axis,Sensor::X |Sensor::Y | Sensor::Z);
	this->orientationSensor->init(body);

	//Create acceleration sensor
	auto accSensor = std::make_shared<DerivativeSensor>(new SpeedSensor(1.0, SpeedSensor::TranslationalRel, SpeedSensor::XYZ));
	accSensor->init(body);
	accSensor->setBaseName("Linear Acceleration in ");
	accSensor->setNames({"x (body frame)", "y (body frame)", "z (body frame)"});
	addSensor(accSensor);

	/* Infra-red sensors */
	auto irSensorBank = std::make_shared<RaySensorBank>();
	// Initialising infra-red sensor bank
	irSensorBank->setInitData(odeHandle, osgHandle, TRANSM(0,0,0));
	irSensorBank->setBaseName("IR ");
	irSensorBank->setNames({"left", "left front", "front left","front right",
		"right front", "right","back left", "back right"});

	// Registering the sensor in the bank (set of ir sensors), fixed to body
	// For the first sensor it is rotated to point forward
	// translation from center of body to outside and middle of height
	// pose is relative to the parent body - no need to multiply by 'pose'.
	// Maximum range of sensor value.
	// drawAll will display a line and the sensor body in the rendered scene.
	irSensorBank->registerSensor(new IRSensor(), body,
			Matrix::rotate(-M_PI / 2.0, 1, 0, 0) *
			//                                Matrix::rotate( M_PI / 2.0, 0, 0, 1) *
			Matrix::translate(-conf.bodyRadius * sin(-M_PI*.1),
					conf.bodyRadius * cos(-M_PI*.1), conf.bodyHeight / 2.0),
					conf.irRange,
					RaySensor::drawNothing);

	irSensorBank->registerSensor(new IRSensor(), body,
			Matrix::rotate(-M_PI / 2.0, 1, 0, 0) *
			Matrix::rotate( M_PI / 3.5, 0, 0, 1) *
			Matrix::rotate( -M_PI / 2, 0, 0, 1) *
			Matrix::translate(-conf.bodyRadius * sin(-M_PI * .25),
					conf.bodyRadius * cos(-M_PI * .25), conf.bodyHeight / 2.0),
					conf.irRange,
					RaySensor::drawNothing);

	irSensorBank->registerSensor(new IRSensor(), body,
			Matrix::rotate(-M_PI / 2.0, 1, 0, 0) *
			Matrix::rotate( -M_PI / 2, 0, 0, 1) *
			Matrix::translate(-conf.bodyRadius * sin(M_PI * .05 - M_PI/2.),
					conf.bodyRadius * cos(M_PI * .05- M_PI/2.), conf.bodyHeight / 2.0),
					conf.irRange,
					RaySensor::drawNothing);

	irSensorBank->registerSensor(new IRSensor(), body,
			Matrix::rotate(-M_PI / 2.0, 1, 0, 0) *
			Matrix::rotate( -M_PI / 2, 0, 0, 1) *
			Matrix::translate(conf.bodyRadius * sin(M_PI * .05+ M_PI/2.),
					conf.bodyRadius * cos(M_PI * .05+ M_PI/2.), conf.bodyHeight / 2.0),
					conf.irRange,
					RaySensor::drawNothing);
	irSensorBank->registerSensor(new IRSensor(), body,
			Matrix::rotate(-M_PI / 2.0, 1, 0, 0) *
			Matrix::rotate(-M_PI / 3.5, 0, 0, 1) *
			Matrix::rotate( -M_PI / 2, 0, 0, 1) *
			Matrix::translate(conf.bodyRadius * sin(M_PI * .25 + M_PI/2.),
					conf.bodyRadius * cos(M_PI * .25 + M_PI/2.), conf.bodyHeight / 2.0),
					conf.irRange,
					RaySensor::drawNothing);
	irSensorBank->registerSensor(new IRSensor(), body,
			Matrix::rotate(-M_PI / 2.0, 1, 0, 0) *
			Matrix::rotate( -M_PI, 0, 0, 1) *
			Matrix::translate(conf.bodyRadius * sin(-M_PI*.1 + M_PI),
					conf.bodyRadius * cos(-M_PI*.1 + M_PI), conf.bodyHeight / 2.0),
					conf.irRange,
					RaySensor::drawNothing);

	irSensorBank->registerSensor(new IRSensor(), body,
			Matrix::rotate(M_PI / 2.0, 1, 0, 0) *
			Matrix::rotate( -M_PI/2., 0, 0, 1) *
			Matrix::translate(-conf.bodyRadius * sin(M_PI * .9 - M_PI/2.),
					conf.bodyRadius * cos(M_PI * .9 - M_PI/2.), conf.bodyHeight / 2.0),
					conf.irRange,
					RaySensor::drawNothing);

	irSensorBank->registerSensor(new IRSensor(), body,
			Matrix::rotate(M_PI / 2.0, 1, 0, 0) *
			Matrix::rotate( -M_PI/2., 0, 0, 1) *
			Matrix::translate(conf.bodyRadius * sin(M_PI * .9 + M_PI/2.),
					conf.bodyRadius * cos(M_PI * .9 + M_PI/2.), conf.bodyHeight / 2.0),
					conf.irRange,
					RaySensor::drawNothing);
	addSensor(irSensorBank);

}
}
