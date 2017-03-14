#include "tribot.h"

#include <ode_robots/primitive.h>

namespace lpzrobots {

  /**
   * Constructor
   */
  Tribot::Tribot(const OdeHandle& odeHandle,
                 const OsgHandle& osgHandle,
                 const TribotConfig& config,
                 const std::string& name)
    : OdeRobot(odeHandle, osgHandle, name, "1.0"), conf(config)
  {
  }

  Tribot::~Tribot()
  {
  }

  void Tribot::placeIntern(const osg::Matrix& pose)
  {
    osg::Matrix initialPose;
    initialPose = osg::Matrix::translate(osg::Vec3(0, 0, conf.wheelRadius) * pose);

    create(initialPose);
  }

  enum Side {
    left,
    right
  };

  Primitive * Tribot::createWheel(int side, const osg::Matrix& pose) {
    Primitive* lWheel = new Cylinder(conf.wheelRadius, conf.wheelHeight);
    lWheel->setTexture("Images/chess.rgb");
    lWheel->init(odeHandle, conf.wheelMass, osgHandle);

    double transX = conf.bodyRadius + conf.wheelHeight / 2.0;
    if(side == 1) {
      transX *= -1.;
    }
    osg::Matrix lWheelPose =
      osg::Matrix::rotate(M_PI / 2.0, 0, 1, 0) *
      osg::Matrix::translate(transX, .0, .0) *
      pose;

    lWheel->setPose(lWheelPose);
    objects.push_back(lWheel);

    return lWheel;
  }

  void Tribot::createMotor(std::string name, HingeJoint* joint)
  {
    auto motor = std::make_shared<AngularMotor1Axis>(odeHandle,
                                                     joint,
                                                     conf.wheelMotorPower);
    motor->setBaseName(name);
    motor->setVelovityFactor(conf.wheelMotorMaxSpeed);
    addSensor(motor);
    addMotor(motor);
  }

  void Tribot::create(const osg::Matrix& pose)
  {
    // Set the body type
    Primitive* body = new Cylinder(conf.bodyRadius, conf.bodyHeight);
    body->setTexture("Images/purple_velour.jpg");
    body->init(odeHandle, conf.bodyMass, osgHandle);
    body->setPose(pose);
    objects.push_back(body);

    // Left Wheel
    Primitive* lWheel = createWheel(0, pose);
    auto* bodyLeftWheelJoint = new HingeJoint(body,
                                              lWheel,
                                              lWheel->getPosition(),
                                              Axis(0,0,1) * lWheel->getPose());
    bodyLeftWheelJoint->init(odeHandle, osgHandle);
    joints.push_back(bodyLeftWheelJoint);

    // Right Wheel
    Primitive* rWheel = createWheel(1, pose);
    auto* bodyRightWheelJoint = new HingeJoint(body,
                                              rWheel,
                                              rWheel->getPosition(),
                                              Axis(0,0,1) * rWheel->getPose());
    bodyRightWheelJoint->init(odeHandle, osgHandle);
    joints.push_back(bodyRightWheelJoint);

    // Motors
    createMotor(std::string("left motor"), bodyLeftWheelJoint);
    createMotor(std::string("right motor"), bodyRightWheelJoint);
  }
}
