#include "factory.h"

// joint needed for fixation of the robot in the beginning
#include <ode_robots/joint.h>

// add head file for creating a sphere by Ren ------------
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>
#include <selforg/abstractcontroller.h>
#include <ode_robots/color.h>

// Getting the GlobalData implementation code to get members
#include <ode_robots/globaldata.h>

using namespace lpzrobots;

Factory::Factory(const lpzrobots::OdeHandle& odeHandle,
                 const lpzrobots::OsgHandle& osgHandle,
                 const lpzrobots::GlobalData& global) :
  odeHandle(odeHandle), osgHandle(osgHandle), global(global) {}

lpzrobots::PassiveBox * Factory::createBox(double length,
                                  double width,
                                  double height,
                                  osg::Vec3 position) {
  lpzrobots::PassiveBox * passiveBox
    = new lpzrobots::PassiveBox(odeHandle, osgHandle,
                                osg::Vec3(length, width, height));
  passiveBox->setColor(lpzrobots::Color(1,0,0));
  passiveBox->setPose(osg::Matrix::rotate(2, 0, 0, 1) * osg::Matrix::translate(2.0, 0.0, 0.0));

  return passiveBox;
}

lpzrobots::PassiveSphere * Factory::createSphere(double size,
                                        osg::Vec3 position) {
  lpzrobots::PassiveSphere* sphere = new lpzrobots::PassiveSphere(odeHandle, osgHandle, size);
  sphere->setPosition(position);
  sphere->setTexture("Images/dusty.rgb");
  sphere->setColor(lpzrobots::Color(1,0,0));

  return sphere;
}

void Factory::initFixedJoint(lpzrobots::AbstractObstacle* fixedObject) {
  std::vector<lpzrobots::AbstractObstacle*> obstacles = global.obstacles;
  obstacles.push_back(fixedObject);
  lpzrobots::FixedJoint* fixator
    = new lpzrobots::FixedJoint(fixedObject->getMainPrimitive(), global.environment);
  fixator->init(odeHandle, osgHandle);
}
