#ifndef __FACTORY_H
#define __FACTORY_H

// add head file for creating a sphere by Ren ------------
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>
#include <selforg/abstractcontroller.h>
#include <ode_robots/color.h>

/**
 * This Factory will make it easier and faster to quickly create objects by re-using some
 * variables which is usually used during that phase
 */
class Factory {
 private:
  // re-used variables
  const lpzrobots::OdeHandle& odeHandle;
  const lpzrobots::OsgHandle& osgHandle;
  const lpzrobots::GlobalData& global;
 public:
  // Construct
  Factory(const lpzrobots::OdeHandle& odeHandle,
          const lpzrobots::OsgHandle& osgHandle,
          const lpzrobots::GlobalData& global);
  // Deconstruct
  virtual ~Factory() {};
  // Methods
  lpzrobots::PassiveBox * createBox(double length,
                                    double width,
                                    double height,
                                    osg::Vec3 position);

  lpzrobots::PassiveSphere * createSphere(double size,
                                          osg::Vec3 position);

  void initFixedJoint(lpzrobots::AbstractObstacle * fixedObject);
};

#endif
