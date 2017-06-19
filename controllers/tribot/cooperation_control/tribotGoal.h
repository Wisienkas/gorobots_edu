// HEADER GUARD
#ifndef __GOROBOTS_EDU_PROJECTS_TRIBOT_GOAL_H
#define __GOROBOTS_EDU_PROJECTS_TRIBOT_GOAL_H

#include <ode_robots/primitive.h>

namespace tribot {
  struct TribotGoal
  {
    osg::Vec3 position;
    double winDistance;
  TribotGoal(Position position, double winDistance)
  : position(position.x, position.y, position.z), winDistance(winDistance) {};
  TribotGoal(osg::Vec3 position, double winDistance)
  : position(position), winDistance(winDistance) {};
  };
}

#endif // end of __GOROBOTS_EDU_PROJECTS_TRIBOT_GOAL_H
