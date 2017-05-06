// HEADER GUARD
#ifndef __GOROBOTS_EDU_PROJECTS_TRIBOT_GOAL_H
#define __GOROBOTS_EDU_PROJECTS_TRIBOT_GOAL_H

namespace tribot {
  struct TribotGoal
  {
    osg::Vec3 position;
  TribotGoal(Position position) : position(position.x, position.y, position.z) {};
  TribotGoal(osg::Vec3 position) : position(position) {};
  };
}

#endif // end of __GOROBOTS_EDU_PROJECTS_TRIBOT_GOAL_H
