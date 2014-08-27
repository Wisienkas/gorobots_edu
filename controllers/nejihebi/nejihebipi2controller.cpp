/***************************************************************************
 *   Copyright (C) 2012 by                                                 *
 *    Timo Nachstedt <nachstedt@physik3.gwdg.de>                           *
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

#include "nejihebipi2controller.h"

#include "controllers/nejihebi/nejihebiinterface.h"

#include <iostream>
#include <stdlib.h>
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include "Eigen/Eigen"
#include <algorithm>
#include <stack>
#include <map>
#include <utility>
using namespace Eigen;
using namespace std;

#include "PI2Wholesteps.h"

float dt = 0.2;

NejihebiPi2Controller::NejihebiPi2Controller(void) {
  t = 0;
}

NejihebiPi2Controller::~NejihebiPi2Controller(void) {
  for (int i = 0; i < 4; i++)
    nejihebi->setRotationUnitSpeed(i, 0); //screw speed initialized to zero

}

void NejihebiPi2Controller::init(NejihebiInterface * neji) {
  nejihebi = neji;

  // set joint configuration in radian
  float joint1 = 0;
  float joint2 = 0;
  float joint3 = 0;

  // set some basic commands
  nejihebi->setYawJointGoalAngle(0, joint1 * 180 / M_PI); //setting joint angles to zero
  nejihebi->setYawJointGoalAngle(1, joint2 * 180 / M_PI);
  nejihebi->setYawJointGoalAngle(2, joint3 * 180 / M_PI);
  nejihebi->setPitchJointGoalAngle(0, 0);
  nejihebi->setPitchJointGoalAngle(1, 0);
  nejihebi->setPitchJointGoalAngle(2, 0);
  nejihebi->setYawJointTorqueLimit(0, 1);
  nejihebi->setYawJointTorqueLimit(1, 1);
  nejihebi->setYawJointTorqueLimit(2, 1);
  nejihebi->setJointTorqueLimit(NejihebiInterface::PITCH, 0, 1);
  nejihebi->setJointTorqueLimit(NejihebiInterface::PITCH, 1, 1);
  nejihebi->setJointTorqueLimit(NejihebiInterface::PITCH, 2, 1);

  //float x,y;
  //cout<<"give goal x position \n";
  //cin>>x;
  //cout<<"give goal y position \n";
  //cin>>y;

  // Give goal position in x and y
  float x = -1;
  float y = -1;
  float goal[2] = { x, y };
  PI2Wholesteps p; //object for main starting clsss wholesteps

  // initial robot configuration initialized in state vector robot_head
  float robot_head[6] = { 0, 0, 0, joint1, joint2, joint3 };
  //robot_head has initial robot head (x,y), psi orientation, and 3 joint angles.

  // parameters concerning time
  /**********************************************************/
  int timesteps = 100;                //set t as the size of timesteps required
  // dt in global
  int a = timesteps / dt;
  float t[a];
  t[0] = 0;

  /***********************************************************/

  // start of Learning
  cout << "\n Starting PI2 Learning \n";
  float phi[3] = { robot_head[3], robot_head[4], robot_head[5] }; // with only intial robot joint angles
  Eigen::MatrixXd u_final = p.wholesteps(robot_head, t, phi, goal);
  cout << "\n PI2 Learning Ends \n";
  // end of PI2 Learning;  u_final has the final learned parameters

  // giving final learned screw velocities to the robot
  double a_u = u_final(0, 0);
  a_u = a_u / (0.8 * timesteps);
  nejihebi->setRotationUnitSpeed(0, a_u);

  a_u = u_final(0, 1);
  a_u = a_u / (0.8 * timesteps);
  nejihebi->setRotationUnitSpeed(1, a_u);

  a_u = u_final(0, 2);
  a_u = a_u / (0.8 * timesteps);
  nejihebi->setRotationUnitSpeed(2, a_u);

  a_u = u_final(0, 3);
  a_u = a_u / (0.8 * timesteps);
  nejihebi->setRotationUnitSpeed(3, a_u);

  cout << "\n final learned screw velocities are: \n"
      << u_final(0, 0) / (0.8 * timesteps) << ","
      << u_final(0, 1) / (0.8 * timesteps) << ","
      << u_final(0, 2) / (0.8 * timesteps) << ","
      << u_final(0, 3) / (0.8 * timesteps);

  cout << "\n given goal positions are: \n" << x << " and " << y;
}

void NejihebiPi2Controller::step() {

  /*if (t%50==0) {
   switch ((t/50)%5) {
   case 0:
   nejihebi->setRotationUnitSpeed(0,-2);
   nejihebi->setRotationUnitSpeed(1, 2);
   nejihebi->setRotationUnitSpeed(2,-2);
   nejihebi->setRotationUnitSpeed(3, 2);
   break;
   case 1:
   nejihebi->setRotationUnitSpeed(0, 2);
   nejihebi->setRotationUnitSpeed(1, 1);
   nejihebi->setRotationUnitSpeed(2,-1);
   nejihebi->setRotationUnitSpeed(3,-2);
   break;
   case 2:
   nejihebi->setRotationUnitSpeed(0, 1);
   nejihebi->setRotationUnitSpeed(1, 1);
   nejihebi->setRotationUnitSpeed(2, 1);
   nejihebi->setRotationUnitSpeed(3, 1);
   break;
   case 3:
   nejihebi->setRotationUnitSpeed(0,-2);
   nejihebi->setRotationUnitSpeed(1,-1);
   nejihebi->setRotationUnitSpeed(2, 1);
   nejihebi->setRotationUnitSpeed(3, 2);
   break;
   case 4:
   nejihebi->setRotationUnitSpeed(0, 2);
   nejihebi->setRotationUnitSpeed(1,-2);
   nejihebi->setRotationUnitSpeed(2, 2);
   nejihebi->setRotationUnitSpeed(3,-2);
   break;
   }
   }
   t++;*/
}
