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

#include "nejihebicontroller.h"

#include "controllers/nejihebi/nejihebiinterface.h"

NejihebiController::NejihebiController(void)
{
  t = 0;
}

NejihebiController::~NejihebiController(void)
{
  for (int i=0;i<4;i++) nejihebi->setRotationUnitSpeed(i,0);
}

void NejihebiController::init(NejihebiInterface * neji)
{
  nejihebi = neji;

  // set some basic commands
  nejihebi->setYawJointGoalAngle(0, 0);
  nejihebi->setYawJointGoalAngle(1, 0);
  nejihebi->setYawJointGoalAngle(2, 0);
  nejihebi->setPitchJointGoalAngle(0,0);
  nejihebi->setPitchJointGoalAngle(1,0);
  nejihebi->setPitchJointGoalAngle(2,0);
  nejihebi->setYawJointTorqueLimit(0,1);
  nejihebi->setYawJointTorqueLimit(1,1);
  nejihebi->setYawJointTorqueLimit(2,1);
  nejihebi->setJointTorqueLimit(NejihebiInterface::PITCH,0,1);
  nejihebi->setJointTorqueLimit(NejihebiInterface::PITCH,1,1);
  nejihebi->setJointTorqueLimit(NejihebiInterface::PITCH,2,1);

}


void NejihebiController::step()
{

  if (t%50==0) {
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
  t++;
}
