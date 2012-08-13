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

#include "nejihebiinterface.h"

NejihebiInterface::~NejihebiInterface(void)
{
}

double NejihebiInterface::getYawJointAngle(unsigned int index)
{
  return getJointAngle(YAW, index);
}

double NejihebiInterface::getYawJointGoalAngle(unsigned int index)
{
  return getJointGoalAngle(YAW, index);
}

double NejihebiInterface::getYawJointLoad(unsigned int index)
{
  return getJointLoad(YAW, index);
}

double NejihebiInterface::getYawJointSpeed(unsigned int index)
{
  return getJointSpeed(YAW, index);
}

double NejihebiInterface::getYawJointTemperature(unsigned int index)
{
  return getJointTemperature(YAW, index);
}

double NejihebiInterface::getYawJointTorqueLimit(unsigned int index)
{
  return getJointTorqueLimit(YAW, index);
}

double NejihebiInterface::getYawJointVoltage(unsigned int index)
{
  return getJointVoltage(YAW, index);
}

void NejihebiInterface::setPitchJointGoalAngle(unsigned int index, const double angle)
{
  setJointGoalAngle(PITCH, index, angle);
}

void NejihebiInterface::setYawJointGoalAngle(unsigned int index, const double angle)
{
  setJointGoalAngle(YAW, index, angle);
}

void NejihebiInterface::setYawJointMovingSpeed(unsigned int index, const double angle)
{
  setJointMovingSpeed(YAW, index, angle);
}

void NejihebiInterface::setYawJointTorqueEnabled(unsigned int index, const bool enabled)
{
  setJointTorqueEnabled(YAW, index, enabled);
}

void NejihebiInterface::setYawJointTorqueLimit(unsigned int index, const double value)
{
  setJointTorqueLimit(YAW, index, value);
}
