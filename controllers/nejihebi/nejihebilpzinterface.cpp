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

#include "nejihebilpzinterface.h"

NejihebiLpzInterface::NejihebiLpzInterface(NejihebiControllerBase*
    acontroller)
:AbstractController("NejihebiLpzController", "v0.1"), controller(acontroller){
  jointMaxSpeed = 0;
  screwMaxSpeed = 0;
}

void NejihebiLpzInterface::init(int sensornumber, int motornumber,
    RandGen* randGen ) {
  controller->init(this); // todo: get frequency from argument
  NejihebiControllerBase::InspectableValueList l =
      controller->getInspectableValues();
  for (NejihebiController::InspectableValueList::iterator it = l.begin();
      it != l.end(); it++) {
    addInspectableValue(it->name, it->value, it->description);
  }
}

int NejihebiLpzInterface::getSensorNumber() const {
  return 5*screwnumber - 4;
}

int NejihebiLpzInterface::getMotorNumber() const {
  return 5*screwnumber - 4;
}

void NejihebiLpzInterface::step(const sensor* x, int number_sensors,
    motor* y, int number_motors) {
  stepNoLearning(x, number_sensors, y, number_motors);
}


void NejihebiLpzInterface::stepNoLearning(const sensor* x,
    int number_sensors, motor* y, int number_motors) {
  for (int i=0; i<screwnumber; i++) {
    screws[i].angle = 180*x[i];
  }
  for (int i=0; i<screwnumber-1; i++) {
    joints[YAW]  [i].angle = 180*x[screwnumber+4*i+0];
    joints[YAW]  [i].speed = jointMaxSpeed * x[screwnumber+4*i+1];
    joints[PITCH][i].angle = 180*x[screwnumber+4*i+2];
    joints[PITCH][i].speed = jointMaxSpeed * x[screwnumber+4*i+3];
  }

  controller->step();

  for (int i=0; i<screwnumber; i++) {
    y[i] = screws[i].speed/screwMaxSpeed;
  }
  for (int i=0; i<screwnumber-1; i++) {
    y[  screwnumber  +2*i]   = joints[YAW]  [i].goalAngle/180.0;
    y[  screwnumber  +2*i+1] = joints[PITCH][i].goalAngle/180.0;
    y[3*screwnumber-2+2*i]   = joints[YAW]  [i].torqueLimit;
    y[3*screwnumber-2+2*i+1] = joints[PITCH]  [i].torqueLimit;
  }
}

void NejihebiLpzInterface::setRobotMaxSpeed(const double& screwMax,
    const double& jointMax) {
  screwMaxSpeed = screwMax;
  jointMaxSpeed = jointMax;
}

bool NejihebiLpzInterface::store(FILE* f) const {
  return false;
}

bool NejihebiLpzInterface::restore(FILE* f) {
  return false;
}

double NejihebiLpzInterface::getJointAngle(JointType type, unsigned int index)
{
  return joints[type][index].angle;
}

double NejihebiLpzInterface::getJointGoalAngle(JointType type,
    unsigned int index) {
  return joints[type][index].goalAngle;
}

double NejihebiLpzInterface::getJointLoad(JointType type, unsigned int index) {
  return joints[type][index].load;
}

double NejihebiLpzInterface::getJointSpeed(JointType type,
    unsigned int index) {
  return joints[type][index].speed;
}

double NejihebiLpzInterface::getJointTemperature(JointType type,
    unsigned int index) {
  return joints[type][index].temperature;
}

double NejihebiLpzInterface::getJointTorqueLimit(JointType type,
    unsigned int index){
  return joints[type][index].torqueLimit;
}

double NejihebiLpzInterface::getJointVoltage(JointType type,
    unsigned int index) {
  return joints[type][index].voltage;
}

double NejihebiLpzInterface::getRotationUnitAngle(unsigned int index) {
  return screws[index].angle;
}

void NejihebiLpzInterface::setJointGoalAngle(JointType type,
    unsigned int index, const double angle) {
  joints[type][index].goalAngle = angle;
}

void NejihebiLpzInterface::setJointMovingSpeed(JointType type,
    unsigned int index, const double speed) {
  joints[type][index].movingSpeed = speed;
}

void NejihebiLpzInterface::setJointTorqueEnabled(JointType type,
    unsigned int index, const bool enabled) {
  joints[type][index].torqueEnabled = enabled;
}

void NejihebiLpzInterface::setJointTorqueLimit(JointType type,
    unsigned int index, const double value) {
  joints[type][index].torqueLimit = value;
}

void NejihebiLpzInterface::setRotationUnitSpeed(unsigned int index,
    const double speed) {
  screws[index].speed = speed;
}
