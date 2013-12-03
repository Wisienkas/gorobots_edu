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

#ifndef GOROBOTS_CONTROLLER_NEJIHEBI_NEJIHEBIINTERFACE_H_
#define GOROBOTS_CONTROLLER_NEJIHEBI_NEJIHEBIINTERFACE_H_

// include some functionalities from standard library
#include <vector>
#include <map>

/**
 * Interface for Nejihebi Robots
 *
 * This parent class is shared by the interface to the simulated nejihebi robot
 * as well as the interface to the real machine. All methods provided by the
 * interface are guaranteed to work with the real machine.
 */
class NejihebiInterface
{
  public:
    /**
     * Enumeration type for joint type
     *
     * YAW:   left-right joint
     * PITCh: up-down joint
     */
    enum JointType {YAW, PITCH};

    /**
     * destructor
     */
    virtual ~NejihebiInterface();

    /**
     * Returns present joint angle in degrees
     *
     * @param type joint type
     * @param index joint index (0 is foremost)
     * @return joint angle in degrees
     */
    virtual double getJointAngle(JointType type, unsigned int index) = 0;

    /**
     * Returns servo motor goal angle in degrees
     *
     * @param type joint type
     * @param index joint index (0 is foremost)
     * @return goal angle in degrees
     */
    virtual double getJointGoalAngle(JointType type, unsigned int index) = 0;

    /**
     * Returns present joint load in per cent of maximum torque
     *
     * @param type joint type
     * @param index joint index (0 is foremost)
     * @return joint load in per cent
     */
    virtual double getJointLoad(JointType type, unsigned int index) = 0;

    /**
     * Returns present joint speed in rpm
     *
     * @param type joint type
     * @param index joint index (0 is foremost)
     * @return joint speed in rpm
     */
    virtual double getJointSpeed(JointType type, unsigned int index) = 0;

    /**
     * Returns present servo temperature in Celsius
     *
     * @param type joint type
     * @param index joint index (0 is foremost)
     * @return servo temperature in Celsius
     */
    virtual double getJointTemperature(JointType type, unsigned int index) = 0;

    /**
     * Returns joint torque limit in per cent
     *
     * @param type joint type
     * @param index joint index (0 is foremost)
     * @return joint torque limit in per cent
     */
    virtual double getJointTorqueLimit(JointType type, unsigned int index) = 0;

    /**
     * Returns present servo voltage in Volts
     *
     * @param type joint type
     * @param index joint index (0 is foremost)
     * @return servo voltage in Volts
     */
    virtual double getJointVoltage(JointType type, unsigned int index) = 0;

    /**
     * Sets the goal angle for a joint servo
     *
     * @param type joint type
     * @param index joint index (0 is foremost)
     * @param angle goal angle in degrees
     */
    virtual void setJointGoalAngle(JointType type, unsigned int index,
        const double angle) = 0;

    /**
     * Sets the maximum moving speed for a joint servo
     *
     * @param type joint type
     * @param index joint index (0 is foremost)
     * @param speed maximum moving speed in rpm
     */
    virtual void setJointMovingSpeed(JointType type, unsigned int index,
        const double speed) = 0;

    /**
     * Disables/Enables free run mode for a joint servo
     *
     * @param type joint type
     * @param index joint index (0 is foremost)
     * @param enabled true disables, false enables free run mode
     */
    virtual void setJointTorqueEnabled(JointType type, unsigned int index,
        const bool enabled) = 0;

    /**
     * Sets the maximum used torque for a joint servo
     *
     * @param type joint type
     * @param index joint index (0 is foremost)
     * @param values maximum used torque in per cent of maximum torque
     */
    virtual void setJointTorqueLimit(JointType type, unsigned int index,
        const double value) = 0;

    /**
     * Returns the present angle of a screw in degrees
     *
     * @param index screw index (0 is foremost)
     * @return present rotation angle of screw unit
     */
    virtual double getRotationUnitAngle(unsigned int index) = 0;    
    
    /**
     * Sets desired speed of screw in rad/s
     *
     * @param index screw index (0 is foremost)
     * @param speed desired speed in rad/s
     */
    virtual void setRotationUnitSpeed(unsigned int index, const double speed)=0;

    /**
     * Returns present angle of a yaw (left-right) joint in degrees
     *
     * @param index index of yaw joint (0 is foremost)
     * @return present angle in degrees
     */
    double getYawJointAngle(unsigned int index);

    /**
     * Returns goal angle for a yaw (left-right) joint in degrees
     *
     * @param index index of yaw joint (0 is foremost)
     * @return goal angle in degrees
     */
    double getYawJointGoalAngle(unsigned int index);

    /**
     * Returns present load of yaw (left-right) joint in per cent
     *
     * @param index index of yaw joint (0 is foremost)
     * @return present load in per cent of maximum torque
     */
    double getYawJointLoad(unsigned int index);

    /**
     * Returns present speed of yaw (left-right) joint in rpm
     *
     * @param index index of yaw joint (0 is foremost)
     * @return present speed in rpm
     */
    double getYawJointSpeed(unsigned int index);

    /**
     * Returns present temperature of yaw (left-right) joint servo in Celsius
     *
     * @param index index of yaw joint (0 is foremost)
     * @return present temperature in Celsius
     */
    double getYawJointTemperature(unsigned int index);

    /**
     * Returns torque limit of yaw (left-right) joint servo in per cent
     *
     * @param index index of yaw joint (0 is foremost)
     * @return maximum used torque in per cent of maximum possible torque
     */
    double getYawJointTorqueLimit(unsigned int index);

    /**
     * Returns present voltage of yaw (left-right) joint servo in Volts
     *
     * @param index index of yaw joint (0 is foremost)
     * @return present voltage in Volts
     */
    double getYawJointVoltage(unsigned int index);
    
    /**
     * Sets goal angle for pitch (up-down) joint in degrees
     *
     * @param index index of pitch joint (0 is foremost)
     * @param angle desired goal angle in degrees
     */
    void setPitchJointGoalAngle(unsigned int index, const double angle); 

    /**
     * Sets goal angle for yaw (left-right) joint in degrees
     *
     * @param index index of yaw joint (0 is foremost)
     * @param angle desired goal angle in degrees
     */
    void setYawJointGoalAngle(unsigned int index, const double angle);

    /**
     * Sets the maximum moving speed for a yaw joint servo
     *
     * @param index yaw joint index (0 is foremost)
     * @param speed maximum moving speed in rpm
     */
    void setYawJointMovingSpeed(unsigned int index, const double speed);

    /**
     * Disables/Enables free run mode for a yaw joint servo
     *
     * @param index joint index (0 is foremost)
     * @param enabled true disables, false enables free run mode
     */
    void setYawJointTorqueEnabled(unsigned int index, const bool enabled);

    /**
     * Sets the maximum used torque for a yaw joint servo
     *
     * @param index joint index (0 is foremost)
     * @param values maximum used torque in per cent of maximum possible torque
     */
    void setYawJointTorqueLimit(unsigned int index, const double value);
};

#endif /* GOROBOTS_CONTROLLER_NEJIHEBI_NEJIHEBIINTERFACE_H_ */
