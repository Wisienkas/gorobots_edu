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
#ifndef GOROBOTS_CONTROLLER_NEJIHEBI_NEJIHEBIBUFFER_H_
#define GOROBOTS_CONTROLLER_NEJIHEBI_NEJIHEBIBUFFER_H_

#include "nejihebiinterface.h"
#include "nejihebicontroller.h"
#include <selforg/abstractcontroller.h>

// forward declarations
class NejihebiControllerBase;

/**
 * Connector between general Nejihebi controllers and lpzrobots
 *
 * This class connnects your general nejihebi controller that is also suitable
 * for the real machine with the lpzrobots simulation. The class derives from
 * NejihebiInterface, so that it looks like a Nejihebi robot for the controller.
 * On the other side it uses the lpzrobots mechanism to control the simulated
 * robot.
 *
 * To use this connector, first create an instance of your controller and pass
 * it as argument to the constructor of this class. An instance of
 * NejihebiLpzInterface can afterwards be used like an normal lpzrobots
 * controller.
 */
class NejihebiLpzInterface : public AbstractController,
public NejihebiInterface {
  public:
    /**
     * Constructor
     *
     * @param controller pointer to an instance of your nejihebi controller
     */
    NejihebiLpzInterface(NejihebiControllerBase* controller);

    /**
     * Initializes the controller
     *
     * initialisation of the controller with the given sensor/ motornumber.
     * Must be called before use. The random generator is optional.
     */
    virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

    /**
     * Returns number of sensors
     *
     * @return Number of sensors the controller was initialised with or 0 if
     *         not initialised
     */
    virtual int getSensorNumber() const;

    /**
     * Returns number of motors
     *
     * @return Number of motors the controller was initialised with or 0 if not
     * initialised
     */
    virtual int getMotorNumber() const;

    /**
     * Performs one step (includes learning)
     *
     * Calculates motor commands from sensor inputs.
     *
     * @param sensors sensors inputs scaled to [-1,1]
     * @param sensornumber length of the sensor array
     * @param motors motors outputs. MUST have enough space for motor values!
     * @param motornumber length of the provided motor array
    */
    virtual void step(const sensor*, int number_sensors, motor*,
        int number_motors);

    /**
     * Performs one step without learning
     *
     * @see step
     */
    virtual void stepNoLearning(const sensor*, int number_sensors,
        motor*, int number_motors);

    void setRobotMaxSpeed(const double& screwMax, const double& jointMax);

    /** stores the controller values to a given file. */
    virtual bool store(FILE* f) const;

    /** loads the controller values from a given file. */
    virtual bool restore(FILE* f);

    double getJointAngle(JointType type, unsigned int index);
    double getJointGoalAngle(JointType type, unsigned int index);
    double getJointLoad(JointType type, unsigned int index);
    double getJointSpeed(JointType type, unsigned int index);
    double getJointTemperature(JointType type, unsigned int index);
    double getJointTorqueLimit(JointType type, unsigned int index);
    double getJointVoltage(JointType type, unsigned int index);
    double getRotationUnitAngle(unsigned int index);

    void setJointGoalAngle(JointType type, unsigned int index, const double angle);
    void setJointMovingSpeed(JointType type, unsigned int index, const double speed);
    void setJointTorqueEnabled(JointType type, unsigned int index, const bool enabled);
    void setJointTorqueLimit(JointType type, unsigned int index, const double value);
    void setRotationUnitSpeed(unsigned int index, const double speed);

  private:

    struct JointData {
      double angle;
      double goalAngle;
      double load;
      double movingSpeed;
      double speed;
      double temperature;
      bool torqueEnabled;
      double torqueLimit;
      double voltage;
    };

    struct ScrewData {
        double angle;
        double speed;
    };

    std::map<JointType, std::map<int, JointData> > joints;
    std::map<int, ScrewData> screws;

    NejihebiControllerBase* controller;
    static const int screwnumber = 4;

    double screwMaxSpeed;
    double jointMaxSpeed;
};


#endif /* GOROBOTS_CONTROLLER_NEJIHEBI_NEJIHEBIBUFFER_H_ */
