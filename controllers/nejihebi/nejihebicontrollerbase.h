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

#ifndef GOROBOTS_CONTROLLER_NEJIHEBI_NEJIHEBICONTROLLERBASE_H_
#define GOROBOTS_CONTROLLER_NEJIHEBI_NEJIHEBICONTROLLERBASE_H_

// include some functionalities from standard library
#include <string>
#include <list>

// forward declarations
class NejihebiInterface;

/**
 * Base Class for Nejihebi Controllers
 *
 * This is the base class for nejihebi controllers that work for the simulated
 * robot as well as for the real machine. You should derive your controller from
 * this class and use the methods provided by the NejihebiInterface class to
 * controll the robot within the step() method.
 * You can furthermore use the system for inspectable values to make internal
 * values available for lpzrobots and e.g. the guilogger
 */
class NejihebiControllerBase {
  public:

    /**
     * Structure encapsulating relevant information for inspectable values.
     */
    struct InspectableValue {
      std::string name;
      std::string description;
      double const* value;
    };

    typedef std::list<InspectableValue> InspectableValueList;

    /**
     * The destructor
     */
    virtual ~NejihebiControllerBase() {}

    /**
     * Returns list of inspectable values
     *
     * This method is used by NejihebiLpzInterface to forward the inspectable
     * values to the Inspectable engine of lpzrobots.
     *
     * @return list of structures containing the information over the registered
     *         inspectable values.
     */
    InspectableValueList getInspectableValues();

    /**
     * Initialize the controller
     *
     * This method is called after the controller was created.
     *
     * @param nejihebi pointer to nejihebi robot interface to be controlled
     */
    virtual void init(NejihebiInterface* nejihebi) = 0;

    /**
     * Performs one control step
     *
     * Do all your robot control in this method
     */
    virtual void step() = 0;

  protected:
    /**
     * Registers a new inspectable value
     *
     * @param name Name of the inspectable value
     * @param value Pointer to the value
     * @param description Longer description of the inspectable value
     */
    void addInspectableValue(std::string name, double const* value,
        std::string description);

    /**
     * Convenience function to convert an integer to a string
     */
    std::string itos(const int& i);
  private:

    /**
     * Contains the list of registered inspectable values
     */
    InspectableValueList inspectableValues;
};


#endif /* GOROBOTS_CONTROLLER_NEJIHEBI_NEJIHEBICONTROLLERBASE_H_ */
