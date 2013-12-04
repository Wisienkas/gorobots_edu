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

#ifndef NEJIHEBICONTROLLER_H_
#define NEJIHEBICONTROLLER_H_

#include "controllers/nejihebi/nejihebicontrollerbase.h"
#include <map>

// forward declarations
class NejihebiInterface;

/**
 * Example Nejihebi Controller
 *
 * This is a very simple example controller for the Nejihebi Robot. It
 * demonstrates different movement gaits. Without further adjustments it can
 * be used for the simulated as wells as for the real machine.
 */
class NejihebiController : public NejihebiControllerBase{
public:
  NejihebiController(void);
  ~NejihebiController(void);
  void init(NejihebiInterface* nejihebi);
  void step();
private:
  NejihebiInterface * nejihebi;
  int t;
};

#endif
