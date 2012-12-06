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

#include "nejihebicontrollerbase.h"
#include <sstream>

void NejihebiControllerBase::addInspectableValue(std::string name,
    double const* value, std::string description) {
  InspectableValue v;
  v.name        = name;
  v.description = description;
  v.value       = value;
  inspectableValues.push_back(v);
}

NejihebiControllerBase::InspectableValueList
NejihebiControllerBase::getInspectableValues() {
  return inspectableValues;
}

std::string NejihebiControllerBase::itos(const int& i) {
  std::stringstream s;
  s << i;
  return s.str();
}

