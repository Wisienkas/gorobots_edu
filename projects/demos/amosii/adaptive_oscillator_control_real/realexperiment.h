/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Göttingen                           *
 *    timo.nachstedt@gmail.com                                             *
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
 *   $Log: main.cpp,v $                                                    *
 *                                                                         *
 ***************************************************************************/

#ifndef REALEXPERIMENT_H_
#define REALEXPERIMENT_H_

#include <list>
#include "realexperimentglobaldata.h"
#include "selforg/plotoption.h"

namespace lpzrobots {

  class RealExperiment {
    public:
      typedef std::list<PlotOption> PlotOptionList;

      RealExperiment();
      virtual ~RealExperiment();
      int run(int argc, char** argv);
      void setControlFrequency(const double& f);
      void setNoise(const double& n);
    protected:
      virtual void start(RealExperimentGlobalData& globaldata, PlotOptionList plotoptions) = 0;
    private:
      int contains(char **list, int len, const char *str);
      long long usecs();

      double controlFrequency;
      RealExperimentGlobalData globaldata;
      double noise;
  };

}

#endif /* REALEXPERIMENT_H_ */
