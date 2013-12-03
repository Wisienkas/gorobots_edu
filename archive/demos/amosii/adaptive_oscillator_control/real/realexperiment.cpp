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


#include "realexperiment.h"

#include "realexperimentconsole.h"
#include "cmdline.h"
#include <selforg/agent.h>
#include <cstring>
#include <sys/time.h>

namespace lpzrobots {

  RealExperiment::RealExperiment() {
    controlFrequency = 25;
    noise = 0.0;
  }

  RealExperiment::~RealExperiment() {

  }


  int RealExperiment::contains(char **list, int len, const char *str) {
    for (int i = 0; i < len; i++) {
      if (strcmp(list[i], str) == 0)
        return i + 1;
    }
    return 0;
  }

  int RealExperiment::run(int argc, char** argv) {
    std::list<PlotOption> plotoptions;
    if (int index = contains(argv, argc, "-g"))
      if (argc > index)
        plotoptions.push_back(PlotOption(GuiLogger, atoi(argv[index])));
    if (contains(argv, argc, "-f")) plotoptions.push_back(PlotOption(File));
    if (contains(argv, argc, "-n")) plotoptions.push_back(PlotOption(MatrixViz));
    if (contains(argv, argc, "-h")) {
      std::cout << "Usage: " << argv[0] << " [-g N] [-f] [-n] [p PORT]\n"
       << "\t-g N\tstart guilogger with interval N\n\t-f\twrite logfile\n"
       << "\t-n\tstart neuronviz\n"
       << "\t-h\tdisplay this help\n";
      exit(0);
    }

    initializeConsole();

    start(globaldata,plotoptions);

    showParams(globaldata.configs);
    std::cout << "\nPress c to invoke parameter input shell\n"
              << "The output of the program is more fun then useful ;-).\n"
              << "The number are the sensors and the position there value.\n"
              << "You probably want to use the guilogger with e.g.: -g 1\n";

    cmd_handler_init();
    long int t=0;

    long long start = usecs();
    long long pausetime = 0;
    bool stop = false;
    while(!stop) {
      t++;
      globaldata.time = usecs()-start-pausetime;
      while ( globaldata.time*1e-6*controlFrequency<double(t)){}

      for (unsigned int i=0; i<globaldata.agents.size(); i++)
        globaldata.agents[i]->step(noise,t);


      //int key = wgetch (stdscr);
      if(control_c_pressed()) {
        long long pausestart = usecs();
        if(!handleConsole(globaldata)) stop=true;
        cmd_end_input();
        pausetime+=usecs()-pausestart;
      }
    };

    for (unsigned int i=0; i<globaldata.agents.size(); i++)
      delete globaldata.agents[i];

    closeConsole();

    const double elapsed_secs  = 1e-6 * globaldata.time;
    const double frequency     = double(t)/elapsed_secs;
    std::cout << std::endl;
    std::cout << "average frequency = " << frequency << " Hz" <<std::endl;

    return 0;
  }

  void RealExperiment::setControlFrequency(const double& f) {
    controlFrequency = f;
  }

  void RealExperiment::setNoise(const double& n) {
    noise = n;
  }

  long long RealExperiment::usecs() {
    timeval time;
    gettimeofday(&time, 0);
    return 1e6*time.tv_sec+time.tv_usec;
  }
}
