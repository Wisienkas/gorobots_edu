// Header guard
#ifndef __LEGWHEELSIM_H
#define __LEGWHEELSIM_H

#include <legWheelBot.h>

//#include "utils/sim_robots/legwheelbot/legWheelBot.h"
#include <legWheelBotDifferentialDriveController.h>
#include <ode_robots/simulation.h>
#include <terrainGenerator.h>
#include <ga-mpi/ga.h>

using namespace lpzrobots;

class LegWheelSim : public lpzrobots::Simulation 
{
public:
  LegWheelBotDifferentialDriveController* controller;
  TerrainType terrainType;
  LegWheelBotConf conf;
  
  LegWheelSim(TerrainType terrainType, LegWheelBotConf conf = LegWheelBot::getDefaultConf());
  ~LegWheelSim();
    
  virtual void start(const OdeHandle&, const OsgHandle&, GlobalData& globalData);  
};

#endif // Header guard
