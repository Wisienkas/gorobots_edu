// HEADER GUARD
#ifndef __GOROBOTS_EDU_PROJECTS_TRIBOT_SIMULATION_H
#define __GOROBOTS_EDU_PROJECTS_TRIBOT_SIMULATION_H

#include <ode_robots/taskedsimulation.h>
#include <ode_robots/abstractground.h>
#include <ode_robots/pos.h>
#include <ode_robots/globaldata.h>
#include <ode_robots/osghandle.h>
#include <ode_robots/simulationtaskhandle.h>
#include <selforg/position.h>
#include "tribotSimTaskHandle.h"
#include "tribot.h"
#include <ode_robots/odeagent.h>
#include <ode_robots/abstractobstacle.h>

#include <ode_robots/playground.h>
// Contains important piece of code to make playground.h compile apparently
#include <ode_robots/passivesphere.h>

#include <string>
#include <fstream>
#include <iostream>

namespace tribot {
  class TribotSimulation : public lpzrobots::TaskedSimulation {
  private:
    std::fstream * stream;
    TribotSimTaskHandle tribotSimTaskHandle;

    void initializeCamera(lpzrobots::Pos from, lpzrobots::Pos to);
    lpzrobots::Playground * initializePlayground(const lpzrobots::GlobalData& globalData,
                                      const lpzrobots::OdeHandle& odeHandle,
                                      const lpzrobots::OsgHandle& osgHandle);
    lpzrobots::OdeAgent * createRobot(const lpzrobots::OdeHandle& odeHandle,
                                      const lpzrobots::OsgHandle& osgHandle,
                                      lpzrobots::GlobalData& global,
                                      const TribotAgentConfig& agentConfig,
                                      const Position& goal);
    lpzrobots::AbstractObstacle * initializeGoal(const lpzrobots::OdeHandle& odeHandle,
                                           const lpzrobots::OsgHandle& osgHandle,
                                           lpzrobots::GlobalData& globalData);
    void initializeRobots(const lpzrobots::OdeHandle& odeHandle,
                          const lpzrobots::OsgHandle& osgHandle,
                          lpzrobots::GlobalData& global,
                          const Position& goal);
    void initializeSimulationParameters(const lpzrobots::GlobalData & globalData);
    std::string getLine(lpzrobots::Tribot * bot);
    void setupFileStream(std::string filename);
  public:
    TribotSimulation(TribotSimTaskHandle);
    ~TribotSimulation();

    /**
     * All objects, agents etc should be initialized in this method
     */
    virtual void start (const lpzrobots::OdeHandle &,
           const lpzrobots::OsgHandle &,
           lpzrobots::GlobalData &globalData,
           lpzrobots::SimulationTaskHandle &simTaskHandle,
           int taskId);

    /**
     * Allows for inspection at each step
     */
    virtual void addCallback (lpzrobots::GlobalData &globalData,
                 bool draw,
                 bool pause,
                 bool control,
                 lpzrobots::SimulationTaskHandle &,
                 int taskId);
    virtual bool restart(const lpzrobots::OdeHandle &,
                          const lpzrobots::OsgHandle &,
                          lpzrobots::GlobalData &globalData,
                          lpzrobots::SimulationTaskHandle &,
                          int taskId);
    virtual bool command(const lpzrobots::OdeHandle &,
                         const lpzrobots::OsgHandle &,
                         lpzrobots::GlobalData &globalData,
                         int key,
                         bool down,
                         lpzrobots::SimulationTaskHandle &,
                         int taskId);
  };
}

#endif // end of __GOROBOTS_EDU_PROJECTS_TRIBOT_SIMULATION_H
