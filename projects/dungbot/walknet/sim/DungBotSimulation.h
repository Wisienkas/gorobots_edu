/*****************************************************************************
*   "THE BEER-WARE LICENSE" (Revision 43):
*   This software was written by Theis Str�m-Hansen <thstroemhansen@gmail.com>
*   and Mathias Thor <mathias.thor@gmail.com>
*   As long as you retain this notice you can do whatever you want with it.
*   If we meet some day, and you think this stuff is worth it, you can buy me
*   a beer in return.
*
*   Should this software ever become self-aware, remember: I am your master
*****************************************************************************/

#ifndef ODE_ROBOTS_ROBOTS_DUNGBOTSIMULATION_H_
#define ODE_ROBOTS_ROBOTS_DUNGBOTSIMULATION_H_
#include <stdio.h>

#include <ode_robots/simulation.h>
#include <selforg/noisegenerator.h>
#include <ode_robots/odeagent.h>
#include <ode_robots/speedsensor.h>
#include <selforg/one2onewiring.h>
#include <ode_robots/playground.h>
#include <ode_robots/passivebox.h>
#include <ode_robots/passivesphere.h>

#include "dungbot.h"
#include "DungBotEmptyController.h"

namespace lpzrobots
{
	class DungBotSimulation : public Simulation
	{
		public:
			DungBotSimulation( void );
			virtual ~DungBotSimulation( void );
			virtual void start( const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global );
			virtual void addCallback( GlobalData& globalData, bool draw, bool pause, bool control );
			virtual bool command ( const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down );
			virtual void bindingDescription( osg::ApplicationUsage & au ) const ;
			virtual bool restart( const OdeHandle&, const OsgHandle&, GlobalData& globalData );

		protected:
			double simulation_time_seconds;
			int number_of_runs;
			int trial_number;
			OdeAgent* agent;
			DungBot* robot;
			DungBotEmptyController* controller;
			lpzrobots::Joint* robotfixator;

		private:
			void setSimulationDuration(double seconds);
			void instantiateAgent(GlobalData& global);
			void addPlayground(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global);
	};
} /* namespace lpzrobots */

#endif /* ODE_ROBOTS_ROBOTS_DUNGBOTSIMULATION_H_ */
