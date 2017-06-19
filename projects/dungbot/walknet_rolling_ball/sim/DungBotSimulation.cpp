/*****************************************************************************
*   "THE BEER-WARE LICENSE" (Revision 43):
*   This software was written by Theis Strøm-Hansen <thstroemhansen@gmail.com>
*   and Mathias Thor <mathias.thor@gmail.com>
*   As long as you retain this notice you can do whatever you want with it.
*   If we meet some day, and you think this stuff is worth it, you can buy me
*   a beer in return.
*
*   Should this software ever become self-aware, remember: I am your master
*****************************************************************************/

#include "DungBotSimulation.h"

namespace lpzrobots
{
	DungBotSimulation::DungBotSimulation( void )
	{
		setTitle("DUNGBOT SIM");
		setGroundTexture("../../../../../pmanoonpong-lpzrobots-fork/ode_robots/osg/data/Images/sand_texture.jpg"); //sand_texture.jpg //whiteground_crosses.jpg
		simulation_time_seconds = 0.0;
		trial_number = 0;
		agent = NULL;
	}

	DungBotSimulation::~DungBotSimulation( void )
	{
	}

	bool DungBotSimulation::command(const lpzrobots::OdeHandle&,
		  const lpzrobots::OsgHandle&,
		  lpzrobots::GlobalData& globalData,
		  int key,
		  bool down)
	{
		if( down )
		{
			// only when key is pressed, not when released
			switch (char(key))
			{
				case 'x':
					if( robotfixator )
					{
						std::cout << "Dropping robot" << std::endl;
						delete robotfixator;
						robotfixator = NULL;
					}
					break;
				case 'c':
					if( ballfixator )
					{
						std::cout << "Releasing ball" << std::endl;
						delete ballfixator;
						ballfixator = NULL;
					}
					break;
				default:
					return false;
					break;
			}
		}
		return false;
	}

	void DungBotSimulation::bindingDescription( osg::ApplicationUsage& au ) const
	{
	}

	void DungBotSimulation::start( const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global )
	{
		// Configure environment
		setCameraHomePos(Pos(-2.85247, 2.68423, 2.9186),  Pos(-131.376, -16.7443, 0));

		global.odeConfig.setParam( "controlinterval", 1 );
		global.odeConfig.setParam( "simstepsize", 0.01 );
		global.odeConfig.setParam( "gravity", -9.8 );

		//add playground
		//addPlayground(odeHandle,osgHandle,global);

		// Configure simulation
		simulation_time_seconds = 300; // Gaittest - 60s. Walk 300s.
		number_of_runs = 1;
		instantiateAgent( global );
	}

	void DungBotSimulation::instantiateAgent( GlobalData& global )
	{
		// Instantiate robot
		DungBotConf conf = DungBot::getDefaultConf();
		robot = new DungBot( odeHandle, osgHandle, conf, "Dungbot_Robot" );
		robot->place( Pos( 0.0, 0.0, 0.12 ) ); // No head = 0.21 --> Head = 12.

		// Instantiate controller
		controller = new DungBotEmptyController( "DungBotEmptyController" );

		// Create the wiring
		auto wiring = new One2OneWiring( new NoNoise() );

		// Create Agent
		agent = new OdeAgent( global );
		agent->init( controller, robot, wiring );
		global.agents.push_back( agent );
		global.configs.push_back( agent );

		setSimulationDuration( simulation_time_seconds );

		if(true){ // TODO IF TRUE, THEN A BALL SPAWNS
			PassiveSphere* s1 = new PassiveSphere(odeHandle, osgHandle, 0.27, 1, true);
			s1->setPosition(osg::Vec3(0.48, 0.0, 0.0));  // No head = 0.4 --> Head = 48.

			s1->setTexture("../../../../../pmanoonpong-lpzrobots-fork/ode_robots/osg/data/Images/ground_texture3.jpg");
			//Substance surface_ball(100,0,75,0);
			//s1->setSubstance( surface_ball );
			global.obstacles.push_back(s1);

			ballfixator = new FixedJoint(s1->getMainPrimitive(), global.environment);
			ballfixator->init(odeHandle, osgHandle, true);
		}

		// create a fixed joint to hold the robot in the air at the beginning
		robotfixator = new lpzrobots::FixedJoint(robot->getMainPrimitive(),global.environment);
		robotfixator->init(odeHandle, osgHandle, true);

		// inform global variable over everything that happened:
		global.configs.push_back(robot);
		global.agents.push_back(agent);
		global.configs.push_back(controller);

		std::cout << "\n"
		<<" ____                    ____        _     ____  _           \n"
		<<"|  _ \\ _   _ _ __   __ _| __ )  ___ | |_  / ___|(_)_ __ ___  \n"
		<<"| | | | | | | '_ \\ / _` |  _ \\ / _ \\| __| \\___ \\| | '_ ` _ \\ \n"
		<<"| |_| | |_| | | | | (_| | |_) | (_) | |_   ___) | | | | | | |\n"
		<<"|____/ \\__,_|_| |_|\\__, |____/ \\___/ \\__| |____/|_|_| |_| |_|\n"
		<<"                   |___/       "
				<< "Press X to free DungBot!\n" << std::endl;
	}


	void DungBotSimulation::addCallback( GlobalData& globalData, bool draw, bool pause, bool control )
	{
		if( globalData.sim_step >= simulation_time )
		{
			simulation_time_reached=true;
		}
	}

	bool DungBotSimulation::restart( const OdeHandle&, const OsgHandle&, GlobalData& global )
	{
		if( this->currentCycle == number_of_runs )
			return false;

		if( agent!=0 )
		{
			OdeAgentList::iterator itr = find( global.agents.begin(), global.agents.end(),agent );
			if ( itr!=global.agents.end() )
			{
				global.agents.erase( itr );
			}
			delete agent;
			agent = 0;
		}

		instantiateAgent( global );

		return true;
	}

	void DungBotSimulation::setSimulationDuration( double seconds )
	{
		simulation_time = (long)( seconds/0.01 );
	}

	void DungBotSimulation::addPlayground( const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
	{
		// implement playground here.
	    lpzrobots::Playground* playground = new lpzrobots::Playground(odeHandle, osgHandle, osg::Vec3(12, 0.05, 0.7),0.047);
	    Substance PGsubstance;
	    PGsubstance.toRubber( 50 );
	    playground->setGroundSubstance(PGsubstance);
	    //playground->setTexture(0,0,lpzrobots::TextureDescr("/home/mat/dark-mosaic.png",-1.5,-3));
	    playground->setGroundColor(Color(0.372, 0.737, 0.360));  //http://doc.instantreality.org/tools/color_calculator/
	    //playground->setColor(Color(0.737, 0.647, 0.360));
	    playground->setColor(Color(0.5,0.1,0.1,0.15)); // inner wall invisible
	    playground->setPosition( osg::Vec3( 5, 0, -0.2 ) );
	    global.obstacles.push_back( playground );
	}

} /* namespace lpzrobots */


