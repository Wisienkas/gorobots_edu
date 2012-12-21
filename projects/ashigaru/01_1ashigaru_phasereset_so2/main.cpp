/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
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
 *   $Log$
 *   Revision 1.4  2011-06-03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.3  2009/08/05 23:25:57  martius
 *   adapted small things to compile with changed Plotoptions
 *
 *   Revision 1.2  2008/05/01 22:03:55  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.1  2007/03/05 10:18:24  fhesse
 *   nimm2_eating created
 *
 ***************************************************************************/

/*
1, template_ashigaru
 this is the program for 1 ashigaru
*/

#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>
#include <ode_robots/octaplayground.h>
#include <ode_robots/passivesphere.h>

#include <selforg/invertmotornstep.h>
#include <selforg/invertmotorspace.h>
#include <selforg/invertnchannelcontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/stl_adds.h>

#include "ode_robots/ashigaru.h"
#include "testController.h"

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace ASHIGARU;

class ThisSim : public Simulation {
protected:
	Ashigaru* ashigaru;
	AbstractController* controller;
	lpzrobots::Joint* robotFixator;

public:
  // Constructor
  ThisSim():Simulation(),ashigaru(0),controller(0),robotFixator(0)
  {}

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
  {
    //setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));
    setCameraHomePos(
        lpzrobots::Pos(-0.0114359, 1.66848, 0.922832),
        lpzrobots::Pos(0.866, -0.43884, 0)
    );

    // initialization
    // - set noise to 0.1
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)
    global.odeConfig.noise=0.0;
    //    global.odeConfig.setParam("gravity", 0);
    global.odeConfig.setParam("controlinterval", 10);
    global.odeConfig.setParam("simstepsize", 0.01);

    // use Playground as boundary:
    OctaPlayground* playground = new OctaPlayground(odeHandle, osgHandle, osg::Vec3(100, 0.2, 0.4), 4);
    playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
    global.obstacles.push_back(playground);

    // The definition of the values
    //OdeRobot* ashigaru;
    AbstractWiring* wiring;
    OdeAgent* agent;

    // make wireing
    wiring = new One2OneWiring(new ColorUniformNoise(0.1));

    // make controller
    TestControllerConf confy;
    confy = TestController::getDefaultConf();
    controller = new TestController();//  TestController::TestController(confy);
    controller->setParam("eps",0.1);
    controller->setParam("factor_a",0.01);

    // make robot
    AshigaruConf conf = Ashigaru::getDefaultConf();
    ashigaru = new Ashigaru(odeHandle, osgHandle, conf, "Ashigaru");
    ashigaru->setColor(Color(1.0,1.0,0));
    ashigaru->place(osg::Matrix::translate(.0, .0, 0.1));

    // make agent
    agent = new OdeAgent(global);
    agent->init(controller, ashigaru, wiring);

    // create a fixed joint to hold the robot in the air at the beginning SUSPEND
    robotFixator = new lpzrobots::FixedJoint(
        ashigaru->getMainPrimitive(),
        global.environment);

    robotFixator->init(odeHandle, osgHandle, false);

    //pause = true;


    // make connection and something
    global.configs.push_back(ashigaru);
    global.agents.push_back(agent);
    global.configs.push_back(controller);



    //pause = true;

  }

  //Function die eingegebene Befehle/kommandos verarbeitet
  virtual bool command (const OdeHandle&, const OsgHandle&, GlobalData& globalData, int key, bool down)
  {
    if (!down) return false;    
    bool handled = false;
    FILE* f;
    switch ( key )
    {
      case 's' :
    	  f=fopen("controller","wb");
    	  controller->store(f) && printf("Controller stored\n");
    	  fclose(f);
    	  handled = true; break;
      case 'l' :
    	  f=fopen("controller","rb");
    	  controller->restore(f) && printf("Controller loaded\n");
    	  handled = true; break;
      case 'x':
        if (robotFixator) {
          std::cout << "dropping robot" << std::endl;
          delete robotFixator;
          robotFixator = NULL;
        }
        break;

	fclose(f);
    }

    fflush(stdout);
    return handled;
  }

  virtual void bindingDescription(osg::ApplicationUsage & au) const {
    au.addKeyboardMouseBinding("Teachung: t","toggle mode");
    au.addKeyboardMouseBinding("Teaching: u","forward");
    au.addKeyboardMouseBinding("Teaching: j","backward");
    au.addKeyboardMouseBinding("Simulation: s","store");
    au.addKeyboardMouseBinding("Simulation: l","load");
  }

  
};

int main (int argc, char **argv)
{ 
  ThisSim sim;
  // run simulation
  return sim.run(argc, argv) ? 0 : 1;
}
 
