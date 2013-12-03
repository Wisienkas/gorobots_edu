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
5, template ashigaru5
 this is the program for straight connected ashigaru
and apply simple oscilator to ashieve the same result which I got before by SO2 Oscilator
*/

#include <sstream>
#include <iostream>

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
#include "testController5.h"

#define Num 3


// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace ASHIGARU;


class ThisSim : public Simulation {
protected:
	Ashigaru* ashiUnit[Num];
//	Ashigaru2* ashiUnit2;
//	Ashigaru2* ashiUnit3;
//	Ashigaru2* ashiUnit4;

	AbstractController* controller[Num];
	TestController5* tController[Num];

//	AbstractController* controller2;
//	AbstractController* controller3;
//	AbstractController* controller4;


	lpzrobots::Joint* envFixator;
	lpzrobots::Joint* robotFixator[ Num -1 ];

public:
  // Constructor
  ThisSim():Simulation() //ashiUnit1(0),controller(0),robotFixator(0)
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
    global.odeConfig.noise=0.05;
    //    global.odeConfig.setParam("gravity", 0);
    global.odeConfig.setParam("controlinterval", 5);
    global.odeConfig.setParam("simstepsize", 0.01);

    // use Playground as boundary:
    OctaPlayground* playground = new OctaPlayground(odeHandle, osgHandle, osg::Vec3(20, 0.2, 0.4), 4);
    playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
    global.obstacles.push_back(playground);

    // The definition of the values
    //OdeRobot* ashiUnit1;
    AbstractWiring* wiring[Num];
    //AbstractWiring* wiring2;
    //AbstractWiring* wiring3;

    OdeAgent* agent[Num];
    //OdeAgent* agent2;
    //OdeAgent* agent3;

    // make robot
    for(int i = 0; i< Num; i++){
    	// make wireing
    	wiring[i] = new One2OneWiring(new ColorUniformNoise(0.0));

    	// make controller
        TestController5Conf confy;
        confy = TestController5::getDefaultConf();
        if( 0){
        	confy.disLeg = true;
        }else{
        	confy.disLeg = false;
        }

        std::ostringstream ost;
        ost << "ashigaru" << i;
        std::string ashName = ost.str();
        char ashName2[30];
        strcpy(ashName2,ashName.c_str());

        tController[i] = new TestController5(ashName2, 0.1, confy);//  TestController::TestController(confy);
        controller[i] = tController[i];

        // make robot
        AshigaruConf conf = Ashigaru::getDefaultConf();

        // Put the robot on initial position
        //    this position is important because by this position, the connection relation is determined.
        ashiUnit[i] = new Ashigaru(odeHandle, osgHandle, conf, ashName2);
        ashiUnit[i]->setColor(Color(1.0,1.0,0));
        ashiUnit[i]->place( osg::Matrix::rotate(0., osg::Vec3(1, 0, 0)) * osg::Matrix::translate( ((double)i) * conf.rate * conf.connectLength, .0, 0.2));

        // make agent
        agent[i] = new OdeAgent(global);
        agent[i]->init(controller[i], ashiUnit[i], wiring[i]);

        // Create Joint
        if(i == 0){
            // create a fixed joint to hold the robot in the air at the beginning SUSPEND
            envFixator = new lpzrobots::FixedJoint(
                    ashiUnit[i]->getMainPrimitive(),
                    global.environment);
            envFixator->init(odeHandle, osgHandle, false);
        }else if(i > 0){
        	// Create ASHIGARU connection
        	robotFixator[i-1] = new lpzrobots::FixedJoint(
        			ashiUnit[i-1]->getTibiaPrimitive(Ashigaru::L0),
        			ashiUnit[i]->getMainPrimitive());
            	//global.environment);
        	robotFixator[i-1]->init(odeHandle, osgHandle, false);
        }

        // make connection and something
        global.configs.push_back(ashiUnit[i]);
        global.configs.push_back(controller[i]);
        global.agents.push_back(agent[i]);
    }

    // Observer Controller setting
    tController[0]->setObserverMode();
    for(int i = 0; i< Num; i++){
    	tController[0]->addObject(tController[i]);
    }

    /*
    // make wireing
    wiring[] = new One2OneWiring(new ColorUniformNoise(0.0));
    wiring2 = new One2OneWiring(new ColorUniformNoise(0.));
    wiring3 = new One2OneWiring(new ColorUniformNoise(0.0));


    // make controller
    TestController5Conf confy;
    confy = TestController5::getDefaultConf();
    confy.disLeg = false;
    controller = new TestController5("data1", 0.1, confy);//  TestController::TestController(confy);
    controller->setParam("eps",0.1);
    controller->setParam("factor_a",0.01);

    confy.disLeg = true;
    controller2 = new TestController5("data2", 0.1, confy);//  TestController::TestController(confy);
    controller2->setParam("eps",0.1);
    controller2->setParam("factor_a",0.01);

    confy.disLeg = false;
    controller3 = new TestController5("data3", 0.1, confy);//  TestController::TestController(confy);
    controller3->setParam("eps",0.1);
    controller3->setParam("factor_a",0.01);



    // make robot
    Ashigaru2Conf conf = Ashigaru2::getDefaultConf();

    // Put the robot on initial position
    //    this position is important because by this position, the connection relation is determined.
    ashiUnit1 = new Ashigaru2(odeHandle, osgHandle, conf, "Ashigaru1");
    ashiUnit1->setColor(Color(1.0,1.0,0));
    ashiUnit1->place( osg::Matrix::rotate(0., osg::Vec3(1, 0, 0)) * osg::Matrix::translate(.0, .0, 0.2));

    ashiUnit2 = new Ashigaru2(odeHandle, osgHandle, conf, "Ashigaru2");
    ashiUnit2->setColor(Color(1.0,0.5,0));
    ashiUnit2->place( osg::Matrix::rotate(0., osg::Vec3(1, 0, 0)) * osg::Matrix::translate(conf.rate * conf.connectLength, .0, 0.2));

    ashiUnit3 = new Ashigaru2(odeHandle, osgHandle, conf, "Ashigaru");
    ashiUnit3->setColor(Color(1.0,0.5,0));
    ashiUnit3->place(osg::Matrix::translate(2. * conf.rate * conf.connectLength, .0, 0.2));

    // make agent
    agent = new OdeAgent(global);
    agent->init(controller, ashiUnit1, wiring);

    agent2 = new OdeAgent(global);
    agent2->init(controller2, ashiUnit2, wiring2);

    agent3 = new OdeAgent(global);
    agent3->init(controller3, ashiUnit3, wiring2);


    // create a fixed joint to hold the robot in the air at the beginning SUSPEND
    envFixator = new lpzrobots::FixedJoint(
            ashiUnit1->getMainPrimitive(),
            global.environment);
    envFixator->init(odeHandle, osgHandle, false);

    // Create ASHIGARU connection
    robotFixator = new lpzrobots::FixedJoint(
        ashiUnit1->getTibiaPrimitive(Ashigaru2::L0),
        ashiUnit2->getMainPrimitive());
        //global.environment);
    robotFixator->init(odeHandle, osgHandle, false);



    robotFixator = new lpzrobots::FixedJoint(
        ashiUnit2->getTibiaPrimitive(Ashigaru2::L0),
        ashiUnit3->getMainPrimitive());
        //global.environment);
    robotFixator->init(odeHandle, osgHandle, false);


    //pause = true;


    // make connection and something
    global.configs.push_back(ashiUnit1);
    global.configs.push_back(ashiUnit2);
    global.configs.push_back(ashiUnit3);
    global.configs.push_back(controller);
    global.configs.push_back(controller2);
    global.configs.push_back(controller3);

    global.agents.push_back(agent);
    global.agents.push_back(agent2);
    global.agents.push_back(agent3);

     */

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
    	  controller[0]->store(f) && printf("Controller stored\n");
    	  fclose(f);
    	  handled = true; break;
      case 'l' :
    	  f=fopen("controller","rb");
    	  controller[0]->restore(f) && printf("Controller loaded\n");
    	  handled = true; break;
      case 'x':
        if (envFixator) {
          std::cout << "dropping robot" << std::endl;
          delete envFixator;
          envFixator = NULL;
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
 
