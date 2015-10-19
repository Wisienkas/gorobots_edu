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
 *   $Log: main.cpp,v $                                                    *
 *                                                                         *
 ***************************************************************************/

// include simulation environment stuff
#include <ode_robots/simulation.h>
// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>
// playground
#include <ode_robots/playground.h>
// simple wiring
#include <selforg/one2onewiring.h>

// joint needed for fixation of the robot in the beginning
#include <ode_robots/joint.h>

#include "../common/runbot.h"
#include "../common/muscleRunbotController.h"
#include "../common/runbotANNController.h"
#include <ode_robots/passivebox.h> //needed for obstacles

#include <ode_robots/terrainground.h>
class ThisSim: public lpzrobots::Simulation {
public:

	ThisSim() {
		addPaletteFile("colors/UrbanExtraColors.gpl");
		addColorAliasFile("colors/UrbanColorSchema.txt");
		// you can replace color mappings in your own file, see colors/UrbanColorSchema.txt
		// addColorAliasFile("myColorSchema.txt");
		//setGroundTexture("Images/wood.jpg"); // gets its color from the schema
		fixed = false;
		//setTitle("centered text");
		//setCaption("right aligned text");
	}

	/**
	 * starting function (executed once at the beginning of the simulation loop)
	 */
	virtual void start(const lpzrobots::OdeHandle& odeHandle,
			const lpzrobots::OsgHandle& osgHandle,
			lpzrobots::GlobalData& global) {

	  bool showRealController = false;
		// set initial camera position
    setCameraHomePos(
        lpzrobots::Pos(0, 0, 0.922832),
        lpzrobots::Pos(178.866, -7.43884, 0));

		// set simulation parameters
		global.odeConfig.setParam("controlinterval",2);
  //  global.odeConfig.setParam("gravity",-098.1);
		global.odeConfig.setParam("simstepsize", 0.002);

		//Adding Runbot
		//lpzrobots::RunbotConf myRunbotConf = lpzrobots::Runbot::getDefaultConf();

    lpzrobots::RunbotConf myRunbotConf = lpzrobots::Runbot::getDefaultConf();
    myRunbotConf.armLength = 100;
    myRunbotConf.scale = 0.01;
    printf("creating runbot\n");
    runbot = new lpzrobots::Runbot(odeHandle,
        osgHandle, myRunbotConf,
        "Runbot_with_muscles");

    printf("placing runbot\n");
    runbot->place(osg::Matrix::translate(0.0,0.0,0.0));
    printf("returned..");
    switchFixate();


    printf("fixating runbot\n");
    // create a fixed joint to hold the center of runbot stable to walk around
    if (myRunbotConf.armLength > 0){
      lpzrobots::FixedJoint* centerFix = new lpzrobots::FixedJoint(runbot->getCenterAnchor(),global.environment);
      centerFix->init(odeHandle, osgHandle, false);
    }


    controller = new MuscleRunbotController("muscleRunbotController","0.1");



    One2OneWiring* runWiring = new One2OneWiring(new ColorUniformNoise());

    lpzrobots::OdeAgent* runAgent = new lpzrobots::OdeAgent(global);
    runAgent->init(controller, runbot, runWiring);

		// inform global variable over everything that happened:
		global.configs.push_back(runbot);
		global.agents.push_back(runAgent);
		global.configs.push_back(controller);
	if (showRealController){
		runbot2 = new lpzrobots::Runbot(odeHandle,osgHandle, myRunbotConf,"Runbot_without_muscles");
		runbot2->place(osg::Matrix::translate(0.0,0.0,0.0));
		controller2 = new RunbotANNController("realRunbotController","0.1");
		if (myRunbotConf.armLength > 0){
			lpzrobots::FixedJoint* centerFix2 = new lpzrobots::FixedJoint(runbot2->getCenterAnchor(),global.environment);
    		centerFix2->init(odeHandle, osgHandle, false);
		}

		One2OneWiring* runWiring2 = new One2OneWiring(new ColorUniformNoise(),AbstractWiring::Controller,0,"Wiring2");
	    lpzrobots::OdeAgent* runAgent2 = new lpzrobots::OdeAgent(global);
		runAgent2->init(controller2, runbot2, runWiring2);

		// inform global variable over everything that happened:
		global.configs.push_back(runbot2);
		global.agents.push_back(runAgent2);
		global.configs.push_back(controller2);
    	lpzrobots::OdeHandle ode = odeHandle;
	    for (lpzrobots::Primitives::iterator i = runbot->getAllPrimitives().begin(); i < runbot->getAllPrimitives().end(); i++){
	        for (lpzrobots::Primitives::iterator j = runbot2->getAllPrimitives().begin(); j < runbot2->getAllPrimitives().end(); j++){

	        	lpzrobots::Primitive* x,* y;
	        	x = *i;
	        	y = *j;
	        	ode.addIgnoredPair(x,y);
	        }
	    }
	}

	}

	/**
	 * add own key handling stuff here, just insert some case values
	 */
	virtual bool command(const lpzrobots::OdeHandle&,
			const lpzrobots::OsgHandle&, lpzrobots::GlobalData& globalData,
			int key, bool down) {
		if (down) { // only when key is pressed, not when released
			switch (char(key)) {
			case 'x':
			  switchFixate();
				break;
			default:
				return false;
				break;
			}
		}
		return false;
	}
protected:
	AbstractController* controller;
	AbstractController* controller2;
	lpzrobots::Runbot* runbot;
	lpzrobots::Runbot* runbot2;
	bool fixed;

	void switchFixate(){
	  if (runbot){
	    if (fixed)  runbot->unFixate(globalData);
	    else runbot->fixate(globalData);
	    fixed = !fixed;
	  }
	}

	void create_step_for_runbot(double pos, double length, double height,osg::Vec3 center,double distance){
	  osg::Matrixd trans = osg::Matrixd::translate(distance,0,0);
	  osg::Matrixd rot = osg::Matrixd::rotate(pos,osg::Vec3d(0,0,-1));
    osg::Matrixd trans2 = osg::Matrixd::translate(center+osg::Vec3(0,0,height/2));
    lpzrobots::PassiveBox* the_step = new lpzrobots::PassiveBox(odeHandle,osgHandle.changeColor(lpzrobots::Color(1-(height*50),1/(height*200),1-(1/height*500))),osg::Vec3(0.5,length,height),0.0);
    the_step->setPose(trans*rot*trans2);
    globalData.obstacles.push_back(the_step);
	}

	void create_stairs_for_runbot(double heightStart,double heightStop, double fromPos, double toPos,unsigned int stairsCount,osg::Vec3 center, double dist){
	  double steprange = (toPos - fromPos)/(double)(stairsCount);
	  double stepheight = (heightStop-heightStart)/(double)(stairsCount);
	  for (int i = 0; i< stairsCount;i++)
	    create_step_for_runbot(fromPos+i*steprange,steprange*2,heightStart+i*stepheight,center,dist);
	}
};

int main(int argc, char **argv) {	ThisSim sim;
	sim.setGroundTexture("Images/flowerground2.rgb");
	return sim.run(argc, argv) ? 0 : 1;
}



