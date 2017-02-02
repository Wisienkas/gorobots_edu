// include simulation environment stuff
#include <ode_robots/simulation.h>
// include agent (class for holding a robot, a controller and a wiring)
#include <ode_robots/odeagent.h>
// playground
#include <ode_robots/playground.h>

#include <ode_robots/terrainground.h>

// simple wiring
#include <selforg/one2onewiring.h>
// the robot
#include <ode_robots/amosII.h>

// joint needed for fixation of the robot in the beginning
#include <ode_robots/joint.h>

// add head file for creating a sphere by Ren ------------
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>
#include <selforg/abstractcontroller.h>
#include <ode_robots/color.h>
#include <iostream>
#include <terrainGenerator.h>

using namespace std;
using namespace lpzrobots;

double fRand ( double fMin, double fMax )
{
    double f = ( double ) rand() / RAND_MAX;
    return fMin + f * ( fMax - fMin );
}

float TerrainGenerator::setupTerrain (
    const lpzrobots::OdeHandle& odeHandle,
    const lpzrobots::OsgHandle& osgHandle,
    lpzrobots::GlobalData& global,
    TerrainType terrain,
    float terrain_noise_prcnt,
    float height_scaling_factor)
{

    // declare terrain params
    double terrain_roughness;
    double terrain_slip;
    double terrain_hardness;
    double terrain_elasticity;
    double terrain_height;
    int terrain_height_map;
    Color terrain_color;

    // define individual terrain parameters here
    switch ( terrain ) {
    case concrete:	// concrete
// 	setTitle("concrete");
        terrain_roughness = 10.0;
        terrain_slip = 0.0;
        terrain_hardness = 100.0;
        terrain_elasticity = 0.0;
        terrain_height = 0.5;
        terrain_height_map = 3;
        terrain_color = Color ( 156.0/255.0,159.0/255.0,166.0/255.0 );
        break;
    case mud:	// mud
//         setTitle ( "mud" );
        terrain_roughness = 0.5;
        terrain_slip = 5.0;
        terrain_hardness = 0.5;
        terrain_elasticity = 0.5;
        terrain_height = 0.2;
        terrain_height_map = 1;
        terrain_color = Color ( 100.0/255.0,100.0/255.0,100.0/255.0 );
        break;
    case ice:	// ice
//         setTitle ( "ice" );
	terrain_roughness = 0.0;
	terrain_slip = 100.0;
	terrain_hardness = 100.0;
	terrain_elasticity = 0.0;
	terrain_height = 0.0;
        terrain_height_map = 2;
        terrain_color = Color ( 215.0/255.0,227.0/255.0,255.0/255.0 );
        break;
    case rock:	// rock
//         setTitle ( "rock" );
        terrain_roughness = 10.0;
        terrain_slip = 0.0;
        terrain_hardness = 100.0;
        terrain_elasticity = 0.0;
        terrain_height = 0.5;
        terrain_height_map = 2;
        terrain_color = Color ( 110.0/255.0,90.0/255.0,60.0/255.0 );
        break;
    case sand:	// sand
//         setTitle ( "sand" );
        terrain_roughness = 1.0;
        terrain_slip = 0.1;
        terrain_hardness = 30.0;
        terrain_elasticity = 0.0;
        terrain_height = 0.5;
        terrain_height_map = 1;
        terrain_color = Color ( 242.0/255.0,238.0/255.0,124.0/255.0 );
        break;
    case gravel:	// gravel
//         setTitle ( "gravel" );
        terrain_roughness = 7.0;
        terrain_slip = 0.1;
        terrain_hardness = 100.0;
        terrain_elasticity = 0.0;
        terrain_height = 0.3;
        terrain_height_map = 1;
        terrain_color = Color ( 115.0/255.0,127.0/255.0,156.0/255.0 );
        break;
    case grass:	// grass
//         setTitle ( "grass" );
        terrain_roughness = 5.0;
        terrain_slip = 0.0;
        terrain_hardness = 30.0;
        terrain_elasticity = 0.6;
        terrain_height = 0.1;
        terrain_height_map = 1;
        terrain_color = Color ( 35.0/255.0,150.0/255.0,20.0/255.0 );
        break;
    case swamp:	// swamp
//         setTitle ( "swamp" );
        terrain_roughness = 0.0;
        terrain_slip = 5.0;
        terrain_hardness = 0.0;
        terrain_elasticity = 0.0;
        terrain_height = 0.1;
        terrain_height_map = 1;
        terrain_color = Color ( 50.0/255.0,75.0/255.0,50.0/255.0 );
        break;
    case tiles:	// tiles
//         setTitle ( "tiles" );
        terrain_roughness = 5.0;
        terrain_slip = 30.0;
        terrain_hardness = 100.0;
        terrain_elasticity = 0.0;
        terrain_height = 0.1;
        terrain_height_map = 1;
        terrain_color = Color ( 250.0/255.0,200.0/255.0,150.0/255.0 );
        break;
    case snow: // snow
//         setTitle ( "snow" );
        terrain_roughness = 0.0;
        terrain_slip = 80.0;
        terrain_hardness = 20.0;
        terrain_elasticity = 0.0;
        terrain_height = 0.2;
        terrain_height_map = 1;
        terrain_color = Color ( 255.0/255.0,255.0/255.0,255.0/255.0 );
        break;
    case rubber: // rubber
//         setTitle ( "rubber" );
        terrain_roughness = 8.0;
        terrain_slip = 0.0;
        terrain_hardness = 80.0;
        terrain_elasticity = 2.0;
        terrain_height = 0.0;
        terrain_height_map = 1;
        terrain_color = Color ( 0.0/255.0,0.0/255.0,0.0/255.0 );
        break;
    case carpet: // carpet
//         setTitle ( "carpet" );
        terrain_roughness = 3.0;
        terrain_slip = 0.0;
        terrain_hardness = 40.0;
        terrain_elasticity = 0.3;
        terrain_height = 0.1;
        terrain_height_map = 1;
        terrain_color = Color ( 135.0/255.0,100.0/255.0,150.0/255.0 );
        break;
    case wood: // wood
//         setTitle ( "wood" );
        terrain_roughness = 6.0;
        terrain_slip = 0.0;
        terrain_hardness = 80.0;
        terrain_elasticity = 0.2;
        terrain_height = 0.1;
        terrain_height_map = 2;
        terrain_color = Color ( 90.0/255.0,65.0/255.0,0.0/255.0 );
        break;
    case plastic: // plastic
//         setTitle ( "plastic" );
        terrain_roughness = 1.0;
        terrain_slip = 2.0;
        terrain_hardness = 60.0;
        terrain_elasticity = 0.5;
        terrain_height = 0.0;
        terrain_height_map = 1;
        terrain_color = Color ( 150.0/255.0,250.0/255.0,190.0/255.0 );
        break;
    case foam: // foam
//         setTitle ( "foam" );
        terrain_roughness = 5.0;
        terrain_slip = 0.0;
        terrain_hardness = 0.0;
        terrain_elasticity = 0.7;
        terrain_height = 0.1;
        terrain_height_map = 1;
        terrain_color = Color ( 220.0/255.0,230.0/255.0,150.0/255.0 );
        break;
    default:
//         setTitle ( "default" );
        terrain_roughness = 10.0;
        terrain_slip = 0.0;
        terrain_hardness = 100.0;
        terrain_elasticity = 0.0;
        terrain_height = 0.0;
        terrain_height_map = 1;
        terrain_color = Color ( 255.0/255.0,255.0/255.0,255.0/255.0 );
    }

    if ( terrain_noise_prcnt > 0.0 ) {
        // Adding noise
        terrain_roughness += fRand ( -10.0*terrain_noise_prcnt, 10.0*terrain_noise_prcnt );
        terrain_slip  += fRand ( -10.0*terrain_noise_prcnt, 10.0*terrain_noise_prcnt );
        terrain_hardness += fRand ( -100.0*terrain_noise_prcnt, 100.0*terrain_noise_prcnt );
        terrain_elasticity += fRand ( -2.0*terrain_noise_prcnt, 2.0*terrain_noise_prcnt );
        terrain_height += fRand ( -0.1*terrain_noise_prcnt, 0.1*terrain_noise_prcnt );
    }

    // limits
    terrain_roughness = max ( 0.0, terrain_roughness );
    terrain_slip = max ( 0.0, terrain_slip );
    terrain_hardness = max ( 0.0, terrain_hardness );
    terrain_elasticity = max ( 0.0, terrain_elasticity );
    terrain_height = max ( 0.01, terrain_height * height_scaling_factor );

    string terrainFile = heightMapPrefix + to_string ( terrain_height_map ) + heightMapSufix;
    cout << terrain_roughness << ", " << terrain_slip << ", " << terrain_hardness << ", " << terrain_elasticity << ", " << terrain_height << endl;

    //**************Change Material substance*********//
    Substance roughterrainSubstance ( terrain_roughness, terrain_slip, terrain_hardness, terrain_elasticity ); //(roughness(0:smooth/1:rough), slip(0:friction/100:sliding), hardness(0:soft/100:hard), elasticity(0:hard/1:elastic))
    OdeHandle oodeHandle = odeHandle;
    oodeHandle.substance = roughterrainSubstance;
    //**************Change Material substance*********//

    float playgroundSize = 140;
    float playgroundXYfactor = 10;
    float playgroundWallThickness = 0.1;
    float terrainObjectSpacing = 0.005;

    Playground* playground = new Playground ( odeHandle, osgHandle,osg::Vec3 ( playgroundSize, .2, 5 ), playgroundXYfactor, false);
//     
// //     playground->setGroundColor(terrain_color);
// //     playground->setGroundSubstance(roughterrainSubstance);
// //     playground->setGroundTexture(terrainFile);
//     
    playground->setColor ( Color ( 1., .784, .082, .3 ) );
    playground->setPosition ( osg::Vec3 ( .0, ( ( playgroundSize*playgroundXYfactor ) /2. ) - playgroundSize/2., .1 ) );

    // Adding playground to obstacles list
    global.obstacles.push_back ( playground );

    for ( int j = 0; j < playgroundXYfactor; j++ ) {
        TerrainGround* terrainground =
            new TerrainGround ( oodeHandle, osgHandle.changeColor ( terrain_color ),terrainFile,"", /*AREA-SIZE-X->*/playgroundSize - (2*playgroundWallThickness + 2*terrainObjectSpacing), /*AREA-SIZE-Y->*/playgroundSize-terrainObjectSpacing, /*HEIGHT->*/terrain_height, OSGHeightField::Sum);
        terrainground->setPose ( osg::Matrix::translate ( 0,j*playgroundSize,0 ) );
        global.obstacles.push_back ( terrainground );
    }
    
    return terrain_height;
}
