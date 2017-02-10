// Header guard
#ifndef __TERRAINGENERATOR_H
#define __TERRAINGENERATOR_H
#include "ode_robots/odeagent.h"
#include "ode_robots/odehandle.h"

// Below is a very hackish way to use parralell arrays, 
// because c++ does not support conversions between strings and enum values like Java and C# 
enum TerrainType {
  concrete, 
  mud,
  ice,
  sand,
  gravel,
  grass,
  swamp,
  rock,
  tiles,
  snow,
  rubber,
  carpet,
  wood,
  plastic,
  foam
};

static const std::vector<TerrainType> AllTerrains = {
    concrete, 
    mud,
    ice,
    sand,
    gravel,
    grass,
    swamp,
    rock,
    tiles,
    snow,
    rubber,
    carpet,
    wood,
    plastic,
    foam
};

static const std::vector<std::string> AllTerrainTypeNames = {
    "concrete", 
    "mud",
    "ice",
    "sand",
    "gravel",
    "grass",
    "swamp",
    "rock",
    "tiles",
    "snow",
    "rubber",
    "carpet",
    "wood",
    "plastic",
    "foam"
};

// Where are the terrain heigh ppm files located
static std::string heightMapPrefix = "terrains/";
static std::string heightMapSufix = ".ppm";

class TerrainGenerator {
  public:
  // returns the height of the terrain
    static float setupTerrain (
      const lpzrobots::OdeHandle& odeHandle,
      const lpzrobots::OsgHandle& osgHandle,
      lpzrobots::GlobalData& global,
      TerrainType terrain,
      float terrain_noise_prcnt = 0.0,
      float height_scaling_factor = 1);
};

#endif