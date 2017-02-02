// Header guard
#ifndef __LEGWHEELBOTPOPULATIONEVALUATOR_H
#define __LEGWHEELBOTPOPULATIONEVALUATOR_H

#include <ga-mpi/ga.h>
#include <terrainGenerator.h>

#include <iostream>
#include <fstream>

extern std::vector<TerrainType> Evaluator_TerrainsToEvaluate;
extern int Evaluator_argc;
extern char **Evaluator_argv;
extern std::ofstream resultsFile;
extern int rank;

class LegWheelBotPopulationEvaluator {
  public:
    LegWheelBotPopulationEvaluator();
    static void evaluate(GAPopulation &);
    static float objective(GAGenome &);
};

#endif // Header guard
