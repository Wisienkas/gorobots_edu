#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <ctime>

// ode_robots
#include <ode_robots/base.h>
#include <ode_robots/simulation.h>
#include <ode_robots/odeagent.h>
#include <ode_robots/playground.h>
#include <ode_robots/passivebox.h>

// selforg
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>

//ga-mpi
#include "mpi.h"
#include <ga-mpi/ga.h>
#include <ga-mpi/std_stream.h>

#include <legWheelBotPopulationEvaluator.h>

// The robot
#include "legWheelBot.h"

// The controller
#include "legWheelBotDifferentialDriveController.h"

// The simmulation
#include "legWheelSim.h"

using namespace lpzrobots;
using namespace std;

//Used for MPI to parrelelize evolution
int mpi_tasks, mpi_rank;

// Default values for genetic algorithm
int popsize = 10;
int ngen = 10;
float pmut   = 0.05;
float pcross = 0.65;
unsigned int seed = 1234;
int simTimeMinutes = 1;

//indicating whether we should execute a testrun or real runEvolution
int testnumber = 0;

// Used if program is launched for a single genome evaluation
bool onlyeval;
TerrainType evalterrain;
float rsl, lsl;
int rns, lns;
std::string resultsFileName;

// Method declarations
bool parseArguments ( int argc, char **argv );
const vector<string> explode ( const string& s, const char& c );
const int findIdx ( vector<string> vector, string& element );
int testRun1 ( int argc, char **argv );
int testRun2 ( int argc, char **argv );
int doSingleGenomeEvaluation ( int argc, char **argv );
int runEvolution ( int argc, char **argv );

int main ( int argc, char **argv )
{
    if ( !parseArguments ( argc, argv ) ) { // Means arguments for the program were wrong
        std::cerr << "Error parsing arguments\n";
        return 1;
    }
    
    if(onlyeval) { // Means this program is spawned to perform a single genome evaluation
      return doSingleGenomeEvaluation(argc, argv);
    }
    
    else if ( testnumber == 0 ) { // Means the -testrun argument was not provided and we should run an evolution
        return runEvolution ( argc, argv );
	
    } else { // The testrun argument was specified so we execute a testrun method // Just used for debugging during development
        switch ( testnumber ) {
        case 1:
            return testRun1 ( argc,argv );
        case 2:
            return testRun2 ( argc,argv );
        }
    }

    return 0;
}

// This method is used to evaluate a single genome in a single terrains
// It's called from behind the scenes and should usually not be used directley
int doSingleGenomeEvaluation ( int argc, char **argv )
{
    std::ofstream resultsFile;
    resultsFile.open (resultsFileName, std::ios::out);

    //Get robot configuration
    LegWheelBotConf conf = LegWheelBot::getDefaultConf();
    conf.leftWheel.spokeLength = lsl;
    conf.leftWheel.noOfSpokes = lns;
    conf.rightWheel.spokeLength = rsl;
    conf.rightWheel.noOfSpokes = rns;
    
    // New simulation
    LegWheelSim* sim = new LegWheelSim ( evalterrain, conf );
    sim->setTitle ( "Legwheelbot sim " + AllTerrainTypeNames[evalterrain] );

    //Run simulation
    bool simSucess = sim->run ( Evaluator_argc, Evaluator_argv );

    float partialScore = 0.;
    if ( simSucess ) {
        partialScore = std::abs ( sim->controller->travelledDist );
	resultsFile << partialScore;
    }
    
    resultsFile.close();
    
    return simSucess ? 10 : 0;
}

// Returns true if arguments were sucessfully parsed and false otherwise
bool parseArguments ( int argc, char **argv )
{
    if(argc == 1) { // No arguments were passed to the program
        testnumber = 1; // Just execute the first test
        cout << "No arguments were passed to the program. Will just execute a test simulation";
        return true;
    }

    int popsizeIdx = Base::contains ( argv, argc, "-popsize" );
    int ngenIdx = Base::contains ( argv, argc, "-ngen" );
    int pmutIdx = Base::contains ( argv, argc, "-pmut" );
    int pcrossIdx = Base::contains ( argv, argc, "-pcross" );
    int testnumberIdx = Base::contains ( argv, argc, "-testrun" );
    // -r is used as random seed argument for lpzrobots
    // specifying a random seed, the evolution will be exactly the same each time you use that seed number
    int seedIdx = Base::contains ( argv, argc, "-r" );
    int terrainsToEvalIdx = Base::contains ( argv, argc, "-terrains" );
    int simtimeIdx = Base::contains ( argv, argc, "-simtime" );
    onlyeval = Base::contains ( argv, argc, "-onlyeval" ) != 0;
    int onlyEvalTerrainIdx = Base::contains( argv, argc, "-evalterrain" );
    int lslIdx = Base::contains ( argv, argc, "-lsl" );
    int lnsIdx = Base::contains ( argv, argc, "-lns" );
    int rslIdx = Base::contains ( argv, argc, "-rsl" );
    int rnsIdx = Base::contains ( argv, argc, "-rns" );
    int resultFileidx = Base::contains ( argv, argc, "-resultfile" );

    if ( popsizeIdx != 0 ) {
        popsize = atoi ( argv[popsizeIdx] );
    }
    if ( ngenIdx != 0 ) {
        ngen = atoi ( argv[ngenIdx] );
    }
    if ( seedIdx != 0 ) {
        seed = atoi ( argv[seedIdx] );
    }
    if ( pmutIdx != 0 ) {
        pmut = atof ( argv[pmutIdx] );
    }
    if ( pcrossIdx != 0 ) {
        pcross = atof ( argv[pcrossIdx] );
    }
    if ( testnumberIdx != 0 ) {
        testnumber = atoi ( argv[testnumberIdx] );
    } else if ( simtimeIdx == 0 ) {
        //If not test then ensure simtime is specified
        cerr << "You must specify the -simtime argument.\n";
        return false;
    }

    if ( terrainsToEvalIdx != 0 ) {
        vector< string > terrains = explode ( argv[terrainsToEvalIdx],'-' );
        vector<TerrainType> terrainsEnum;
        for ( string t : terrains ) {
            int idx = findIdx ( AllTerrainTypeNames, t );
            if ( idx != -1 ) {
                Evaluator_TerrainsToEvaluate.push_back ( AllTerrains[idx] );
            }
        }
    } else if ( testnumberIdx == 0 ) {
        std::cerr << "No argument specified for terains. Eg. like this: -terrains rock-grass-ice\n";
        return false;
    }
    
    if(onlyeval != 0) {
      string terrain = argv[onlyEvalTerrainIdx];
      evalterrain = AllTerrains[findIdx ( AllTerrainTypeNames, terrain )];
      lsl = atof ( argv[lslIdx] );
      lns = atoi ( argv[lnsIdx] );
      rsl = atof ( argv[rslIdx] );
      rns = atoi ( argv[rnsIdx] );
      resultsFileName = argv[resultFileidx];
    }

    //Pass on the arguments to the pop evaluator
    Evaluator_argc = argc;
    Evaluator_argv = argv;

    return true;  
}

//Run a simple visual test simulation
int testRun1 ( int argc, char **argv )
{
    std::cout << "Testrun 1...\n";
    for(int i = 0; i < argc; i++) {
       std::cout << argv[i] << " ";
    }
    std::cout << "\n";
    
    LegWheelBotConf conf = LegWheelBot::getDefaultConf();
    conf.leftWheel.spokeLength=1.45968;
    conf.leftWheel.noOfSpokes=29;
    conf.rightWheel.spokeLength=2.5;
    conf.rightWheel.noOfSpokes=22;
    
    // New simulation
    LegWheelSim* sim = new LegWheelSim ( rock, conf);

    // Set Title of simulation
    sim->setTitle ( "Test simulation 2" );

    // Simulation begins
    int exitcode = sim->run ( argc, argv ) ? 0 : 1;

    cout << "Exitcode was from the simmulation was " << exitcode;

    return 0;
}

// Tests all terraintypes visually
int testRun2 ( int argc, char **argv )
{
    for ( TerrainType t : AllTerrains ) {
        std::cout << "Testrun 2..." << t;
        LegWheelSim* sim = new LegWheelSim ( t );
        sim->setTitle ( "Test simulation 2" );
        if ( !sim->run ( argc, argv ) ) {
            return 1;
        }
    }
    return 0;
}

// Run a batch of evolution in parralell using MPI
int runEvolution ( int argc, char **argv )
{
    // MPI init
    MPI_Init ( &argc, &argv );
    MPI_Comm_size ( MPI_COMM_WORLD, &mpi_tasks );
    MPI_Comm_rank ( MPI_COMM_WORLD, &mpi_rank );


    std::cout << "popsize: " + std::to_string ( popsize );
    std::cout << "mpitasks: " + std::to_string ( mpi_tasks );
    std::cout << "mpiranks: " + std::to_string ( mpi_rank );

    // popsize / mpi_tasks must be an integer
    popsize = mpi_tasks * int ( ( double ) popsize/ ( double ) mpi_tasks + 0.999 );

    std::cout << "popsize after: " + std::to_string ( popsize );

    // Create the phenotype for four variables.  The number of bits you can use to
    // represent any number is limited by the type of computer you are using.
    // For this case we use 5 or 10 bits for each variable in the genome
    GABin2DecPhenotype map;

    LegWheelBotConf defaultConf = LegWheelBot::getDefaultConf();
    float minSpokeLength = defaultConf.bodyHeight/2; //Any shorter and the robot wheels wont reach the ground
    float maxSpokeLength = 2.5; // Any higher and the robot tilts because it gets too high
    float maxNoOfSpokes = 30; // More than this and the simmulation becomes much slower

    //Left wheel spokelength
    map.add ( /*nbits->*/ 10, /*minvalue->*/minSpokeLength, /*maxvalue->*/maxSpokeLength );
    //Left wheel number of spokes
    map.add ( 5, 1, maxNoOfSpokes );
    //Right wheel spokeLength
    map.add ( 10, minSpokeLength, maxSpokeLength );
    //Right wheel number of spokes
    map.add ( 5, 1, maxNoOfSpokes );

    // Create the template genome using the phenotype map we just made.
    GABin2DecGenome genome ( map, LegWheelBotPopulationEvaluator::objective );

    // Now create the GA using the genome and run it.
    GAPopulation pop ( genome, popsize );
    pop.evaluator ( LegWheelBotPopulationEvaluator::evaluate );

    GASimpleGA ga ( pop );
    GALinearScaling scaling;
    ga.nGenerations ( ngen );
    ga.pMutation ( pmut );
    ga.pCrossover ( pcross );
    ga.elitist ( _GABoolean::gaTrue );
    ga.scaling ( scaling );

    // Make sure we have a folder for holding temporary data during evolution and a folder where results are put to
    if ( mpi_rank == 0 ) {
	system("mkdir -p results_legwheelbot");
	system("mkdir -p tmp_data_legwheelbot");
    } else {
        ga.scoreFilename ( "/dev/null" );
    }

    ga.nBestGenomes ( 1 );
    ga.scoreFrequency ( 1 );
    ga.flushFrequency ( 1 );
    ga.selectScores ( GAStatistics::AllScores );
    
    // Pass MPI data to the GA class
    ga.mpi_rank ( mpi_rank );
    ga.mpi_tasks ( mpi_tasks );
    ga.evolve ( seed );

    // Dump the GA results to file
    if ( mpi_rank == 0 ) {
        GAPopulation bestPopulation = ga.population().best(); //ga.statistics().bestPopulation();

        for ( int i = 0; i < bestPopulation.size(); i++ ) {
            genome = bestPopulation.individual ( i );

            printf ( "GA result:\n" );
            printf ( "lsl = %f, lns = %f, rsl = %f, rns = %f\n",
                     genome.phenotype ( 0 ), genome.phenotype ( 1 ), genome.phenotype ( 2 ), genome.phenotype ( 3 ) );

            // ### Use below code if you are running in an environment which support graphics to view the final best legwhellbot ###
	    //LegWheelSim sim ;
            // sim.setTitle("Best individual");
            // sim.x = genome.phenotype(0);
            // sim.y = genome.phenotype(1);

	    // int argc = 1;
	    // char* argv[] = {"sim"};
	    // sim.run(argc, argv) ? 0 : 1;
        }
    }

    MPI_Finalize();

    return 0;
}

// Utility method to split a string on a certain char and return a vector of the split string
const vector<string> explode ( const string& s, const char& c )
{
    string buff {""};
    vector<string> v;

    for ( auto n:s ) {
        if ( n != c ) {
            buff+=n;
        } else if ( n == c && buff != "" ) {
            v.push_back ( buff );
            buff = "";
        }
    }
    if ( buff != "" ) {
        v.push_back ( buff );
    }

    return v;
}

// Utility method to find and return the index of a given element in a vector
// Returns -1 if not found
const int findIdx ( vector<string> vector, string& element )
{
    auto it = std::find ( vector.begin(), vector.end(), element );
    if ( it == vector.end() ) {
        return -1;
    } else {
        return std::distance ( vector.begin(), it );
    }
}