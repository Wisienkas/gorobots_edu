#include <legWheelBotPopulationEvaluator.h>
#include <legWheelSim.h>
#include <legWheelBot.h>

#include <ga-mpi/ga.h>
#include <mpi.h>
#include <thread>
#include <future>

std::vector<TerrainType> Evaluator_TerrainsToEvaluate;
int Evaluator_argc;
char **Evaluator_argv;
std::ofstream resultsFile;
int rank;

void runEvaluation (int idx, TerrainType t, std::string filename, float lsl, int lns, float rsl, int rns, std::vector<int> &exitcodes);

void LegWheelBotPopulationEvaluator::evaluate ( GAPopulation &p )
{
    rank = p.vmpi_rank;

    if ( p.geneticAlgorithm()->statistics().initial() == 0. ) {
        //This is the very first initial generation so create new files
        resultsFile.open ( "results_legwheelbot/results_process_" + std::to_string ( rank ) + ".txt", std::ios::out );
        resultsFile << "generation,left_spoke_length,left_noof_spokes,right_spoke_length,right_noof_spokes";
        for ( TerrainType t : Evaluator_TerrainsToEvaluate ) {
            resultsFile << "," << AllTerrainTypeNames[t];
        }
        resultsFile << "\n";
    } else {
        resultsFile.open ( "results_legwheelbot/results_process_" + std::to_string ( rank ) + ".txt", std::ios::out | std::ios::app );
    }

    // MPI aux vars
    int mpi_rc;
    MPI_Status mpi_Stat;

    // Array to store the scores of the individuals
    float *mpi_score;
    mpi_score = ( float* ) malloc ( p.size() *sizeof ( float ) );

    // Each thread computes individuals from is to (ie-1)
    int is, ie;
    is = ( int ) ( ( float ) p.size() / ( float ) p.vmpi_tasks* ( ( float ) p.vmpi_rank ) );
    ie = ( int ) ( ( float ) p.size() / ( float ) p.vmpi_tasks* ( ( float ) p.vmpi_rank+1.0 ) );
    if ( ie>p.size() || p.vmpi_rank== ( p.vmpi_tasks-1 ) ) {
        ie = p.size();
    }

    // Individual loop
    for ( int i=is; i<ie; i++ ) {
        p.individual ( i ).evaluate();
        mpi_score[i] = p.individual ( i ).score();
    }

    // The master process:
    if ( p.vmpi_rank == 0 ) {
        // recives the partial scores
        for ( int i=1; i<p.vmpi_tasks; i++ ) {
            is = ( int ) ( ( float ) p.size() / ( float ) p.vmpi_tasks* ( ( float ) i ) );
            ie = ( int ) ( ( float ) p.size() / ( float ) p.vmpi_tasks* ( ( float ) i+1.0 ) );
            mpi_rc = MPI_Recv ( mpi_score+is, ie-is, MPI_FLOAT, i, 1, MPI_COMM_WORLD, &mpi_Stat );
        }
        // and sends the whole array
        for ( int i=1; i<p.vmpi_tasks; i++ ) {
            mpi_rc = MPI_Send ( mpi_score, p.size(), MPI_FLOAT, i, 1, MPI_COMM_WORLD );
        }
    }
    // The rest:
    else {
        // sends the partial computed scores
        mpi_rc = MPI_Send ( mpi_score+is, ie-is, MPI_FLOAT, 0, 1, MPI_COMM_WORLD );
        // and recieves the whole array
        mpi_rc = MPI_Recv ( mpi_score, p.size(), MPI_FLOAT, 0, 1, MPI_COMM_WORLD, &mpi_Stat );
    }

    // Update the scores of the individuals
    for ( int i=0; i<p.size(); i++ ) {
        p.individual ( i ).score ( mpi_score[i] );
    }

    resultsFile.close();
}

float LegWheelBotPopulationEvaluator::objective ( GAGenome& c )
{
    GABin2DecGenome &genome = ( GABin2DecGenome & ) c;

    //Get the generation nbr of this genome
    int generation = c.geneticAlgorithm()->generation();
    if ( c.geneticAlgorithm()->statistics().initial() != 0. ) {
        generation++;
    }

    //Initialize sore at 0
    float score = 0;

    //Get robot configuration
    LegWheelBotConf conf = LegWheelBot::getDefaultConf();
    conf.leftWheel.spokeLength = genome.phenotype ( 0 );
    conf.leftWheel.noOfSpokes = genome.phenotype ( 1 );
    conf.rightWheel.spokeLength = genome.phenotype ( 2 );
    conf.rightWheel.noOfSpokes = genome.phenotype ( 3 );

    resultsFile << generation
                << "," << conf.leftWheel.spokeLength
                << "," << conf.leftWheel.noOfSpokes
                << "," << conf.rightWheel.spokeLength
                << "," << conf.rightWheel.noOfSpokes;

    std::vector<std::thread> threads;
    std::vector<int> returnCodes(Evaluator_TerrainsToEvaluate.size());
    std::vector<std::string> filenames;

    int idx = 0;
    for ( TerrainType t : Evaluator_TerrainsToEvaluate ) {
        std::string paritalName = "tmp_data_legwheelbot/partial_data_" + std::to_string ( rank ) + "_" + std::to_string(t);
        filenames.push_back (paritalName);
	
        threads.push_back ( std::thread (runEvaluation,idx,t,paritalName,conf.leftWheel.spokeLength, conf.leftWheel.noOfSpokes,conf.rightWheel.spokeLength,conf.rightWheel.noOfSpokes,ref(returnCodes) ));
	idx++;
    }

    for ( int i = 0; i<threads.size(); i++ ) {
        threads[i].join();
        
        if ( returnCodes[i] != 10 ) {
            resultsFile << "," << 0.;
        } else {
            std::ifstream ifs ( filenames[i] );
            std::string partialResult ( ( std::istreambuf_iterator<char> ( ifs ) ),
                                        ( std::istreambuf_iterator<char>() ) );

            resultsFile << "," << partialResult;
            score += std::stof ( partialResult );
        }
    }

    resultsFile << "\n";

    return score;
}

void runEvaluation (int idx, TerrainType t, std::string filename, float lsl, int lns, float rsl, int rns, std::vector<int> &exitcodes)
{
      // Evaluation of a single genome is performed by calling the program again with the -onlyeval flag
      // This will launch the perform single evaluation method from the main.cpp
      // This ensures that if the simmulator fails it will not crash the main process running the genetic algorithm
      std::stringstream stream;
      stream << "./";
      for(int i = 0; i < Evaluator_argc; i++) {
	stream << Evaluator_argv[i] << " ";
      }
      stream << "-evalterrain " << AllTerrainTypeNames[t] << " "
      << "-onlyeval " << lsl << " "
      << "-lsl " << lsl << " "
      << "-lns " << lns << " "
      << "-rsl " << rsl << " "
      << "-rns " << rns << " "
      << "-resultfile " << filename;

    //std::cout <<"Command to execute = " << command << "\n";
    int returncode = system ( stream.str().c_str() );
    std::cout << "Exitcode = " << WEXITSTATUS ( returncode ) << "\n";

    exitcodes[idx] = ( WEXITSTATUS ( returncode ) );
}
