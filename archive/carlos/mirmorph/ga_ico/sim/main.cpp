#include "mirmorphsimulation.h"
#include <ea.h>

int argc_;
char** argv_;
int sim = 0;

double ffunction(Chromosome<double> *a){
    std::cout << "Simulation: " << sim++ << std::endl;
    MirmorphSimulation simulation(*a);

    double time = simulation.getResults(argc_, argv_);

    return time;
}

int main(int argc, char** argv){
    argc_ = argc;
    argv_ = argv;

    EA<double> *c;

    std::string str(argv[argc-1]);
    std::string generations_str(argv[argc-2]);
    int generations;
    if(!str.compare("-checkpoint")){
        std::cout << "Starting evolution from checkpoint" << std::endl;

        generations = std::stoi(generations_str.substr(1));

        c = new EA<double>("checkpoint.csv");
    }
    else{
        generations = std::stoi(str.substr(1));

		std::ofstream file;
		file.open("results.log", std::ios_base::out | std::ios_base::trunc);
		file.close();

        c = new EA<double>(20, 10, VALUE, 0.0, 1.0, FLAT, 0.75, UNIFORM, 0.5, RANK, 16, ELITISM, false);
        c->randomInitialization();
    }

    std::cout << generations << std::endl;

    c->setFitnessFunction(ffunction);
    c->printPopulation();
    c->plot_live = false;

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    Chromosome<double> *res = c->evolve(generations);
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();

    c->printPopulation();

    std::cout << std::endl << res->getFitness() << std::endl;

    std::cout << "Duration: " << duration << std::endl;

    return 0;
}
