template <class T>
Chromosome<T>::Chromosome(encoding_type Chromosome_encoding_type, T min, T max)
{
    this->chromosome_encoding = Chromosome_encoding_type;
    fitness_value = 0;
    evaluated = false;
    size = 0;
    lower_boundary = min;
    upper_boundary = max;
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator = std::default_random_engine(seed);
    chromosome_values.clear();
}

template <class T>
Chromosome<T>::Chromosome()
{
    lower_boundary = 0;
    upper_boundary = 1;
    size = 1;
    fitness_value = 0;
    evaluated = false;
    chromosome_encoding = encoding_type::BINARY;
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator = std::default_random_engine(seed);
    chromosome_values.clear();
}

template <class T>
Chromosome<T>::Chromosome(const Chromosome& chromosome_copy)
{
    lower_boundary = chromosome_copy.lower_boundary;
    upper_boundary = chromosome_copy.upper_boundary;
    size = chromosome_copy.size;
    fitness_value = chromosome_copy.fitness_value;
    evaluated = true;
    chromosome_encoding = chromosome_copy.chromosome_encoding;
    generator = chromosome_copy.generator;
    for (unsigned int value = 0; value < size; ++value) {
      chromosome_values[value] = chromosome_copy.chromosome_values[value];
    }
}

template <class T>
Chromosome<T>& Chromosome<T>::operator=(const Chromosome& chromosome_copy)
{
    lower_boundary = chromosome_copy.lower_boundary;
    upper_boundary = chromosome_copy.upper_boundary;
    size = chromosome_copy.size;
    fitness_value = chromosome_copy.fitness_value;
    evaluated = true;
    chromosome_encoding = chromosome_copy.chromosome_encoding;
    generator = chromosome_copy.generator;
    for (unsigned int value = 0; value < size; ++value) {
      chromosome_values.push_back(chromosome_copy.chromosome_values[value]);
    }
    return *this;
}

template <class T>
Chromosome<T>::~Chromosome()
{
    //delete chromosome_values;
}

template <class T>
void Chromosome<T>::setChromosome(std::vector<T> values)
{
    size = values.size();
    //chromosome_values = new T[size];

    for (unsigned int i = 0; i < size; i++) {
      chromosome_values.push_back(values[i]);
    }
}

template <class T>
void Chromosome<T>::setChromosome(T values[], int size)
{
    this->size = size;
    chromosome_values = new T[size];

    for (unsigned int i = 0; i < size; i++) {
      chromosome_values[i] = values[i];
    }
}

template <class T>
std::vector<T> Chromosome<T>::getChromosome()
{
    return chromosome_values;
}

template <class T>
void Chromosome<T>::copyChromosome(std::vector<T>& chromosome_values)
{
    chromosome_values.clear();
    for (unsigned int gen = 0; gen < size; ++gen)
      chromosome_values.push_back(getChromosome()[gen]);
}

template <class T>
void Chromosome<T>::setFitness(double value)
{
    fitness_value = value;
    evaluated = true;
}

template <class T>
bool Chromosome<T>::isEvaluated(){
    return evaluated;
}

template <class T>
double Chromosome<T>::getFitness()
{
    return fitness_value;
}

template <class T>
void Chromosome<T>::bitStringMutation()
{
    double probability;
    for (unsigned int i = 0; i < size; i++) {
      probability = std::generate_canonical<double, std::numeric_limits<double>::digits>(generator);
      if (probability > 1.0 / size) {
        chromosome_values[i] = !chromosome_values[i];
      }
    }
}

template <class T>
void Chromosome<T>::flipBitMutation()
{
    for (unsigned int i = 0; i < size; i++) {
      chromosome_values[i] = !chromosome_values[i];
    }
}

template <class T>
void Chromosome<T>::boundaryMutation(double mutation_rate)
{
    double ud, probability;
    for (unsigned int i = 0; i < size; i++) {
      probability = std::generate_canonical<double, std::numeric_limits<double>::digits>(generator);
      if (probability < mutation_rate) {
        ud = std::generate_canonical<double, std::numeric_limits<double>::digits>(generator);
        if (ud > 0.5) {
          chromosome_values[i] = upper_boundary;
        }
        else {
          chromosome_values[i] = lower_boundary;
        }
      }
    }
}

template <class T>
void Chromosome<T>::uniformMutation(double mutation_rate)
{
    double probability;
    for (unsigned int i = 0; i < size; i++) {
      probability = std::generate_canonical<double, std::numeric_limits<double>::digits>(generator);
      if (probability < mutation_rate) {
        double random_value = std::generate_canonical<double, std::numeric_limits<double>::digits>(generator);
        chromosome_values[i] = random_value * (upper_boundary - lower_boundary) + lower_boundary;
      }
    }
}

template <class T>
void Chromosome<T>::gaussianMutation(double mutation_rate)
{
    double probability;
    std::normal_distribution<double> distribution = std::normal_distribution<double>((lower_boundary + upper_boundary) / 2, lower_boundary / 2 + upper_boundary);
    for (unsigned int i = 0; i < size; i++) {
      probability = std::generate_canonical<double, std::numeric_limits<double>::digits>(generator);
      if (probability < mutation_rate) {
        chromosome_values[i] += distribution(generator);
        if (chromosome_values[i] > upper_boundary)
          chromosome_values[i] = upper_boundary;
        else if (chromosome_values[i] < lower_boundary)
          chromosome_values[i] = lower_boundary;
      }
    }
}

template <class T>
void Chromosome<T>::printChromosome()
{
    for (unsigned int i = 0; i < size; i++) {
      std::cout << chromosome_values[i] << " ";
    }
    std::cout << std::endl;
}

template <class T>
EA<T>::EA(int population, int size, encoding_type chromosome_encoding, T min, T max, crossover_type crossover, double crossover_rate, mutation_type mutation, double mutation_rate, selection_type selection, int selection_size, replacement_type replacement)
{
    population_size = population;
    Chromosome_size = size;
    encoding = chromosome_encoding;
    lower_boundary = min;
    upper_boundary = max;
    this->crossover = crossover;
    this->mutation = mutation;
    this->mutation_rate = mutation_rate;
    this->selection = selection;
    this->crossover_rate = crossover_rate;
    if (selection_size % 2 != 0) {
      std::cerr << "The selection size has to be an even number." << std::endl;
    }
    this->selection_size = selection_size;
    this->replacement = replacement;
    generation = 0;

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator = std::default_random_engine(seed);
}

template <class T>
EA<T>::~EA()
{
}

template <class T>
void EA<T>::setFitnessFunction(double (*function)(Chromosome<T>*))
{
    fitnessFunction = function;
}

template <class T>
void EA<T>::randomInitialization()
{
    Chromosome<T>* c;
    for (unsigned int i = 0; i < population_size; i++) {
      c = new Chromosome<T>(encoding, lower_boundary, upper_boundary);
      std::vector<T> values;
      for (unsigned int j = 0; j < Chromosome_size; j++) {
        T value = (T)(std::generate_canonical<double, std::numeric_limits<double>::digits>(generator) * (upper_boundary - lower_boundary) + lower_boundary);
        values.push_back(value);
      }
      c->setChromosome(values);
      population.push_back(c);
    }
}

template <class T>
void EA<T>::initializePopulation(std::vector<Chromosome<T>*> initial_population)
{
    if (initial_population.size() != population_size) {
      std::cerr << "Wrong vector size. It doesn't match the population size!" << std::endl;
    }
    else {
      population = initial_population;
    }
}

template <class T>
void EA<T>::printPopulation()
{
    for (unsigned int i = 0; i < population.size(); i++) {
      std::cout << i + 1 << "(" << population[i]->getFitness() << ")"
          << ": ";
      population[i]->printChromosome();
    }
}

template <class T>
void EA<T>::evaluate()
{
    for (unsigned int i = 0; i < population.size(); i++) {
      if (!population[i]->isEvaluated()){
        population[i]->setFitness(fitnessFunction(population[i]));
      }
    }
}

template <class T>
void EA<T>::evaluateOffspring()
{
    for (unsigned int i = 0; i < offspring.size(); i++) {
      if (offspring[i]){
        offspring[i]->setFitness(fitnessFunction(offspring[i]));
      }
    }
}

template <class T>
void EA<T>::evaluateThreaded(unsigned int threads)
{
    boost::threadpool::fifo_pool tp(threads);
    std::vector<boost::threadpool::future<double>> results;
    std::vector<int> indexes;
    for (unsigned int i = 0; i < population.size(); i++) {
      if(!population[i]->isEvaluated()){
        std::function<double()> func = std::bind<double>(&EA<T>::evaluateOne, std::ref(*this), population[i]);
        results.push_back(boost::threadpool::schedule(tp, func));
        indexes.push_back(i);
      }
    }
    tp.wait();

    for(unsigned int i=0; i<indexes.size(); i++){
      population[indexes[i]]->setFitness(results[i].get());
    }
}

template <class T>
void EA<T>::evaluateOffspringThreaded(unsigned int threads)
{
    boost::threadpool::fifo_pool tp(threads);
    std::vector<boost::threadpool::future<double>> results;
    std::vector<int> indexes;
    for (unsigned int i = 0; i < offspring.size(); i++) {
      if(offspring[i]){
        std::function<double()> func = std::bind<double>(&EA<T>::evaluateOne, std::ref(*this), offspring[i]);
        results.push_back(boost::threadpool::schedule(tp, func));
        indexes.push_back(i);
      }
    }
    tp.wait();

    for(unsigned int i=0; i<indexes.size(); i++){
      offspring[indexes[i]]->setFitness(results[i].get());
    }
}

template <class T>
double EA<T>::evaluateOne(Chromosome<T>* i){
    return fitnessFunction(i);
}

template <class T>
void EA<T>::select()
{
    std::sort(population.begin(), population.end(), ChromosomeComp<T>);

    selected.clear();

    switch (selection) {
      case ROULETTE: {
        T roulette_sum = 0;
        for (unsigned int i = 0; i < population.size(); i++) {
          roulette_sum += population[i]->getFitness();
        }

        for (unsigned int i = 0; i < selection_size; i++) {
          std::uniform_real_distribution<double> distribution(0.0, roulette_sum);
          T roulette_rand = (T)distribution(generator);
          T pick_sum = 0;
          for (unsigned int j = 0; j < population.size(); j++) {
            pick_sum += population[j]->getFitness();
            if (pick_sum >= roulette_rand) {
              selected.push_back(population[j]);
              roulette_sum -= population[j]->getFitness();
              population.erase(population.begin() + j);
              break;
            }
          }
        }
        std::sort(selected.begin(), selected.end());
        break;
      }

      case RANK: {
        std::vector<double> ranks;
        double rank_sum = 0;
        for (unsigned int i = 0; i < population.size(); i++) {
          rank_sum += population[i]->getFitness();
        }
        for (unsigned int i = 0; i < population.size(); i++) {
          ranks.push_back(population[i]->getFitness() * 100.0 / rank_sum);
        }
        for (unsigned int i = 0; i < selection_size; i++) {
          std::uniform_int_distribution<int> distribution(0, 100);
          int rank_rand = distribution(generator);
          double pick_sum = 0;
          for (unsigned int j = 0; j < population.size(); j++) {
            pick_sum += ranks[j];
            if (pick_sum >= rank_rand) {
              selected.push_back(population[j]);
              population.erase(population.begin() + j);
              ranks.clear();
              rank_sum = 0;
              for (unsigned int k = 0; k < population.size(); k++) {
                rank_sum += population[k]->getFitness();
              }
              for (unsigned int k = 0; k < population.size(); k++) {
                ranks.push_back(population[k]->getFitness() * 100 / rank_sum);
              }
              break;
            }
          }
        }
        std::sort(selected.begin(), selected.end());
        break;
      }

      case TOURNAMENT: {
        std::vector<int> candidates;
        for (unsigned int i = 0; i < selection_size; i++) {
          std::uniform_int_distribution<int> distribution(0, population.size() - 1);
          for (unsigned int j = 0; j < (population_size - selection_size) / 2; j++) {
            candidates.push_back(distribution(generator));
          }
          int best = 0;
          double max = 0;
          for (unsigned int j = 0; j < candidates.size(); j++) {
            if (population[candidates[j]]->getFitness() > max) {
              max = population[candidates[j]]->getFitness();
              best = candidates[j];
            }
          }
          selected.push_back(population[best]);
          population.erase(population.begin() + best);
          candidates.clear();
        }
        std::sort(selected.begin(), selected.end());
        break;
      }
    }
}

template <class T>
void EA<T>::reproduce()
{
    offspring.clear();

    switch (crossover) {
      case SINGLE_POINT:
        for (unsigned int i = 0; i < selected.size(); i += 2) {
          double probability = std::generate_canonical<double, std::numeric_limits<double>::digits>(generator);
          if (probability < crossover_rate) {
            Chromosome<T>* child1 = new Chromosome<T>(encoding, lower_boundary, upper_boundary);
            Chromosome<T>* child2 = new Chromosome<T>(encoding, lower_boundary, upper_boundary);
            std::vector<T> values1, values2, parent1, parent2;
            selected[i]->copyChromosome(parent1);
            if ((i + 1) >= selected.size())
              selected[0]->copyChromosome(parent2);
            else
              selected[i + 1]->copyChromosome(parent2);
            for (unsigned int j = 0; j < Chromosome_size; j++) {
              if (j < Chromosome_size / 2) {
                values1.push_back(parent1[j]);
                values2.push_back(parent2[j]);
              }
              else {
                values1.push_back(parent2[j]);
                values2.push_back(parent1[j]);
              }
            }
            child1->setChromosome(values1);
            child2->setChromosome(values2);
            offspring.push_back(child1);
            offspring.push_back(child2);
          }
          else {
            offspring.push_back(NULL);
            offspring.push_back(NULL);
          }
        }
        break;

      case TWO_POINT:
        for (unsigned int i = 0; i < selected.size(); i += 2) {
          double probability = std::generate_canonical<double, std::numeric_limits<double>::digits>(generator);
          if (probability < crossover_rate) {
            Chromosome<T>* child1 = new Chromosome<T>(encoding, lower_boundary, upper_boundary);
            Chromosome<T>* child2 = new Chromosome<T>(encoding, lower_boundary, upper_boundary);
            std::vector<T> values1, values2, parent1, parent2;
            selected[i]->copyChromosome(parent1);
            if ((i + 1) >= selected.size())
              selected[0]->copyChromosome(parent2);
            else
              selected[i + 1]->copyChromosome(parent2);
            for (unsigned int j = 0; j < Chromosome_size; j++) {
              if (j < Chromosome_size / 3 || j > Chromosome_size * 2 / 3) {
                values1.push_back(parent1[j]);
                values2.push_back(parent2[j]);
              }
              else {
                values1.push_back(parent2[j]);
                values2.push_back(parent1[j]);
              }
            }
            child1->setChromosome(values1);
            child2->setChromosome(values2);
            offspring.push_back(child1);
            offspring.push_back(child2);
          }
          else {
            offspring.push_back(NULL);
            offspring.push_back(NULL);
          }
        }
        break;

      case UNIFORM_CROSSOVER: {
        std::vector<bool> mask;
        for (unsigned int i = 0; i < Chromosome_size; i++) {
          double probability = std::generate_canonical<double, std::numeric_limits<double>::digits>(generator);
          if (probability > 0.5) {
            mask.push_back(true);
          }
          else {
            mask.push_back(false);
          }
        }

        for (unsigned int i = 0; i < selected.size(); i += 2) {
          double probability = std::generate_canonical<double, std::numeric_limits<double>::digits>(generator);
          if (probability < crossover_rate) {
            Chromosome<T>* child1 = new Chromosome<T>(encoding, lower_boundary, upper_boundary);
            Chromosome<T>* child2 = new Chromosome<T>(encoding, lower_boundary, upper_boundary);
            std::vector<T> values1, values2, parent1, parent2;
            unsigned int random_parent1 = (std::rand() % (int)(selected.size()));
            unsigned int random_parent2 = (std::rand() % (int)(selected.size()));
            selected[random_parent1]->copyChromosome(parent1);
            selected[random_parent2]->copyChromosome(parent2);
            for (unsigned int j = 0; j < Chromosome_size; j++) {
              if (mask[j]) {
                values1.push_back(parent1[j]);
                values2.push_back(parent2[j]);
              }
              else {
                values1.push_back(parent2[j]);
                values2.push_back(parent1[j]);
              }
            }
            child1->setChromosome(values1);
            child2->setChromosome(values2);
            offspring.push_back(child1);
            offspring.push_back(child2);
          }
          else {
            offspring.push_back(NULL);
            offspring.push_back(NULL);
          }
        }
        break;
      }
    }

    for (unsigned int i = 0; i < offspring.size(); i++) {
      if (offspring[i]) {
        switch (mutation) {
          case BIT_STRING:
            offspring[i]->bitStringMutation();
            break;

          case FLIP_BIT:
            offspring[i]->flipBitMutation();
            break;

          case BOUNDARY:
            offspring[i]->boundaryMutation(mutation_rate);
            break;

          case NON_UNIFORM:
            offspring[i]->uniformMutation(1.0 / generation);
            break;

          case UNIFORM:
            offspring[i]->uniformMutation(mutation_rate);
            break;

          case GAUSSIAN:
            offspring[i]->gaussianMutation(mutation_rate);
            break;
        }
      }
    }
}

template <class T>
void EA<T>::replace()
{
    switch (replacement) {
      case GENERATIONAL:
        for (unsigned int i = 0; i < offspring.size(); i++) {
          if (offspring[i]) {
            population.push_back(offspring[i]);
          }
          else {
            population.push_back(selected[i]);
          }
        }
        break;

      case STEADY_STATE:
        for (unsigned int i = 0; i < offspring.size(); i += 2) {
          if (offspring[i] && offspring[i + 1]) {
            std::vector<Chromosome<T>*> replacing_order;
            replacing_order.push_back(selected[i]);
            replacing_order.push_back(selected[i + 1]);
            replacing_order.push_back(offspring[i]);
            replacing_order.push_back(offspring[i + 1]);
            std::sort(replacing_order.begin(), replacing_order.end(), ChromosomeComp<T>);

            population.push_back(replacing_order.back());
            replacing_order.pop_back();
            population.push_back(replacing_order.back());
            replacing_order.clear();
          }
          else {
            population.push_back(selected[i]);
            population.push_back(selected[i + 1]);
          }
        }
        break;

      case ELITISM:
        for (unsigned int i = 0; i < selected.size(); i++) {
          population.push_back(selected[i]);
        }
        std::sort(population.begin(), population.end(), ChromosomeCompInv<T>);
        for (unsigned int i = 0; i < offspring.size(); i++) {
          if (offspring[i]) {
            population.pop_back();
          }
        }
        for (unsigned int i = 0; i < offspring.size(); i++) {
          if (offspring[i]) {
            population.push_back(offspring[i]);
          }
        }
        break;
    }
}

template <class T>
void EA<T>::evolve(unsigned int generations, int solution)
{
    bool solution_found = false;
    for (unsigned int i = 0; i < generations; i++) {
      evaluate();
      for (unsigned int i = 0; i < population.size(); i++) {
        if (population[i]->getFitness() == solution) {
          solution_found = true;
          std::cout << generation << " generations. Found solution: ";
          population[i]->printChromosome();
        }
      }
      if (solution_found) {
        break;
      }

      generation++;

      evaluate();
      select();
      reproduce();
      replace();
    }
    std::cout << "Solution not found..." << std::endl;
}

template <class T>
Chromosome<T>* EA<T>::evolve(unsigned int generations)
{
    Gnuplot livePlot;
    livePlot << "set terminal wxt noraise\n";
    std::ofstream fitnessFile;
    fitnessFile.open("fitness_funtion.cvs", std::ios::trunc);
    fitnessFile << "Generation,Fitness value,";
    for (unsigned int gen_number = 0; gen_number < population[0]->getChromosome().size(); ++gen_number)
      fitnessFile << "Gen " << gen_number << ",";
    fitnessFile << std::endl;
    fitnessValuesMax.clear();
    evaluate();
    for (unsigned int i = 0; i < generations; i++) {

      generation++;

      select();
      reproduce();
      evaluateOffspring();
      replace();

      fitnessValuesMax.push_back(std::make_pair(generation, population[0]->getFitness()));
      fitnessFile << generation << "," << population[0]->getFitness();
      for (auto gen : population[0]->getChromosome())
        fitnessFile << "," << gen;
      fitnessFile << std::endl;

      livePlot << "plot '-' with lines title 'Fitness values'\n";
      livePlot.send1d(fitnessValuesMax);
      livePlot.flush();
    }
    fitnessFile.close();

    int best_index = 0;
    double max = 0;
    for (unsigned int i = 0; i < population.size(); i++) {
      if (population[i]->getFitness() > max) {
        max = population[i]->getFitness();
        best_index = i;
      }
    }

    return population[best_index];
}

template <class T>
Chromosome<T>* EA<T>::evolveThreaded(unsigned int generations, unsigned int threads)
{
    Gnuplot livePlot;
    livePlot << "set terminal wxt noraise\n";
    std::ofstream fitnessFile;
    fitnessFile.open("fitness_funtion.cvs", std::ios::trunc);
    fitnessFile << "Generation,Fitness value,";
    for (unsigned int gen_number = 0; gen_number < population[0]->getChromosome().size(); ++gen_number)
      fitnessFile << "Gen " << gen_number << ",";
    fitnessFile << std::endl;
    fitnessValuesMax.clear();

    evaluateThreaded(threads);
    for (unsigned int i = 0; i < generations; i++) {

      generation++;

      select();
      reproduce();
      evaluateOffspringThreaded(threads);
      replace();
      std::sort(population.begin(), population.end(), ChromosomeComp<T>);

      fitnessValuesMax.push_back(std::make_pair(generation, population.back()->getFitness()));
      fitnessFile << generation << "," << population.back()->getFitness();
      for (auto gen : population.back()->getChromosome())
        fitnessFile << "," << gen;
      fitnessFile << std::endl;

      livePlot << "plot '-' with lines title 'Fitness values'\n";
      livePlot.send1d(fitnessValuesMax);
      livePlot.flush();
    }

    fitnessFile.close();

    int best_index = 0;
    double max = population[0]->getFitness();
    for (unsigned int i = 0; i < population.size(); i++) {
      if (population[i]->getFitness() > max) {
        max = population[i]->getFitness();
        best_index = i;
      }
    }

    return population[best_index];
}

template <class T>
bool ChromosomeComp(Chromosome<T>* A, Chromosome<T>* B)
{
    return A->getFitness() < B->getFitness();
}

template <class T>
bool ChromosomeCompInv(Chromosome<T>* A, Chromosome<T>* B)
{
    return A->getFitness() > B->getFitness();
}
