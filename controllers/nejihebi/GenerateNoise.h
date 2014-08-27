/*
 * GenerateNoise.h
 *
 *  Created on: 13 Jun 2014
 *      Author: Chatterjee
 */

#ifndef __GenerateNoise_H
#define __GenerateNoise_H

#include <math.h>
#include <stdlib.h>
#include <map>
#include <utility>
#include <vector>

using namespace std;

class StatisticalDistribution {
 public:
  StatisticalDistribution();
  virtual ~StatisticalDistribution();

  // Distribution functions
  double pdf(double x);
  double cdf(double x);

  // Inverse cumulative distribution functions (quantile function)
  double inv_cdf(double quantile);

  // Descriptive mena, variance set
  double mean();
  double var();
  double stdev();

  // Obtain a sequence of random draws from this distribution
  vector<double> random_draws(vector<double> uniform_draws,
		  vector<double> dist_draws);
  vector<double> standardNoise(int rollout);
};

#endif


