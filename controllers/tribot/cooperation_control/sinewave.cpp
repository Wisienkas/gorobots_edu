#include "sinewave.h"
#include <algorithm>
#include <functional>
#include <iostream>

namespace tribot {

  Jitter Sinewave::jitter = Jitter();

  double generatePhaseshift(double angle, double frequency) {
    return 2 * M_PI * frequency * tribot::EAR_DISTANCE_MM * sin(angle) / tribot::SPEED_OF_SOUND;
  }

  Sinewave::Sinewave(double frequency, double phaseshift, bool noise):
    frequency(frequency),
    phaseshift(phaseshift),
    noise(noise)
    {
  }

  std::vector<double> Sinewave::sample(std::vector<double> series) {
    std::vector<double> result;
    result.resize(series.size());

    std::transform(series.begin(),
                   series.end(),
                   result.begin(),
                   [this] (const double& x) { return this->sample(x); });
    return result;
  }

  std::vector<double> Sinewave::sample(double from, double to, double stepSize) {
    std::vector<double> result;
    for (double t = from; t < to; t += stepSize) {
      result.push_back(this->sample(t));
    }
    return result;
  }

  double Sinewave::sample(double t) {
    return sin(2.0 * M_PI * frequency * t + phaseshift + getNoise());
  }

  double Sinewave::getNoise() {
    return noise ? jitter.get() : 0.0;
  }

}
