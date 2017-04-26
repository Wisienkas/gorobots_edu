#include "braiten_berg.h"

#include "toolbox.h"
#include <cmath>

namespace tribot {
  BraitenBerg::BraitenBerg(double weight): weight(weight) {
  }

  Output BraitenBerg::calculateOutput(double angle) {
    std::vector<std::vector<double>> values = toolbox::sinewaveSampling(angle,
                                                                        SAMPLES,
                                                                        SAMPLE_RATE,
                                                                        FREQUENCY);
    std::vector<double> leftSampling = values.front();
    std::vector<double> rightSampling = values.back();

    return normalizedEarOutput(leftSampling, rightSampling);
  }

  /**
   * Braitenberg use
   *  - the left input to regulate the right motor
   *  - the right input to regulate the left motor
   * That is why the outputs are shifted around
   */
  Output BraitenBerg::normalizedEarOutput(std::vector<double> left, std::vector<double> right) {
    lizardEar.filter(left, right);

    Output output;
    output.right = std::log10(lizardEar.sumR) * DB_SCALAR;
    output.left = std::log10(lizardEar.sumL) * DB_SCALAR;

    return output;
  }
}
