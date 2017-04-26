#include "toolbox.h"
// Used to include things as the PI value as M_PI
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include "sinewave.h"

namespace toolbox {

  double trimRadian(double theta) {
    double mod = fmod(theta, M_PI * 2);
    if(mod < 0) {
      mod += (2 * M_PI);
    }
    return mod;
  }

  double toDegrees(double radian) {
    return radian * (180 / M_PI);
  }

  double toRadian(double degrees) {
    return degrees * (M_PI / 180);
  }

  /**
   * Algorithm as in matlab
   * ----------------------
   * sample_rate = 50000;
   * samples = 500;
   *
   * t = 0:1/sample_rate:sample_rate/samples;
   *
   * phase_shift_L = 0.0;
   * phase_shift_R = 2*pi*freq*13*sin(angle)/340000;
   *
   * sinewave_L = sin(2*pi*freq*t + phase_shift_L);
   * sinewave_R = sin(2*pi*freq*t + phase_shift_R);
   */
  std::vector<std::vector<double>> sinewaveSampling(double angle,
                                                    int samples,
                                                    int samplerate,
                                                    int frequency) {
    //std::cout << "Creating sinewaves\n";
    double leftPhaseshift = 0.0;
    double rightPhaseshift = tribot::generatePhaseshift(angle, frequency);

    //std::cout << "phase: " << rightPhaseshift << "\n";

    tribot::Sinewave leftSinewave = tribot::Sinewave(frequency, leftPhaseshift);
    tribot::Sinewave rightSinewave = tribot::Sinewave(frequency, rightPhaseshift);

    double from = 0.0;
    double to = (double) samples / samplerate;
    double step = 1.0 / samplerate;
    //std::cout << "Sampling...\n";

    std::vector<std::vector<double>> result;
    result.push_back(leftSinewave.sample(from, to, step));
    result.push_back(rightSinewave.sample(from, to, step));

    return result;
  }

  bool isSameSign(double a, double b) {
    if (std::fabs(a) < 0.001 || std::fabs(b) < 0.001) {
      return true;
    } else if (a > 0) {
      return b > 0;
    } else {
      return b < 0;
    }
  }
}
