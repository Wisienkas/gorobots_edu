// Header Guard
#ifndef __GOROBOTS_EDU_PROJECTS_TRIBOT_TOOLBOX_H
#define __GOROBOTS_EDU_PROJECTS_TRIBOT_TOOLBOX_H

#include <vector>

namespace toolbox {

  /**
   * Will contain the positive remainder
   * for a radian value "theta"
   */
  double trimRadian(double theta);

  /**
   * Will convert radian to degrees
   */
  double toDegrees(double radian);

  /**
   * Converts degrees to radian
   */
  double toRadian(double degrees);

  /**
   * Phaseshift Values
   */
  std::vector<std::vector<double>> sinewaveSampling(double angle,
                                                    int samples,
                                                    int sample_rate,
                                                    int frequency);
}

#endif // end of __GOROBOTS_EDU_PROJECTS_TRIBOT_TOOLBOX_H
