// Header Guard
#ifndef __TOOLBOX_H
#define __TOOLBOX_H

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
}

#endif
