#include "toolbox.h"
#define _USE_MATH_DEFINES
#include <math.h>

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
}
