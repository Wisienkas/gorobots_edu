#include "jitter.h"

namespace tribot {
  Jitter::Jitter() : unif(0, 1) {}

  double Jitter::get() {
    return unif(re);
  }
}
