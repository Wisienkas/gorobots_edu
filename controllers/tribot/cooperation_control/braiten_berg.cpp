#include "braiten_berg.h"

#include <cmath>
#include "sound.h"

namespace tribot {
  BraitenBerg::BraitenBerg(SoundGenerator soundGenerator): soundGenerator(soundGenerator) {
  }

  Output BraitenBerg::calculateOutput(double angle) {
    Sound sounds = soundGenerator.sample(angle,
                                         SAMPLES,
                                         SAMPLE_RATE,
                                         FREQUENCY);
    lizardEar.filter(sounds.left, sounds.right);
    return Output(std::log10(lizardEar.sumL) * DB_SCALAR, std::log10(lizardEar.sumR) * DB_SCALAR);
  }

}
