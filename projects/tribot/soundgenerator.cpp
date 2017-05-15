#include "soundgenerator.h"

#include "sinewave.h"

namespace tribot {

  Sound SoundGenerator::sample(double angle, int samples, int samplerate, int frequency) {
    double leftPhaseshift = 0.0;
    double rightPhaseshift = tribot::generatePhaseshift(angle, frequency);

    tribot::Sinewave leftSinewave = tribot::Sinewave(frequency, leftPhaseshift);
    tribot::Sinewave rightSinewave = tribot::Sinewave(frequency, rightPhaseshift);

    double from = 0.0;
    double to = (double) samples / samplerate;
    double step = 1.0 / samplerate;

    Sound sounds;
    sounds.left = leftSinewave.sample(from, to, step);
    sounds.right = rightSinewave.sample(from, to, step);

    return sounds;
  }

}
