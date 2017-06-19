#include "soundgenerator.h"

#include "sinewave.h"
#include <iostream>
namespace tribot {

  SoundGenerator::SoundGenerator(bool noise) {
    this->noise = noise;
  }

  SoundGenerator::SoundGenerator(){}

  Sound SoundGenerator::sample(double angle, int samples, int samplerate, int frequency) {
    double leftPhaseshift = 0.0;
    double rightPhaseshift = tribot::generatePhaseshift(angle, frequency);

    tribot::Sinewave leftSinewave = tribot::Sinewave(frequency, leftPhaseshift, noise);
    tribot::Sinewave rightSinewave = tribot::Sinewave(frequency, rightPhaseshift, noise);

    double from = 0.0;
    double to = (double) samples / samplerate;
    double step = 1.0 / samplerate;

    Sound sounds;
    sounds.left = leftSinewave.sample(from, to, step);
    sounds.right = rightSinewave.sample(from, to, step);

    return sounds;
  }

}
