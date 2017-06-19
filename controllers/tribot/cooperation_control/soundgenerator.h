#ifndef __GOROBOTS_EDU_PROJECTS_TRIBOT_SOUND_GENERATOR_H
#define __GOROBOTS_EDU_PROJECTS_TRIBOT_SOUND_GENERATOR_H

#include "sound.h"

namespace tribot {

  class SoundGenerator {
    bool noise = false;
  public:
    SoundGenerator(bool);
    SoundGenerator();
    Sound sample(double angle, int samples, int samplerate, int frequency);
  };

}

#endif // end of __GOROBOTS_EDU_PROJECTS_TRIBOT_SOUND_GENERATOR_H
