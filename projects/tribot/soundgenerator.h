#ifndef __GOROBOTS_EDU_PROJECTS_TRIBOT_SOUND_GENERATOR_H
#define __GOROBOTS_EDU_PROJECTS_TRIBOT_SOUND_GENERATOR_H

#include "sound.h"

namespace tribot {

  class SoundGenerator {

  public:
    Sound sample(double angle, int samples, int samplerate, int frequency);
  };

}

#endif // end of __GOROBOTS_EDU_PROJECTS_TRIBOT_SOUND_GENERATOR_H
