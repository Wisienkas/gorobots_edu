#!/bin/bash
# TODO MAKE ONE FOR ROLLING ASWELL
if [[ $1 == s ]]; then
  # Change Makefile.conf
  sed -i 's/$(DUNGBOT)\/dungbot \\/$(DUNGBOT)\/dungbot_simple \\/g' Makefile.conf
  echo "changed Makefile.conf"
  # Change DungBotSimulation includes
  sed -i 's/#include <dungbot.h>/#include <dungbot_simple.h>/g' DungBotSimulation.h
  echo "changed DungBotSimulation.h"
  # Change walknetSeparateLeg.cpp
  sed -i 's/if(false){ \/\/ use the simple robot/if(true){ \/\/ use the simple robot/g' ./../../../../controllers/dungbot/walknetSeparateLeg.cpp
  sed -i 's/swingNet1( sensor, viaAngle, jointVel );/swingNetSimple( sensor, viaAngle, jointVel );/g' ./../../../../controllers/dungbot/walknetSeparateLeg.cpp
  sed -i 's/stanceNet1( sensor, viaAngle, jointVel );/stanceNetSimple( sensor, viaAngle, jointVel );/g' ./../../../../controllers/dungbot/walknetSeparateLeg.cpp
  echo "changed walknetSeparateLeg.cpp"
  # Change DungBotEmptyController
  sed -i 's/stand( angleVector );/standsimple( angleVector );/g' DungBotEmptyController.cpp
  sed -i 's/rollstand( angleVector );/standsimple( angleVector );/g' DungBotEmptyController.cpp
  echo "changed DungBotEmptyController.cpp"
elif [[ $1 == o ]]; then
  # Change Makefile.conf
  sed -i 's/$(DUNGBOT)\/dungbot_simple \\/$(DUNGBOT)\/dungbot \\/g' Makefile.conf
  echo "changed Makefile.conf"
  # Change DungBotSimulation includes
  sed -i 's/#include <dungbot_simple.h>/#include <dungbot.h>/g' DungBotSimulation.h
  echo "changed DungBotSimulation.h"
  # Change walknetSeparateLeg.cpp
  sed -i 's/if(true){ \/\/ use the simple robot/if(false){ \/\/ use the simple robot/g' ./../../../../controllers/dungbot/walknetSeparateLeg.cpp
  sed -i 's/if(false){ \/\/ Use for standard dungbot/if(true){ \/\/ Use for standard dungbot/g' ./../../../../controllers/dungbot/walknetSeparateLeg.cpp
  sed -i 's/if(true){ \/\/ use for rolling/if(false){ \/\/  use for rolling/g' ./../../../../controllers/dungbot/walknetSeparateLeg.cpp
  sed -i 's/swingNetSimple( sensor, viaAngle, jointVel );/swingNet1( sensor, viaAngle, jointVel );/g' ./../../../../controllers/dungbot/walknetSeparateLeg.cpp
  sed -i 's/stanceNetSimple( sensor, viaAngle, jointVel );/stanceNet1( sensor, viaAngle, jointVel );/g' ./../../../../controllers/dungbot/walknetSeparateLeg.cpp
  echo "changed walknetSeparateLeg.cpp"
  # Change DungBotEmptyController
  sed -i 's/standsimple( angleVector );/stand( angleVector );/g' DungBotEmptyController.cpp
  sed -i 's/rollstand( angleVector );/stand( angleVector );/g' DungBotEmptyController.cpp
  echo "changed DungBotEmptyController.cpp"
else
  echo "UNKNOWN ARGUMENT"
  echo "EXPECTS 'o' (original) or 's' (simple)"
fi

make clean
