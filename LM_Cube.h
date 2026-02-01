#pragma once
#include "Arduino.h"

#define DEFAULT_SIDE_REMAP {0, 1, 2, 3, 4, 5}

class LM_Cube
{
private:
  /* data */
public:
  LM_Cube(/* args */);
  ~LM_Cube();

  void begin();
  void process();

  // void compute_cube_state(float ,float , float , float , float , float );
  void getTopSide(float , float );

  void mapSides();
  void handleFlip();
  void update_leds(uint8_t topSide);

  int8_t sideRemap[6] = DEFAULT_SIDE_REMAP;
  bool apMode=false;
  uint8_t currentSideUnmapped = 1;
  uint8_t currentSide = 1;
  uint8_t lastSide = 0;
  // uint8_t lastTopSideTemp = 0;
  unsigned long lastFlipTiemstamp = 0;
  unsigned long lastFlipSecs = 0;
  // unsigned long lastFlipMillisTemp = 0;
  String lastFlipDateTime = "";
  double* angle = {};

  bool flipACK;

  uint8_t topSideRaw = 0;
  uint8_t topSide = 0;
};
