#pragma once
#include "Arduino.h"

struct CubeOrientation {
  uint8_t top;
  uint8_t bottom;
  uint8_t front;
  uint8_t back;
  uint8_t left;
  uint8_t right;
};


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


  // index: topSideRaw (0–5)
  // sorrend: {top, bottom, front, back, left, right}
  const uint8_t cubeMap[6][6] = {
    // top = 0
    {0, 5, 1, 4, 2, 3},

    // top = 1
    {1, 4, 5, 0, 2, 3},

    // top = 2
    {2, 3, 1, 4, 5, 0},

    // top = 3
    {3, 2, 1, 4, 0, 5},

    // top = 4
    {4, 1, 0, 5, 2, 3},

    // top = 5
    {5, 0, 1, 4, 3, 2}
  };

  CubeOrientation getFullOrientation();
  void updateStability(float roll, float pitch);
  float getRotationSpeed(float gyroX, float gyroY, float gyroZ);


  void updateRotationState(float gyroX, float gyroY, float gyroZ);
  uint8_t rotationAxis;
  uint8_t rotationDirection;
  uint8_t rotationSpeed; 

  bool isStable, balancesOnEdge, balancesOnCorner;

};
