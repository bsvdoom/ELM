// #include "esphome.h"
#include "LM_Cube.h"
#include <math.h>



LM_Cube::LM_Cube(/* args */)
{
}

LM_Cube::~LM_Cube()
{
}


void LM_Cube::begin() {


}

void LM_Cube::process() {
  
}


void LM_Cube::update_leds(uint8_t topSide) {
  // Placeholder for LED update logic
  // This function should contain the code to update the LEDs based on the topSide parameter


}


void LM_Cube::getTopSide(float roll, float pitch) {
  if ( pitch > 45 && pitch < 135 ) { //roll left side // 3
    currentSideUnmapped = 3;
  }
  else if ( pitch > -135 && pitch < -45 ) { //roll right side // 4
    currentSideUnmapped = 4;
  }
  else if ( abs(roll) < 45 ) { //roll top // 1, 2, 5

    if (pitch < 45 && pitch > -45) { // 1
      currentSideUnmapped = 1;
    }

  }
  else if ( abs(roll) > 135 ) { //roll bottom side // 2, 6, 5

    if (pitch < 45 && pitch > -45) { // 6
      currentSideUnmapped = 6;
    }

  } else {
    if (roll > 45 && roll < 135 ) { // 5
      currentSideUnmapped = 2;
    }
    else if (roll < -45 && roll > -135) { // 2
      currentSideUnmapped = 5;
    }
  }
  
  if (currentSideUnmapped < 1 || currentSideUnmapped > 6) {
    currentSideUnmapped = 1;
  }

  topSideRaw = currentSideUnmapped - 1;

}


CubeOrientation LM_Cube::getFullOrientation() {
  CubeOrientation o;

  uint8_t t = topSideRaw;

  o.top    = cubeMap[t][0];
  o.bottom = cubeMap[t][1];
  o.front  = cubeMap[t][2];
  o.back   = cubeMap[t][3];
  o.left   = cubeMap[t][4];
  o.right  = cubeMap[t][5];

  return o;
}


void LM_Cube::updateStability(float roll, float pitch) {

  float tilt = sqrt(roll * roll + pitch * pitch);

  isStable = false;
  balancesOnEdge = false;
  balancesOnCorner = false;

  if (tilt < 15) {
    isStable = true;
  }
  else if (tilt >= 30 && tilt < 60) {
    balancesOnEdge = true;
  }
  else if (tilt >= 60) {
    balancesOnCorner = true;
  }
}


float LM_Cube::getRotationSpeed(float gyroX, float gyroY, float gyroZ) {
  return sqrt(gyroX*gyroX + gyroY*gyroY + gyroZ*gyroZ);
}

void LM_Cube::updateRotationState(float gyroX, float gyroY, float gyroZ) {

  float absX = abs(gyroX);
  float absY = abs(gyroY);
  float absZ = abs(gyroZ);

  rotationSpeed = getRotationSpeed(gyroX, gyroY, gyroZ);

  if (rotationSpeed < 10) {
    rotationAxis = 0; // nincs forgás
    rotationDirection = 0;
    return;
  }

  if (absX > absY && absX > absZ) {
    rotationAxis = 1; // X tengely
    rotationDirection = (gyroX > 0) ? 1 : -1;
  }
  else if (absY > absZ) {
    rotationAxis = 2; // Y tengely
    rotationDirection = (gyroY > 0) ? 1 : -1;
  }
  else {
    rotationAxis = 3; // Z tengely
    rotationDirection = (gyroZ > 0) ? 1 : -1;
  }
}




void LM_Cube::handleFlip() {

  // Serial.println("lastFlipSecs");
  // Serial.println(floor(millis()/1000));
  // Serial.println(lastFlipSecs);
  // Serial.println((floor(millis()/1000) - lastFlipSecs));

  // if ((floor(millis()/1000) - lastFlipSecs) >= RtcMemory.myData->settings.cube.flipTiltTime && !flipACK ) {  //tilt time to rotate  reached and no ack yet

  // //&& (strcmp(RtcMemory.myData->settings.cube.TimeflipSides[lastSide-1].localId, SLEEP_SIDE) != 0) //new side is NOT a sleep side,and
  //   if (lastSide > 0 ) { //if  old side was NOT NOTHING
  //     wifiWake();
  //     NetworkHTTPClientTimeflip.syncIntervals();
  //     wifiSleep();
  //     flipACK=true;
      
  //   }


  //   if (strcmp(RtcMemory.myData->settings.cube.TimeflipSides[currentSide-1].localId, SLEEP_SIDE) == 0) {  //if new side is a sleep side
  //     goToSleep();
  //   }

  // }
}
