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

  mapSides();

}


void LM_Cube::mapSides() {
  topSide = sideRemap[topSideRaw];
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
