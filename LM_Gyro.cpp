#include "LM_Gyro.h"

#include "Arduino.h"
#include <math.h>


bool LM_Gyro::begin() {
  if (!bus) return false;
  return startGyro();
  
}

void LM_Gyro::process() {
  
}


// === Utility Functions ===
// Write a single byte
void LM_Gyro::writeByte(uint8_t reg, uint8_t value) {
  if (!bus) return;
  uint8_t data[2] = {reg, value};
  bus->write(address, data, 2);
}

// Read a single byte
uint8_t LM_Gyro::readByte(uint8_t reg) {
  if (!bus) return 0;
  uint8_t data = 0;
  bus->write(address, &reg, 1); // set register
  bus->read(address, &data, 1);
  return data;
}

// Read multiple bytes
void LM_Gyro::readBytes(uint8_t reg, uint8_t count, uint8_t *dest) {
  if (!bus) return;
  bus->write(address, &reg, 1);  // set starting register
  bus->read(address, dest, count);
}

// // === Self-Test ===
// void LM_Gyro::MPU6050SelfTest(float *destination) {
//   // const uint8_t self_test_x = 0x0D, SELF_TEST_Y = 0x0E, SELF_TEST_Z = 0x0F, SELF_TEST_A = 0x10;
//   writeByte(ACCEL_CONFIG, 0xF0); // Enable accel self test
//   writeByte(GYRO_CONFIG, 0xE0); // Enable gyro self test
//   delay(250);

//   uint8_t rawData[4];
//   rawData[0] = readByte(SELF_TEST_X);
//   rawData[1] = readByte(SELF_TEST_Y);
//   rawData[2] = readByte(SELF_TEST_Z);
//   rawData[3] = readByte(SELF_TEST_A);

//   uint8_t selfTest[6];
//   selfTest[0] = (rawData[0] >> 3) | ((rawData[3] & 0x30) >> 4);
//   selfTest[1] = (rawData[1] >> 3) | ((rawData[3] & 0x0C) >> 2);
//   selfTest[2] = (rawData[2] >> 3) | ((rawData[3] & 0x03) >> 0);
//   selfTest[3] = rawData[0] & 0x1F;
//   selfTest[4] = rawData[1] & 0x1F;
//   selfTest[5] = rawData[2] & 0x1F;

//   for (int i = 0; i < 6; i++) {
//     destination[i] = 100.0 + 100.0 * ((float)selfTest[i] - 100.0f) / 100.0f;
//   }

//   ESP_LOGI("mpu", "Self-test results: %.2f %.2f %.2f %.2f %.2f %.2f",
//            destination[0], destination[1], destination[2],
//            destination[3], destination[4], destination[5]);
// }

// === Calibration ===
// void LM_Gyro::calibrateMPU6050(float *gyroBias, float *accelBias) {
//   // Simplified placeholder — full version from your C++ code can be adapted if needed
//   ESP_LOGI("mpu", "Calibrating (placeholder)...");
//   gyroBias[0] = gyroBias[1] = gyroBias[2] = 0;
//   accelBias[0] = accelBias[1] = accelBias[2] = 0;
// }


// // === MPU6050 Initialization ===
// bool LM_Gyro::init6050MPUnew() {
//   delay(100);

//   // WHO_AM_I check
//   // const uint8_t WHO_AM_I = 0x75;
//   uint8_t c = readByte(WHO_AM_I_MPU6050);
//   if (c != 0x68) {
//     ESP_LOGE("mpu", "MPU6050 not responding! WHO_AM_I=0x%02X", c);
//     return false;
//   }

//   // Reset and wake up
//   writeByte(PWR_MGMT_1, 0x00); // PWR_MGMT_1: wake
//   writeByte(CONFIG, 0x00); // CONFIG
//   writeByte(GYRO_CONFIG, 0x00); // GYRO_CONFIG ±250dps
//   writeByte(ACCEL_CONFIG, 0x00); // ACCEL_CONFIG ±2g
//   delay(100);

//   ESP_LOGI("mpu", "MPU6050 WHO_AM_I=0x68 detected.");
//   ESP_LOGI("mpu", "Performing self-test...");
//   float selftest[6];
//   MPU6050SelfTest(selftest);

//   if (selftest[0] < 1.0f && selftest[1] < 1.0f && selftest[2] < 1.0f &&
//       selftest[3] < 1.0f && selftest[4] < 1.0f && selftest[5] < 1.0f) {
//     ESP_LOGI("mpu", "Self-test passed, starting calibration...");
//     float gyroBias[3], accelBias[3];
//     calibrateMPU6050(gyroBias, accelBias);
//     ESP_LOGI("mpu", "Calibration done. Gyro bias: %.2f %.2f %.2f", gyroBias[0], gyroBias[1], gyroBias[2]);
//     return true;
//   } else {
//     ESP_LOGE("mpu", "Self-test failed!");
//     return false;
//   }
// }


bool LM_Gyro::startGyro() {

// #if ARDUINO >= 157
//   Wire.setClock(400000UL); // Set I2C frequency to 400kHz
// #else
//   TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
// #endif

//   delay(100);

  // Read the WHO_AM_I register, this is a good test of communication
  uint8_t c = readByte(WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050

  //Serial.println("connecting to MPU6050...");

  if (c == 0x68) { // WHO_AM_I should always be 0x68
    //Serial.println("MPU6050 is online...");

    MPU6050SelfTest(SelfTest); // Start by performing self test and reporting values
    //    Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0], 1); Serial.println("% of factory value");
    //    Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1], 1); Serial.println("% of factory value");
    //    Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2], 1); Serial.println("% of factory value");
    //    Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3], 1); Serial.println("% of factory value");
    //    Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4], 1); Serial.println("% of factory value");
    //    Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5], 1); Serial.println("% of factory value");

    if (SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {

      //Serial.println("Pass Selftest!");
      calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
      delay(100);
      initMPU6050();
      //Serial.println("MPU6050 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

    } else {
      // criticalLed(ledError);
      //Serial.print("Could not connect to MPU6050: 0x");
      //Serial.println(c, HEX);
      // while (1) ; // Loop forever if communication doesn't happen
      return false;
    }

  } else {

    //Serial.print("MPU6050 is not connected: 0x");
    //Serial.println(c, HEX);

    // criticalLed(ledError);

    // goToSleep();
    return false;
  }
  return true;
}




// Accelerometer and gyroscope self test; check calibration wrt factory settings
void LM_Gyro::MPU6050SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
  uint8_t rawData[4];
  uint8_t selfTest[6];
  float factoryTrim[6];

  // Configure the accelerometer for self-test
  writeByte(ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
  writeByte(GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  delay(250);  // Delay a while to let the device execute the self-test
  rawData[0] = readByte(SELF_TEST_X); // X-axis self-test results
  rawData[1] = readByte(SELF_TEST_Y); // Y-axis self-test results
  rawData[2] = readByte(SELF_TEST_Z); // Z-axis self-test results
  rawData[3] = readByte(SELF_TEST_A); // Mixed-axis self-test results
  // Extract the acceleration test results first
  selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
  selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 2 ; // YA_TEST result is a five-bit unsigned integer
  selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 0 ; // ZA_TEST result is a five-bit unsigned integer
  // Extract the gyration test results first
  selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
  selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
  selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer
  // Process results to allow final comparison with factory set values
  factoryTrim[0] = (4096.0 * 0.34) * (pow( (0.92 / 0.34) , (((float)selfTest[0] - 1.0) / 30.0))); // FT[Xa] factory trim calculation
  factoryTrim[1] = (4096.0 * 0.34) * (pow( (0.92 / 0.34) , (((float)selfTest[1] - 1.0) / 30.0))); // FT[Ya] factory trim calculation
  factoryTrim[2] = (4096.0 * 0.34) * (pow( (0.92 / 0.34) , (((float)selfTest[2] - 1.0) / 30.0))); // FT[Za] factory trim calculation
  factoryTrim[3] =  ( 25.0 * 131.0) * (pow( 1.046 , ((float)selfTest[3] - 1.0) ));         // FT[Xg] factory trim calculation
  factoryTrim[4] =  (-25.0 * 131.0) * (pow( 1.046 , ((float)selfTest[4] - 1.0) ));         // FT[Yg] factory trim calculation
  factoryTrim[5] =  ( 25.0 * 131.0) * (pow( 1.046 , ((float)selfTest[5] - 1.0) ));         // FT[Zg] factory trim calculation

  //  Output self-test results and factory trim calculation if desired
  //  Serial.println(selfTest[0]); Serial.println(selfTest[1]); Serial.println(selfTest[2]);
  //  Serial.println(selfTest[3]); Serial.println(selfTest[4]); Serial.println(selfTest[5]);
  //  Serial.println(factoryTrim[0]); Serial.println(factoryTrim[1]); Serial.println(factoryTrim[2]);
  //  Serial.println(factoryTrim[3]); Serial.println(factoryTrim[4]); Serial.println(factoryTrim[5]);

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
  // To get to percent, must multiply by 100 and subtract result from 100
  for (int i = 0; i < 6; i++) {
    destination[i] = 100.0 + 100.0 * ((float)selfTest[i] - factoryTrim[i]) / factoryTrim[i]; // Report percent differences
  }

}


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void LM_Gyro::calibrateMPU6050(float * dest1, float * dest2)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  // reset device, reset all registers, clear gyro and accelerometer bias registers
  writeByte(PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);

  // get stable time source
  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  writeByte(PWR_MGMT_1, 0x01);
  writeByte(PWR_MGMT_2, 0x00);
  delay(200);

  // Configure device for bias calculation
  writeByte(INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(FIFO_EN, 0x00);      // Disable FIFO
  writeByte(PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);

  // Configure MPU6050 gyro and accelerometer for bias calculation
  //  writeByte(MPU6050_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(CONFIG, 0x00);      // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  //  writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(SMPLRT_DIV, 0x07);  // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  writeByte(GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(USER_CTRL, 0x40);   // Enable FIFO
  writeByte(FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
  delay(80); // accumulate 80 samples in 80 milliseconds = 960 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  writeByte(FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];

  }
  accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;

  if (accel_bias[2] > 0L) {
    accel_bias[2] -= (int32_t) accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
  }
  else {
    accel_bias[2] += (int32_t) accelsensitivity;
  }

  // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0] / 4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1] / 4)       & 0xFF;
  data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2] / 4)       & 0xFF;

  // Push gyro biases to hardware registers; works well for gyro but not for accelerometer
  //  writeByte(MPU6050_ADDRESS, XG_OFFS_USRH, data[0]);
  //  writeByte(MPU6050_ADDRESS, XG_OFFS_USRL, data[1]);
  //  writeByte(MPU6050_ADDRESS, YG_OFFS_USRH, data[2]);
  //  writeByte(MPU6050_ADDRESS, YG_OFFS_USRL, data[3]);
  //  writeByte(MPU6050_ADDRESS, ZG_OFFS_USRH, data[4]);
  //  writeByte(MPU6050_ADDRESS, ZG_OFFS_USRL, data[5]);

  dest1[0] = (float) gyro_bias[0] / (float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
  dest1[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

  // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
  // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
  // non-zero values. In addition, bit 0 of the lower uint8_t must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readBytes(YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readBytes(ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower uint8_t of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for (ii = 0; ii < 3; ii++) {
    if (accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1] / 8);
  accel_bias_reg[2] -= (accel_bias[2] / 8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

  // Push accelerometer biases to hardware registers; doesn't work well for accelerometer
  // Are we handling the temperature compensation bit correctly?
  //  writeByte(MPU6050_ADDRESS, XA_OFFSET_H, data[0]);
  //  writeByte(MPU6050_ADDRESS, XA_OFFSET_L_TC, data[1]);
  //  writeByte(MPU6050_ADDRESS, YA_OFFSET_H, data[2]);
  //  writeByte(MPU6050_ADDRESS, YA_OFFSET_L_TC, data[3]);
  //  writeByte(MPU6050_ADDRESS, ZA_OFFSET_H, data[4]);
  //  writeByte(MPU6050_ADDRESS, ZA_OFFSET_L_TC, data[5]);

  // Output scaled accelerometer biases for manual subtraction in the main program
  dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
  dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
  dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;
}


//===================================================================================================================
//====== Set of useful function to access acceleration, gyroscope, and temperature data
//===================================================================================================================

void LM_Gyro::getGres() {
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    case GFS_250DPS:
      gRes = 250.0 / 32768.0;
      break;
    case GFS_500DPS:
      gRes = 500.0 / 32768.0;
      break;
    case GFS_1000DPS:
      gRes = 1000.0 / 32768.0;
      break;
    case GFS_2000DPS:
      gRes = 2000.0 / 32768.0;
      break;
  }
}

void LM_Gyro::getAres() {
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    case AFS_2G:
      aRes = 2.0 / 32768.0;
      break;
    case AFS_4G:
      aRes = 4.0 / 32768.0;
      break;
    case AFS_8G:
      aRes = 8.0 / 32768.0;
      break;
    case AFS_16G:
      aRes = 16.0 / 32768.0;
      break;
  }
}



void LM_Gyro::initKalman(float accX, float accY, float accZ) {
// Serial.print(F("Starting  kalman: "));
  /* Set kalman and gyro starting angle */
  
  #ifdef GYRO_NATIVE_BIAS

  readBytes(0x3B, 6, i2cData);
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  #endif

  // float accX = id(mpu_accel_x).state;
  // float accY = id(mpu_accel_y).state;
  // float accZ = id(mpu_accel_z).state;

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}







void LM_Gyro::getAngle(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ) {

  // If data ready bit set, all data registers have new data
  // if (readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) { // check if data ready interrupt NOT
  // Serial.println("check if data ready interrupt NOT");
    // rollAngle = 0;
    // pitchAngle = 0;
    // return;
    // }
    //  

    //  i2c   
    //        
    //    
    //    
   
    #ifdef GYRO_NATIVE_BIAS
   
    uint8_t i2cData[14]; // Buffer for I2C data
    readBytes(0x3B, 14, i2cData); // Read the 14 raw data registers into data array
    accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
    accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
    tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
    gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
    gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
    gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

    Serial.print(accX); Serial.print("\t");
    Serial.print(accY); Serial.print("\t");
    Serial.print(accZ); Serial.print("\t");
    Serial.print(gyroX); Serial.print("\t");
    Serial.print(gyroY); Serial.print("\t");  
    Serial.print(gyroZ); Serial.print("\t");

    #endif

    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();

    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
    #ifdef RESTRICT_PITCH // Eq. 25 and 26
      double roll  = atan2(accY, accZ) * RAD_TO_DEG;
      double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
    #else // Eq. 28 and 29
      double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
      double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
    #endif

    double gyroXrate = gyroX / 131.0; // Convert to deg/s
    double gyroYrate = gyroY / 131.0; // Convert to deg/s

    #ifdef RESTRICT_PITCH
      // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
      if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
        kalmanX.setAngle(roll);
        compAngleX = roll;
        kalAngleX = roll;
        gyroXangle = roll;
      } else
        kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

      if (abs(kalAngleX) > 90)
        gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
    #else
      // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
      if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
        kalmanY.setAngle(pitch);
        compAngleY = pitch;
        kalAngleY = pitch;
        gyroYangle = pitch;
      } else
        kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

      if (abs(kalAngleY) > 90)
        gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
    #endif

    gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;
    //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
    //gyroYangle += kalmanY.getRate() * dt;

    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

    // Reset the gyro angle when it has drifted too much
    if (gyroXangle < -180 || gyroXangle > 180)
      gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
      gyroYangle = kalAngleY;

    /* Print Data */
    #if 1 // Set to 1 to activate
      Serial.print("\t");
      Serial.print(roll); Serial.print("\t");
      Serial.print(kalAngleX); Serial.print("\t");
      Serial.print("\t");
      Serial.print(pitch); Serial.print("\t");
      Serial.print(kalAngleY); Serial.print("\t");
    #endif

    // #if 0 // Set to 1 to print the temperature

    //   double temperature = (double)tempRaw / 340.0 + 36.53;
    //   Serial.print(temperature); Serial.print("\t");
    // #endif
    
    rollAngle = kalAngleX;
    pitchAngle = kalAngleY;

    // rollAngle = 3.0;
    // pitchAngle = 4.0;
  
  // }

}



// Configure the motion detection control for low power accelerometer mode
void LM_Gyro::LowPowerAccelOnlyMPU6050()
{
  //return;

  // The sensor has a high-pass filter necessary to invoke to allow the sensor motion detection algorithms work properly
  // Motion detection occurs on free-fall (acceleration below a threshold for some time for all axes), motion (acceleration
  // above a threshold for some time on at least one axis), and zero-motion toggle (acceleration on each axis less than a
  // threshold for some time sets this flag, motion above the threshold turns it off). The high-pass filter takes gravity out
  // consideration for these threshold evaluations; otherwise, the flags would be set all the time!

  uint8_t c = readByte(PWR_MGMT_1);
  writeByte(PWR_MGMT_1, c & ~0x30); // Clear sleep and cycle bits [5:6]
  writeByte(PWR_MGMT_1, c |  0x30); // Set sleep and cycle bits [5:6] to zero to make sure accelerometer is running

  c = readByte(PWR_MGMT_2);
  writeByte(PWR_MGMT_2, c & ~0x38); // Clear standby XA, YA, and ZA bits [3:5]
  writeByte(PWR_MGMT_2, c |  0x00); // Set XA, YA, and ZA bits [3:5] to zero to make sure accelerometer is running

  c = readByte(ACCEL_CONFIG);
  writeByte(ACCEL_CONFIG, c & ~0x07); // Clear high-pass filter bits [2:0]
  // Set high-pass filter to 0) reset (disable), 1) 5 Hz, 2) 2.5 Hz, 3) 1.25 Hz, 4) 0.63 Hz, or 7) Hold
  writeByte(ACCEL_CONFIG,  c | 0x00);  // Set ACCEL_HPF to 0; reset mode disbaling high-pass filter

  c = readByte(CONFIG);
  writeByte(CONFIG, c & ~0x07); // Clear low-pass filter bits [2:0]
  writeByte(CONFIG, c |  0x00);  // Set DLPD_CFG to 0; 260 Hz bandwidth, 1 kHz rate

  c = readByte(INT_ENABLE);
  writeByte(INT_ENABLE, c & ~0xFF);  // Clear all interrupts
  writeByte(INT_ENABLE, 0x40);  // Enable motion threshold (bits 5) interrupt only

  // Motion detection interrupt requires the absolute value of any axis to lie above the detection threshold
  // for at least the counter duration
  writeByte(MOT_THR, 0x80); // Set motion detection to 0.256 g; LSB = 2 mg
  writeByte(MOT_DUR, 0x01); // Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate

  delay (100);  // Add delay for accumulation of samples

  c = readByte(ACCEL_CONFIG);
  writeByte(ACCEL_CONFIG, c & ~0x07); // Clear high-pass filter bits [2:0]
  writeByte(ACCEL_CONFIG, c |  0x07);  // Set ACCEL_HPF to 7; hold the initial accleration value as a referance

  c = readByte(PWR_MGMT_2);
  writeByte(PWR_MGMT_2, c & ~0xC7); // Clear standby XA, YA, and ZA bits [3:5] and LP_WAKE_CTRL bits [6:7]
  writeByte(PWR_MGMT_2, c |  0x47); // Set wakeup frequency to 5 Hz, and disable XG, YG, and ZG gyros (bits [0:2])

  c = readByte(PWR_MGMT_1);
  writeByte(PWR_MGMT_1, c & ~0x20); // Clear sleep and cycle bit 5
  writeByte(PWR_MGMT_1, c |  0x20); // Set cycle bit 5 to begin low power accelerometer motion interrupts

  //  writeByte(MPU6050_ADDRESS, INT_PIN_CFG, 0x1A); // we need 210
  //writeByte(MPU6050_ADDRESS, SIGNAL_PATH_RESET, 0x07); //Reset all internal signal paths in the MPU-6050 by writing 0x07 to register 0x68;
  writeByte(I2C_SLV0_ADDR, 0x20); //write register 0x37 to select how to use the interrupt pin. For an active high, push-pull signal that stays until register (decimal) 58 is read, write 0x20.

  writeByte(MOT_THR, 2); //Write the desired Motion threshold to register 0x1F (For example, write decimal 20).
  writeByte(MOT_DUR, 1); //Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate

  delay (100);  // Add delay for accumulation of samples

  //writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x01); //Write register 28 (==0x1C) to set the Digital High Pass Filter, bits 3:0. For example set it to 0x01 for 5Hz. (These 3 bits are grey in the data sheet, but they are used! Leaving them 0 means the filter always outputs 0.)
  writeByte(MOT_DETECT_CTRL, 0x15); //to register 0x69, write the motion detection decrement and a few other settings (for example write 0x15 to set both free-fall and motion decrements to 1 and accelerometer start-up delay to 5ms total by adding 1ms. )
  writeByte(INT_ENABLE, 0x40); //write register 0x38, bit 6 (0x40), to enable motion detection interrupt.
  writeByte(0x37, 0); // now INT pin is active low

}

void LM_Gyro::FullPowerAccelOnlyMPU6050()
{
  //return;

  uint8_t c = readByte(PWR_MGMT_1);
  writeByte(PWR_MGMT_1, c & ~0x30); // Clear sleep and cycle bits [5:6]
  writeByte(PWR_MGMT_1, c |  0x00); // Set sleep and cycle bits [5:6] to zero to make sure accelerometer is running

  c = readByte(PWR_MGMT_2);
  writeByte(PWR_MGMT_2, c & ~0x38); // Clear standby XA, YA, and ZA bits [3:5]
  writeByte(PWR_MGMT_2, c |  0x00); // Set XA, YA, and ZA bits [3:5] to zero to make sure accelerometer is running

}

void LM_Gyro::initMPU6050()
{
  //return;
  // Serial.println("Initialize MPU6050 device");

  //  wake up device-don't need this here if using calibration function below
  //  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  //  delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

  // get stable time source
  writeByte(PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

  // Configure Gyro and Accelerometer
  // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
  // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
  //  writeByte(MPU6050_ADDRESS, CONFIG, 0x03);
  writeByte(CONFIG, 0x00); // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  //  writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz sample rate
  writeByte(SMPLRT_DIV, 0x07);  // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c =  readByte(GYRO_CONFIG);
  writeByte(GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
  writeByte(GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro

  // Set accelerometer configuration
  c =  readByte(ACCEL_CONFIG);
  writeByte(ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
  writeByte(ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
  writeByte(INT_PIN_CFG, 0x08); // was 02
  //writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
}

