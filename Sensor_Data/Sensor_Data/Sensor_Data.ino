#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <vl53l4cd_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>

#define DEV_I2C Wire
#define SerialPort Serial

//Wiring For SPI: SCK-SCL; MISO-SDOAG,SDOM; MOSI-SDA; pick pins for the two chip selects
#define LSM9DS1_XGCS 2
#define LSM9DS1_MCS 3
// hardware SPI! In this case, only CS pins are passed in
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);

float gyro_bias_x;
float gyro_bias_y;
float gyro_bias_z;
int samples = 500;

// define tof components
VL53L4CD tof1(&DEV_I2C, 12);
VL53L4CD tof2(&DEV_I2C, 10);
VL53L4CD tof3(&DEV_I2C, 11);
VL53L4CD tof4(&DEV_I2C, 9);

int data[10];
int filtered_data[10];

unsigned long previousMillis = 0;  // store last time action was taken
const long interval = 10;         // interval at which to do something (ms)
unsigned long current_millis;

void setup() {
  // Initialize serial for output.
  SerialPort.begin(115200);

  // Initialize I2C bus.
  DEV_I2C.begin();

  // Configure VL53L4CD tofs
  tof1.begin();
  tof2.begin();
  tof3.begin();
  tof4.begin();

  // Switch off VL53L4CD satellite component.
  tof1.VL53L4CD_Off();
  tof2.VL53L4CD_Off();
  tof3.VL53L4CD_Off();
  tof4.VL53L4CD_Off();
  delay(100);

  // Initialize TOF1
  tof1.begin();
  tof1.InitSensor(0x10);
  delay(100);

  // Initialize TOF2
  tof2.begin();
  tof2.InitSensor(0x20);
  delay(100);

  // Initialize TOF3
  tof3.begin();
  tof3.InitSensor(0x30);
  delay(100);

  // Initialize TOF4
  tof4.begin();
  tof4.InitSensor(0x40);
  delay(100);

  // Turn all sensors back on with assigned addresses
  tof1.VL53L4CD_On();
  tof2.VL53L4CD_On();
  tof3.VL53L4CD_On();
  tof4.VL53L4CD_On();
  delay(500);

  // low power mode. This should give the best accuracy
  tof1.VL53L4CD_SetRangeTiming(200, 0);
  tof2.VL53L4CD_SetRangeTiming(200, 0);
  tof3.VL53L4CD_SetRangeTiming(200, 0);
  tof4.VL53L4CD_SetRangeTiming(200, 0);

  // Start Measurements
  tof1.VL53L4CD_StartRanging();
  tof2.VL53L4CD_StartRanging();
  tof3.VL53L4CD_StartRanging();
  tof4.VL53L4CD_StartRanging();

  //init IMU
  while (!lsm.begin()) {
  }

  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G, lsm.LSM9DS1_ACCELDATARATE_10HZ);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);

  for (int i = 0; i < samples; i++) {
    lsm.read();
    gyro_bias_x += lsm.gyroData.x;
    gyro_bias_y += lsm.gyroData.y;
    gyro_bias_z += lsm.gyroData.z;
    delay(5);  // Small delay between samples
  }

  gyro_bias_x /= samples;
  gyro_bias_y /= samples;
  gyro_bias_z /= samples;
}

void loop() {
  current_millis = millis();

  if (current_millis - previousMillis >= interval) {
    previousMillis = current_millis;


    uint8_t NewDataReady = 0;
    VL53L4CD_Result_t results;
    uint8_t status;
    char report[64];


    do {
      status = tof1.VL53L4CD_CheckForDataReady(&NewDataReady);
    } while (!NewDataReady);

    if ((!status) && (NewDataReady != 0)) {
      // (Mandatory) Clear HW interrupt to restart measurements
      tof1.VL53L4CD_ClearInterrupt();

      // Read measured distance and saves in data
      tof1.VL53L4CD_GetResult(&results);
      data[0] = results.distance_mm;
    }

    uint8_t NewDataReady2 = 0;
    VL53L4CD_Result_t results2;
    uint8_t status2;
    char report2[64];

    do {
      status2 = tof2.VL53L4CD_CheckForDataReady(&NewDataReady2);
    } while (!NewDataReady2);

    if ((!status2) && (NewDataReady2 != 0)) {
      // (Mandatory) Clear HW interrupt to restart measurements
      tof2.VL53L4CD_ClearInterrupt();

      // Read measured distance and saves in data
      tof2.VL53L4CD_GetResult(&results2);
      data[1] = results2.distance_mm;
    }

    uint8_t NewDataReady3 = 0;
    VL53L4CD_Result_t results3;
    uint8_t status3;
    char report3[64];

    do {
      status3 = tof3.VL53L4CD_CheckForDataReady(&NewDataReady3);
    } while (!NewDataReady3);

    if ((!status3) && (NewDataReady3 != 0)) {
      // (Mandatory) Clear HW interrupt to restart measurements
      tof3.VL53L4CD_ClearInterrupt();

      // Read measured distance and saves in data
      tof3.VL53L4CD_GetResult(&results3);
      data[2] = results3.distance_mm;
    }

    uint8_t NewDataReady4 = 0;
    VL53L4CD_Result_t results4;
    uint8_t status4;
    char report4[64];

    do {
      status4 = tof4.VL53L4CD_CheckForDataReady(&NewDataReady4);
    } while (!NewDataReady4);

    if ((!status4) && (NewDataReady4 != 0)) {
      // (Mandatory) Clear HW interrupt to restart measurements
      tof4.VL53L4CD_ClearInterrupt();

      // Read measured distance and saves in data
      tof4.VL53L4CD_GetResult(&results4);
      data[3] = results4.distance_mm;
    }

    lsm.read(); /* ask it to read in the data */

    // saves IMU values in data, accel in m/s^2, gyro in degree/s
    data[4] = lsm.accelData.x;
    data[5] = lsm.accelData.y;
    data[6] = lsm.accelData.z;
    float gyro_x_dps = (lsm.gyroData.x - gyro_bias_x) * 0.00875;
    float gyro_y_dps = (lsm.gyroData.y - gyro_bias_y) * 0.00875;
    float gyro_z_dps = (lsm.gyroData.z - gyro_bias_z) * 0.00875;

    // Integrate gyro data (degrees)
    float gyro_pitch = data[7] + gyro_x_dps * (interval / 1000.0);  // from gyro X
    float gyro_roll = data[8] + gyro_y_dps * (interval / 1000.0);   // from gyro Y
    float gyro_yaw = data[9] + gyro_z_dps * (interval / 1000.0);    // from gyro Z

    // Calculate accelerometer-based pitch and roll (in degrees)
    float acc_pitch = atan2(lsm.accelData.y, sqrt(pow(lsm.accelData.x, 2) + pow(lsm.accelData.z, 2))) * 180.0 / PI;
    float acc_roll = atan2(-lsm.accelData.x, lsm.accelData.z) * 180.0 / PI;

    // Complementary filter to fuse gyro and accel
    const float alpha = 0.90;                                // High-pass filter for gyro, low-pass for accel
    data[7] = alpha * gyro_pitch + (1 - alpha) * acc_pitch;  // Fused pitch
    data[8] = alpha * gyro_roll + (1 - alpha) * acc_roll;    // Fused roll
    data[9] = gyro_yaw;                                      // Yaw from gyro only (no accelerometer contribution)

    for (int i = 0; i < 10; i++) {
      if (i < 4) {
        filtered_data[i] = (0.5) * data[i] + (1 - 0.5) * filtered_data[i];
      } else {
        // filtered_data[i] = (0.9) * data[i] + (1 - 0.9) * filtered_data[i];
        filtered_data[i] = data[i];
      }
    }

    for (int i = 0; i < 10; i++) {
      Serial.print(filtered_data[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
}

// function to setup resolution for IMU
void setupSensor() {
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G, lsm.LSM9DS1_ACCELDATARATE_10HZ);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
}
