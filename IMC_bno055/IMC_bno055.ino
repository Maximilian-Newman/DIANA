const char busID = '1';
String VERSION = "0.2";

/*
Connected to main DIANA flight computer via serial connection
can have multiple in parallel as long as all have a different busID



MIT License

Copyright (c) 2026 Maximilian Newman Loussouarn

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "BNO055_support.h"
#include <Wire.h>
struct bno055_t myBNO;
struct bno055_euler eulerData;
struct bno055_linear_accel acceleration;
struct bno055_gyro gyro;

bool working = true;
unsigned long lastUpdate = 0;

unsigned long freqTimer = 0;
unsigned int numSamples = 0;
unsigned int lastSampleRate = 0;

float accX = 0;
float accY = 0;
float accZ = 0;
float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;

float DECLINATION = -14.0;
float ACC_DAMP_FACTOR = 0.8; // 0.0 to 1.0
float GYRO_DAMP_FACTOR = 0.8;

void setup() {
  Serial.begin(115200); // debug channel
  Serial1.begin(115200);
  Wire.begin();
  BNO_Init(&myBNO);
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
  bno055_set_accel_unit(0); // ms^-2
  bno055_set_gyro_unit(0); // degrees/s
  bno055_set_tilt_unit(0); // degrees
  delay(1000);
  lastUpdate = millis();
  freqTimer = millis();
}

void loop() {
  if (working) {
    bno055_read_gyro_xyz(&gyro);
    bno055_read_linear_accel_xyz(&acceleration);
    bno055_read_euler_hrp(&eulerData);
    lastUpdate = millis();
    numSamples += 1;

    accX = ACC_DAMP_FACTOR * accX + (1 - ACC_DAMP_FACTOR) * float(acceleration.x);
    accY = ACC_DAMP_FACTOR * accY + (1 - ACC_DAMP_FACTOR) * float(acceleration.y);
    accZ = ACC_DAMP_FACTOR * accZ + (1 - ACC_DAMP_FACTOR) * float(acceleration.z);
    gyroX = GYRO_DAMP_FACTOR * gyroX + (1 - GYRO_DAMP_FACTOR) * float(gyro.x);
    gyroY = GYRO_DAMP_FACTOR * gyroY + (1 - GYRO_DAMP_FACTOR) * float(gyro.y);
    gyroZ = GYRO_DAMP_FACTOR * gyroZ + (1 - GYRO_DAMP_FACTOR) * float(gyro.z);

    if (eulerData.p == 0 and eulerData.h == 0 and eulerData.r == 0 and acceleration.x == 0 and acceleration.y == 0 and acceleration.z == 0 and gyro.x == 0 and gyro.y == 0 and gyro.z == 0){
      working = false;
      accX = 0;
      accY = 0;
      accZ = 0;
      gyroX = 0;
      gyroY = 0;
      gyroZ = 0;
    }


    if (freqTimer - millis() > 1000) {
      freqTimer += 1000;
      lastSampleRate = numSamples;
      //Serial1.println();
      //Serial1.println(numSamples); // getting 150 Hz sample rate
      numSamples = 0;
    }
  }



  if (Serial1.available()){
    delay(2);
    char reqBus = Serial1.read();
    char cmd = Serial1.read();

    //Serial.write(reqBus);
    //Serial.write(cmd);
    //Serial.write('\n');

    if (reqBus == busID){
      //Serial.println("this IMC");

      if (cmd == 'r'){
        //Serial.println("read detected");
        if (working) {
          Serial1.write('1');

          Serial1.print(float(eulerData.p) / 16.00);
          Serial1.write(',');
          Serial1.print(float(eulerData.r) / 16.00);
          Serial1.write(',');
          Serial1.print(float(eulerData.h) / 16.00 + DECLINATION);
          Serial1.write(',');
          Serial1.print(gyroX);
          Serial1.write(',');
          Serial1.print(gyroY);
          Serial1.write(',');
          Serial1.print(gyroZ);
          Serial1.write(',');
          Serial1.print(accX);
          Serial1.write(',');
          Serial1.print(accY);
          Serial1.write(',');
          Serial1.print(accZ);
          Serial1.write(',');
          Serial1.print(lastSampleRate);
          Serial1.write('\n');
        }
        else {
          Serial1.write('0');
        }
      }

      else if (cmd == 'f') {working = false;}
      else if (cmd == 'w') {working = true;}
      else if (cmd == 'v') {Serial1.print(VERSION + "\n");}
    }
  }
}
