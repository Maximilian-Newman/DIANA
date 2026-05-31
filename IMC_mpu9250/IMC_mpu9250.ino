const char busID = '0';
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


#include "MPU9250.h" // https://github.com/hideakitai/MPU9250
MPU9250 mpu;
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

float ACC_DAMP_FACTOR = 0.8; // 0.0 to 1.0
float GYRO_DAMP_FACTOR = 0.8;

void setup() {
  Serial.begin(115200); // debug channel
  Serial1.begin(115200);
  Wire.begin();
  delay(2000);

  mpu.verbose(false);
  
  if (!mpu.setup(0x68)) {
    working = false;
  }
  mpu.setMagneticDeclination(-14);
  delay(1000);
  lastUpdate = millis();
  freqTimer = millis();
}

void loop() {
  if (working) {
    if (mpu.update()) {
      lastUpdate = millis();
      numSamples += 1;

      accX = ACC_DAMP_FACTOR * accX + (1 - ACC_DAMP_FACTOR) * mpu.getAccX();
      accY = ACC_DAMP_FACTOR * accY + (1 - ACC_DAMP_FACTOR) * mpu.getAccY();
      accZ = ACC_DAMP_FACTOR * accZ + (1 - ACC_DAMP_FACTOR) * mpu.getAccZ();
      gyroX = GYRO_DAMP_FACTOR * gyroX + (1 - GYRO_DAMP_FACTOR) * mpu.getGyroX();
      gyroY = GYRO_DAMP_FACTOR * gyroY + (1 - GYRO_DAMP_FACTOR) * mpu.getGyroY();
      gyroZ = GYRO_DAMP_FACTOR * gyroZ + (1 - GYRO_DAMP_FACTOR) * mpu.getGyroZ();
    }

    else if (millis() - lastUpdate > 50){
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

          Serial1.print(mpu.getPitch());
          Serial1.write(',');
          Serial1.print(mpu.getRoll());
          Serial1.write(',');
          Serial1.print(mpu.getYaw());
          Serial1.write(',');
          Serial1.print(mpu.getGyroX());
          Serial1.write(',');
          Serial1.print(mpu.getGyroY());
          Serial1.write(',');
          Serial1.print(mpu.getGyroZ());
          Serial1.write(',');
          Serial1.print(mpu.getAccX());
          Serial1.write(',');
          Serial1.print(mpu.getAccY());
          Serial1.write(',');
          Serial1.print(mpu.getAccZ());
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
