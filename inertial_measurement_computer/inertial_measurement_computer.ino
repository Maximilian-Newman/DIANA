const char busID = '0';
String VERSION = "0.1";

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

void setup() {
  Serial.begin(115200);
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
    }

    else if (millis() - lastUpdate > 50){
      working = false;
      Serial.println("EEEEEEEEEEE");
    }


    if (freqTimer - millis() > 1000) {
      freqTimer += 1000;
      lastSampleRate = numSamples;
      //Serial.println();
      //Serial.println(numSamples); // getting 150 Hz sample rate
      numSamples = 0;
    }
  }



  if (Serial.available()){
    delay(1);
    char reqBus = Serial.read();
    char cmd = Serial.read();

    //Serial.write(reqBus);
    //Serial.write(cmd);

    if (reqBus == busID){
      if (cmd == 'r'){
        if (working) {
          Serial.write('1');

          Serial.print(mpu.getPitch());
          Serial.write(',');
          Serial.print(mpu.getRoll());
          Serial.write(',');
          Serial.print(mpu.getPitch());
          Serial.write(',');
          Serial.print(lastSampleRate);
          Serial.write('\n');
        }
        else {
          Serial.print("0\n");
        }
      }

      else if (cmd == 'f') {working = false;}
      else if (cmd == 'w') {working = true;}
      else if (cmd == 'v') {Serial.println(VERSION);}
    }
  }
}
