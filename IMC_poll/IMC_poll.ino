/*
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




String IMC_BUS_COMPATIBLE_VERSION = "0.3";

byte numIMC = 0;
char imcAddresses[] = {255, 255, 255, 255, 255}; // addr 255 is reserved for 'not connected'
bool imcStatus[] = {false, false, false, false, false};
unsigned long imcNextReattempt[] = {0, 0, 0, 0, 0}; // if recieved a 2 / not atabilized response
float imcPitch[] = {0, 0, 0, 0, 0};
float imcRoll[] = {0, 0, 0, 0, 0};
float imcHeading[] = {0, 0, 0, 0, 0};
float imcGyroX[] = {0, 0, 0, 0, 0};
float imcGyroY[] = {0, 0, 0, 0, 0};
float imcGyroZ[] = {0, 0, 0, 0, 0};
float imcAccX[] = {0, 0, 0, 0, 0};
float imcAccY[] = {0, 0, 0, 0, 0};
float imcAccZ[] = {0, 0, 0, 0, 0};
unsigned int imcSampleRate[] = {0, 0, 0, 0, 0};
unsigned int imcWarning[] = {0, 0, 0, 0, 0}; // measures drift over time relative to other imc's. shuts off if error is sustained
byte workingIMCs = 0;
float pitch = 0;
float roll = 0;
float heading = 0;
float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;
float accX = 0;
float accY = 0;
float accZ = 0;

void setupIMCs() {
  while (Serial1.available()) {Serial1.read();}

  for (byte i=0; i<5; i++) {
    imcAddresses[i] = 255;
    imcStatus[i] = false;
    imcPitch[i] = 0;
    imcRoll[i] = 0;
    imcHeading[i] = 0;
    imcGyroX[i] = 0;
    imcGyroY[i] = 0;
    imcGyroZ[i] = 0;
    imcAccX[i] = 0;
    imcAccY[i] = 0;
    imcAccZ[i] = 0;
    imcSampleRate[i] = 0;
    imcWarning[i] = 0;
  }
  numIMC = 0;
  workingIMCs = 0;

  for (byte i=0; i<255; i++) {
    Serial1.write(i);
    Serial1.write('v');
    delay(15);
    if (Serial1.available()){
      Serial.print("Found IMC on addr ");
      Serial.println(i);

      String conVers = Serial1.readStringUntil('\n');
      if (conVers == IMC_BUS_COMPATIBLE_VERSION) {
        imcAddresses[numIMC] = i;
        imcStatus[numIMC] = true;
        numIMC += 1;
        Serial.println("Successfully connected IMC");
        if (numIMC == 5) {break;} // avoid memory leak if too many IMC's connected
      }
      else {
        Serial.print("Error: incompatible IMC firmware version: ");
        Serial.println(conVers);
      }
    }
  }
}


void pollIMCs(){
  workingIMCs = 0;
  for (byte i=0; i<numIMC; i++){

    if (imcNextReattempt[i] > 0 and imcNextReattempt[i] < millis()) {
      imcStatus[i] = true; // attempt reconnect after timeout if  previously recieved a '2 / not stabilized' response
      imcNextReattempt[i] = 0;
    }

    if (imcStatus[i] == true) {
      while (Serial1.available()) {Serial1.read();}
      Serial1.write(imcAddresses[i]);
      Serial1.write('r');
      byte timeoutTimer = 0;
      while (!Serial1.available() and timeoutTimer < 100) {
        delay(1);
        timeoutTimer += 1;
      }
      char r = Serial1.read();

      if (r == '1') {
        imcPitch[i] = Serial1.readStringUntil(',').toFloat();
        imcRoll[i] = Serial1.readStringUntil(',').toFloat();
        imcHeading[i] = Serial1.readStringUntil(',').toFloat();
        imcGyroX[i] = Serial1.readStringUntil(',').toFloat();
        imcGyroY[i] = Serial1.readStringUntil(',').toFloat();
        imcGyroZ[i] = Serial1.readStringUntil(',').toFloat();
        imcAccX[i] = Serial1.readStringUntil(',').toFloat();
        imcAccY[i] = Serial1.readStringUntil(',').toFloat();
        imcAccZ[i] = Serial1.readStringUntil(',').toFloat();
        imcSampleRate[i] = Serial1.readStringUntil('\n').toFloat();
        workingIMCs += 1;
      }

      else if (r == '2') {
        imcStatus[i] = false;
        imcNextReattempt[i] = millis() + 1000;
      }
      else {
        imcStatus[i] = false;
        imcSampleRate[i] = 0;
      }
    }
  }
}

void mergeIMCs() {
  pitch = 0;
  roll = 0;
  heading = 0;
  workingIMCs = 0;
  gyroX = 0;
  gyroY = 0;
  gyroZ = 0;
  accX = 0;
  accY = 0;
  accZ = 0;

  for (byte i=0; i<numIMC; i++){
    if (imcStatus[i] == true) {
      pitch += imcPitch[i];
      roll += imcRoll[i];
      heading += imcHeading[i];
      gyroX += imcGyroX[i];
      gyroY += imcGyroY[i];
      gyroZ += imcGyroZ[i];
      accX += imcAccX[i];
      accY += imcAccY[i];
      accZ += imcAccZ[i];
      workingIMCs += 1;
    }
  }

  pitch /= workingIMCs;
  roll /= workingIMCs;
  heading /= workingIMCs;
  gyroX /= workingIMCs;
  gyroY /= workingIMCs;
  gyroZ /= workingIMCs;
  accX /= workingIMCs;
  accY /= workingIMCs;
  accZ /= workingIMCs;
}

void setup() {
  pinMode(19, INPUT_PULLUP);
  delay(1000);
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial1.setTimeout(100); // avoid long blocks on readStringUntil() if communication is interrupted with IMC's
  setupIMCs();
}

void loop() {
  if (Serial.available()) {
    byte index = Serial.read() - '0';
    byte instruct = Serial.read();
    if (index < 5) {
      if (instruct == '1') {
        imcStatus[index] = true;
      }
      Serial1.write(imcAddresses[index]);
      Serial1.write(instruct);
    }
  }


  delay(100);
  pollIMCs();
  mergeIMCs();

  Serial.print(imcSampleRate[0]);
  Serial.print(" ");
  Serial.print(imcSampleRate[1]);
  Serial.print("     ");
  Serial.print(pitch);
  Serial.print(" ");

  Serial.print(imcPitch[0]);
  Serial.print(" ");
  Serial.print(imcPitch[1]);
/*
  Serial.print(roll);
  Serial.print(" ");
  Serial.print(heading);
  Serial.print(" ");
  Serial.print(gyroX);
  Serial.print(" ");
  Serial.print(gyroY);
  Serial.print(" ");
  Serial.print(gyroZ);
  Serial.print(" ");
  Serial.print(accX);
  Serial.print(" ");
  Serial.print(accY);
  Serial.print(" ");
  Serial.print(accZ);*/


  Serial.println();
}
