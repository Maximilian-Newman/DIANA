// Completely untested, will test when I return to Montreal


#include <Servo.h>

Servo AoAServo;
int AoA = 0;

void setup() {
  AoAServo.attach(6);
  AoAServo.write(90);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  Serial.begin(115200);
}

void loop() {
  AoAServo.write(AoA + 90);
  Serial.println(AoA);

  bool over = !digitalRead(4);
  bool under = !digitalRead(5);

  if (over and under) {Serial.println("ERROR");}
  else if (over) {AoA += 1;}
  else if (under) {AoA -= 1;}

  if (AoA > 60) {AoA = 60;}
  if (AoA < -60) {AoA = -60;}
}
