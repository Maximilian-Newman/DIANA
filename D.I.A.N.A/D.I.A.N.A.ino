/* 

Demonstrator for Intelligent Automatically Navigating Aircraft

This software is still in development
DO NOT USE IN FLIGHT! (yet)

Currently in the middle of a major overhaul, different parts not compatible with each other.

*/


const float declinationAngle = ((1 + (57.0 / 60.0)) / (180 / PI)); // magnetic declination at location of flight

// Hardware connections
const byte yawChannel = 3;
const byte thrustChannel = 4;
const byte pitchChannel = 5;
const byte rollChannel = 6;
const byte sonarTrigPin = 46;
const byte sonarEchoPin = 47;
const byte IR_reciever = 2;
const byte pitot1Pin = A0;
const byte pitot2Pin = A1;


// performance limits
const byte MAX_PITCH = 20;
const byte MAX_ROLL = 40;
const byte MAX_PITCH_RATE = 10;
const byte MAX_ROLL_RATE = 10;
const byte MAX_ROLL_REDUCED = 10; // max roll during certain critical modes of flight
const byte MAX_YAW = 25;
const unsigned int STALL_SPEED = 0; // not known yet
const unsigned int MAX_SPEED = 0; // needs to be configured
const int APPROACH_ANGLE = -3; // angle to follow when on ILS approach
const float MAX_THRUST_CHANGE_RATE = 0.04; // limit increases in thrust to avoid stalling motors
const byte MAX_THRUST = 100; // maximum thrust to allow (technical maximum is 180, but can't be sustained without damaging the batteries)

const byte GROUND_HEIGHT = 20; // height measured when on the ground
const byte PITOT_DIFF_TOLERANCE = 4; // to be adjusted, maximum difference in dynamic pressure in kPa readings to allow before dissactivating

// Servo limits
const byte ELEVATOR_MIN = 60;
const byte ELEVATOR_MAX = 165;
const byte AILERON_OFFSET_R = 90;
const byte AILERON_OFFSET_L = 90;
const byte AILERON_MAX_DEFLECTION = 40;


const unsigned int logInterval = 5000; // time between samples in flight data recorder


#include <Servo.h>
#include <SD.h>
#include <TinyGPS++.h>
#include "Wire.h"
#include <MPU6050_light.h>
#include <DFRobot_QMC5883.h>
//#include "WatchDog.h"
#include <BMP180I2C.h>
#include <IRremote.h>
#include <LoRa.h>
#include <setjmp.h>

Servo elevators;
Servo aileronL;
Servo aileronR;
Servo leftMotor;
Servo rightMotor;
Servo sonarPitch;

File dataFile;
TinyGPSPlus gps;
MPU6050 gyro(Wire);
DFRobot_QMC5883 compass(&Wire, 0x1E);
BMP180I2C staticPort(0x77);

const byte MODE_GROUND = 0;
const byte MODE_MAN_FULL = 1;
const byte MODE_MAN_PROT = 2;
const byte MODE_MAN_RATES = 3;
const byte MODE_ANGLE_HOLD = 4;
const byte MODE_ALTITUDE_HOLD = 5;
const byte MODE_HEADING_HOLD = 5;
const byte MODE_THRUST_SPEED_HOLD = 5;
const byte MODE_OPEN_CLIMB = 6;
const byte MODE_YAW_ACC = 7;
const byte MODE_YAW_VANE = 8;
const byte MODE_GPS_WAYPOINT = 9;
const byte MODE_ILS = 10;
const byte MODE_FLARE = 11;

byte modeV = MODE_GROUND; // vertical
byte modeH = MODE_GROUND; // roll
byte modeY = MODE_GROUND; // yaw
byte modeT = MODE_GROUND; // thrust

int targetHeading = 0;
int targetRoll = 0;
unsigned int targetAltitude = 0;
unsigned int targetSpeed = 0;
unsigned int elevatorAngle = 90;
int aileronAngle = 0;
int currentPitch = 0;
int currentRoll = 0;
int sideslipAcc = 0;
float currentPitchRate = 0;
float currentRollRate = 0;
float targetLongitude = 0;
float targetLattitude = 0;
int currentAirspeed = 0;
double currentAltitude = 0;
unsigned long staticPressure = 0;
long dynamicPressure = 0;
long pitotError1 = 0; // systematic error to correct for
long pitotError2 = 0; // systematic error to correct for
long staticError = 0; // systematic error to correct for
double groundAltitude = 0;
int currentHeading = 0;
int runwayHeading = 0;
unsigned long nextLogTime = 0;
const char compile_date[] = __DATE__ " " __TIME__;
unsigned int fileNum = 0;
bool gyroIsValid = true;
bool pitotIsValid = true;
unsigned long lastStaticPortUpdate = 0;
unsigned long lastGroundCom = 0;
unsigned long lastGroundPingRQST = 0;
bool pressureCalibrated = false;
bool groundAltitudeKnown = false;
bool setupFinished = false;
const unsigned int ILS_TIMEOUT = 800;
#define numILSBeacons 5
unsigned long lastSeenILS[numILSBeacons];
bool capturedILS[numILSBeacons];
bool leftMotorOn = false;
bool rightMotorOn = false;
bool inESCReset = false;
unsigned long flareStartTime = 0;
byte thrust = 0;
byte lastThrustL = 0;
byte lastThrustR = 0;
int lastHeight = 0;
int yaw = 0;
bool reducedRoll = false;
int targetPitch = 0;
float targetPitchRate = 0;
float targetRollRate = 0;

int groundProximity(){
  if (currentPitch > -30 and currentPitch < 30) {sonarPitch.write(110 + currentPitch);}
  else {sonarPitch.write(110);}

  digitalWrite(sonarTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(sonarTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sonarTrigPin, LOW);
  long duration = pulseIn(sonarEchoPin, HIGH, 30000);
  int distance = duration * 0.034 / 2;
  
  if (currentPitch < -10) {distance -= 20;}
  else if (currentPitch < 10) {distance -= 5;} // still needs work here

  if (distance > 400){
    distance = 500;
  }
  if (distance < 0) {distance = 0;}
  if (distance > lastHeight+5) {distance = lastHeight + 1;} // avoid random errors

  lastHeight = distance;
  return distance;
}

int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue){
  int ch = pulseIn(channelInput, HIGH, 500);
  if (ch < 100) return defaultValue;
  if (ch < 840) return minLimit;
  if (ch > 1685) return maxLimit;
  return map(ch, 840, 1685, minLimit, maxLimit);
}

int rescale(int val, int fromS, int fromE, int toS, int toE) {
  if (val > fromE){
      val = toE;
  }
  else if (val < fromS){
    val = toS;
  }
  else {
    val = map(val, fromS, fromE, toS, toE);
  }
  return val;
}

unsigned long holdBraudcastsUntil = 0;



void update_GPS() {
  while (Serial3.available()){
    gps.encode(Serial3.read());
  }
}


void update_gyro() {
  if (gyroIsValid){ // replace with new external IMC code when finalized
    gyro.update();
  }


  currentPitch = gyro.getAngleX() - 14;
  currentRoll = gyro.getAngleY();
  currentPitchRate = (currentPitchRate + gyro.getGyroX()) / 2;
  currentRollRate = (currentRollRate + gyro.getGyroY()) / 2;
  sideslipAcc = sideslipAcc / 2 + gyro.getAccX() * 50.0;

  currentHeading = 0;
}




void update_pressure() {

  if (pitotIsValid or true) {
    //int p1 = ((analogRead(pitot1Pin) - pitotError1) / 1024 - 0.5) * 5; // conversion from voltage to kPa
    /* redundany code to activate when second pitot installed

    int p2 = ((analogRead(pitot2Pin) - pitotError2) / 1024 - 0.5) * 5;
    int diff = p2 - p1;
    if (diff < 0) {diff = -diff;}
    if (diff > PITOT_DIFF_TOLERANCE) {pitotIsValid = false;}
    dynamicPressure = (p1 + p2) / 2;*/

    dynamicPressure = analogRead(pitot1Pin) - 549;  // using simplified model

    if (dynamicPressure < -300) {
      pitotIsValid = false;
    }
  }
}


double anonymize_coordinate(double value) {
  int diff = value;
  diff = diff / 10;
  diff = diff * 10;
  return value - diff;
}


void start_data_recorder() {
  if (dataFile){
    return;
  }
  fileNum = 0;
  while (SD.exists("/DIANA/REC/R" + String(fileNum) + ".TXT")){
    fileNum += 1;
  }

  if (dataFile = SD.open("/DIANA/REC/R" + String(fileNum) + ".TXT", FILE_WRITE)) {

    dataFile.print(F("D.I.A.N.A. Flight System\nCompiled: "));
    dataFile.print(compile_date);
    dataFile.print(F("\nFlight start time: "));
    if (gps.date.isValid()) {
      dataFile.print(gps.date.day());
      dataFile.print("/");
      dataFile.print(gps.date.month());
      dataFile.print("/");
      dataFile.print(gps.date.year());
      dataFile.print("   ");
      dataFile.print(gps.time.hour());
      dataFile.print(":");
      dataFile.print(gps.time.minute());
      dataFile.print(":");
      dataFile.print(gps.time.second());
      dataFile.print(F(" UTC"));
    }
    else{
      dataFile.print(F("Unknown, no GPS"));
    }
    dataFile.print(F("\n\n\ntime\tvertical mode\thorizontal mode\tairspeed\tpitch\troll\theading\taltitude\tstatic press\theight\tsideslipAcc\tyaw\tnum satellites\tlongitude\tlattitude\ttarget pitch\ttarget roll\ttarget pitch rate\t target roll rate\ttargetheading\ttarget altitude\televator\taileron\tthrust L\t thrust R\terrors and events\n"));
    nextLogTime = millis() + logInterval;
    //braudcast("Initialized flight data recorder: /DIANA/REC/R" + String(fileNum) + ".TXT");


  } else{
    //braudcast(F("File not created! The flight can't take place without a flight data recorder."));
    //while (true);
  }
}


















void targetPitch_from_targetAltitude() {
  targetPitch = targetAltitude - currentAltitude;
  if (targetPitch < -MAX_PITCH) {targetPitch = -MAX_PITCH;}
  if (targetPitch > MAX_PITCH) {targetPitch = MAX_PITCH;}
}

void targetPitchRate_from_targetPitch() {
  if (targetPitch > MAX_PITCH) {targetPitch = MAX_PITCH;}
  if (targetPitch < -MAX_PITCH) {targetPitch = -MAX_PITCH;}
  targetPitchRate = targetPitch - currentPitch;
  if (targetPitchRate < -MAX_PITCH_RATE) {targetPitchRate = -MAX_PITCH_RATE;}
  if (targetPitchRate > MAX_PITCH_RATE) {targetPitchRate = MAX_PITCH_RATE;}
}

void elevator_from_targetPitchRate() {
  if (targetPitchRate > currentPitchRate) {elevatorAngle += 1;}
  if (targetPitchRate < currentPitchRate) {elevatorAngle -= 1;}

  if (elevatorAngle < ELEVATOR_MIN) {elevatorAngle = ELEVATOR_MIN;}
  if (elevatorAngle > ELEVATOR_MAX) {elevatorAngle = ELEVATOR_MAX;}
}



void targetRoll_from_targetHeading() {
  int diff = targetHeading - currentHeading;
  if (diff < -180) {diff += 360;}
  else if (diff > 180) {diff -= 360;}
  targetRoll = 2*diff;
  if (targetRoll > MAX_ROLL) {targetRoll = MAX_ROLL;}
  if (targetRoll < -MAX_ROLL) {targetRoll = -MAX_ROLL;}
}

void targetRollRate_from_targetRoll() {
  if (targetRoll > MAX_ROLL) {targetRoll = MAX_ROLL;}
  if (targetRoll < -MAX_ROLL) {targetRoll = -MAX_ROLL;}
  targetRollRate = targetRoll - currentRoll;
  if (targetRollRate > MAX_ROLL_RATE) {targetRollRate = MAX_ROLL_RATE;}
  if (targetRollRate < -MAX_ROLL_RATE) {targetRollRate = -MAX_ROLL_RATE;}
}

void ailerons_from_targetRollRate() {
  if (targetRollRate > currentRollRate) {aileronAngle += 1;}
  else if (targetRollRate < currentRollRate) {aileronAngle -= 1;}

  if (aileronAngle > AILERON_MAX_DEFLECTION) {aileronAngle = AILERON_MAX_DEFLECTION;}
  if (aileronAngle < -AILERON_MAX_DEFLECTION) {aileronAngle = -AILERON_MAX_DEFLECTION;}
}


void pitch_protections() {
  if (currentPitch > MAX_PITCH * 1.1) {
    targetPitch = 0.9 * MAX_PITCH;
    targetPitchRate_from_targetPitch();
    elevator_from_targetPitchRate();
  }
  if (currentPitch < -MAX_PITCH * 1.1) {
    targetPitch = -0.9 * MAX_PITCH;
    targetPitchRate_from_targetPitch();
    elevator_from_targetPitchRate();
  }


  if (currentAirspeed < STALL_SPEED) {
    reducedRoll = true;
    thrust = MAX_THRUST;
    targetPitch = -MAX_PITCH;
    targetPitchRate_from_targetPitch();
    elevator_from_targetPitchRate();
  }

  else if (currentAirspeed > MAX_SPEED) {
    thrust = 0;
  }
}


void roll_protections() {
  if (reducedRoll and currentRoll > MAX_ROLL_REDUCED * 1.1) {
    targetRoll = 0.9 * MAX_ROLL_REDUCED;
    targetRollRate_from_targetRoll();
    ailerons_from_targetRollRate();
  }
  else if (reducedRoll and currentRoll < -MAX_ROLL_REDUCED * 1.1) {
    targetRoll = -0.9 * MAX_ROLL_REDUCED;
    targetRollRate_from_targetRoll();
    ailerons_from_targetRollRate();
  }
  else if (currentRoll > MAX_ROLL * 1.1) {
    targetRoll = 0.9 * MAX_ROLL;
    targetRollRate_from_targetRoll();
    ailerons_from_targetRollRate();
  }
  else if (currentRoll < -MAX_ROLL * 1.1) {
    targetRoll = -0.9 * MAX_ROLL;
    targetRollRate_from_targetRoll();
    ailerons_from_targetRollRate();
  }
}

void go_around() {
  modeV = MODE_OPEN_CLIMB;
  modeH = MODE_ANGLE_HOLD;
  modeT = MODE_OPEN_CLIMB;
  targetRoll = 0;

  if (groundAltitudeKnown) {targetAltitude = groundAltitude + 30;}
  else {targetAltitude = currentAltitude + 30;}
}










void setup() {
  Serial.begin(115200);
  Serial1.begin(115200); // bluetooth connection to ground systems
  Serial2.begin(115200); // arduino nano connected to pitot tube
  Serial3.begin(9600); // GPS
  Wire.begin();
  //IrReceiver.begin(7);
  //elevators.attach(8);
  //aileronL.attach(23);
  //aileronR.attach();
  //leftMotor.attach(48);
  //rightMotor.attach(49);
  //sonarPitch.attach(45);

  pinMode(sonarTrigPin, OUTPUT);
  pinMode(sonarEchoPin, INPUT);

  leftMotor.write(0);
  rightMotor.write(0);
  elevators.write(90);
  aileronL.write(90);
  aileronR.write(90);



  // init flight data recorder
  if (!SD.begin(53)) {
    //braudcast(F("SD card not found! The flight can't take place without a flight data recorder."));
    //while (true);
  }
  else{
    SD.mkdir("/DIANA");
    SD.mkdir("/DIANA/REC");
    SD.mkdir("/DIANA/WAYPTS");
  }



  update_pressure();

  nextLogTime = millis() + logInterval;
  //braudcast(F("Initialization Complete"));
  setupFinished = true;
}




















void loop() {

  unsigned long t = millis();

  String command1 = "";
  String command2 = "";

  if (Serial.available()) {
    command1 = Serial.readStringUntil(' ');
    command2 = Serial.readStringUntil('\n');
    command1.toUpperCase();
  }
  else if (Serial1.available()) {
    lastGroundCom = millis();
    command1 = Serial1.readStringUntil(' ');
    command2 = Serial1.readStringUntil('\n');
    command1.toUpperCase();

    Serial.println(command1 + " " + command2);
  }
  
  if (command1 != "") {
    if (command1 == "SMV") {modeV = command2.toInt();}
    if (command1 == "SMH") {modeH = command2.toInt();}
    if (command1 == "SMY") {modeH = command2.toInt();}
    if (command1 == "SMT") {modeH = command2.toInt();}

    if (command1 == "TH") {
      int target = command2.toInt();
      if (target >= 0 and target < 360) {
        targetHeading = target;
      }
    }

    if (command1 == "TA") {
      int target = command2.toInt();
      if (target > 0) {
        targetAltitude = target;
      }
    }

    if (command1 == "RHDG") {
      runwayHeading = command2.toInt();
      //braudcast("new runway heading: " + String(runwayHeading), true);
    }

    if (command1 == "R" and modeV == MODE_GROUND) {


      if (command2 == "G"){ // recalibrate gyroscope
        
      }

      else if (command2 == "M1") { // recalibrate motor ESC's step 1 (battery disconnected)
        leftMotor.write(180);
        rightMotor.write(180);
        inESCReset = true;
      }
      else if (command2 == "M2") { // recalibrate motor ESC's step 2 (battery connected)
        leftMotor.write(1);
        rightMotor.write(1);
        lastThrustL = 0;
        lastThrustR = 0;
        inESCReset = false;
      }
    }
  }

  if ((lastGroundCom == 0 or millis() - lastGroundCom > 30000) and millis() - lastGroundPingRQST > 2000) {
    Serial1.write(1);
    lastGroundPingRQST = millis();
  }


  if (IrReceiver.decode()) {
    if (IrReceiver.decodedIRData.command > 0 and IrReceiver.decodedIRData.command <= numILSBeacons) {
      lastSeenILS[IrReceiver.decodedIRData.command -1] = millis();
    }
    IrReceiver.resume();
  }

  for (byte i=0; i<numILSBeacons; i++) {
    if (millis() - lastSeenILS[i] < ILS_TIMEOUT) {
      capturedILS[i] = true;
    }
    else {
      capturedILS[i] = false;
    }
  }















  update_gyro();
  update_GPS();

  reducedRoll = false;
  double currentLongitude = gps.location.lng();
  double currentLattitude = gps.location.lat();
  int height = groundProximity();
  update_pressure();



  int yawcont = readChannel(yawChannel, -10, 10, 0);
  if (yawcont <= -9) {
    modeV = MODE_MAN_FULL;
    modeH = MODE_MAN_FULL;
    modeY = MODE_MAN_FULL;
    modeT = MODE_MAN_FULL;
  }
  if (yawcont >= 9 and modeV != MODE_MAN_FULL) {
    modeV = MODE_MAN_PROT;
    modeH = MODE_MAN_PROT;
    modeT = MODE_MAN_PROT;
  }

  if (modeY == MODE_YAW_ACC) {
    if (gyroIsValid) {
      if (sideslipAcc > 0) {yaw += 1;}
      else {yaw -= 1;}
    }

    else {
      modeY = MODE_MAN_FULL;
    }
  }

  if (height < 120) {reducedRoll = true;}


  if (gps.location.isValid() and gps.satellites.value() >= 5 and modeV == MODE_GROUND) {
    groundAltitude = gps.altitude.meters();
    if (!groundAltitudeKnown){
      groundAltitudeKnown = true;
      //braudcast(F("ground GPS altitude set"), true);
    }
  }

  if (gps.location.isValid() and gps.satellites.value() >= 5 and groundAltitudeKnown) {
    currentAltitude = gps.altitude.meters() - groundAltitude;
  }

  if (pitotIsValid){
    //currentAirspeed = pow(2*dynamicPressure / 1.225, 0.5);
    currentAirspeed = dynamicPressure; // using simplified model
  }
  


  if (modeV == MODE_GROUND) {
    elevatorAngle = 90;
    aileronAngle = 0;
    thrust = 0;
  }

    
 if (modeV == MODE_MAN_RATES) {
    targetPitchRate = readChannel(pitchChannel, -MAX_PITCH_RATE, MAX_PITCH_RATE, 0);
    elevator_from_targetPitchRate();
    pitch_protections();
  }

  if (modeV == MODE_MAN_PROT) {
    elevatorAngle = readChannel(pitchChannel, ELEVATOR_MIN, ELEVATOR_MAX, 90);
    pitch_protections();
  }

  if (modeV == MODE_MAN_FULL) {
    elevatorAngle = readChannel(pitchChannel, ELEVATOR_MIN, ELEVATOR_MAX, 90);
  }


  if (modeV == MODE_ALTITUDE_HOLD) {
    int input = readChannel(pitchChannel, -30, 30, 0);
    if (input > 10) {
      targetAltitude -= 1;
    }
    else if (input < -10) {
      targetAltitude += 1;
    }

    targetPitch_from_targetAltitude();
    targetPitchRate_from_targetPitch();
    elevator_from_targetPitchRate();
  }

  if (modeH == MODE_HEADING_HOLD) {

    int input = readChannel(rollChannel, -30, 30, 0);
    if (input > 10) {
      targetHeading += 1;
    }
    else if (input < -10) {
      targetHeading -= 1;
    }

    if (targetHeading >= 360) {targetHeading -= 360;}
    if (targetHeading < 0) {targetHeading += 360;}

    targetRoll_from_targetHeading();
    targetRollRate_from_targetRoll();
    ailerons_from_targetRollRate();
  }

  if (modeH == MODE_ANGLE_HOLD) {
    targetRollRate_from_targetRoll();
    ailerons_from_targetRollRate();
  }

  if (modeV == MODE_OPEN_CLIMB) {
    if (currentPitch < 3) {
      targetPitch = 4;
      targetPitchRate_from_targetPitch();
    }
    else {
      targetPitchRate = currentAirspeed - STALL_SPEED*1.5;
      if (targetPitchRate > MAX_PITCH_RATE) {targetPitchRate = MAX_PITCH_RATE;}
      if (targetPitchRate < -MAX_PITCH_RATE) {targetPitchRate = -MAX_PITCH_RATE;}
    }
    elevator_from_targetPitchRate();
    modeT = MODE_OPEN_CLIMB;
  }

  if (modeH == MODE_GPS_WAYPOINT) {

    float d_latt = targetLattitude - currentLattitude;
    float d_long = targetLongitude - currentLongitude;
    targetHeading = tan(d_long/d_latt) * 180/PI;
    if (d_long > 0 and d_latt < 0) {targetHeading += 90;}
    if (d_long < 0 and d_latt < 0) {targetHeading -= 180;}
    if (d_long < 0 and d_latt > 0) {targetHeading -= 90;}

    targetRoll_from_targetHeading();
    targetRollRate_from_targetRoll();
    ailerons_from_targetRollRate();
  }

  if (modeV == MODE_ILS) {
    if (height > 300){targetSpeed = STALL_SPEED * 1.5;}
    else{targetSpeed = STALL_SPEED*1.1;}

    if (capturedILS[0] and capturedILS[1]) {targetPitch = APPROACH_ANGLE;}
    else if (capturedILS[0]) {targetPitch = APPROACH_ANGLE - 5;}
    else if (capturedILS[1]) {targetPitch = APPROACH_ANGLE + 5;}
    else {
      //braudcast(F("Go-around initiated, reason: lost glideslope"), true);
      go_around();
    }

    if (capturedILS[4]) {
      if (height > 150) {
        //braudcast(F("Go-around initiated, reason: too high at threshold"), true);
        go_around();
      }
      else {
        modeV = MODE_FLARE;
        modeH = MODE_ANGLE_HOLD;
        targetRoll = 0;
        modeT = MODE_FLARE;
        modeY = MODE_FLARE;
      }
    }

    if (height < 350 and modeH != "ILS") {
      //braudcast(F("Go-Around, reason: localiser not intercepted"), true);
      go_around();
    }

    if (height < 100) {
      //braudcast(F("Go-Around, reason: threshold beacon not found"), true);
      go_around();
    }
  }

  if (modeH == MODE_ILS) {
    targetRoll_from_targetHeading();
    targetRollRate_from_targetRoll();
    ailerons_from_targetRollRate();

    if (capturedILS[2] and capturedILS[3]) {targetHeading = runwayHeading;}
    else if (capturedILS[2]) {targetHeading = runwayHeading - 5;}
    else if (capturedILS[3]) {targetHeading = runwayHeading + 5;}
    else {
      //braudcast(F("Go-around, reason: lost localiser"), true);
      go_around();
    }

    if (height < 350 and modeV != MODE_ILS) {
      //braudcast(F("Go-Around, reason: glideslope not intercepted"), true);
      go_around();
    }
  }

  if (modeV == MODE_FLARE) {
    reducedRoll = true;
    thrust = 0;
    targetPitch = 5; // to be adjusted after test flights
    if (millis() - flareStartTime > 5000) {
      //braudcast(F("Go-around initiated, reason: 5 second flare rule"), true);
      go_around();
    }
    if (height <= GROUND_HEIGHT) {
      modeV = MODE_GROUND;
      modeH = MODE_GROUND;
      modeY = MODE_GROUND;
      modeT = MODE_GROUND;
    }

    if (readChannel(thrustChannel, 0, 100, 0) > 90) {
      go_around();
      //braudcast: manual go-around triggered
    }
  }






































  if (modeT == MODE_THRUST_SPEED_HOLD) {
    if (targetSpeed > currentAirspeed and thrust < MAX_THRUST) {thrust += 1;}
    else if (thrust > 0) {thrust -= 1;}
  }



  if (elevatorAngle < ELEVATOR_MIN) {elevatorAngle = ELEVATOR_MIN;}
  if (elevatorAngle > ELEVATOR_MAX) {elevatorAngle = ELEVATOR_MAX;}

  if (aileronAngle < -AILERON_MAX_DEFLECTION) {aileronAngle = -AILERON_MAX_DEFLECTION;}
  if (aileronAngle > AILERON_MAX_DEFLECTION) {aileronAngle = AILERON_MAX_DEFLECTION;}

  if (yaw < 5 and yaw > -5) {yaw = 0;}














  int thrustL = thrust + yaw;
  int thrustR = thrust - yaw;


  if (thrustL > 10) {leftMotorOn = true;} // prevent bouncing in signal at low thrust levels from stalling the motors
  if (thrustL < 3) {leftMotorOn = false;}
  if (leftMotorOn == false){
    thrustL = 0;
  }


  if (thrustR > 10) {rightMotorOn = true;} // prevent bouncing in signal at low thrust levels from stalling the motors
  if (thrustR < 3) {rightMotorOn = false;}
  if (rightMotorOn == false){
    thrustR = 0;
  }

  // prevent sudden increases in thrust, which can stall motors, make increase gradual.
  if (thrustL - lastThrustL > MAX_THRUST_CHANGE_RATE * 500) {
    if (millis() - t >= 500) {
      thrustL = lastThrustL + MAX_THRUST_CHANGE_RATE * 500;
    }
    else{
      thrustL = lastThrustL + MAX_THRUST_CHANGE_RATE * (millis() - t);
    }
  }


  if (thrustR - lastThrustR > MAX_THRUST_CHANGE_RATE * 500) {
    if (millis() - t >= 500) {
      thrustR = lastThrustR + MAX_THRUST_CHANGE_RATE * 500;
    }
    else{
      thrustR = lastThrustR + MAX_THRUST_CHANGE_RATE * (millis() - t);
    }
  }


  if (thrustL < 0) {thrustL = 0;}
  if (thrustR < 0) {thrustR = 0;}
  if (thrustL > MAX_THRUST) {thrustL = MAX_THRUST;}
  if (thrustR > MAX_THRUST) {thrustR = MAX_THRUST;}


  elevators.write(elevatorAngle);
  aileronL.write(aileronAngle + AILERON_OFFSET_L);
  aileronR.write(aileronAngle + AILERON_OFFSET_R);

  if (!inESCReset) {
    leftMotor.write(thrustL);
    rightMotor.write(thrustR);
    lastThrustL = thrustL;
    lastThrustR = thrustR;
  }

  if (modeT != MODE_GROUND and !dataFile) {
    start_data_recorder();
  }

  if (nextLogTime < millis() and dataFile) {
    nextLogTime += logInterval;
    dataFile.print(millis() / 1000); dataFile.print("\t");
    dataFile.print(modeV); dataFile.print("\t");
    dataFile.print(modeH); dataFile.print("\t");
    dataFile.print(currentAirspeed); dataFile.print("\t");
    dataFile.print(currentPitch); dataFile.print("\t");
    dataFile.print(currentRoll); dataFile.print("\t");
    dataFile.print(currentHeading); dataFile.print("\t");
    dataFile.print(currentAltitude); dataFile.print("\t");
    dataFile.print(staticPressure); dataFile.print("\t");
    dataFile.print(height); dataFile.print("\t");
    dataFile.print(sideslipAcc); dataFile.print("\t");
    dataFile.print(yaw); dataFile.print("\t");
    dataFile.print(gps.satellites.value()); dataFile.print("\t");
    dataFile.print(anonymize_coordinate(currentLongitude)); dataFile.print("\t");
    dataFile.print(anonymize_coordinate(currentLattitude)); dataFile.print("\t");
    dataFile.print(targetPitch); dataFile.print("\t");
    dataFile.print(targetRoll); dataFile.print("\t");
    dataFile.print(targetPitchRate); dataFile.print("\t");
    dataFile.print(targetRollRate); dataFile.print("\t");
    dataFile.print(targetHeading); dataFile.print("\t");
    dataFile.print(targetAltitude); dataFile.print("\t");
    dataFile.print(elevatorAngle); dataFile.print("\t");
    dataFile.print(aileronAngle); dataFile.print("\t");
    dataFile.print(thrustL); dataFile.print("\t");
    dataFile.print(thrustR); dataFile.print("\t");
    dataFile.print("\n");
    if (modeV == MODE_GROUND) {
      dataFile.close();
    }
  }
}
