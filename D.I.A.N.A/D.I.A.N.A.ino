/* 

Demonstrator for Intelligent Automatically Navigating Aircraft

This software is still in development
DO NOT USE IN FLIGHT! (yet)



WARNING: Terrain escape protection is temporarily disactivated for gound testing purposes.




Flight Mode System Requirements:

Required systems are cumulative and include those from all previous categories

Cat 0 - requires flight data recorder (temporarily bypassible for ground testing)

Cat 1 - requires gyroscope and altitude information (either working recalibrated static pressure or minimum 5 GPS sattelites and known terrain altitude)

Cat 2 - requires magnetic compass

Cat 3 - requires minimum 5 GPS satellites




Flight Modes:


Global Modes:

GROUND -    Cat 0
            Engines off, brakes on, ignores all inputs until mode changed

MAN FULL -  (full manual) - Cat 0
            direct radio control over control surfaces.
            No protections. (normal RC plane)
            Should only be reverted to during landings and emergencies / when sensor inputs are bad

MAN PROT -  (protected manual) - Cat 1
            direct radio control over control surfaces.
            must not be used for landing
            should be avoided at low altitudes, stall protection may result in crash
            will reduce to MAN FULL mode if not in Cat 1 conditions
            System will override in following cases:
              - excessive pitch
              - excessive roll
              - stall
              - overspeed
              - ground proximity terrain escape (reduced margin)

MAN RATE -  (manual pitch/roll rate) - Cat 1
            radio control inputs set target pitch and roll rates
            must not be used for landing
            should be avoided at low altitudes, stall protection may result in crash
            will reduce to MAN FULL mode if not in Cat 1 conditions
            System will override in following cases:
              - excessive pitch
              - excessive roll
              - stall
              - overspeed
              - ground proximity terrain escape (reduced margin)

MAN SIMP -  (simplified manual) - Cat 1
            Radio control inputs correspond to pitch angle and roll angle
            must not be used for landing
            will reduce to MAN FULL mode if not in Cat 1 conditions
            System will override in following cases:
              - excessive pitch
              - excessive roll
              - stall
              - overspeed
              - ground proximity terrain escape (reduced margin)




Vertical Modes:

CLIMB -     (maximum climb rate) - Cat 1
            radio controls increment/decrement target altitude
            Climbs just above stall speed at full thrust
            Once target altitude is reached, enters HOLD mode

HOLD -      (altitude hold) - cat 2
            radio controls increment/decrement target altitude
            will reduce to MAN PROT mode if not in Cat 2 conditions

ILS -       (autoland ILS Final approach glideslope) - Cat 2 + infrared ILS reception
            follows ILS glideslope path to the runway
            enters FLARE mode if passes runway threshold beacon below 2m
            enters CLIMB mode if passes runway threshold beacon above 2m
            enters CLIMB mode if lower than 1m without passing runway threshold beacon
            enters CLIMB mode if loses ILS contact
            enters CLIMB mode if horizontal mode is not also ILS below 3.5 m
            reduces to MAN PROT if not in Cat 2 conditions

FLARE -     (autoland flare) - Cat 2
            flares and in final seconds of flight during autolands.
            thrust reduced to 0
            used simultaneously with horizontal hold at runway heading
            pitch maintained constant at (not yet decided)º
            enters GROUND mode once wheels touch the runway
            enters CLIMB mode if activated for longer than 5 seconds without landing
            enters CLIMB mode if remote control thrust increased to maximum
            will reduce to MAN FULL mode if not in Cat 2 conditions (protections may cause unwanted nosedive, so no MAN PROT)



Horizontal Modes:

LVL -       (hold wings level) - Cat 1
            will reduce to MAN FULL if not in Cat 1 conditions
            will promote to HOLD if in Cat 2 conditions

HOLD -      (heading hold) - cat 2
            radio controls increment/decrement target heading
            will reduce to LVL mode if not in Cat 2 conditions

WAYPT -     (follow waypoint) - Cat 3
            automatically navigates to selected GPS coordinates
            when target reached, enters HOLD mode
            will reduce to HOLD mode if not in Cat 3 conditions

ILS -       (autoland ILS Final approach localiser) - Cat 2 + infrared ILS reception
            follows ILS localiser path to the runway
            enters CLIMB HDG mode if loses ILS contact
            enters CLIMB mode if vertical mode is not also ILS below 3.5 m
            reduces to MAN PROT if not in Cat 2 conditions



general control information:
 - maximum left rudder  -> sets to MAN FULL
 - maximum right rudder -> sets to MAN PROT
 - yaw control is automatic in all modes except MAN FULL
*/


const float declinationAngle = ((1 + (57.0 / 60.0)) / (180 / PI)); // magnetic declination at location of flight

// Hardware connections
#define yawChannel 3
#define thrustChannel 4
#define pitchChannel 5
#define rollChannel 6
#define sonarTrigPin 46
#define sonarEchoPin 47
#define IR_reciever 2
#define pitot1Pin A0
//#define pitot2Pin A1 // redundant pitot tube will be added in the future


// performance limits
#define MAX_AUTO_CAT 3 // limit for flight mode category to allow during the flight (see above)
#define MAX_PITCH 20
#define MAX_ROLL 30
#define MAX_PITCH_RATE 10
#define MAX_ROLL_RATE 10
#define MAX_ROLL_REDUCED 10 // max roll during certain critical modes of flight
#define MAX_YAW 25
#define STALL_SPEED 0 // not known yet
#define MAX_SPEED 0 // needs to be configured
#define SAFE_CLIMB_HEIGHT 20 // height to ascend during take-off, go-around, and terrain escape
#define APPROACH_ANGLE -3 // angle to follow when on ILS approach
#define MAX_THRUST_CHANGE_RATE 0.04 // limit increases in thrust to avoid stalling motors
#define MAX_THRUST 100 // maximum thrust to allow (technical maximum is 180, but can't be sustained without damaging the batteries)

#define GROUND_HEIGHT 20 // height measured when on the ground
#define PITOT_DIFF_TOLERANCE 4 // to be adjusted, maximum difference in dynamic pressure in kPa readings to allow before dissactivating

// Servo limits
#define ELEVATOR_MIN 60
#define ELEVATOR_MAX 165
#define AILERON_MIN 45
#define AILERON_MAX 135


#define logInterval 5000 // time between samples in flight data recorder


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
Servo ailerons;
Servo leftMotor;
Servo rightMotor;
Servo sonarPitch;

File dataFile;
TinyGPSPlus gps;
MPU6050 gyro(Wire);
DFRobot_QMC5883 compass(&Wire, 0x1E);
BMP180I2C staticPort(0x77);


String modeV = "GROUND";
String modeH = "GROUND";
int targetHeading = 0;
int targetRoll = 0;
unsigned int targetAltitude = 0;
unsigned int targetSpeed = 0;
unsigned int elevatorAngle = 90;
unsigned int aileronAngle = 90;
int currentPitch = 0;
int currentRoll = 0;
int sideslip = 0;
float currentPitchRate = 0;
float currentRollRate = 0;
float targetLongitude = 0;
float targetLattitude = 0;
int currentAirspeed = 0;
double currentAltitude = 0;
unsigned long staticPressure = 0;
long dynamicPressure = 0;
unsigned long groundPressure = 101325;
unsigned int groundTemperature = 273; // in kelvin
long pitotError1 = 0; // systematic error to correct for
long pitotError2 = 0; // systematic error to correct for
long staticError = 0; // systematic error to correct for
double groundAltitude = 0;
int currentHeading = 0;
int runwayHeading = 0;
unsigned long nextLogTime = 0;
String braudcastedData = "";
const char compile_date[] = __DATE__ " " __TIME__;
unsigned int fileNum = 0;
bool gyroWorking = true;
bool compassWorking = true;
bool pitotWorking = true;
bool staticPressWorking = true;
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
bool inESPReset = false;
unsigned long flareStartTime = 0;
byte thrust = 0;
byte lastThrustL = 0;
byte lastThrustR = 0;
int lastHeight = 0;
int autoYaw = 0;

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
  int ch = pulseIn(channelInput, HIGH, 30000);
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

void braudcast(String message, bool log = false, bool hold = true){ // transmit message to operator via bluetooth

  if (log) {
    message.replace("\n", "  \\n  ");
    if (braudcastedData.indexOf(message) != -1){
      return; // avoid flood of repeat messages
    }

    if (braudcastedData != "") {
      braudcastedData += "\t";
    }
    braudcastedData += message;
  }

  if (hold or millis() > holdBraudcastsUntil) {
    Serial.println(message);

    while (message.length() > 10) {
      Serial1.print(message.substring(0, 10));
      message.remove(0, 10);
      delay(20);
    }
    Serial1.println(message);

    holdBraudcastsUntil = millis() + 1000;
  }
}

void update_GPS() {
  while (Serial3.available()){
    gps.encode(Serial3.read());
  }
}

/*
void WDT_trigger() {
  gyroWorking = false;
  WatchDog::stop();
  braudcast(F("Gyroscope timeout error"), true);
  if (setupFinished) {
    goto LOOP_START;
  }
}*/

void update_gyro() {
  if (gyroWorking){
    //WatchDog::start();
    gyro.update();
    //WatchDog::stop();
  }
}


int compassOffsets[] = {
  277, // 0
  283, // 10
  287, // 20
  292, // 30
  296, // 40
  301, // 50
  -55, // 60
  -52, // 70
  -51, // 80
  -50, // 90
  -51, // 100
  -54, // 110
  -56, // 120
  -60, // 130
  -65, // 140
  -71, // 150
  -79, // 160
  -84, // 170
  -91, // 180
  -97, // 190
  -103, // 200
  -111, // 210
  -117, // 220
  -123, // 230
  -126, // 240
  -126, // 250
  -126, // 260
  -126, // 270
  -124, // 280
  -120, // 290
  -114, // 300
  -109, // 310
  -104, // 320
  -90, // 330
  -94, // 340
  -88, // 350
  277, // 360
};

unsigned int get_heading() {
  compass.setDeclinationAngle(0);
  sVector_t mag = compass.readRaw();
  compass.getHeadingDegrees();
  int hdg = mag.HeadingDegress;
  
  for (byte i=0; i<36; i++) {
    if (hdg >= 10 * i and hdg < 10 * (i+1)) {
      int offset1 = compassOffsets[i];
      int offset2 = compassOffsets[i+1];
      if (offset1 > 0 and offset2 < 0) {offset2 = 360+offset2;}
      if (offset2 > 0 and offset1 < 0) {offset1 = 360+offset1;}

      int offset = (offset1 * (10*(i+1) - hdg) + offset2 * (hdg - 10*i) ) / 10;
      hdg += offset;
      break;
    }
  }

  while (hdg >= 360) {hdg -= 360;}
  while (hdg < 0) {hdg += 360;}

  return hdg;
}


void update_pressure() {
  // static pressure:
  if (staticPressWorking) {
    if (staticPort.hasValue()) {
      staticPressure = staticPort.getPressure();
      lastStaticPortUpdate = millis();
    }
    staticPort.measurePressure();//prepare measurement for next loop pass
  }

  if (pitotWorking or true) {
    //int p1 = ((analogRead(pitot1Pin) - pitotError1) / 1024 - 0.5) * 5; // conversion from voltage to kPa
    /* redundany code to activate when second pitot installed

    int p2 = ((analogRead(pitot2Pin) - pitotError2) / 1024 - 0.5) * 5;
    int diff = p2 - p1;
    if (diff < 0) {diff = -diff;}
    if (diff > PITOT_DIFF_TOLERANCE) {pitotWorking = false;}
    dynamicPressure = (p1 + p2) / 2;*/

    dynamicPressure = analogRead(pitot1Pin) - 549;  // using simplified model

    if (dynamicPressure < -300) {
      pitotWorking = false;
      braudcast(F("Pitot unavailable"), true);
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
    dataFile.print(F("\n\n\ntime\tvertical mode\thorizontal mode\tairspeed\tpitch\troll\theading\taltitude\tstatic press\theight\tsideslip\tyaw\tnum satellites\tlongitude\tlattitude\ttarget pitch\ttarget roll\ttarget pitch rate\t target roll rate\ttargetheading\ttarget altitude\televator\taileron\tthrust L\t thrust R\terrors and events\n"));
    nextLogTime = millis() + logInterval;
    braudcast("Initialized flight data recorder: /DIANA/REC/R" + String(fileNum) + ".TXT");


  } else{
    braudcast(F("File not created! The flight can't take place without a flight data recorder."));
    //while (true);
  }
}






void set_mode(String newMode) {
  Serial.println(newMode);

  if (newMode == "HOLD") {
    targetHeading = currentHeading;
    targetAltitude = currentAltitude;
    modeV = "HOLD";
    modeH = "HOLD";
  }
  else if (newMode == "CLIMB"){
    targetAltitude = currentAltitude + SAFE_CLIMB_HEIGHT;
    modeV = "CLIMB";
  }
  else if (newMode == "FLARE"){
    flareStartTime = millis();
    targetHeading = runwayHeading;
    modeV = "FLARE";
    modeH = "HOLD";
  }
  else if (newMode == "GROUND") {
    modeV = "GROUND";
    modeH = "GROUND";
  }
  else if (newMode == "MAN FULL") {
    modeV = "MAN FULL";
    modeH = "MAN FULL";
  }
  else if (newMode == "MAN PROT") {
    modeV = "MAN PROT";
    modeH = "MAN PROT";
  }
  else if (newMode == "MAN RATE") {
    modeV = "MAN RATE";
    modeH = "MAN RATE";
  }
  else if (newMode == "MAN SIMP") {
    modeV = "MAN SIMP";
    modeH = "MAN SIMP";
  }
  else if (newMode == "WAYPT") {
    targetAltitude = currentAltitude;
    modeV = "HOLD";
    modeH = "WAYPT";
  }
  else {
    braudcast("ERROR: invalid mode change to '" + newMode +"'", true);
  }

  if (newMode != "GROUND") {
    start_data_recorder();
  }
}










void setup() {
  Serial.begin(115200);
  Serial1.begin(115200); // bluetooth connection to ground systems
  Serial2.begin(115200); // arduino nano connected to pitot tube
  Serial3.begin(9600); // GPS
  Wire.begin();
  IrReceiver.begin(7);
  elevators.attach(8);
  ailerons.attach(23);
  leftMotor.attach(48);
  rightMotor.attach(49);
  sonarPitch.attach(45);

  pinMode(sonarTrigPin, OUTPUT);
  pinMode(sonarEchoPin, INPUT);

  leftMotor.write(0);
  rightMotor.write(0);
  elevators.write(90);
  ailerons.write(90);



  // init flight data recorder
  if (!SD.begin(53)) {
    braudcast(F("SD card not found! The flight can't take place without a flight data recorder."));
    //while (true);
  }
  else{
    SD.mkdir("/DIANA");
    SD.mkdir("/DIANA/REC");
    SD.mkdir("/DIANA/WAYPTS");
  }

//  WatchDog::init(WDT_trigger, OVF_500MS);
//  WatchDog::stop();

  byte gyroStatus = gyro.begin();
  if (gyroStatus == 0){
    gyro.setGyroOffsets(0, 0, 0);
    gyro.setAccOffsets(0, 0, 0);
  }
  else{
    braudcast("MPU6050 gyroscope error status: " + String(gyroStatus), true);
    gyroWorking = false;
  }

  if (!compass.begin()) {
    braudcast(F("Failed to connect to magnetic compass"), true);
    compassWorking = false;
  }

  if (staticPort.begin()) {
    staticPort.resetToDefaults();
    staticPort.setSamplingMode(BMP180MI::MODE_UHR);
  }
  else{
    braudcast(F("Failed to initialize static port pressure"), true);
    staticPressWorking = false;
  }

  update_pressure();

  nextLogTime = millis() + logInterval;
  braudcast(F("Initialization Complete"));
  setupFinished = true;
}




















void loop() {
  LOOP_START:

  unsigned long t = millis();
  update_gyro();
  
  String dataBraudcast = "mode: " + modeV + " / " + modeH;

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
    if (command1 == "SM") {set_mode(command2);}
    if (command1 == "SMV") {modeV = command2;}
    if (command1 == "SMH") {modeH = command2;}

    if (command1 == "TH") {
      int target = command2.toInt();
      if (target >= 0 and target < 360) {
        targetHeading = target;
      }
      else {
        braudcast("Couldn't change target heading to: " + command2, true);
      }
    }

    if (command1 == "TA") {
      int target = command2.toInt();
      if (target > 0) {
        targetAltitude = target;
      }
      else {
        braudcast("Couldn't change target altitude to: " + command2, true);
      }
    }

    if (command1 == "WAYPT") {
      if (File waypointFile = SD.open("DIANA/WAYPTS/" + command2, FILE_READ)) {
        command2 = waypointFile.readString();
        waypointFile.close();
      }
      float longitude = command2.substring(0, command2.indexOf(',')).toFloat();
      command2.remove(0, command2.indexOf(','));
      float lattitude = command2.substring(0, command2.indexOf(',')).toFloat();
      command2.remove(0, command2.indexOf(','));
      float altitude = command2.toFloat();
      
      if (altitude > 0) {
        targetLongitude = longitude;
        targetLattitude = lattitude;
        targetAltitude = altitude;
        set_mode("WAYPT");
      }
    }

    if (command1 == "RHDG") {
      runwayHeading = command2.toInt();
      braudcast("new runway heading: " + String(runwayHeading), true);
    }

    if (command1 == "R" and modeV == "GROUND") {

      if (command2 == "P"){ // recallibrate pressure
        if (modeV == "GROUND"){
          pitotError1 = analogRead(pitot1Pin);
          //pitotError2 = analogRead(pitot2Pin);
          staticError += staticPressure - groundPressure;
          groundPressure += staticPressure - groundPressure;
          Serial.println(staticError);
          pressureCalibrated = true;
          braudcast(F("Static port calibrated"), true);
        }
        else {
          braudcast(F("Couldn't recalibrate static port, Not in GROUND mode"), true);
        }
      }

      else if (command2 == "G"){ // recalibrate gyroscope
        if (modeV == "GROUND"){
          gyro.calcOffsets();
          braudcast(F("Gyroscope calibrated"), true);
        }
        else {
          braudcast(F("Couldn't recalibrate gyroscope, Not in GROUND mode"), true);
        }
      }

      else if (command2 == "M1") { // recalibrate motor ESP's step 1 (battery disconnected)
        leftMotor.write(180);
        rightMotor.write(180);
        inESPReset = true;
      }
      else if (command2 == "M2") { // recalibrate motor ESP's step 2 (battery connected)
        leftMotor.write(1);
        rightMotor.write(1);
        lastThrustL = 0;
        lastThrustR = 0;
        inESPReset = false;
      }
    }

    if (command1 == "SYS_P") {
      groundPressure = command2.toInt() + staticError;
    }

    if (command1 == "SYS_T") {
      groundTemperature = command2.toInt();
    }
  }

  if ((lastGroundCom == 0 or millis() - lastGroundCom > 30000) and millis() - lastGroundPingRQST > 2000) {
    Serial1.write(1);
    lastGroundPingRQST = millis();
  }

  update_gyro();

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

  byte conditionsCat = 0;
  //Serial.print(gyroWorking); Serial.print("\t"); Serial.print(pitotWorking); Serial.print("\t"); Serial.print(millis() - lastStaticPortUpdate); Serial.print("\n"); 
  if (gyroWorking and pitotWorking and ((millis() - lastStaticPortUpdate < 1000 and pressureCalibrated) or (gps.location.isValid() and gps.satellites.value() >= 5 and groundAltitudeKnown))) {
    conditionsCat = 1;
    if (compassWorking) {
      conditionsCat = 2;
      if (gps.location.isValid() and gps.satellites.value() >= 5) {
        conditionsCat = 3;
      }
    }
  }
  
  if (conditionsCat > MAX_AUTO_CAT) {
    conditionsCat = MAX_AUTO_CAT;
  }















  update_GPS();
  update_gyro();

  bool gotoTargetPitch = true;
  bool gotoTargetRoll = true;
  bool gotoTargetRollRate = true;
  bool gotoTargetPitchRate = true;
  bool gotoTargetAlt = false;
  bool gotoTargetHeading = false;
  bool thrustToTargetSpeed = false;
  bool reducedRoll = false;
  double currentLongitude = gps.location.lng();
  double currentLattitude = gps.location.lat();
  currentPitch = gyro.getAngleX() - 14;
  currentRoll = gyro.getAngleY();
  currentPitchRate = (currentPitchRate + gyro.getGyroX()) / 2;
  currentRollRate = (currentRollRate + gyro.getGyroY()) / 2;
  int targetPitch = 0;
  float targetPitchRate = 0;
  float targetRollRate = 0;
  sideslip = sideslip / 2 + gyro.getAccX() * 50.0;
  int yaw = 0;
  currentHeading = get_heading();
  int height = groundProximity();
  update_pressure();


  if (gyroWorking and modeH != "MAN FULL" and modeH != "GROUND") {
    if (sideslip > 0) {autoYaw += 1;}
    else {autoYaw -= 1;}
    yaw = autoYaw;
  }

  if (height < 120) {reducedRoll = true;}

  update_gyro();

  if (gps.location.isValid() and gps.satellites.value() >= 5 and modeV == "GROUND") {
    groundAltitude = gps.altitude.meters();
    if (!groundAltitudeKnown){
      groundAltitudeKnown = true;
      braudcast(F("ground GPS altitude set"), true);
    }
  }

  if (gps.location.isValid() and gps.satellites.value() >= 5 and groundAltitudeKnown) {
    currentAltitude = gps.altitude.meters() - groundAltitude;
  }
  else if (millis() - lastStaticPortUpdate < 1000){
    currentAltitude = groundTemperature * (pow((float)staticPressure/groundPressure, (float)-9.80665/287*0.0065) - 1) / -0.0065;
  }

  if (pitotWorking){
    //currentAirspeed = pow(2*dynamicPressure / 1.225, 0.5);
    currentAirspeed = dynamicPressure; // using simplified model
  }
  

  update_gyro();

  if (modeV == "GROUND") {
    dataBraudcast += "   Conditions: " + String(conditionsCat) + "   GPS: " + String(gps.satellites.value()) + "   height: " + String(height) + "   QFE: " + groundPressure + "   static press: " + String(staticPressure) + "   alt: " + String(currentAltitude);
    gotoTargetPitch = false;
    gotoTargetRoll = false;
    gotoTargetPitchRate = false;
    gotoTargetRollRate = false;
    elevatorAngle = 90;
    aileronAngle = 90;
    thrust = 0;
  }

  if (modeV.startsWith("MAN")) {
    dataBraudcast += "   speed: " + String(currentAirspeed) + "   sideslip: " + String(sideslip) + "   HDG: " + String(currentHeading);
    thrust = readChannel(thrustChannel, 0, MAX_THRUST, 0);

    if (modeV == "MAN SIMP") {
      gotoTargetPitch = true;
      gotoTargetRoll = true;
      targetPitch = readChannel(pitchChannel, -MAX_PITCH, MAX_PITCH, 0);
      targetRoll = readChannel(rollChannel, -MAX_ROLL, MAX_ROLL, 0);
    }
    else if (modeV == "MAN RATE") {
      gotoTargetPitch = false;
      gotoTargetRoll = false;
      gotoTargetPitchRate = true;
      gotoTargetRollRate = true;
      targetPitchRate = readChannel(pitchChannel, -MAX_PITCH_RATE, MAX_PITCH_RATE, 0);
      targetRollRate = readChannel(rollChannel, -MAX_ROLL_RATE, MAX_ROLL_RATE, 0);

      dataBraudcast += "   target pitch r: " + String(targetPitchRate) + "   pitch rate: " + String(currentPitchRate);
    }
    else{
      elevatorAngle = readChannel(pitchChannel, ELEVATOR_MIN, ELEVATOR_MAX, 90);
      aileronAngle = readChannel(rollChannel, AILERON_MIN, AILERON_MAX, 90);
      gotoTargetPitch = false;
      gotoTargetRoll = false;
      gotoTargetPitchRate = false;
      gotoTargetRollRate = false;
      if (modeV == "MAN FULL") {yaw = readChannel(yawChannel, -MAX_YAW, MAX_YAW, 0);}
    }

    if (conditionsCat < 1 and modeV != "MAN FULL") {set_mode("MAN FULL");}
  }

  //else {
    int yawcont = readChannel(yawChannel, -10, 10, 0);
    if (yawcont <= -9 and modeV != "MAN FULL") {set_mode("MAN FULL");}
    if (yawcont >= 9 and modeV != "MAN PROT" and modeV != "MAN FULL")  {set_mode("MAN PROT");}
  //}


  if (modeV == "HOLD") {
    dataBraudcast += "   target ALT: " + String(targetAltitude) + "   ALT: " + String(currentAltitude);
    gotoTargetAlt = true;
    int input = readChannel(pitchChannel, -30, 30, 0);
    if (input > 10) {
      targetAltitude -= 1;
      braudcast("target ALT: " + String(targetAltitude), false, true);
    }
    else if (input < -10) {
      targetAltitude += 1;
      braudcast("target ALT: " + String(targetAltitude), false, true);
    }
  }

  if (modeH == "HOLD") {
    dataBraudcast += "   target HDG: " + String(targetHeading) + "   HDG: " + String(currentHeading);
    gotoTargetHeading = true;

    int input = readChannel(rollChannel, -30, 30, 0);
    if (input > 10) {
      targetHeading += 1;
      braudcast("target HDG: " + String(targetHeading), false, true);
    }
    else if (input < -10) {
      targetHeading -= 1;
      braudcast("target HDG: " + String(targetHeading), false, true);
    }

    if (targetHeading >= 360) {targetHeading -= 360;}
    if (targetHeading < 0) {targetHeading += 360;}
    if (conditionsCat < 2) {modeH = "LVL";} // no set_mode() function, because vertical mode should be left unaffected
  }

  if (modeH == "LVL") {
    gotoTargetRoll = true;
    targetRoll = 0;
    if (conditionsCat < 1) {set_mode("MAN FULL");}
    if (conditionsCat > 1) {
      modeH = "HOLD"; // no set_mode() function, because vertical mode should be left unaffected
      targetHeading = currentHeading;
    }
  }

  if (modeV == "CLIMB") {
    dataBraudcast += "   speed: " + String(currentAirspeed) + "   target ALT: " + String(targetAltitude) + "   ALT: " + String(currentAltitude);
    reducedRoll = true;
    gotoTargetAlt = false;
    if (currentPitch < 3) {
      targetPitch = 3;
    }
    else {
      gotoTargetPitch = false;
      gotoTargetPitchRate = true;
      targetPitchRate = rescale(currentAirspeed - STALL_SPEED*1.5, -10, 10, -MAX_PITCH_RATE, MAX_PITCH_RATE);
    }
    thrust = MAX_THRUST;
    if (conditionsCat < 1) {set_mode("MAN FULL");}
  }

  if (modeH == "WAYPT") {
    dataBraudcast += "target long: " + String(targetLongitude) + "   target latt: " + String(targetLattitude) + "   target HDG: " + String(targetHeading) + "   HDG: " + String(currentHeading);

    gotoTargetHeading = true;
    float d_latt = targetLattitude - currentLattitude;
    float d_long = targetLongitude - currentLongitude;
    targetHeading = tan(d_long/d_latt) * 180/PI;
    if (d_long > 0 and d_latt < 0) {targetHeading += 90;}
    if (d_long < 0 and d_latt < 0) {targetHeading -= 180;}
    if (d_long < 0 and d_latt > 0) {targetHeading -= 90;}
  }

  if (modeV == "ILS") {
    thrustToTargetSpeed = true;
    if (height > 300){targetSpeed = STALL_SPEED * 1.5;}
    else{targetSpeed = STALL_SPEED*1.1;}

    if (capturedILS[0] and capturedILS[1]) {targetPitch = APPROACH_ANGLE;}
    else if (capturedILS[0]) {targetPitch = APPROACH_ANGLE - 5;}
    else if (capturedILS[1]) {targetPitch = APPROACH_ANGLE + 5;}
    else {
      braudcast(F("Go-around initiated, reason: lost glideslope"), true);
      set_mode("CLIMB");
    }

    if (capturedILS[4]) {
      if (height > 150) {
        braudcast(F("Go-around initiated, reason: too high at threshold"), true);
        set_mode("CLIMB");
      }
      else {
        set_mode("FLARE");
      }
    }

    if (height < 350 and modeH != "ILS") {
      braudcast(F("Go-Around, reason: localiser not intercepted"), true);
      set_mode("CLIMB");
    }

    if (height < 100) {
      braudcast(F("Go-Around, reason: threshold beacon not found"), true);
      set_mode("CLIMB");
    }

    if (conditionsCat < 2) {set_mode("MAN PROT");}
  }

  if (modeH == "ILS") {
    gotoTargetHeading = true;

    if (capturedILS[2] and capturedILS[3]) {targetHeading = runwayHeading;}
    else if (capturedILS[2]) {targetHeading = runwayHeading - 5;}
    else if (capturedILS[3]) {targetHeading = runwayHeading + 5;}
    else {
      braudcast(F("Go-around, reason: lost localiser"), true);
      set_mode("CLIMB");
    }

    if (height < 350 and modeV != "ILS") {
      braudcast(F("Go-Around, reason: glideslope not intercepted"), true);
      set_mode("CLIMB");
    }

    if (conditionsCat < 2) {set_mode("MAN PROT");}
  }

  if (modeV == "FLARE") {
    reducedRoll = true;
    thrust = 0;
    targetPitch = 5; // to be adjusted after test flights
    if (millis() - flareStartTime > 5) {
      braudcast(F("Go-around initiated, reason: 5 second rule"), true);
      set_mode("CLIMB");
    }
    if (height <= GROUND_HEIGHT) {
      set_mode("GROUND");
    }

    if (conditionsCat < 2) {set_mode("MAN FULL");}
    if (readChannel(thrustChannel, 0, 100, 0) > 90) {set_mode("CLIMB");}
  }



























  update_gyro();


  if (gotoTargetAlt) {
    targetPitch = rescale(targetAltitude - currentAltitude, -10, 10, -MAX_PITCH, MAX_PITCH);
    gotoTargetPitch = true;
  }
  if (gotoTargetHeading) {
    gotoTargetRoll = true;
    int diff = targetHeading - currentHeading;
    if (diff < -180) {diff += 360;}
    else if (diff > 180) {diff -= 360;}
    targetRoll = rescale(diff, -5, 5, -MAX_ROLL, MAX_ROLL);
  }

  update_gyro();

  // emergency protections
  if (modeV != "MAN FULL" and modeV != "GROUND") {
    if (currentPitch > MAX_PITCH * 1.1) {
      targetPitch = 0.9 * MAX_PITCH;
      gotoTargetPitch = true;
    }
    if (currentPitch < -MAX_PITCH * 1.1) {
      targetPitch = -0.9 * MAX_PITCH;
      gotoTargetPitch = true;
    }


    if (currentAirspeed < STALL_SPEED) {
      reducedRoll = true;
      thrust = MAX_THRUST;
      thrustToTargetSpeed = false;
      targetPitch = -MAX_PITCH;
      gotoTargetPitch = true;
    }
    else if (currentAirspeed < STALL_SPEED * 1.4 and modeV != "FLARE" and modeV != "ILS") {
      reducedRoll = true;
      targetSpeed = STALL_SPEED * 1.4;
      thrustToTargetSpeed = true;
    }

    else if (currentAirspeed > MAX_SPEED) {
      thrust = 0;
      thrustToTargetSpeed = false;
    }

    //if (modeV != "FLARE" and modeV != "ILS" and modeV != "CLIMB" and ( (height < 390 and modeV.startsWith("MAN") == false) or height < 150 )) {
    //  braudcast(F("Terrain escape"), true);
    //  set_mode("CLIMB");
    //}

    if (reducedRoll and currentRoll > MAX_ROLL_REDUCED * 1.1) {
      targetRoll = 0.9 * MAX_ROLL_REDUCED;
      gotoTargetRoll = true;
    }
    else if (reducedRoll and currentRoll < -MAX_ROLL_REDUCED * 1.1) {
      targetRoll = -0.9 * MAX_ROLL_REDUCED;
      gotoTargetRoll = true;
    }
    else if (currentRoll > MAX_ROLL * 1.1) {
      targetRoll = 0.9 * MAX_ROLL;
      gotoTargetRoll = true;
    }
    else if (currentRoll < -MAX_ROLL * 1.1) {
      targetRoll = -0.9 * MAX_ROLL;
      gotoTargetRoll = true;
    }
  }

  update_gyro();

  
  if (gotoTargetPitch) {
    gotoTargetPitchRate = true;
    if (targetPitch > MAX_PITCH) {targetPitch = MAX_PITCH;}
    if (targetPitch < -MAX_PITCH) {targetPitch = -MAX_PITCH;}
    targetPitchRate = rescale(targetPitch - currentPitch, -10, 10, -MAX_PITCH_RATE, MAX_PITCH_RATE);
  }

  if (gotoTargetRoll) {
    gotoTargetRollRate = true;
    if (targetRoll > MAX_ROLL) {targetRoll = MAX_ROLL;}
    if (targetRoll < -MAX_ROLL) {targetRoll = -MAX_ROLL;}
    targetRollRate = rescale(targetRoll - currentRoll, -10, 10, -MAX_ROLL_RATE, MAX_ROLL_RATE);
  }


  update_gyro();



  if (gotoTargetPitchRate) {
    if (targetPitchRate > MAX_PITCH_RATE) {targetPitchRate = MAX_PITCH_RATE;}
    if (targetPitchRate < -MAX_PITCH_RATE) {targetPitchRate = -MAX_PITCH_RATE;}
    elevatorAngle += rescale((targetPitchRate - currentPitchRate) *10, -MAX_PITCH_RATE*10, MAX_PITCH_RATE*10, -ELEVATOR_MAX/10, ELEVATOR_MAX/10);
    //if (targetPitchRate > currentPitchRate) {elevatorAngle += 1;}
    //else {elevatorAngle -= 1;}
  }

  if (gotoTargetRollRate) {
    if (targetRollRate > MAX_ROLL_RATE) {targetRollRate = MAX_ROLL_RATE;}
    if (targetRollRate < -MAX_ROLL_RATE) {targetRollRate = -MAX_ROLL_RATE;}
    aileronAngle += rescale((targetRollRate - currentRollRate) *10, -MAX_ROLL_RATE*10, MAX_ROLL_RATE*10, -AILERON_MAX/10, AILERON_MAX/10);
  }




  if (thrustToTargetSpeed) {
    if (targetSpeed > currentAirspeed and thrust < MAX_THRUST) {thrust += 1;}
    else if (thrust > 0) {thrust -= 1;}
  }

  update_gyro();


  if (elevatorAngle < ELEVATOR_MIN) {elevatorAngle = ELEVATOR_MIN;}
  if (elevatorAngle > ELEVATOR_MAX) {elevatorAngle = ELEVATOR_MAX;}

  if (aileronAngle < AILERON_MIN) {aileronAngle = AILERON_MIN;}
  if (aileronAngle > AILERON_MAX) {aileronAngle = AILERON_MAX;}

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

  update_gyro();

  elevators.write(elevatorAngle);
  ailerons.write(aileronAngle);
  if (!inESPReset) {
    leftMotor.write(thrustL);
    rightMotor.write(thrustR);
    lastThrustL = thrustL;
    lastThrustR = thrustR;
  }

  update_gyro();

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
    dataFile.print(sideslip); dataFile.print("\t");
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
    dataFile.print(braudcastedData + "\n");
    braudcastedData = "";
    if (modeV == "GROUND") {
      dataFile.close();
    }
  }

  update_gyro();
  braudcast(dataBraudcast, false, false);
}