#include "Propulsion.h"
#include "ArmServo.h"
#include "MissionHelper.h"
#include "Sensors.h"
//#include "Enes100.h"

MissionHelper mission;
ArmServo armServo;
Propulsion propulsion(mission);
Sensors sensors;

double midpointY = 1.0; // 1 meter,we need to make sure this is right (enes100 library and stuff has lots of mistakes)
double sidePointMissionSite = 0.2; // outer edge of mission zone
double bottomObstacleField = 0.1; // theoretical "bottom" of obstacle field
double southTopEdge = 0.8; // dISTANCE BEFORE ROBOT ARM IS HOVERING OVER DATA POINT
double northBottomEdge = 1.2; // these are guesses :<
double obstacleExists = 10; // cm, theoretical value that an object exists if ultrasonic sensor value reads <= to this value
int timeToClearObstacleY = 1000; // ms, time until the robot has cleared an obstacle in the y dir
int timeToClearObstacleX = 1000; // ms, time until the robot has cleared an obstacle in the x dir
double limboMidPoint = 1.5; // Y coord of midpoint of limbo 
int timeToClearLimbo = 500; // time to clear limbo
double dutyCycle;
boolean magnetic;

void setup() {
  mission.start();
  mission.updateCurrLocation();

}

void loop() {
  mission.updateCurrLocation();
 
  doMission((mission.getY() > midpointY)); 
  //if the y position is greater than the midpoint, then the robot is on the north side and the mission is on the south side. else, vice versa.
  
  // nav code
  prepareNav();
  findHole();
  prepareNav();
  findHole();
  clearLimbo();

  // runs infinite while loop
  finish(); 
  // pog
}

void doMission(boolean robotIsNorth) { // true if north, false if south
  mission.updateCurrLocation();
  armServo.runArmUp();
  if (robotIsNorth) {
    propulsion.turnTo(-PI/2);
    mission.updateCurrLocation();
    while(mission.getY() > southTopEdge){     // POINT AT WHICH ARM IS HOVERING OVER MIDPOINT OF DATA EXTRACTION POINT
     propulsion.driveFwd();
     mission.updateCurrLocation();
    }
    propulsion.stopMotors();
    mission.updateCurrLocation();
  } else {
      propulsion.turnTo(PI/2);
      mission.updateCurrLocation();
      while(mission.getY() < northBottomEdge) { // POINT AT WHICH ARM IS HOVERING OVER MIDPOINT OF DATA EXTRACTION POINT
        propulsion.driveFwd();
        mission.updateCurrLocation();
      }
      propulsion.stopMotors();
      mission.updateCurrLocation();
  }

  // doing the mission stuff
  armServo.runArmDown();
  sensors.dutyCircuitReady();
  mission.sendDutyCycle(sensors.readDutyCycle());
  mission.sendMagnetic(sensors.useReed());
  armServo.runArmUp();

  // navigate to center of field
  if(robotIsNorth) { // robot WAS north, and is now south
    while(mission.getY() < midpointY) {
      propulsion.driveBackwd(); 
      mission.updateCurrLocation();
    }
  } else {          // robot WAS south, and is now north
    while(mission.getY() > midpointY) {
      propulsion.driveBackwd();
      mission.updateCurrLocation();
    }
  }
  propulsion.stopMotors();
  mission.updateCurrLocation();

  // navigate to side edge of mission site
  armServo.runArmDown();
  propulsion.turnTo(0);
  mission.updateCurrLocation();
  while (mission.getX() < sidePointMissionSite) {
    propulsion.driveFwd();
    mission.updateCurrLocation();
  }
  propulsion.stopMotors();
  mission.updateCurrLocation();
  
}

void prepareNav() {
  propulsion.turnTo(PI/2);
  mission.updateCurrLocation();
  while(mission.getY() > bottomObstacleField) {
    propulsion.driveBackwd();
    mission.updateCurrLocation();
  }
  propulsion.stopMotors();
  mission.updateCurrLocation();
}
void findHole() {
  while(sensors.getUltrasonic() < obstacleExists) {
    propulsion.driveFwd();
    mission.updateCurrLocation();
  }
  propulsion.stopMotors();
  mission.updateCurrLocation();

  propulsion.driveFwd();
  delay(timeToClearObstacleY);
  propulsion.stopMotors();
  mission.updateCurrLocation();

  propulsion.turnTo(0);
  mission.updateCurrLocation();
  propulsion.driveFwd();
  delay(timeToClearObstacleX);
  propulsion.stopMotors();
  mission.updateCurrLocation();
}

void clearLimbo() {
  propulsion.turnTo(PI/2);
  mission.updateCurrLocation();
  while(mission.getY() < limboMidPoint) {
    propulsion.driveFwd();
    mission.updateCurrLocation();
  }
  propulsion.stopMotors();

  propulsion.turnTo(0);
  mission.updateCurrLocation();
  propulsion.driveFwd();
  delay(timeToClearLimbo);
  propulsion.stopMotors();
  mission.updateCurrLocation();
}

void finish() {
  while(true){
    //nothing
  }
}

void testMagSensAndSend() {
  Serial.println("Starts in 5 sec");
  delay(5000);
  Serial.println("Start");
  bool mag = sensors.useReed();
  Serial.println(mag);
  mission.sendMagnetic(mag);
  Serial.println("Done");
  while(1);
}

void testNoVSTurn() {
  propulsion.turn90NoVS(1);
  delay(1000);
  propulsion.turn90NoVS(0);
  delay(1000);
  propulsion.turn90NoVS(1);
  delay(1000);
  while(1);
}

void testReed() {
  Serial.println("Starts in 5 sec");
  delay(5000);
  Serial.println("Start");
  Serial.println(sensors.useReed());
  while(1);
}

void testDuty() {
  Serial.println(sensors.readDutyCycle());
}

void testHall() {
  Serial.println(sensors.getHallEff());
}

void testWifiTX() {
  mission.sendToVS("Hi Vision System :)");
  String loc = String(mission.getX(), 3) + " " + String(mission.getY(), 3);
  mission.sendToVS("Our current XY location is: " + loc);
}

void testWifiRX() {
  String x = String(mission.getX(), 3);
  Serial.println("X loc: " + x);
  String y = String(mission.getY(), 3);
  Serial.println("Y loc: " + y);
  String t = String(mission.getTheta(), 3);
  Serial.println("T loc: " + t);
  delay(50);
}

void testProp() {
  propulsion.driveFwdS(255);
  delay(10000);
  //propulsion.driveBackwdS(255);
  //delay(5000);
  propulsion.stopMotors();
  delay(2000);
}

void testServo() {
  armServo.runServoCCW();
  delay(2000);
  armServo.stopServoRotation();
  armServo.runServoCW();
  delay(2000);
  armServo.stopServoRotation();
  delay(2000);
}

void testUS() {
  float x = sensors.getUltrasonic();
  Serial.println(x);
}
