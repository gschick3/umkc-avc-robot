// this is a test
#define MotorDirection 8
#define MotorSpeed 9

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "Move.h"
#include <LiquidCrystal.h>

/*
   CONDITIONS UNDER WHICH THIS WORKED:
   -TinyGPS++ Library V1.0.2
   -In my room, near a window
   -Weather: Clear skies 
   -2/5/22 5:00PM
   -Both Modules
   -Wait for like 5-10 min for GPS fix
   -The code says RXPin = 4, TXPin = 3, but I had
    the actual pins flipped, but the code unchanged
*/
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;
const int  pwm1 = 11; // Power to right motor
const int  pwm2 = 9; // Power to left motor
const int  dir1 = 12; // Direction of right motor (low is forward)
const int  dir2 = 10; // Direction of left motor (low is forward)

const int rs = A5, en = A4, d4 = A3, d5 = A2, d6 = A1, d7 = A0;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


void displayInfo();
void getWaypoints(int);

Move move(pwm1, pwm2, dir1, dir2);

TinyGPSPlus gps;// The TinyGPS++ object
SoftwareSerial ss(RXPin, TXPin); // The serial connection to the GPS device

double originLat, originLong;
double wayLat[16] = {39.040138, 39.040088}; // hard coded
double wayLong[16] = {-94.572181, -94.572212}; // hard coded
int numWays = 1; // total number of waypoints minus 1
int wayCount; // which waypoint you're on
volatile byte state = LOW;
volatile byte prevState = LOW;
unsigned long lockTime;
const int interruptPin = 2;

int delayTime = 200; // general delay time in milliseconds

void setup() {
  wayCount = 0;
  
  //Set up GPS
  Serial.begin(115200);
  ss.begin(GPSBaud);


  //Display
  lcd.begin(16, 2);
  lcd.print("Locking on...");

  // Motor Power (0-255)
  move.power(40);
  

  // Set interrupt pin
  // pinMode(interruptPin, INPUT);
  // attachInterrupt(digitalPinToInterrupt(interruptPin), wayPointInterrupt, CHANGE); 


  // Activate GPS and wait for Lock
  do{
    while (ss.available() > 0){
      if (gps.encode(ss.read())) {
        originLat = gps.location.lat();
        originLong = gps.location.lng();
        displayInfo();
      }
    }
    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
      Serial.println(F("No GPS detected: check wiring."));
      while(true);
    }
  } while(gps.location.isValid() == false);
  lockTime = millis();
  Serial.println("-------------LOCK ACHIEVED-------------");
  Serial.print("Approximate lock time: ");
  Serial.print(lockTime/1000);
  Serial.println();
  delay(delayTime);
}

void loop()
{
  while (ss.available() > 0){
    if (gps.encode(ss.read())){
      if (originLat == gps.location.lat() && originLong == gps.location.lng()) {
        move.forward();
        delay(delayTime * 4);
        continue;
      }
      unsigned long distanceToWaypoint = getDistanceFromTarget(wayLat[wayCount], wayLong[wayCount]);
      if (distanceToWaypoint < 2) {
        if (wayCount != numWays) {
          wayCount++;
          distanceToWaypoint = getDistanceFromTarget(wayLat[wayCount], wayLong[wayCount]);
        }
        else {
          move.forward();
          delay(2000);
          move.halt();
          delay(10000);
        }
        continue;
      }
      turnTowardsWaypoint(wayLat[wayCount], wayLong[wayCount]);
      if (distanceToWaypoint < 3 && wayCount != numWays) {
        move.power(130);
      }
      else {
        move.power(255); 
      }
      displayInfo();
      displayLCD();
      move.forward();
      delay(delayTime);
    }
  }
}

void displayLCD(){
  lcd.clear();
  //lcd.print(gps.location.lat(), 6);
  lcd.print(gps.course.deg(), 6);
  lcd.setCursor(0,1);
  //lcd.print(gps.location.lng(), 6);
  lcd.print(TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      wayLat[wayCount], 
      wayLong[wayCount]));
  lcd.setCursor(10, 0);
  lcd.print("(^-^)/");
  lcd.setCursor(12,2);
  lcd.print("W");
  lcd.print(wayCount);
}

void turnTowardsWaypoint(double targetLat, double targetLong){
  // (Current lat, current long, target lat, target long)
  double courseToTarget =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      targetLat, 
      targetLong);

  double dir = abs(courseToTarget - gps.course.deg());

  if (dir > 180) dir -= 360; 
  
  if ((dir < 10 && dir > 0) || (dir > -10 && dir < 0)) // don't do anything if the change would be less than 10 degrees
    return;
  /*
  else if (courseToTarget < 20 && courseToTarget > 0) {
    move.set(move.getPower(), move.getPower()/4*3, LOW, LOW);
    delay(delayTime);
  }
  else if (courseToTarget > -20 && courseToTarget < 0) {
    move.set(move.getPower()/4*3, move.getPower(), LOW, LOW);
    delay(delayTime);
  }
  */
  else {
    move.halt();
    delay(delayTime);
    move.turn(dir);
  }
}

unsigned long getDistanceFromTarget(double targetLat, double targetLong) {
  // (current lat, current long, target lat, target long)
  unsigned long distanceToTarget = // distance in meters
      (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      targetLat, 
      targetLong);
  return distanceToTarget;
  /*
  Serial.print("Distance to Target: "); 
  Serial.print(distanceToTarget, 9);
  Serial.print(", Course: ");
  Serial.print(courseToTarget, 6);
  Serial.print(", hdop: ");
  Serial.print(gps.hdop.hdop());
  Serial.print(", age: ");
  Serial.print(gps.location.age());
  Serial.println();
  */
}

void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Time: "));
  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}
