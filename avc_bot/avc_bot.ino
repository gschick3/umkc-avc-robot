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

Move move(pwm1, pwm2, dir1, dir2);

TinyGPSPlus gps; // The TinyGPS++ object
SoftwareSerial ss(RXPin, TXPin); // The serial connection to the GPS device

double originLat, originLong;

double wayLat[16] = {39.040149}; // hard coded
double wayLong[16] = {-94.572029}; // hard coded
int numWays = 0; // total number of waypoints minus 1
int wayCount; // which waypoint you're on

unsigned long lockTime;

int delayTime = 200; // general delay time in milliseconds

int degree; // degrees away from next waypoint

void setup() {
  wayCount = 0;
  degree = 0;
  
  Serial.begin(115200);
  
  //Set up GPS
  ss.begin(GPSBaud);

  // Motor Power (0-255)
  move.power(255);

  // test movement
  move.forward();
  delay(1000);
//  move.turn(90);
  move.backward();
  delay(1000);
//  move.turn(180);
  move.halt();
  delay(2000);
  move.turn(90);
  move.turn(-90);
  move.halt();
  delay(5000);

  
  // Activate GPS and wait for Lock
  do {
    //Display
    lcd.begin(16, 2);
    lcd.print("Locking on...");
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
  smartDelay(delayTime);
}

void loop()
{
  while (ss.available() > 0)
  {
    if (gps.encode(ss.read()))
    {
      if(checkErrors(5, 200)) {
        move.halt();
        continue;
      }
      
      displayInfo();
      displayLCD(degree);
      
      if (originLat == gps.location.lat() && originLong == gps.location.lng()) {
        move.forward();
        smartDelay(delayTime * 20);
        continue;
      }
      
      unsigned long distanceToWaypoint = getDistanceFromTarget(wayLat[wayCount], wayLong[wayCount]);
      if (distanceToWaypoint < 2) {
        if (wayCount != numWays) {
          wayCount++;
          distanceToWaypoint = getDistanceFromTarget(wayLat[wayCount], wayLong[wayCount]);
        }
        else {
          //move.forward();
          //smartDelay(2000);
          move.halt();
          smartDelay(10000);
        }
        continue;
      }
      
      degree = turnTowardsWaypoint(wayLat[wayCount], wayLong[wayCount]);
      
      if (distanceToWaypoint < 3 && wayCount != numWays) {
        move.power(130);
      }
      else {
        move.power(255); 
      }
      move.forward();
      smartDelay(delayTime * 5);
    }
  }
}

/************************************************************************************
 * ERROR CHECKING
 ***********************************************************************************/
bool checkErrors(int hdop, int age) {
  hdop *= 100;
  if(gps.hdop.value() > hdop || !gps.location.isValid() || !gps.course.isValid() || gps.location.age() > age) {
    printError(hdop, age);
    return true;
  }
  return false;
}

void printError(int hdop, int age) {
  lcd.clear();
  lcd.print("ERROR");
  lcd.setCursor(1, 0);
  if (!gps.location.isValid()) {
    lcd.print("Location invalid.");
    return;
  }
  else if (!gps.course.isValid()) {
    lcd.print("Course invalid.");
    return;
  }
  else if(gps.hdop.value() > hdop) {
    lcd.print("hdop:");
    lcd.print(gps.hdop.value() / 100.0);
  }
  else if(gps.location.age() > age) {
    lcd.print("age:");
    lcd.print(gps.location.age());
  }
}

/************************************************************************************
 * LOCATION FINDING
 ***********************************************************************************/
int turnTowardsWaypoint(double targetLat, double targetLong){
  // (Current lat, current long, target lat, target long)
  double courseToTarget =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      targetLat, 
      targetLong);

  int dir = abs(courseToTarget - gps.course.deg());

  if (dir > 180) dir -= 360; 
  
  if ((dir < 10 && dir > 0) || (dir > -10 && dir < 0)) // don't do anything if the change would be less than 10 degrees
    return 0;
  /*
  else if (courseToTarget < 20 && courseToTarget > 0) {
    move.set(move.getPower(), move.getPower()/4*3, LOW, LOW);
    smartDelay(delayTime);
  }
  else if (courseToTarget > -20 && courseToTarget < 0) {
    move.set(move.getPower()/4*3, move.getPower(), LOW, LOW);
    smartDelay(delayTime);
  }
  */
  else {
    //move.halt();
    //smartDelay(delayTime);
    move.turn(dir);
  }
  return dir;
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
}

/************************************************************************************
 * INFORMATION DISPLAY
 ***********************************************************************************/
void displayLCD(int degree){
  lcd.clear();
  lcd.print(gps.location.lat(), 6);
  //lcd.print(gps.course.deg(), 6);
  lcd.setCursor(12, 0);
  lcd.print(degree);
  lcd.setCursor(0,1);
  lcd.print(gps.location.lng(), 6);
  /*
  lcd.print(TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      wayLat[wayCount], 
      wayLong[wayCount]));
  lcd.setCursor(10, 0);
  lcd.print("(^-^)/");
  */
  lcd.setCursor(12,2);
  lcd.print("W");
  lcd.print(wayCount);
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

/************************************************************************************
 * DELAY
 ***********************************************************************************/
void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

/************************************************************************************
 * MOVE CLASS MEMBER FUNCTIONS
 ***********************************************************************************/
void Move::turn(int degree){
  int orginalPower = getPower();
  int timeDelay = 375; // how much it takes to turn 15 degrees
  power(150);

  if (degree < 0){
    left();
    degree *= -1;
  }
  else{ 
    right();
  }
  
  smartDelay((degree/15) * timeDelay); // degrees are given as multiples of 15

  power(orginalPower); // sets power to before it started to turn.
}
