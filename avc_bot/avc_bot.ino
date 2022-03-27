// this is a test
#define MotorDirection 8
#define MotorSpeed 9

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "Move.h"

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

void displayInfo();
void getWaypoints(int);

Move move(pwm1, pwm2, dir1, dir2);

TinyGPSPlus gps;// The TinyGPS++ object
SoftwareSerial ss(RXPin, TXPin); // The serial connection to the GPS device

double wayLat[16];
double wayLong[16];
volatile byte pointCount = 0;
volatile byte state = LOW;
volatile byte prevState = LOW;
unsigned long lockTime;
const int interruptPin = 2;
const int countLED[] = {10, 11, 12, 13};

void setup() {
  //Set up GPS
  Serial.begin(115200);
  ss.begin(GPSBaud);
  move.power(40);
  // Set interrupt pin
  // pinMode(interruptPin, INPUT);
  // attachInterrupt(digitalPinToInterrupt(interruptPin), wayPointInterrupt, CHANGE); 

  // Set pins for LED output
  /*for(int i = 0; i < 4; i++){
    pinMode(countLED[i], OUTPUT);
  }*/

  // Activate GPS and wait for Lock
  do{
   while (ss.available() > 0){
      if (gps.encode(ss.read()))
        displayInfo();
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
  delay(200);
}

void loop()
{
  while (ss.available() > 0){
    if (gps.encode(ss.read())){
      
      displayInfo();
   /*   for (int j = 0; j < pointCount; j++){
        Serial.print("Lat: ");
        Serial.print(wayLat[j], 6);
        Serial.print(", Long: ");
        Serial.print(wayLong[j], 6);
        Serial.print(", PointCount: ");
        Serial.print(pointCount);
        Serial.println();
      }*/
     move.forward();
      delay(20000);
    }
   
    /* displayWayCount(pointCount);
     */
  }
}

void displayWayCount(byte num){
  for(int i = 0; i < 4; i++){
    if(bitRead(num, i) == 1){
      digitalWrite(countLED[i], HIGH);
    }
    else{
      digitalWrite(countLED[i], LOW);
    }
  }
}
/*
  // (Current lat, current long, target lat, target long)
  double courseToTarget =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      targetLat, 
      targetLong);

  if (courseToTarget > 180) courseToTarget = -1 * (courseToTarget - 180); 

  // (current lat, current long, target lat, target long)
  unsigned long distanceToTarget =
      (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      targetLat, 
      targetLong);
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

// Interrupt Service Routine
void wayPointInterrupt(){
  if(pointCount < 15){
    if (gps.location.isValid()){
      if (gps.encode(ss.read())){
        wayLat[pointCount] = gps.location.lat();
        wayLong[pointCount] = gps.location.lng();
        pointCount = pointCount + 1;
      }
    }
  }
}

/*void getWaypoints(int wayCount) {
  double waypointLats[wayCount];
  double waypointLongs[wayCount];

  for (int i = 0; i < wayCount; i++) {
    Serial.print("Getting waypoint ");
    Serial.println(i+1);
    int x = 0;
    while (x != 1) {
      if (gps.encode(ss.read())){     
        waypointLats[i] = gps.location.lat();
        waypointLongs[i] = gps.location.lng();
      }
      if (gps.location.isValid())
        x = Serial.parseInt();
    }
    displayInfo();
    //Serial.println(waypointLats[i], 6);
    //Serial.println(waypointLongs[i], 6);
  }
  */
