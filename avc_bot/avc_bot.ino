#define MotorDirection 8
#define MotorSpeed 9
#define M_PI 3.141592653589793238

#include <TinyGPS++.h>
#include <SoftwareSerial.h>

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

void displayInfo();
double getDistance(double lat1, double lon1, double lat2, double lon2);

// The TinyGPS++ object
TinyGPSPlus gps;
// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

int wayCount = 0;
int wayLat[5];
int wayLong[5];

void setup() {
  //Set up GPS
  Serial.begin(115200);
  ss.begin(GPSBaud);
  
  //Declaration for the pins used, both should be outputs.
  pinMode(MotorDirection, OUTPUT);
  pinMode(MotorSpeed, OUTPUT);

  // Test coordinates
  wayLat[wayCount] = 39.04;
  wayLong[wayCount] = -94.573;
  wayCount++;

  wayLat[wayCount] = 39.040151;
  wayLong[wayCount] = -94.572768;
  wayCount++;
}

void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
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

// get distance between two lat/lon pairs in feet
double getDistance(double lat1, double lon1, double lat2, double lon2) {
  lat1 *= (M_PI / 180);
  lat2 *= (M_PI / 180);
  lon1 *= (M_PI / 180);
  lon2 *= (M_PI / 180);

  double distFeet = 20924640 * acos((sin(lat1) * sin(lat2)) + cos(lat1) * cos(lat2) * cos(lon2 - lon1));
  return distFeet;
}

// get distance between two lat/lon pairs assuming recangular coordinate system
double getDistance2(double lat1, double lon1, double lat2, double lon2) {
  return sqrt(pow(lat2-lat1, 2)+pow(lon2-lon1, 2));
}

double getAngle(double lat1, double lon1, double lat2, double lon2) {
  
}
/*
void accelerate(bool dir);

void loop() {
  
  //Ramps up the speed in the clockwise direction.
  accelerate(LOW);
  analogWrite(MotorSpeed,0);
  
  //Ramps up the speed in the counter clockwise direction.  
  accelerate(HIGH);
  analogWrite(MotorSpeed,0);
  
}

void accelerate(bool dir){
  digitalWrite(MotorDirection, dir);                  //Loop increases the speed slowly until it reaches max speed.
  for(int SpeedVal = 0; SpeedVal < 255; SpeedVal++){
      analogWrite(MotorSpeed,SpeedVal);
      delay(40);
  }
}
*/
