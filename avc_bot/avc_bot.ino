#define MotorDirection 8
#define MotorSpeed 9

void accelerate(bool dir);

void setup() {
  //Declaration for the pins used, both should be outputs.
  pinMode(MotorDirection, OUTPUT);
  pinMode(MotorSpeed, OUTPUT);
}

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
