#include "Arduino.h"
#include "Move.h"

Move::Move(int pwm1, int pwm2, int dir1, int dir2)
{
	powerValue = 255;
	this->pwm1 = pwm1; // Power to right motor
	this->pwm2 = pwm2; // Power to left motor
	this->dir1 = dir1; // Direction of right motor (low is forward)
	this->dir2 = dir2; // Direction of left motor (low is forward)
}

Move* Move::power(int setPower)
{
	powerValue = setPower;
	return this;
}

int Move::getPower(){
	return powerValue;
}

void Move::set(int right, int left, bool dirR, bool dirL)
{
	analogWrite(pwm1, right);
	analogWrite(pwm2, left);
	digitalWrite(dir1, dirR);   
	digitalWrite(dir2, dirL);
}

void Move::forward()
{
	analogWrite(pwm1, powerValue);
	analogWrite(pwm2, powerValue);
	digitalWrite(dir1, LOW);   
	digitalWrite(dir2, LOW);
}
void Move::backward()
{
	analogWrite(pwm1, powerValue);
	analogWrite(pwm2, powerValue);
	digitalWrite(dir1, HIGH);
	digitalWrite(dir2, HIGH);

}
void Move::left()
{
	analogWrite(pwm1, powerValue);
	analogWrite(pwm2, powerValue);
	digitalWrite(dir1, LOW);
	digitalWrite(dir2, HIGH);
}
void Move::right()
{
	analogWrite(pwm1, powerValue);
	analogWrite(pwm2, powerValue);
	digitalWrite(dir1, HIGH);
	digitalWrite(dir2, LOW);

}
void Move::halt()
{
	analogWrite(pwm1, 0);
	analogWrite(pwm2, 0);
	digitalWrite(dir1, LOW);   
	digitalWrite(dir2, LOW);
}

void Move::turn(int degree){
	int orginalPower = getPower();
	int timeDelay = 100; // how much it takes to turn 15 degrees
	power(255); 

	if (degree < 0){
		left();
		degree *= -1;
	}
	else{ 
		right();
	}
	
	delay((degree/15) * timeDelay); // degrees are given as multiples of 15

	power(orginalPower); // sets power to before it started to turn.
}

	
