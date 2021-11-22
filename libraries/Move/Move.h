#ifndef Move_h
#define Move_h

#include <Arduino.h>

class Move
{
	public:
		Move(int, int, int, int);
		Move* power(int setPower);
		void set(int right, int left, bool dirR, bool dirL);
		void forward();
		void backward();
		void left();
		void right();
		void halt();
		void turn(int degree);
		int getPower();

	private:
		int powerValue;
		int pwm1;
		int pwm2;
		int dir1;
		int dir2;
};


#endif
