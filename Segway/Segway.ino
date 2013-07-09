#include "MPU6050.h"
#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>
#include "PerformanceTimer.h"
#include "SerialExtensions.h"

// Angle
MPU6050 imu;
float angle = 0;
unsigned long oldMillis;

// Pins
// Motor 1
//int m1EnablePin = 7;
//int m1EnablePin2 = 8;
int m1InA = 3;//6;
//int m1InB = 3;//5;

// Motor 2
//int m2EnablePin = 12;
//int m2EnablePin2 = 13;
int m2InA = 11;
//int m2InB = 11;

// Steering
int steeringPin = A0;

// Logging
PerformanceTimer angleTimer;
PerformanceTimer sensorReadTimer;
PerformanceTimer regulateMotorTimer;
PerformanceTimer printDebugInfoTimer;
unsigned long lastPrintTime = 0;
unsigned int sensorReadTime = 0;
unsigned int angleTime = 0;
unsigned int regulateMotorTime = 0;
unsigned int printDebugInfoTime = 0;
unsigned int stallCounter = 0;
unsigned int totalTime = 0;

// Motor regulation
float kp = 6;
float ki = 0;
float kd = 0.3;
float errorSum = 0;
float lastError = 0;
unsigned long lastRegulationTime = 0;
bool motorsDisabled;
int lastMotorSpeed = 0;
bool acceptingInput = false;

// Steering
int minTurn = -256;
int maxTurn = 256;
int maxSteer = 20;

// Misc
char printBuffer[512];

void setup()
{
	Serial.begin(115200);
	
	if (imu.setup())
	{
		Serial.println("Successfully set up MPU-6050");
		imu.calibrate();
		oldMillis = millis();
	}
	else
		Serial.println("MPU-6050 setup failed!");
		
	//TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00);  // Set phase correct PWM on timer 0 (runs pin 5 and 6)
	//TCCR0B = TCCR0B & 0b11111000 | 0x2; // Set PWM freq to 3906.25 Hz on pin 5 and 6
	// pin 9, 10 and 11, 3 run on timer 1 and timer 2 respectively, which are set to fast PWM by default
	
	//TCCR1A = _BV(WGM10);  
	//TCCR1B = (TCCR1B & 0b11111000) | 0x2; // Set timer 1 (pin 9 and 10) to run at 4k Hz
	//TCCR2A = _BV(WGM20);
	TCCR2B = (TCCR2B & 0b11111000) | 0x1; // Timer 2 - 31250 Hz
	
	pinMode(m1InA, OUTPUT);
	//pinMode(m1InB, OUTPUT);
	//pinMode(m1EnablePin, OUTPUT);
	//pinMode(m1EnablePin2, OUTPUT);
	
	//digitalWrite(m1EnablePin, LOW);
	//digitalWrite(m1EnablePin2, LOW);
	
	pinMode(m2InA, OUTPUT);
	//pinMode(m2InB, OUTPUT);
	//pinMode(m2EnablePin, OUTPUT);
	//pinMode(m2EnablePin2, OUTPUT);
	
	pinMode(steeringPin, INPUT);
	
	//digitalWrite(m2EnablePin, LOW);
	//digitalWrite(m2EnablePin2, LOW);
	motorsDisabled = true;
}

void forcePrintTimings()
{
	printBytesSerial<unsigned int>(sensorReadTime);
	printBytesSerial<unsigned int>(angleTime);
	printBytesSerial<unsigned int>(regulateMotorTime);
	printBytesSerial<unsigned int>(printDebugInfoTime);
	printBytesSerial<unsigned int>(totalTime);
	printBytesSerial<int>(static_cast<int>(totalTime > 10000));
}

void printDebugInfo(float accelAngle, float angularVelocity, bool *wrote)
{
	*wrote = false;
	unsigned long time = millis();
	if (time - lastPrintTime > 300)
	{
		Serial.print("Data");
		printBytesSerial<float>(accelAngle);
		printBytesSerial<float>(angularVelocity);
		printBytesSerial<float>(angle);
		printBytesSerial<int>(lastMotorSpeed);
		printBytesSerial<float>(kp);
		printBytesSerial<float>(ki);
		printBytesSerial<float>(kd);

		forcePrintTimings();
		
		lastPrintTime = time;
		*wrote = true;
	}
}

int clamp(int min, int max, int value)
{
	if (value < min)
		value = min;
	if (value > max)
		value = max;
		
	return value;
}

int minVal(int val1, int val2)
{
	return val1 < val2 ? val1 : val2;
}

int maxVal(int val1, int val2)
{
	return val1 > val2 ? val1 : val2;
}

void writeMotors(int pwm, int steering)
{	
	if (pwm == 0 || pwm > 0)
	{
		// Disable
		//digitalWrite(m1EnablePin, LOW);
		//digitalWrite(m1EnablePin2, LOW);
		//
		//digitalWrite(m2EnablePin, LOW);
		//digitalWrite(m2EnablePin2, LOW);
		analogWrite(m1InA, 0);
		analogWrite(m2InA, 0);
		motorsDisabled = true;
	}
	else
	{
		acceptingInput = true;
		int value = abs(pwm);

		// If we have steering, clamp steering so value + steering and value - steering is within 0 and 255
		int high = maxVal(value + steering, value - steering);
		int low = minVal(value + steering, value - steering);
		
		if (high > 255)
		{
			if (steering < 0)
			{
				// value - steering produced this value
				// value - steering = 255
				// steering = value - 255
				steering = value - 255;
			}
			else
			{
				// value + steering produced this value
				steering = 255 - value;
			}
		}
		else if (low < 0)
		{
			if (steering < 0)
			{
				// value + steering produced this value
				steering = 255 - value;
			}
			else
			{
				// value - steering produced this value
				steering = value - 255;
			}
		}
		
		if (pwm < 0)
		{
			// Are we switching direction?
			//if (lastMotorSpeed > 0)
			//{
			//	// Disable half bridges first
			//	digitalWrite(m1EnablePin, LOW);
			//	digitalWrite(m1EnablePin2, LOW);
				
			//	digitalWrite(m2EnablePin, LOW);
			//	digitalWrite(m2EnablePin2, LOW);
			//}
			
			analogWrite(m1InA, value + steering);
			//analogWrite(m1InB, 0);
			
			analogWrite(m2InA, clamp(0, 255, static_cast<int>((value - steering) * 1.00)));
			//analogWrite(m2InB, 0);
			
			// Enable half bridges if we changed direction or they were disabled
			//if (motorsDisabled || lastMotorSpeed > 0)
			//{
			//	digitalWrite(m1EnablePin, HIGH);
			//	digitalWrite(m1EnablePin2, HIGH);
			//	
			//	digitalWrite(m2EnablePin, HIGH);
			//	digitalWrite(m2EnablePin2, HIGH);
			//}
		}
		//else
		//{
		//	// pwm > 0
		//	// Are we switching direction?
		//	if (lastMotorSpeed < 0)
		//	{
		//		// Disable half bridges first
		//		digitalWrite(m1EnablePin, LOW);
		//		digitalWrite(m1EnablePin2, LOW);
		//		
		//		digitalWrite(m2EnablePin, LOW);
		//		digitalWrite(m2EnablePin2, LOW);
		//	}
		//	
		//	analogWrite(m1InA, 0);
		//	analogWrite(m1InB, value + steering);
		//	
		//	analogWrite(m2InA, 0);
		//	analogWrite(m2InB, value - steering);
		//	
		//	if (motorsDisabled || lastMotorSpeed < 0)
		//	{
		//		digitalWrite(m1EnablePin, HIGH);
		//		digitalWrite(m1EnablePin2, HIGH);
		//		
		//		digitalWrite(m2EnablePin, HIGH);
		//		digitalWrite(m2EnablePin2, HIGH);
		//	}
		//}
	}
	
	lastMotorSpeed = pwm;
}

float inverseLerp(float min, float max, float amount)
{
	return (amount - min) / (max - min);
}

float lerp(float min, float max, float value)
{
	return min + (max - min) * value;
}

int getSteering()
{
	int value = analogRead(steeringPin) - 512;
	value = clamp(minTurn, maxTurn, value);
	
	return static_cast<int>(lerp(-maxSteer, maxSteer, inverseLerp(minTurn, maxTurn, value)));
}

void regulateMotors()
{
	float error = 0 - angle;
	errorSum += error; // This only works as long as the time between each step is constant
	float dError = error - lastError; // Also only works when time between regulation is constant!
	lastError = error;
	
	// Output = Kp*error + Ki * int 0..t (error) dt + Kd * d/dt (error)
	float proportion = kp * error;
	float integral = ki * errorSum;  
	float derivative = kd * dError;
	
	int output = clamp(-255, 255, proportion + integral + derivative);
	

	// PID loop
	if (fabs(angle) < 0.8)
	{
		// High pass filter
		writeMotors(0, 0);
	}
	else if (fabs(angle) > 45)
	{
		// Low pass filter
		writeMotors(0, 0);
	}
	else
	{
		//writeMotors(output, getSteering());
		writeMotors(-output, 0);
	}
	
	return;
//	if (fabs(angle) < 2)
//	{
//		digitalWrite(enablePin, LOW);
//		digitalWrite(enablePin2, LOW);
//		analogWrite(inA, 0);
//		analogWrite(inB, 0);
//		
//		motorSpeed = 0;
//	}
//	else if (angle > 0)
//	{
//		digitalWrite(enablePin, LOW);
//		digitalWrite(enablePin2, LOW);
//		analogWrite(inA, clamp(0, 255, static_cast<int>(lerp(0, 255, angle / 90.0f))));
//		analogWrite(inB, 0);
//		digitalWrite(enablePin, HIGH);
//		digitalWrite(enablePin2, HIGH);
//		
//		motorSpeed = clamp(0, 100, static_cast<int>(lerp(0, 100, angle / 90.0f)));
//	}
//	else if (angle < 0)
//	{
//		digitalWrite(enablePin, LOW);
//		digitalWrite(enablePin2, LOW);
//		analogWrite(inA, 0);
//		analogWrite(inB, clamp(0, 255, static_cast<int>(lerp(0, 255, angle / -90.0f))));
//		digitalWrite(enablePin, HIGH);
//		digitalWrite(enablePin2, HIGH);
//		
//		motorSpeed = -clamp(0, 100, static_cast<int>(lerp(0, 100, angle / -90.0f)));
//	}
}

void readRegulationData()
{
	//if (!acceptingInput)
	//	return;
		
	if (Serial.available() >= 16)
	{
		if (Serial.read() == 'K' && Serial.read() == 'S' && Serial.read() == 'E'
				&& Serial.read() == 'G')
		{
			readBytesSerial<float>(&kp);
			readBytesSerial<float>(&ki);
			readBytesSerial<float>(&kd);
		}
	}
	else
	{
		for (int i = 0; i < Serial.available(); i++)
			Serial.read();
	}
}

void logic()
{
	sensorReadTimer.restart();
	float ax, ay, az;
	float gx, gy, gz;
	float temp;
	imu.read(ax, ay, az, gx, gy, gz, temp);
	sensorReadTime = sensorReadTimer.getElapsed();
	
	//az = -az;
	angleTimer.restart();
	unsigned long startTime = micros();
	float accelAngle = 90.0f - atan2(az, ay) * (180.0f / M_PI);
	
	if (accelAngle > 180.0f)
		accelAngle = accelAngle - 360.0f;
	
	float angularVelocity = gx;
	
	//angle = accelAngle;
	unsigned long time = millis();
	angle = 0.98f * (angle + angularVelocity * ((time - oldMillis) / 1000.0f)) + 0.02f * accelAngle;
	
	oldMillis = time;
	
	angleTime = angleTimer.getElapsed();
	
	regulateMotorTimer.restart();
	regulateMotors();
	regulateMotorTime = regulateMotorTimer.getElapsed();
	
	readRegulationData();
	
	printDebugInfoTimer.restart();
	bool wrote;
	printDebugInfo(accelAngle, angularVelocity, &wrote);
	if (wrote) // Only store the time if it actually printed something
		printDebugInfoTime = printDebugInfoTimer.getElapsed();
}

void loop()
{
	unsigned long time = micros();
	logic();
	unsigned long doneTime = micros();
	
	unsigned long elapsed;
	if (doneTime < time)
	{
		// This means that the time overflowed while we were logic'ing the shit out of our Segway
		// Basically what has elapsed is 0xFFFFFFFF - time + 1 + doneTime
		// These are just the ranges [time, 2^32] plus [0, doneTime]
		elapsed = 0xFFFFFFFFul - time + 1 + doneTime;
	}
	else
	{
		elapsed = doneTime - time;
	}
	
	totalTime = elapsed;

	if (elapsed > 10000)
	{
		return;
	}
		
	delayMicroseconds(10000 - elapsed);
}
