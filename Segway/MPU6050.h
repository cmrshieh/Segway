#ifndef MPU6050_H
#define MPU6050_H

#include <Wire.h>
#include <Arduino.h>
#include "MPU6050_defines.h"

class MPU6050
{
private:
	float m_accelZeros[3];
	float m_gyroZeros[3];
	bool m_setup;
	
public:
	MPU6050() : m_setup(false)
	{
	}
	// Calibrates the sensor by doing 100 readings with 10 ms spacing and taking the average
	void calibrate();
	void calibrate(int accelZeros[3], int gyroZeros[3]);
	bool setup();
	
	void read(float &accelX, float &accelY, float &accelZ, float &gyroX, float &gyroY, float &gyroZ, float &temperature);
	void readRaw(int &accelX, int &accelY, int &accelZ, int &gyroX, int &gyroY, int &gyroZ, int &temperature);
};

#endif
