#include <Wire.h>
#include "MPU6050.h"

// --------------------------------------------------------
// MPU6050_read
//
// This is a common function to read multiple bytes 
// from an I2C device.
//
// It uses the boolean parameter for Wire.endTransMission()
// to be able to hold or release the I2C-bus. 
// This is implemented in Arduino 1.0.1.
//
// Only this function is used to read. 
// There is no function for a single byte.
//
int MPU6050_read(int start, uint8_t *buffer, int size)
{
	int i, n, error;

	Wire.beginTransmission(MPU6050_I2C_ADDRESS);
	n = Wire.write(start);
	if (n != 1)
		return (-10);

	n = Wire.endTransmission(false);		// hold the I2C-bus
	if (n != 0)
		return (n);

	// Third parameter is true: relase I2C-bus after data is read.
	Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
	i = 0;
	while(Wire.available() && i<size)
	{
		buffer[i++]=Wire.read();
	}
	if ( i != size)
		return (-11);

	return (0);	// return : no error
}

int MPU6050_read_reg(int reg, uint8_t &val)
{
	int error = MPU6050_read(reg, &val, 1);

	return (error);
}


// --------------------------------------------------------
// MPU6050_write
//
// This is a common function to write multiple bytes to an I2C device.
//
// If only a single register is written,
// use the function MPU_6050_write_reg().
//
// Parameters:
//	 start : Start address, use a define for the register
//	 pData : A pointer to the data to write.
//	 size	: The number of bytes to write.
//
// If only a single register is written, a pointer
// to the data has to be used, and the size is
// a single byte:
//	 int data = 0;				// the data to write
//	 MPU6050_write (MPU6050_PWR_MGMT_1, &c, 1);
//
int MPU6050_write(int start, const uint8_t *pData, int size)
{
	int n, error;

	Wire.beginTransmission(MPU6050_I2C_ADDRESS);
	n = Wire.write(start);				// write the start address
	if (n != 1)
		return (-20);

	n = Wire.write(pData, size);	// write data bytes
	if (n != size)
		return (-21);

	error = Wire.endTransmission(true); // release the I2C-bus
	if (error != 0)
		return (error);

	return (0);				 // return : no error
}

// --------------------------------------------------------
// MPU6050_write_reg
//
// An extra function to write a single register.
// It is just a wrapper around the MPU_6050_write()
// function, and it is only a convenient function
// to make it easier to write a single register.
//
int MPU6050_write_reg(int reg, uint8_t data)
{
	int error = MPU6050_write(reg, &data, 1);

	return (error);
}

bool MPU6050::setup()
{
	Wire.begin();
	int error;
	uint8_t c;
	error = MPU6050_read(MPU6050_WHO_AM_I, &c, 1);
	if (error)
		return false;
		
	error = MPU6050_read(MPU6050_PWR_MGMT_2, &c, 1);
	if (error)
		return false;
		
	// Clear the 'sleep' bit to start the sensor.
	error = MPU6050_write_reg(MPU6050_PWR_MGMT_1, 0);
	
	return error == 0;
}

void MPU6050::calibrate()
{
	long accelVals[3] = {0, 0, 0};
	long gyroVals[3] = {0, 0, 0};
	for (int i = 0; i < 100; i++)
	{
		int temp;
		int ax, ay, az;
		int gx, gy, gz;
		readRaw(ax, ay, az, gx, gy, gz, temp);
		
		accelVals[0] += ax;
		accelVals[1] += ay;
		accelVals[2] += az;
		
		gyroVals[0] += gx;
		gyroVals[1] += gy;
		gyroVals[2] += gz;
		
		delay(10);
	}
	
	m_accelZeros[0] = accelVals[0] / 100.0f;
	m_accelZeros[1] = accelVals[1] / 100.0f;
	// We assume Z has gravity acting on it, so subtract 1g from the zero value
	// Ideally calibration would be done in free fall with no forces acting on any of the axes. This is not very easy, unfortunately.
	m_accelZeros[2] = accelVals[2] / 100.0f - 16384.0f;
	
	m_gyroZeros[0] = gyroVals[0] / 100.0f;
	m_gyroZeros[1] = gyroVals[1] / 100.0f;
	m_gyroZeros[2] = gyroVals[2] / 100.0f;
	
	Serial.print("Done calibrating! Acc: <");
	Serial.print(m_accelZeros[0]);
	Serial.print(", ");
	Serial.print(m_accelZeros[1]);
	Serial.print(", ");
	Serial.print(m_accelZeros[2]);
	Serial.print(">, Gyro: <");
	Serial.print(m_gyroZeros[0]);
	Serial.print(", ");
	Serial.print(m_gyroZeros[1]);
	Serial.print(", ");
	Serial.print(m_gyroZeros[2]);
	Serial.println(">");
}

void MPU6050::calibrate(int accelZeros[3], int gyroZeros[3])
{
	m_accelZeros[0] = accelZeros[0];
	m_accelZeros[1] = accelZeros[1];
	m_accelZeros[2] = accelZeros[2];
	
	m_gyroZeros[0] = gyroZeros[0];
	m_gyroZeros[1] = gyroZeros[1];
	m_gyroZeros[2] = gyroZeros[2];
}

void MPU6050::read(float &accelX, float &accelY, float &accelZ, float &gyroX, float &gyroY, float &gyroZ, float &temperature)
{
	accel_t_gyro_union accel_t_gyro;

	// Read the raw values.
	// Read 14 bytes at once, 
	// containing acceleration, temperature and gyro.
	// With the default settings of the MPU-6050,
	// there is no filter enabled, and the values
	// are not very stable.
	int error = MPU6050_read(MPU6050_ACCEL_XOUT_H, (uint8_t *)&accel_t_gyro, sizeof(accel_t_gyro));

	// Swap all high and low bytes.
	// After this, the registers values are swapped, 
	// so the structure name like x_accel_l does no 
	// longer contain the lower byte.
	uint8_t swap;
	#define SWAP(x,y) swap = x; x = y; y = swap

	SWAP(accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
	SWAP(accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
	SWAP(accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
	SWAP(accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
	SWAP(accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
	SWAP(accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
	SWAP(accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);
	
	accelX = (accel_t_gyro.value.x_accel - m_accelZeros[0]) / 16384.0f;
	accelY = (accel_t_gyro.value.y_accel - m_accelZeros[1]) / 16384.0f;
	accelZ = (accel_t_gyro.value.z_accel - m_accelZeros[2]) / 16384.0f;
	
	gyroX = (accel_t_gyro.value.x_gyro - m_gyroZeros[0]) / 131.0f;
	gyroY = (accel_t_gyro.value.y_gyro - m_gyroZeros[1]) / 131.0f;
	gyroZ = (accel_t_gyro.value.z_gyro - m_gyroZeros[2]) / 131.0f;
	
	temperature = (accel_t_gyro.value.temperature + 12421.0f) / 340.0f;
}

void MPU6050::readRaw(int &accelX, int &accelY, int &accelZ, int &gyroX, int &gyroY, int &gyroZ, int &temperature)
{
	accel_t_gyro_union accel_t_gyro;

	// Read the raw values.
	// Read 14 bytes at once, 
	// containing acceleration, temperature and gyro.
	// With the default settings of the MPU-6050,
	// there is no filter enabled, and the values
	// are not very stable.
	int error = MPU6050_read(MPU6050_ACCEL_XOUT_H, (uint8_t *)&accel_t_gyro, sizeof(accel_t_gyro));

	// Swap all high and low bytes.
	// After this, the registers values are swapped, 
	// so the structure name like x_accel_l does no 
	// longer contain the lower byte.
	uint8_t swap;
	#define SWAP(x,y) swap = x; x = y; y = swap

	SWAP(accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
	SWAP(accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
	SWAP(accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
	SWAP(accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
	SWAP(accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
	SWAP(accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
	SWAP(accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);
	
	accelX = accel_t_gyro.value.x_accel;
    accelY = accel_t_gyro.value.y_accel;
    accelZ = accel_t_gyro.value.z_accel;
	
	gyroX = accel_t_gyro.value.x_gyro;
    gyroY = accel_t_gyro.value.y_gyro;
    gyroZ = accel_t_gyro.value.z_gyro;

	temperature = accel_t_gyro.value.temperature;
}