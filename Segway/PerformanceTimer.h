#ifndef PERFORMANCE_TIMER_H
#define PERFORMANCE_TIMER_H

#include <Arduino.h>

class PerformanceTimer
{
private:
	unsigned long m_startTime;
	unsigned long m_endTime;
	bool m_isRunning;
	
public:
	PerformanceTimer() : m_startTime(0), m_endTime(0), m_isRunning(false)
	{
	}

	unsigned long getStartTime()
	{
		return m_startTime;
	}
	
	unsigned long getElapsed()
	{
		return m_isRunning ? micros() - m_startTime : m_endTime - m_startTime;
	}
	
	void start()
	{
		if (m_startTime == 0)
			m_startTime = micros();
			
		m_isRunning = true;
	}
	
	void restart()
	{
		m_startTime = micros();
		m_isRunning = true;
	}
	
	void stop()
	{
		m_endTime = micros();
		m_isRunning = false;
	}
};

#endif