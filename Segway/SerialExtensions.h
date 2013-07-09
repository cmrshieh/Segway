#ifndef SERIAL_EXTENSIONS_H
#define SERIAL_EXTENSIONS_H

template <class T>
void printBytesSerial(const T &value)
{
	Serial.write(reinterpret_cast<const uint8_t *>(&value), sizeof(T));
}

template <class T>
void readBytesSerial(T *value)
{
	for (size_t i = 0; i < sizeof(T); i++)
	{
		reinterpret_cast<uint8_t *>(value)[i] = Serial.read();
	}
}

#endif