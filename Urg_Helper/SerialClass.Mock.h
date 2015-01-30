
#ifndef SERIALCLASSMOCK_H_INCLUDED
#define SERIALCLASSMOCK_H_INCLUDED

#define ARDUINO_WAIT_TIME 2000

#include "Stdafx.h"
#include "SerialClass.h"

#include <string>


class Serial_Mock : public Serial
{
protected:
	virtual void _init(LPCSTR portName);

public:
	Serial_Mock (LPCSTR portName) : Serial (portName) {}
	Serial_Mock (std::string portName) : Serial(portName) {}
};

#endif // SERIALCLASS_H_INCLUDED

