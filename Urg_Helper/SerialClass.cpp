#include "Stdafx.h"
#include "SerialClass.h"

Serial::Serial() :
	_running(true),
	_packetLock(std::unique_ptr<std::mutex>(new std::mutex())),
	_packetThread(std::bind(&Serial::_packetizeMeCapn, this))
{}

Serial::Serial(LPCSTR portName) :
	_running(true),
	_packetLock(std::unique_ptr<std::mutex>(new std::mutex())),
	_packetThread(std::bind(&Serial::_packetizeMeCapn, this))
{
    _init(portName);
}

Serial::Serial(std::string portName) :
	_running(true),
	_packetLock(std::unique_ptr<std::mutex>(new std::mutex())),
	_packetThread(std::bind(&Serial::_packetizeMeCapn, this))

{
	_init((LPCSTR)portName.c_str());
}

void Serial::_packetizeMeCapn()
{
	while (!this->IsConnected())
	{
		std::this_thread::yield();
	}
	while (_running)
	{
		unsigned char token;
		Packet p;
		// FORGIVE ME.  STATE MACHINES <3 GOTO + LABELS

Start_State:
		while ((token = this->Pop()) != START_TOKEN)
		{
			if (token == EOF) std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}

Timestamp_State:
		char ts_c[sizeof(unsigned long)];
		unsigned int curPos = 0;

		while ((token = this->Pop()) != TYPE_TOKEN)
		{
			if (token == EOF) 
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
				continue;
			}
			//if (token < '0' || token > '9') goto Start_State;
			if (curPos >= sizeof(unsigned long)) goto Start_State;
			ts_c[curPos++] = token;
		}

		// Now cast ts_c
		p.timestamp = *((unsigned long *)ts_c);

Type_State:
		while ((token = this->Pop()) != LENGTH_TOKEN)
		{
			if (token == EOF) 
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
				continue;
			}

			// TODO: Fix this
			if (token != (char)PacketTypes::kQuaternion &&
				token != (char)PacketTypes::kTimeStamp) goto Start_State;
			p.type = (PacketTypes)token;
		}
Length_State:
		while ((token = this->Pop()) != DATA_TOKEN)
		{
			if (token == EOF) 
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
				continue;
			}
			p.length = token;
		}
Data_State:
		unsigned char length = p.length;
		unsigned char* data = (unsigned char*)malloc(length);
		curPos = 0;
		while (curPos < length)
		{
			token = this->Pop();
			if (token == EOF)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
				continue;
			}
			if (token == END_TOKEN) goto Start_State;

			data[curPos++] = token;
		}

		p.data = data;

		// DONE
		_packetLock->lock();
		_packetMap[p.type].push(p);
		_packetLock->unlock();
	}
}

void Serial::_init(LPCSTR portName)
{
	//We're not yet connected
    this->connected = false;

    //Try to connect to the given port throuh CreateFile
    this->hSerial = CreateFile(portName,
            GENERIC_READ | GENERIC_WRITE,
            0,
            NULL,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            NULL);

    //Check if the connection was successfull
    if(this->hSerial==INVALID_HANDLE_VALUE)
    {
        //If not success full display an Error
        if(GetLastError()==ERROR_FILE_NOT_FOUND){

            //Print Error if neccessary
            printf("ERROR: Handle was not attached. Reason: %s not available.\n", portName);

        }
        else
        {
			DWORD retSize;
			LPTSTR pTmp = NULL;

			retSize = FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER |
									FORMAT_MESSAGE_FROM_SYSTEM |
									FORMAT_MESSAGE_ARGUMENT_ARRAY,
									NULL,
									GetLastError(),
									LANG_NEUTRAL,
									(LPTSTR)&pTmp,
									0,
									NULL);
			if (!retSize || pTmp == NULL) {}
			else
			{
				pTmp[strlen(pTmp) - 2] = '\0';
				printf("ERROR: %d: %s", GetLastError(), pTmp);
				LocalFree((HLOCAL)pTmp);
			}
        }
    }
    else
    {
        //If connected we try to set the comm parameters
        DCB dcbSerialParams = {0};

        //Try to get the current
        if (!GetCommState(this->hSerial, &dcbSerialParams))
        {
            //If impossible, show an error
            printf("failed to get current serial parameters!");
        }
        else
        {
            //Define serial connection parameters for the arduino board
            dcbSerialParams.BaudRate=CBR_115200;
            dcbSerialParams.ByteSize=8;
            dcbSerialParams.StopBits=ONESTOPBIT;
            dcbSerialParams.Parity=NOPARITY;

             //Set the parameters and check for their proper application
             if(!SetCommState(hSerial, &dcbSerialParams))
             {
                printf("ALERT: Could not set Serial Port parameters");
             }
             else
             {
                 //If everything went fine we're connected
                 this->connected = true;
                 //We wait 2s as the arduino board will be reseting
                 Sleep(ARDUINO_WAIT_TIME);
             }
        }
    }
}

Serial::~Serial()
{
    //Check if we are connected before trying to disconnect
    if(this->connected)
    {
        //We're no longer connected
        this->connected = false;
        //Close the serial handler
        CloseHandle(this->hSerial);
    }
}

char Serial::Pop()
{
	char inVal = -1;
	DWORD bytesRead = -1;
	ClearCommError(this->hSerial, &this->errors, &this->status);
	if (this->status.cbInQue > 0)
	{
		if(ReadFile(this->hSerial, &inVal, 1, &bytesRead, NULL) && bytesRead != 0)
        {
            return inVal;
        }
	}

	return -1;
}

int Serial::CharsInQueue()
{
	ClearCommError(this->hSerial, &this->errors, &this->status);
	return this->status.cbInQue;
}

int Serial::ReadToChar(char *buffer, char readTo, unsigned int nbChar)
{
    //Number of bytes we'll have read
    DWORD bytesRead = 0;
    //Number of bytes we'll really ask to read
    unsigned int toRead;

    //Use the ClearCommError function to get status info on the Serial port
    ClearCommError(this->hSerial, &this->errors, &this->status);

    //Check if there is something to read
    if(this->status.cbInQue>0)
    {
        //If there is we check if there is enough data to read the required number
        //of characters, if not we'll read only the available characters to prevent
        //locking of the application.
        if(this->status.cbInQue>nbChar)
        {
            toRead = nbChar;
        }
        else
        {
            toRead = this->status.cbInQue;
        }

        //Try to read the require number of chars, and return the number of read bytes on success
		char inChar;
        while ((inChar = this->Pop()) != readTo && bytesRead < nbChar)
        {
			buffer[bytesRead++] = inChar;
        }
		if (bytesRead != nbChar && inChar != readTo)
		{
			std::cout << "Ugh: " << (int)inChar << std::endl;
		}
		return bytesRead;
    }

    //If nothing has been read, or that an error was detected return -1
    return -1;
}

int Serial::ReadData(char *buffer, unsigned int nbChar)
{
    //Number of bytes we'll have read
    DWORD bytesRead;
    //Number of bytes we'll really ask to read
    unsigned int toRead;

    //Use the ClearCommError function to get status info on the Serial port
    ClearCommError(this->hSerial, &this->errors, &this->status);

    //Check if there is something to read
    if(this->status.cbInQue>0)
    {
        //If there is we check if there is enough data to read the required number
        //of characters, if not we'll read only the available characters to prevent
        //locking of the application.
        if(this->status.cbInQue>nbChar)
        {
            toRead = nbChar;
        }
        else
        {
            toRead = this->status.cbInQue;
        }

        //Try to read the require number of chars, and return the number of read bytes on success
        if(ReadFile(this->hSerial, buffer, toRead, &bytesRead, NULL) && bytesRead != 0)
        {
            return bytesRead;
        }

    }

    //If nothing has been read, or that an error was detected return -1
    return -1;
}


bool Serial::WriteData(char *buffer, unsigned int nbChar)
{
    DWORD bytesSend;

    //Try to write the buffer on the Serial port
    if(!WriteFile(this->hSerial, (void *)buffer, nbChar, &bytesSend, 0))
    {
        //In case it don't work get comm error and return false
        ClearCommError(this->hSerial, &this->errors, &this->status);

        return false;
    }
    else
        return true;
}

bool Serial::IsConnected()
{
    //Simply return the connection status
    return this->connected;
}

Serial::Packet Serial::GetPacket(Serial::PacketTypes type)
{
	Packet p;
	if (_packetMap[type].empty()) return p;
	
	_packetLock->lock();
	
	p = _packetMap[type].front();
	_packetMap[type].pop();

	_packetLock->unlock();

	return p;
}

bool Serial::SendPacket(Serial::Packet p)
{
	return true;
}