
#ifndef SERIALCLASS_H_INCLUDED
#define SERIALCLASS_H_INCLUDED

#define ARDUINO_WAIT_TIME 2000
#define START_TOKEN 'S'
#define TYPE_TOKEN 'T'
#define LENGTH_TOKEN 'L'
#define DATA_TOKEN 'D'
#define END_TOKEN 'E'
#define MAX_LEN 64

#include "Stdafx.h"
#include <string>
#include <queue>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>

class Serial
{
public:
	enum class PacketTypes : char {
		kQuaternion = 'q',
		kTimeStamp = 't',
	};

	struct Packet {
		PacketTypes type;
		unsigned char* data;
		unsigned char length;
		unsigned long timestamp;

		// GO AHEAD AND LEAK YOU MOTHERFUCKING SON OF A BITCH
		//~Packet() {
		//	if (data != NULL) free(data);
		//}

		Packet() {
			data = NULL;
			length = 0;
			timestamp = 0;
			type = (PacketTypes)0;
		}
	};

	//Initialize Serial communication with the given COM port
	Serial();
    Serial(LPCSTR portName);
	Serial(std::string portName);
    //Close the connection
    //NOTA: for some reason you can't connect again before exiting
    //the program and running it again
    ~Serial();
    //Check if we are actually connected
    bool IsConnected();

	Packet GetPacket(PacketTypes type);
	bool SendPacket(Packet p);
    bool WriteData(char *buffer, unsigned int nbChar);


protected:
    //Serial comm handler
    HANDLE hSerial;
    //Connection status
    bool connected;
    //Get various information about the connection
    COMSTAT status;
    //Keep track of last error
    DWORD errors;

	MOCKABLE void _init(LPCSTR portName);

	std::atomic<bool> _running;
	std::map<PacketTypes, std::queue<Packet>> _packetMap;
	std::unique_ptr<std::mutex> _packetLock;
	// MUST BE LAST IN LIST
	std::thread _packetThread;

	void _packetizeMeCapn();

	// Let's hide these here...

	//Read data in a buffer, if nbChar is greater than the
    //maximum number of bytes available, it will return only the
    //bytes available. The function return -1 when nothing could
    //be read, the number of bytes actually read.
    int ReadData(char *buffer, unsigned int nbChar);
    //Writes data from a buffer through the Serial connection
    //return true on success.
	char Pop();
	int ReadToChar(char *buffer, char readTo, unsigned int nbChar);
	int CharsInQueue();
  
};

#endif // SERIALCLASS_H_INCLUDED

