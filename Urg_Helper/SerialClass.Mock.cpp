#include "Stdafx.h"
#include "SerialClass.Mock.h"

void Serial_Mock::_init(LPCSTR portName)
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
        //If everything went fine we're connected
        this->connected = true;
    }
}