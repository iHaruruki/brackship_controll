#ifndef SERIAL_H
#define SERIAL_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <sys/wait.h>

typedef unsigned char BYTE;
typedef unsigned long DWORD;

class CSerial {
public:
    CSerial();
    ~CSerial();
    bool InitSerial(char* _comport, int _baudrate);
    bool CloseSerial(void);
    bool Read2(void* _data, int _size);
    bool Write2(void* _data, int _size);

    bool Write(BYTE data);
    bool Read(BYTE* data);
    DWORD CheckSerialData(void);
    void ClearRXbuffer(void);

protected:
    struct termios oldtio, newtio;

private:
    int fd;
    int mBaudrate;
    char* mDeviceName;
    bool mIsInitialised;
    bool InitSetting();
};
#endif
