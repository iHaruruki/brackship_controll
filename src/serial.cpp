// Myserial.cpp : 実装ファイル
//
#include "../include/brackship_controll/serial.hpp"

// CSerial
CSerial::CSerial() {
    mDeviceName = "";
    mBaudrate = 0;
    mIsInitialised = false;
    memset(&newtio, 0, sizeof(newtio));
    memset(&oldtio, 0, sizeof(oldtio));
}

CSerial::~CSerial() {
    tcsetattr(fd, TCSANOW, &oldtio); //設定をもとに戻す
    CloseSerial();
}

bool CSerial::InitSetting() {

    newtio.c_cflag = CS8 | CLOCAL | CREAD;
    // CS8: 8n1(8bit/noparity/1stopbit)
    // CLOCAL: local conection(no modem control)
    // CREAD: make receiving charactors available
    newtio.c_cc[VTIME] = 0; //charactor timer
    newtio.c_lflag = ICANON; //make canonical input available
    newtio.c_iflag = IGNPAR | ICRNL;// IGNPAR: ignore parity errors

    cfsetispeed(&newtio, mBaudrate);
    cfsetospeed(&newtio, mBaudrate);
    tcsetattr(fd, TCSANOW, &newtio);

}

bool CSerial::InitSerial(char* _comport, int _baudrate) {

    if (mIsInitialised) {
        return true;
    } else {
        mDeviceName = _comport;
        mBaudrate = _baudrate;

        std::cout<<mDeviceName<<' '<<mBaudrate<<std::endl;
        fd = open(mDeviceName, O_WRONLY | O_NOCTTY);
        std::cout<<fd<<std::endl;
        if (fd < 0) {
            perror(mDeviceName);
        }
        tcgetattr(fd, &oldtio); // 現在のポート設定を待避

        InitSetting();
        mIsInitialised = true;
        return true;
    }
}

bool CSerial::CloseSerial() {
    return close(fd) < 0 ? false : true;
}

bool CSerial::Write(BYTE _data) {
    return (write(fd, &_data, sizeof(BYTE)) >= 0);
}

bool CSerial::Read(BYTE* _data) {
    return (read(fd, &_data, sizeof(BYTE)) >= 0);
}

bool CSerial::Write2(void* _data, int _size) {
    return (write(fd, _data, _size) >= 0);
}

bool CSerial::Read2(void* _data, int _size) {
    return (read(fd, &_data, _size) >= 0);
}


// 受信データのバイト数を調べて値を返す
DWORD CSerial::CheckSerialData(void) {
// windows:
//    DWORD   dwErrors;  // エラー情報
//    COMSTAT ComStat; // デバイスの状態
//    DWORD   dwCount;   // 受信データのバイト数
//    ClearCommError(hCom, &dwErrors, &ComStat);
//	  dwCount = ComStat.cbInQue;
//    return dwCount;
// linux:
//   TODO
    return 0;
}

void CSerial::ClearRXbuffer(void) {
	//受信バッファをすべてクリア
    // windows:
    //   PurgeComm(hCom, PURGE_RXABORT | PURGE_RXCLEAR);
    // linux:
    //   TODO
}
