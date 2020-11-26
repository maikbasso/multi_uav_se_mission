#ifndef CSERIAL_H
#define CSERIAL_H

#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include <ctime>
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

namespace multi_uav_se_mission{

#define SOCKET_MESSAGE_BLOCK_SIZE 10
#define SOCKET_MESSAGE_BLOCK_SIZE_MIN 4
#define SOCKET_MESSAGE_BLOCK_SIZE_MAX 1024

class CSerial {
private:
    int sock;
    int messageBlockSize;
    bool isNonBlocking;

public:
    CSerial();
    ~CSerial();

    bool openPort(std::string port, int baud = 9600);
    bool closePort();
    bool isOpened();
    void setMessageBlockSize(int blockSize);
    bool setNonBlocking();

    std::vector<unsigned char> readDataBlock();
    int writeDataInBlocks(std::vector<unsigned char> data);

};

}

#endif // CSERIAL_H
