#include <multi_uav_se_mission/CSerial.h>

namespace multi_uav_se_mission{

CSerial::CSerial(){
  this->messageBlockSize = SOCKET_MESSAGE_BLOCK_SIZE;
  this->isNonBlocking = false;
}

CSerial::~CSerial(){
    this->closePort();
}

bool CSerial::openPort(std::string port, int baud){

    this->sock = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

    if (this->sock != -1){
        // Read the configureation of the port

          struct termios options;
          tcgetattr( this->sock, &options );

          /* SEt Baud Rate */

          switch (baud) {
          case 57600:
              cfsetispeed( &options, B57600 );
              cfsetospeed( &options, B57600 );
              break;
          case 9600:
              cfsetispeed( &options, B9600 );
              cfsetospeed( &options, B9600 );
              break;
          case 115200:
              cfsetispeed( &options, B115200 );
              cfsetospeed( &options, B115200 );
              break;
          default:
              cfsetispeed( &options, B9600 );
              cfsetospeed( &options, B9600 );
              break;
          }

          //I don't know what this is exactly

          options.c_cflag |= ( CLOCAL | CREAD );

          // Set the Charactor size

          options.c_cflag &= ~CSIZE; /* Mask the character size bits */
          options.c_cflag |= CS8;    /* Select 8 data bits */

          // Set parity - No Parity (8N1)

          options.c_cflag &= ~PARENB;
          options.c_cflag &= ~CSTOPB;
          options.c_cflag &= ~CSIZE;
          options.c_cflag |= CS8;

          // Disable Hardware flowcontrol

          //  options.c_cflag &= ~CNEW_RTSCTS;  -- not supported

          // Enable Raw Input

          options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

          // Disable Software Flow control

          options.c_iflag &= ~(IXON | IXOFF | IXANY);

          // Chose raw (not processed) output

          options.c_oflag &= ~OPOST;

          if ( tcsetattr(this->sock, TCSANOW, &options ) == -1 )
            std::cout << "Error with tcsetattr!" << std::endl;

          fcntl(this->sock, F_SETFL, FNDELAY);

        return true;
    }
    else{
        fcntl(sock, F_SETFL, FNDELAY);
        return false;
    }
}

bool CSerial::closePort(){

    close(this->sock);

    return true;
}

bool CSerial::isOpened(){
  if(this->sock == -1){
    this->closePort();
    return false;
  }

  return true;
}

void CSerial::setMessageBlockSize(int blockSize){
  if(blockSize > SOCKET_MESSAGE_BLOCK_SIZE_MAX) this->messageBlockSize = SOCKET_MESSAGE_BLOCK_SIZE_MAX;
  else if(blockSize < SOCKET_MESSAGE_BLOCK_SIZE_MIN) this->messageBlockSize = SOCKET_MESSAGE_BLOCK_SIZE_MIN;
  else this->messageBlockSize = blockSize;
}

bool CSerial::setNonBlocking(){
  int status = fcntl(this->sock, F_SETFL, fcntl(this->sock, F_GETFL, 0) | O_NONBLOCK);

  if (status == -1){
    return false;
  }

  this->isNonBlocking = true;

  return true;
}

std::vector<unsigned char> CSerial::readDataBlock(){

  std::vector<unsigned char> data;
  unsigned char buffer[1];

  while(this->isOpened() && data.size() < this->messageBlockSize){
    int n = read(this->sock, buffer, 1);
    if(n > 0){
      data.push_back(buffer[0]);
    }
//    else {
//      if(this->isNonBlocking == false) this->closePort();
//      break;
//    }
  }

  return data;
}

int CSerial::writeDataInBlocks(std::vector<unsigned char> data){

  // complete the last block size
  int r = this->messageBlockSize - (data.size() % this->messageBlockSize);
  if(r >= this->messageBlockSize) r = 0;
  for (int i = 0; i<r; i++) {
    data.push_back('\0');
  }

  // making a pointer that points to the actual vector data
  unsigned char *buffer = &data[0];

  // send all message blocks
  int i = 0;
  int sz = 0;

  for(i = 0; i < data.size() && this->isOpened(); i += this->messageBlockSize ){
    while(this->messageBlockSize - sz && this->isOpened()){
      sz += write(this->sock, buffer+i+sz, this->messageBlockSize-sz);
    }
    sz = 0;
  }

  return i;

}


}
