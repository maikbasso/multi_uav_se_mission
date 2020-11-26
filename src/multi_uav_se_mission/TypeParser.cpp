/*
@author Maik Basso <maik@maikbasso.com.br>
*/

#include <multi_uav_se_mission/TypeParser.h>

namespace multi_uav_se_mission{

TypeParser::TypeParser(){}

TypeParser::~TypeParser(){}

unsigned char TypeParser::int8tToUchar(int8_t value){
  return (unsigned char) value;
}

int8_t TypeParser::ucharToInt8t(unsigned char value){
  return (int8_t) value;
}

std::vector<unsigned char> TypeParser::intToUcharArray(int value){
  std::vector<unsigned char> buffer(4);
  buffer[0] = (value >> 24) & 0xFF;
  buffer[1] = (value >> 16) & 0xFF;
  buffer[2] = (value >> 8) & 0xFF;
  buffer[3] = (value) & 0xFF;
  return buffer;
}

int TypeParser::ucharArrayToInt(std::vector<unsigned char> value){
  if(value.size() != 4){
    return 0;
  }
  else{
    return (value[0] << 24) | (value[1] << 16) | (value[2] << 8) | value[3];
  }
}

std::vector<unsigned char> TypeParser::floatToUcharArray(float value){
  union_float32_uchar4_t aux;
  aux.float_data = value;
  unsigned char buff[4];
  memcpy(buff, &aux.unsigned_char_array[0], sizeof(float));

  std::vector<unsigned char> buffer(4);
  for (int i=0; i<4; i++) {
    buffer[i] = buff[i];
  }

  return buffer;
}

float TypeParser::ucharArrayToFloat(std::vector<unsigned char> value){
  if(value.size() != 4){
    return 0.0;
  }
  else{
    unsigned char buff[4];
    for (int i=0; i<4; i++) {
      buff[i] = value[i];
    }

    union_float32_uchar4_t aux;
    memcpy(&aux.unsigned_char_array[0], buff, sizeof(float));
    return aux.float_data;
  }
}

}
