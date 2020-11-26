/*
@author Maik Basso <maik@maikbasso.com.br>
*/

#ifndef MULTI_UAV_SE_MISSION_TYPE_PARSER_H
#define MULTI_UAV_SE_MISSION_TYPE_PARSER_H

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace multi_uav_se_mission{

typedef union union_float32_uchar4{
  float float_data;
  unsigned char unsigned_char_array[4];
} union_float32_uchar4_t;

class TypeParser {

  private:

	public:
    TypeParser();
    ~TypeParser();

    static const int SIZE_INT8_T_BYTES = 1;
    static const int SIZE_INT_BYTES = 4;
    static const int SIZE_FLOAT_BYTES = 4;

    static unsigned char int8tToUchar(int8_t value);
    static int8_t ucharToInt8t(unsigned char value);

    static std::vector<unsigned char> intToUcharArray(int value);
    static int ucharArrayToInt(std::vector<unsigned char> value);

    static std::vector<unsigned char> floatToUcharArray(float value);
    static float ucharArrayToFloat(std::vector<unsigned char> value);

};

}

#endif // MULTI_UAV_SE_MISSION_TYPE_PARSER_H
