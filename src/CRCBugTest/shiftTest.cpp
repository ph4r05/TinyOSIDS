#include <iostream>
#include <stdint.h>

using namespace std;

//hello.cpp

int main()
{
  //uint8_t crc=0xff;
  uint8_t crc=0x7f;
  uint8_t newCrc = crc << 7;
  cout << "Hello world!\nCRC: " << ((int)crc) \
	  << "\nLQI: " << (crc & 0xf7) \
	  << "\nShifted <<7: " << (crc<<7) \
	  << "\nShifted >>7: " << (crc>>7) \
	  << "\nShifted <<14: " << (crc<<14) \
	  << "\nShifted >>14: " << (crc>>14) \
	  << "\nNewCRC: " << ((int)newCrc) \
	  << "\n";
    return 0;
}
