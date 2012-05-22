
#ifndef TEST_SERIAL_H
#define TEST_SERIAL_H

typedef nx_struct test_serial_msg {
  nx_uint16_t counter;
  nx_uint16_t received;
  nx_uint8_t radioCn;
  nx_uint8_t radioOn;
  nx_uint16_t radioRecv;
  nx_uint16_t radioSent;


} test_serial_msg_t;

enum {
  AM_TEST_SERIAL_MSG = 0x89,
};

#endif
