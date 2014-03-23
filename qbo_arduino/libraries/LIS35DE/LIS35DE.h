
#ifndef LIS35DE_h
#define LIS35DE_h

#define LIS35DE_CTRL_REG1 0x20
#define LIS35DE_CTRL_REG2 0x21
#define LIS35DE_CTRL_REG3 0x22

#include <inttypes.h>

class LIS35DE
{
  public:
    LIS35DE(int);
    void writeRegister(uint8_t address, uint8_t val);
    int readRegister(uint8_t address);
    void getAccelerometerValues(int8_t& x, int8_t& y, int8_t& z);
    int setup();
  private:
    uint8_t deviceAddress;
};


#endif

