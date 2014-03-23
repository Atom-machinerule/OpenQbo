
#ifndef L3G4200D_h
#define L3G4200D_h

#define L3G4200D_CTRL_REG1 0x20
#define L3G4200D_CTRL_REG2 0x21
#define L3G4200D_CTRL_REG3 0x22
#define L3G4200D_CTRL_REG4 0x23
#define L3G4200D_CTRL_REG5 0x24

#include <inttypes.h>

class L3G4200D
{
  public:
    L3G4200D(int);
    void writeRegister(uint8_t address, uint8_t val);
    int readRegister(uint8_t address);
    void getGyroValues(int& x, int& y, int& z);
    int setupScale(int scale);
  private:
    uint8_t deviceAddress;
};


#endif

