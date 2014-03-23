
/******************************************************************************
 * Includes
 ******************************************************************************/

//#include "WConstants.h"
#include "I2C.h"
#include "L3G4200D.h"

/******************************************************************************
 * Definitions
 ******************************************************************************/

/******************************************************************************
 * Constructors
 ******************************************************************************/

/******************************************************************************
 * User API
 ******************************************************************************/

L3G4200D::L3G4200D(int  address) {
    deviceAddress=address;
}

void L3G4200D::writeRegister(uint8_t address, uint8_t val) {
/*
    I2c.beginTransmission(deviceAddress); // start transmission to device 
    I2c.send(address);       // send register address
    I2c.send(val);         // send value to write
    I2c.endTransmission();     // end transmission
*/
    I2c.write(deviceAddress, address, val);
}

int L3G4200D::readRegister(uint8_t address){

    int v=0;
/*
    I2c.beginTransmission(deviceAddress);
    I2c.send(address); // register to read
    I2c.endTransmission();

    I2c.requestFrom(deviceAddress, 1); // read a uint8_t
*/
    I2c.read(deviceAddress,address,(uint8_t)1);
    //while(!I2c.available()) {
    //    // waiting
    //}

    if(I2c.available())
        v = I2c.receive();
    return v;
}

void L3G4200D::getGyroValues(int& x, int& y, int& z){

  uint8_t xMSB = readRegister(0x29);
  uint8_t xLSB = readRegister(0x28);
  x = ((xMSB << 8) | xLSB);

  uint8_t yMSB = readRegister(0x2B);
  uint8_t yLSB = readRegister(0x2A);
  y = ((yMSB << 8) | yLSB);

  uint8_t zMSB = readRegister(0x2D);
  uint8_t zLSB = readRegister(0x2C);
  z = ((zMSB << 8) | zLSB);
}

int L3G4200D::setupScale(int scale){
  //From  Jim Lindblom of Sparkfun's code

  // Enable x, y, z and turn off power down:
  writeRegister(L3G4200D_CTRL_REG1, 0b10001111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(L3G4200D_CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeRegister(L3G4200D_CTRL_REG3, 0b00000000);

  // CTRL_REG4 controls the full-scale range, among other things:

  if(scale == 250){
    writeRegister(L3G4200D_CTRL_REG4, 0b00000000);
  }else if(scale == 500){
    writeRegister(L3G4200D_CTRL_REG4, 0b00010000);
  }else{
    writeRegister(L3G4200D_CTRL_REG4, 0b00110000);
  }

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  writeRegister(L3G4200D_CTRL_REG5, 0b00000000);
}
