
/******************************************************************************
 * Includes
 ******************************************************************************/

//#include "WConstants.h"
#include "I2C.h"
#include "LIS35DE.h"

/******************************************************************************
 * Definitions
 ******************************************************************************/

/******************************************************************************
 * Constructors
 ******************************************************************************/

/******************************************************************************
 * User API
 ******************************************************************************/

LIS35DE::LIS35DE(int  address) {
    deviceAddress=address;
}

void LIS35DE::writeRegister(uint8_t address, uint8_t val) {
/*
    I2c.beginTransmission(deviceAddress); // start transmission to device 
    I2c.send(address);       // send register address
    I2c.send(val);         // send value to write
    I2c.endTransmission();     // end transmission
*/
    I2c.write(deviceAddress, address, val);
}

int LIS35DE::readRegister(uint8_t address){

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

void LIS35DE::getAccelerometerValues(int8_t& x, int8_t& y, int8_t& z){

  x = readRegister(0x29);

  y = readRegister(0x2B);

  z = readRegister(0x2D);
}

int LIS35DE::setup(){
  //From  Jim Lindblom of Sparkfun's code

  // Enable x, y, z and turn off power down:
  writeRegister(LIS35DE_CTRL_REG1, 0b01000111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(LIS35DE_CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeRegister(LIS35DE_CTRL_REG3, 0b00000000);
}
