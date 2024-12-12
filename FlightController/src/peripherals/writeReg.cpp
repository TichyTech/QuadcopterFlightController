#include "writeReg.h"

int writeReg(uint8_t address, uint8_t reg, uint8_t value){
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
  return 0;
}
