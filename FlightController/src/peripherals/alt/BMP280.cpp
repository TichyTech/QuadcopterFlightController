// https://cdn-shop.adafruit.com/datasheets/BST-BMP280-DS001-11.pdf
// BMP280
#include "config.h"
#include "peripherals/writeReg.h"
#include <Arduino.h>
#include "BMP280.h"

#define ALT_ADR 0x76

#define SEALEVEL_HPA 1011
#define ALT_REFRESH_PERIOD 44000  // for 16x 2x overs, 0.5 stdby

#define dig_T1 27364
#define dig_T2 25886
#define dig_T3 50
#define dig_P1 38344
#define dig_P2 -10503
#define dig_P3 3024
#define dig_P4 6858
#define dig_P5 -158
#define dig_P6 -7
#define dig_P7 15500
#define dig_P8 -14600
#define dig_P9 6000

Altimeter::Altimeter(){
  last_alt_val = 0;
  alt_timed_out = 0;  
  last_alt_timestamp = 0;
}

int16_t Altimeter::read_alt16(uint8_t reg){  
  Wire.beginTransmission(ALT_ADR);
  Wire.write(reg | (1 << 7));  // read mode
  Wire.endTransmission();

  Wire.requestFrom(ALT_ADR, (byte)2);
  unsigned long start_time = micros();
  while(Wire.available() < 2){
    if ((micros() - start_time) > SENSOR_TIMEOUT_US){
      alt_timed_out = 1;
      return 0;
    }  
  } 

  uint8_t lbits = Wire.read();
  uint8_t hbits = Wire.read();

  return (int16_t)(hbits << 8 | lbits);
}


inline int32_t Altimeter::read_alt24(uint8_t reg){
  Wire.beginTransmission(ALT_ADR);  // read temperature
  Wire.write(reg | (1 << 7));
  Wire.endTransmission();
  Wire.requestFrom(ALT_ADR, (byte)3);
  
  unsigned long start_time = micros();
  while (Wire.available()<3) {
    if ((micros() - start_time) > SENSOR_TIMEOUT_US){
      alt_timed_out = 1;
      return 0;
    }    
  }
  
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  uint8_t xlsb = Wire.read();
  return msb << 12 | lsb << 4 | xlsb >> 4;
}


void Altimeter::setup_alt(){
  Serial.println("Setting up altimeter");
  Wire.beginTransmission(ALT_ADR);
  byte error = Wire.endTransmission();
  
  if(error){
    Serial.println("Altimeter not responding");
    while(1){}
  }
  
  // uint8_t measreg = 0x05 << 5 | 0x05 << 2 | 0x03;  // 16x T, 16x P, normal mode
  uint8_t measreg = 0x02 << 5 | 0x05 << 2 | 0x03;  // 2x T, 16x P, normal mode

//  uint8_t configreg = 0x00 << 5 | 0x01 << 2;  // 0.5 ms standby, X2 filter
  // uint8_t configreg = 0x00 << 5 | 0x03 << 2;  // 0.5 ms standby, X8 filter
 uint8_t configreg = 0x00 << 5 | 0x04 << 2;  // 0.5 ms standby, X16 filter
  
  writeReg(ALT_ADR, 0xF4, measreg); // oversampling, mode
  writeReg(ALT_ADR, 0xF5, configreg); // t_standby, filter

  filtered_alt_val = read_alt();
}


float Altimeter::read_alt(){

  if ((micros() - last_alt_timestamp) < ALT_REFRESH_PERIOD) return last_alt_val;  // wait at least 44 ms between measurements (update delay of BMP280)
  else last_alt_timestamp = micros();
  
  int32_t adc_T = read_alt24(0xFA);
  int32_t adc_P = read_alt24(0xF7);

  int32_t var11 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
  int32_t var22 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) *((int32_t)dig_T3)) >> 14;
  t_fine = var11 + var22;

  int64_t var1, var2, p;
  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
  var2 = var2 + (((int64_t)dig_P4)<<35);
  var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
  var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
  if (var1 == 0) return last_alt_val;  // avoid zero division
  
  p = 1048576-adc_P;
  p = (((p<<31)-var2)*3125)/var1;
  var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((int64_t)dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
  
  float var3 = float((uint32_t)p);  // pressure in Pa*256
  var3 = (var3 / (SEALEVEL_HPA*256*100)) - 1;
  float alt = var3*(-8436 + var3*(3415.3142 - 2060.2314*var3));  // Taylor polynomial
  last_alt_val = alt;
  return alt;
}

float Altimeter::get_filtered_alt(){
if ((micros() - last_alt_timestamp) < ALT_REFRESH_PERIOD) return filtered_alt_val;
float read_val = read_alt();
filtered_alt_val = read_val * ALTLPF_RATIO + filtered_alt_val*(1 - ALTLPF_RATIO);  // low pass filter for altimeter
return filtered_alt_val;
}

int32_t Altimeter::read_temp(){
  int32_t adc_T = read_alt24(0xFA);

  int32_t var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
  int32_t var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) *((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;
  int32_t T = (t_fine * 5 + 128) >> 8;
  return T;
}


float Altimeter::read_pressure(){
  int32_t adc_P = read_alt24(0xF7);

  int64_t var1, var2, p;
  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
  var2 = var2 + (((int64_t)dig_P4)<<35);
  var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
  var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
  
  if (var1 == 0) return 0;
  
  p = 1048576-adc_P;
  p = (((p<<31)-var2)*3125)/var1;
  var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((int64_t)dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
  
  return float((uint32_t)p);  // pressure in Pa
}


void Altimeter::get_alt_calibration(){
  
  int16_t ldig_T2 , ldig_T3 , ldig_P2 , ldig_P3, ldig_P4, ldig_P5, ldig_P6, ldig_P7, ldig_P8, ldig_P9; 
  uint16_t ldig_T1, ldig_P1;

  ldig_T1 = (uint16_t) read_alt16(0x88); 
  ldig_T2 = read_alt16(0x8A);  
  ldig_T3 = read_alt16(0x8C);  
  ldig_P1 = (uint16_t) read_alt16(0x8E); 
  ldig_P2 = read_alt16(0x90);  
  ldig_P3 = read_alt16(0x92); 
  ldig_P4 = read_alt16(0x94);  
  ldig_P5 = read_alt16(0x96);  
  ldig_P6 = read_alt16(0x98);  
  ldig_P7 = read_alt16(0x9A); 
  ldig_P8 = read_alt16(0x9C); 
  ldig_P9 = read_alt16(0x9E);

  Serial.println(ldig_T1);
  Serial.println(ldig_T2);
  Serial.println(ldig_T3);
  Serial.println(ldig_P1);
  Serial.println(ldig_P2);
  Serial.println(ldig_P3);
  Serial.println(ldig_P4);
  Serial.println(ldig_P5);
  Serial.println(ldig_P6);
  Serial.println(ldig_P7);
  Serial.println(ldig_P8);
  Serial.println(ldig_P9);
}

