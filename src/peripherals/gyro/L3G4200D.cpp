// https://www.pololu.com/file/0J491/L3G4200D.pdf
// L3G4200D gyro
#include "L3G4200D.h"

#define GYRO_ADR 0x69
// #define DPS_PER_LSB 0.01526  // for +-500 dps FS assuming 16 bits of data
#define DPS_PER_LSB 0.0610351  // for +- 2000 dps FS
#define GYRO_REFRESH_PERIOD 1250  // for 800 Hz

// dynamic bias compensation definitions
#define CUTOFF_FREQUENCY (0.02f)
#define THRESHOLD (3.0f)
#define TIMEOUT (5000)  // milliseconds 

Gyro::Gyro(){
  gyro_bias = {0,0,0};
  last_gyro_vec = {0,0,0};
  gyro_timeout = 0;
  last_gyro_timestamp = 0;

  // dynamic bias compensation initialization
  bias_compensation_on = 0;
  dynamic_bias = {0,0,0};
  bias_timer = 0;
}

void Gyro::setup_gyro() {
  Serial.println("Setting up gyroscope");
  Wire.beginTransmission(GYRO_ADR);
  byte error = Wire.endTransmission();
  if(error){
    Serial.println("Gyroscope not responding");
    while(1){}
  }

  Wire.beginTransmission(GYRO_ADR);
  Wire.write(0x0F | (1 << 7));
  Wire.endTransmission();
  Wire.requestFrom(GYRO_ADR, (byte)1);
  while (!Wire.available()) {}
  uint8_t whoami = Wire.read();
  

  
  // writeReg(GYRO_ADR, 0x23, 0x90);  // Block update until read, FS 500 dps 
 writeReg(GYRO_ADR, 0x23, 0xB0);  // Block update until read, FS 2000 dps 
//  writeReg(GYRO_ADR, 0x20, 0xEF);  // 800 Hz, LP 50 Hz Cutoff
  writeReg(GYRO_ADR, 0x21, 0x29);  // Normal mode HPRESETFILTER, HP 0.1 Hz Cutoff
//  writeReg(GYRO_ADR, 0x20, 0x6F);  // 200 Hz, 50 Hz Cutoff
  // writeReg(GYRO_ADR, 0x20, 0xFF);  // 800 Hz, LP 110 Hz Cutoff
  // writeReg(GYRO_ADR, 0x20, 0xCF);  // 800 Hz, LP 30 Hz Cutoff
  writeReg(GYRO_ADR, 0x20, 0xEF);  // 800 Hz, LP 50 Hz Cutoff
//  writeReg(GYRO_ADR, 0x21, 0x09);  // Normal mode, 1 Hz Cutoff

  filtered_gyro_vec = read_gyro();
}


Vector3 Gyro::read_gyro() {
  uint32_t current_micros = micros();
  if ((current_micros - last_gyro_timestamp) < GYRO_REFRESH_PERIOD) return last_gyro_vec; 
  else last_gyro_timestamp = micros();
  
  Wire.beginTransmission(GYRO_ADR);
  Wire.write(0x28 | (1 << 7));
  Wire.endTransmission();
  Wire.requestFrom(GYRO_ADR, (byte)6);
  while (Wire.available()<6) {
    if ((current_micros - last_gyro_timestamp) > SENSOR_TIMEOUT_US){
      gyro_timeout = 1;
      return zero_3vector;
    }  
  }

  uint8_t xlg = Wire.read();
  uint8_t xhg = Wire.read();
  uint8_t ylg = Wire.read();
  uint8_t yhg = Wire.read();
  uint8_t zlg = Wire.read();
  uint8_t zhg = Wire.read();

  Vector3 gyro_vals;
  gyro_vals(1) =  float((int16_t)(xhg << 8 | xlg));
  gyro_vals(0) =  float((int16_t)(yhg << 8 | ylg));
  gyro_vals(2) =  -float((int16_t)(zhg << 8 | zlg));
  gyro_vals = gyro_vals*DPS_PER_LSB - gyro_bias;  // corrected reading

  // dynamic bias compensation implementation
  if (bias_compensation_on){
    gyro_vals = gyro_vals - dynamic_bias;
    if (abs(gyro_vals(0)) < THRESHOLD || abs(gyro_vals(1)) < THRESHOLD || abs(gyro_vals(2)) < THRESHOLD){  // omega in range
      bias_timer += current_micros/1000;
      if (bias_timer > TIMEOUT){
        dynamic_bias = dynamic_bias + gyro_vals * (2 * PI * CUTOFF_FREQUENCY * GYRO_REFRESH_PERIOD / 1000.0);  // 2 pi f_c / f_s
      }
    }
    else bias_timer = 0;  // omega outside of range

  }

  last_gyro_vec = gyro_vals;
  return gyro_vals; 
}

Vector3 Gyro::get_filtered_gyro(){
  if ((micros() - last_gyro_timestamp) < GYRO_REFRESH_PERIOD) return filtered_gyro_vec; 
  Vector3 gyro_reading = read_gyro();
  filtered_gyro_vec = gyro_reading * GYROLPF_RATIO + filtered_gyro_vec * (1 - GYROLPF_RATIO);
  return filtered_gyro_vec;
};

void Gyro::calibrate_gyro(){
  gyro_bias = {0,0,0};
  Vector3 data = {0,0,0};
  delay(2000);
  for (int i = 0; i < 100; i ++){
    data += read_gyro()/100;
    delayMicroseconds(GYRO_REFRESH_PERIOD + 100);
  }
  gyro_bias = data;
  // printVec3(gyro_bias, 3);
  Serial.println("Gyro calibrated");
  bias_compensation_on = 1;  // turn on dynamic bias compensation from here onward
}
