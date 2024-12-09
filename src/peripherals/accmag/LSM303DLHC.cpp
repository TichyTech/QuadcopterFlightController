// https://www.st.com/resource/en/datasheet/DM00027543.pdf
// LSM303DLHC accelerometer and magnetometer
#include "LSM303DLHC.h"

#define ACC_ADR 0x19
#define MAG_ADR 0x1E
// #define ACC_REFRESH_PERIOD 2500  // for 400 HZ
#define ACC_REFRESH_PERIOD 745  // for 1344 HZ
#define MAG_REFRESH_PERIOD 4555  // for 220 HZ
#define ACC_FS 2.0f  // g
#define MAG_FS 1.3f  // gauss

AccMag::AccMag(){
  acc_bias = {0,0,0};
  acc_timeout = 0;
  mag_timeout = 0;
  // Vector3 mag_bias = {-2.70, 1.63, 15.55};
  // Matrix3 mag_scale = {0.948, 0.080, -0.012, 0.008, 0.933, -0.028, -0.012, -0.028, 1.131};
  // This is the calibration for Frame V4
  mag_bias = {-0.54, -1.95, -13.49};
  mag_scale = {0.941, 0.004, 0.017, 0.004, 0.931, 0.009, 0.017, 0.009, 1.143};

  last_mag_timestamp = 0;
  last_acc_timestamp = 0;
}

void AccMag::setup_acc(){
  Serial.println("Setting up accelerometer");
  Wire.beginTransmission(ACC_ADR);
  byte error = Wire.endTransmission();
  
  if(error){
    Serial.println("Accelerometer not responding");
    while(1){}
  }
      
//  writeReg(ACC_ADR, 0x20, 0x77); // 400Hz,  enable XYZ
  writeReg(ACC_ADR, 0x20, 0x97); // 1344Hz,  enable XYZ
  writeReg(ACC_ADR, 0x23, 0x88); // block update until read, LSB first, FS +-2g, enable High resolution
  filtered_acc_vec = read_acc();
}


void AccMag::setup_mag(){
  Serial.println("Setting up magnetometer");
  Wire.beginTransmission(MAG_ADR);
  byte error = Wire.endTransmission();
  
  if(error){
    Serial.println("Magnetometer not responding");
    while(1){}
  }
  
  writeReg(MAG_ADR, 0x00, 0x1C); // 220 Hz (max)
  writeReg(MAG_ADR, 0x01, 0x20); // gain 1100 (max), +- 1.3 gauss
  writeReg(MAG_ADR, 0x02, 0x00); // continuous conversion mode

  filtered_mag_vec = read_mag();
  };


Vector3 AccMag::read_acc(){
  // fetches latest possible measurement from accelerometer unit
  if ((micros() - last_acc_timestamp) < ACC_REFRESH_PERIOD) return last_acc_vec; 
  else last_acc_timestamp = micros();
  
  Wire.beginTransmission(ACC_ADR);
  Wire.write(0x28 | (1 << 7));
  Wire.endTransmission();
  Wire.requestFrom(ACC_ADR, (byte)6);
  while (Wire.available()<6) {
    if ((micros() - last_acc_timestamp) > SENSOR_TIMEOUT_US){
      acc_timeout = 1;
      return zero_3vector;
    }
  }
  
  uint8_t xla = Wire.read();
  uint8_t xha = Wire.read();
  uint8_t yla = Wire.read();
  uint8_t yha = Wire.read();
  uint8_t zla = Wire.read();
  uint8_t zha = Wire.read();

  // l,h x,y,z upper 12 bits
  Vector3 acc_vals;
  acc_vals(1) = float((int16_t)(xha << 8 | xla)>>4);
  acc_vals(0) = float((int16_t)(yha << 8 | yla)>>4);
  acc_vals(2) = -float((int16_t)(zha << 8 | zla)>>4);
  acc_vals = acc_vals*(ACC_FS/2048) - acc_bias;

  last_acc_vec = acc_vals; 
  return acc_vals;
}

Vector3 AccMag::read_mag(){
  // fetches latest possible measurement from magnetometer unit
  if ((micros() - last_mag_timestamp) < MAG_REFRESH_PERIOD) return last_mag_vec; 
  else last_mag_timestamp = micros();
  
  Wire.beginTransmission(MAG_ADR);
  Wire.write(0x03 | (1 << 7));
  Wire.endTransmission();
  Wire.requestFrom(MAG_ADR, (byte)6);
  while (Wire.available()<6) {
    if ((micros() - last_mag_timestamp) > SENSOR_TIMEOUT_US){
      mag_timeout = 1;
      return zero_3vector;
    }  
  }
  
  uint8_t xhm = Wire.read();
  uint8_t xlm = Wire.read();
  uint8_t zhm = Wire.read();
  uint8_t zlm = Wire.read();
  uint8_t yhm = Wire.read();
  uint8_t ylm = Wire.read();

  // h,l x,z,y lower 16 bits
  Vector3 mag_vals;
  mag_vals(1) =  float((int16_t)(xhm << 8 | xlm));  // flip x,y sign
  mag_vals(0) =  float((int16_t)(yhm << 8 | ylm));
  mag_vals(2) = -float((int16_t)(zhm << 8 | zlm));
  mag_vals = mag_scale*(mag_vals - mag_bias)*(MAG_FS/(2048));  // corrected readings

  last_mag_vec = mag_vals;
  return mag_vals;
}

Vector3 AccMag::get_filtered_acc(){
  if ((micros() - last_acc_timestamp) < ACC_REFRESH_PERIOD) return filtered_acc_vec; 
  Vector3 acc_reading = read_acc();
  filtered_acc_vec = acc_reading * ACCLPF_RATIO + filtered_acc_vec * (1 - ACCLPF_RATIO);
  return filtered_acc_vec;
};

Vector3 AccMag::get_filtered_mag(){
  if ((micros() - last_mag_timestamp) < MAG_REFRESH_PERIOD) return filtered_mag_vec; 
  Vector3 mag_reading = read_mag();
  filtered_mag_vec = mag_reading * MAGLPF_RATIO + filtered_mag_vec * (1 - MAGLPF_RATIO);
  return filtered_mag_vec;
};

void AccMag::calibrate_acc(){  // sensor needs to be perpendicular to gravity vector
  // assuming stationary IMU, we first obtain the direction of gravity
  Vector3 grav_dir = {0,0,0};
  delay(500);
  for (int i = 0; i < 50; i ++){
    grav_dir += read_acc()/50.0f;
    delayMicroseconds(ACC_REFRESH_PERIOD + 100);
  }
  grav_dir = normalize(grav_dir);  // get gravity vector of magnitude 1

  // zero out bias and add average of the samples to it
  acc_bias = {0,0,0};
  for (int i = 0; i < 100; i ++){
    acc_bias += (read_acc() - grav_dir)/100.0f;
    delayMicroseconds(ACC_REFRESH_PERIOD + 100);
  }
  Serial.println("Accelerometer calibrated");
}
