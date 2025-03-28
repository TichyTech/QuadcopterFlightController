// https://www.pololu.com/file/0J491/L3G4200D.pdf
// https://www.elecrow.com/download/L3G4200_AN3393.pdf
// L3G4200D gyro
#include "L3G4200D.h"

#define GYRO_ADR 0x69
#define DPS_PER_LSB 0.01526f  // for +-500 dps FS assuming 16 bits of data
// #define DPS_PER_LSB 0.0610351f  // for +- 2000 dps FS
#define ROLL_MULT 1.1723333f  // sensitivity calibration EKF
#define PITCH_MULT 1.21134444444f  // sensitivity calibration EKF
#define YAW_MULT 1.1556f  // sensitivity calibration 
#define GYRO_REFRESH_RATE 400.0f  // for 800 Hz
#define GYRO_REFRESH_PERIOD (1.0f/GYRO_REFRESH_RATE) // in seconds

// dynamic bias compensation definitions
#define CUTOFF_FREQUENCY (0.02f)
#define THRESHOLD (4.0f)
#define TIMEOUT (300)  // milliseconds 

Gyro::Gyro(){
  gyro_bias = {0,0,0};
  last_gyro_vec = {0,0,0};
  gyro_timeout = 0;
  last_gyro_timestamp = 0;

  // dynamic bias compensation initialization
  bias_compensation_on = 0;
  dynamic_bias = {0,0,0};
  bias_timer = 0;

  stream_buff[0] = {0,0,0};
  stream_buff[1] = {0,0,0};
  stream_buff[2] = {0,0,0};
  stream_buff[3] = {0,0,0};
}

void Gyro::setup_gyro(){
  setup_gyro_stream();
}

void Gyro::setup_gyro_bypass() {
  Serial.println("Setting up gyroscope to BYPASS mode");
  Wire.beginTransmission(GYRO_ADR);
  byte error = Wire.endTransmission();
  if(error){
    Serial.println("Gyroscope not responding");
    while(1){}
  }

  // Wire.beginTransmission(GYRO_ADR);
  // Wire.write(0x0F | (1 << 7));
  // Wire.endTransmission();
  // Wire.requestFrom(GYRO_ADR, (byte)1);
  // while (!Wire.available()) {}
  // uint8_t whoami = Wire.read();
  

  writeReg(GYRO_ADR, 0x20, 0x8F);  // 400 Hz, LP 20 Hz Cutoff
  writeReg(GYRO_ADR, 0x21, 0x29);  // Normal mode HPRESETFILTER, HP 0.1 Hz Cutoff
  writeReg(GYRO_ADR, 0x23, 0x90);  // Block update until read, FS 500 dps 
//  writeReg(GYRO_ADR, 0x23, 0xB0);  // Block update until read, FS 2000 dps 
//  writeReg(GYRO_ADR, 0x20, 0xEF);  // 800 Hz, LP 50 Hz Cutoff
//  writeReg(GYRO_ADR, 0x20, 0x6F);  // 200 Hz, 50 Hz Cutoff
  // writeReg(GYRO_ADR, 0x20, 0xFF);  // 800 Hz, LP 110 Hz Cutoff
  // writeReg(GYRO_ADR, 0x20, 0xCF);  // 800 Hz, LP 30 Hz Cutoff
  // writeReg(GYRO_ADR, 0x20, 0xEF);  // 800 Hz, LP 50 Hz Cutoff
//  writeReg(GYRO_ADR, 0x21, 0x09);  // Normal mode, 1 Hz Cutoff

  filtered_gyro_vec = read_gyro_single();
}

void Gyro::setup_gyro_stream() {
  Serial.println("Setting up gyroscope to BYPASS mode");
  Wire.beginTransmission(GYRO_ADR);
  byte error = Wire.endTransmission();
  if(error){
    Serial.println("Gyroscope not responding");
    while(1){}
  }

  // writeReg(GYRO_ADR, 0x20, 0x8F);  // 400 Hz, LP 110 Hz Cutoff
  // writeReg(GYRO_ADR, 0x21, 0x29);  // Normal mode HPRESETFILTER, HP 0.1 Hz Cutoff
  // writeReg(GYRO_ADR, 0x23, 0x90);  // Block update until read, FS 500 dps 
  // writeReg(GYRO_ADR, 0x24, 0x40);  // EN_FIFO
  // writeReg(GYRO_ADR, 0x2E, 0x40);  // FIFO mode Stream

  writeReg(GYRO_ADR, 0x20, 0xBF);  // 400 Hz, LP 110 Hz Cutoff
  writeReg(GYRO_ADR, 0x21, 0x20);  // Normal mode, HP 30 Hz Cutoff (for 400Hz ODR)
  writeReg(GYRO_ADR, 0x23, 0x90);  // Block update until read, FS 500 dps 
  writeReg(GYRO_ADR, 0x24, 0x50);  // EN_FIFO, HPen
  writeReg(GYRO_ADR, 0x2E, 0x40);  // FIFO mode Stream

  filtered_gyro_vec = read_gyro_single();
}

/**
 * convert 6 byte buffer from the gyro to correctly scaled float Vector3 and static bias corrected
 */
Vector3 Gyro::buff_to_Vec(uint8_t* buff){
  uint8_t xlg = buff[0];
  uint8_t xhg = buff[1];
  uint8_t ylg = buff[2];
  uint8_t yhg = buff[3];
  uint8_t zlg = buff[4];
  uint8_t zhg = buff[5];

  Vector3 gyro_vals;
  gyro_vals(1) =  PITCH_MULT*float((int16_t)(xhg << 8 | xlg));
  gyro_vals(0) =  ROLL_MULT*float((int16_t)(yhg << 8 | ylg));
  gyro_vals(2) =  -YAW_MULT*float((int16_t)(zhg << 8 | zlg));
  gyro_vals = gyro_vals*DPS_PER_LSB - gyro_bias;  // corrected reading
  return gyro_vals;
}

int16_t stream_buff_idx = 0;
Vector3 Gyro::read_gyro_stream(){
  // takes cca 140 + 240*reading us to complete
  Wire.beginTransmission(GYRO_ADR);
  Wire.write(0x2F);  // FIFO_SRC_REG 
  Wire.endTransmission();
  Wire.requestFrom(GYRO_ADR, (byte)1);
  while (!Wire.available()) {}
  byte src_reg = Wire.read();
  uint8_t num_readings = (src_reg & 0x1F);

  if (num_readings > 0){
    Wire.beginTransmission(GYRO_ADR);
    Wire.write(0x28 | (1 << 7));
    Wire.endTransmission();
    Wire.requestFrom(GYRO_ADR, (byte)(6*num_readings));  // request all readings  
    for (int i = 0; i < num_readings; i++){
      while (Wire.available() < 6) {}  // wait for data
      stream_buff_idx = (stream_buff_idx + 1) % 4;
      uint8_t received_data[6];
      for (int j = 0; j < 6; j++) received_data[j] = Wire.read();
      stream_buff[stream_buff_idx] = buff_to_Vec(received_data);
    }
  }

  Vector3 gyro_vals = {0,0,0};
  float coeffs[4] = {0.25, 0.25, 0.25, 0.25};
  for (int off = 0; off < 4; off++){
    uint8_t buff_idx = (stream_buff_idx - off + 4) % 4;
    gyro_vals += coeffs[off] * stream_buff[buff_idx];
  }
  return gyro_vals;
}

Vector3 Gyro::read_gyro_single() {
  uint32_t current_micros = micros();
  if ((current_micros - last_gyro_timestamp) < GYRO_REFRESH_PERIOD*1000000) return last_gyro_vec; 
  
  uint8_t received_data[6];
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

  for (int i = 0; i < 6; i++) received_data[i] = Wire.read();
  Vector3 gyro_vals = buff_to_Vec(received_data);

  // dynamic bias compensation implementation
  if (bias_compensation_on){  // according to Madgwick
    gyro_vals = gyro_vals - dynamic_bias;
    if (abs(gyro_vals(0)) < THRESHOLD || abs(gyro_vals(1)) < THRESHOLD || abs(gyro_vals(2)) < THRESHOLD){  // omega in range
      bias_timer += (current_micros - last_gyro_timestamp)/1000;
      if (bias_timer > TIMEOUT){
        dynamic_bias = dynamic_bias + gyro_vals * (2 * PI_F * CUTOFF_FREQUENCY / GYRO_REFRESH_RATE);  // 2 pi f_c / f_s
      }
    }
    else bias_timer = 0;  // omega outside of range

  }

  last_gyro_vec = gyro_vals;
  last_gyro_timestamp = current_micros;
  return gyro_vals; 
}

Vector3 Gyro::get_filtered_gyro(){
  if ((micros() - last_gyro_timestamp) < GYRO_REFRESH_PERIOD*1000000) return filtered_gyro_vec; 
  Vector3 gyro_reading = read_gyro_stream();
  filtered_gyro_vec = gyro_reading * GYROLPF_RATIO + filtered_gyro_vec * (1 - GYROLPF_RATIO);
  return filtered_gyro_vec;
};

void Gyro::calibrate_gyro(){
  gyro_bias = {0,0,0};
  Vector3 data = {0,0,0};
  delay(2000);
  for (int i = 0; i < 100; i ++){
    data += read_gyro_single()/100.0f;
    delayMicroseconds(GYRO_REFRESH_PERIOD*1000000 + 100);
  }
  gyro_bias = data;
  Vector3 gyro_sigma = {0,0,0}; 
  for (int i = 0; i < 100; i ++){
    Vector3 meas = read_gyro_single();
    Vector3 squares = {meas(0)*meas(0), meas(1)*meas(1), meas(2)*meas(2)};
    gyro_sigma += squares/(99.0f);
    delayMicroseconds(GYRO_REFRESH_PERIOD*1000000 + 100);
  }
  Serial.print("Gyro sigma: ");
  printVec3(gyro_sigma, 2);
  // printVec3(gyro_bias, 3);
  Serial.println("Gyro calibrated");
  bias_compensation_on = 0;  // turn on dynamic bias compensation from here onward
}
