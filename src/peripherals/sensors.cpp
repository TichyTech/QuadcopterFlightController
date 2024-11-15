#include "sensors.h"

#define ADC_TO_VOLTS 0.0045923
#define ADC_OFFSET 35

Battery::Battery(){
  filtered_val = 12.4;
}

float Battery::get_voltage(){
  float voltage = ADC_TO_VOLTS * (analogRead(ADC1_PIN) - ADC_OFFSET);
  return voltage;
}

float Battery::get_filtered_voltage(){
  float current_reading = get_voltage();
  filtered_val = (ADC_TO_VOLTS * current_reading) * BATLPF_RATIO + filtered_val*(1 - BATLPF_RATIO);
  return filtered_val;
}

Sensors::Sensors(){

  gyroscope = Gyro();
  accmag = AccMag();
  altimeter = Altimeter();
  battery = Battery();

  imu_timed_out = false;
  alt_timed_out = false;
}

void Sensors::setup(){
  Serial.println("Setting up peripherals");

  gyroscope.setup_gyro(); // blocking
  accmag.setup_acc();  // blocking
  accmag.setup_mag();  // blocking
  altimeter.setup_alt();  // blocking

  Serial.println("Mounting ADCs and buttons");
  analogReadResolution(12);
  pinMode(ADC1_PIN, INPUT);
  pinMode(ADC2_PIN, INPUT);
  pinMode(SWITCH_PIN, INPUT);
  
  Serial.println("All sensors ready!");

  delay(5000); //delay the calibration process a bit to allow drone to settle on the ground
  gyroscope.calibrate_gyro(); // 3 sec bias calibration
  accmag.calibrate_acc();  // 3 sec bias calibration
}

Measurements Sensors::get_measurements(){  // init all readings to bypass low pass filters in update_measurements()
  Measurements new_m;
  new_m.acc_vec = accmag.read_acc(); 
  new_m.mag_vec = accmag.read_mag();
  new_m.gyro_vec = gyroscope.read_gyro();
  new_m.altitude = altimeter.read_alt();
  new_m.battery = battery.get_voltage();
  new_m.integration_period = update_integration_period();  // first integration period is invalid!!

  imu_timed_out = accmag.acc_timeout || accmag.mag_timeout || gyroscope.gyro_timeout;
  alt_timed_out = altimeter.alt_timed_out;

  return new_m;
}

Measurements Sensors::get_measurements_filtered(){
  Measurements new_m;
  new_m.acc_vec = accmag.get_filtered_acc();  // low pass filter for accelerometer
  new_m.mag_vec = accmag.get_filtered_mag();
  new_m.gyro_vec = gyroscope.get_filtered_gyro();
  new_m.altitude = altimeter.get_filtered_alt();  // low pass filter for altimeter
  new_m.battery = battery.get_filtered_voltage();
  new_m.integration_period = update_integration_period();  // record microseconds since last measurement

  imu_timed_out = accmag.acc_timeout || accmag.mag_timeout || gyroscope.gyro_timeout;
  alt_timed_out = altimeter.alt_timed_out;

  return new_m;
}

float Sensors::update_integration_period(){
  // first integration period is invalid!!
  static unsigned long last_timestamp;  
  unsigned long current_timestamp = micros();
  float integration_period = float(current_timestamp - last_timestamp)/1000000.0;
  last_timestamp = micros();
  return integration_period;
}
