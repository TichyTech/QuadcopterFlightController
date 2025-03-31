#include "serial_comm.h"

/**
 * take received state_data struct, remap values to original ranges and format into string 
 */
void format_state(state_struct s_data, char* buffr, size_t bsize){

  float r = angle_to_float(s_data.roll);
  float p = angle_to_float(s_data.pitch);
  float y = angle_to_float(s_data.yaw);

  float motors[4] = { action_to_float(s_data.motors[0]),
                      action_to_float(s_data.motors[1]),
                      action_to_float(s_data.motors[2]),
                      action_to_float(s_data.motors[3])};

  float PID_outputs[3] = {force_to_float(s_data.PID_outputs[0]),
                          force_to_float(s_data.PID_outputs[1]),
                          force_to_float(s_data.PID_outputs[2])};

  float ref[2] = {angle_to_float(s_data.ref[0]),
                  angle_to_float(s_data.ref[1])};

  float all_floats[12] = {r, p, y, motors[0], motors[1], motors[2], motors[3], PID_outputs[0], 
                          PID_outputs[1], PID_outputs[2], ref[0], ref[1]};

  // format floats into string
  snprintf(buffr, bsize, "State: %lu ",  s_data.ms);
  format_floats_to_buffer(buffr, all_floats, 12);
}

/**
 * take received ekf_struct struct, remap values to original ranges and format into string 
 */
void format_ekf(ekf_struct ekf_data, char* buffr, size_t bsize){

  float r = angle_to_float(ekf_data.roll);
  float p = angle_to_float(ekf_data.pitch);
  float y = angle_to_float(ekf_data.yaw);

  float acc[3] = {  acc_to_float(ekf_data.acc[0]),
                    acc_to_float(ekf_data.acc[1]),
                    acc_to_float(ekf_data.acc[2])};

  float mag[3] = {  mag_to_float(ekf_data.mag[0]),
                    mag_to_float(ekf_data.mag[1]),
                    mag_to_float(ekf_data.mag[2])};

  float gyro[3] = { gyro_to_float(ekf_data.gyro[0]),
                    gyro_to_float(ekf_data.gyro[1]),
                    gyro_to_float(ekf_data.gyro[2])};

  float all_floats[12] = {r, p, y, acc[0], acc[1], acc[2], mag[0], mag[1], mag[2], gyro[0], gyro[1], gyro[2]};

  // format floats into string
  snprintf(buffr, bsize, "EKF: %lu ",  ekf_data.ms);
  format_floats_to_buffer(buffr, all_floats, 12);
}

/**
 * add float array to string with spaces between
 */
void format_floats_to_buffer(char* buffr, float* all_floats, uint8_t num_floats){
  char temp_b[16]; 
  int str_len = strlen(buffr);
  for (int i = 0; i < num_floats; i++){
    dtostrf(all_floats[i], 3, 2, temp_b);
    int float_len = strlen(temp_b);
    for (int j = 0; j < float_len; j++){
      buffr[str_len + j] = temp_b[j];  // copy float to string
    }
    buffr[str_len + float_len] = ' ';  // set a space
    buffr[str_len + float_len + 1] = '\0';  // set a string end char
    str_len += float_len + 1;
  }
}

/**
 * Interpret telemetry message and print it to serial in either human readable or binary format
 */
void print_message_to_serial(telemetry_msg_t tele_msg){
  static uint32_t last_shown = 0;
  if (tele_msg.type == 0){  // state data
    #if PRINT_MIN_DELAY > 0  // wait at least MIN_DELAY before printing telemetry
    uint32_t current_t = millis();
    if (current_t - last_shown < PRINT_MIN_DELAY) return;
    last_shown = current_t;
    #endif
    
    if (HUMAN_READABLE){
        char buffr[100];
        state_struct s_data = tele_msg.data.state_data; 
        format_state(s_data, buffr, sizeof(buffr));  //1168 micros
        Serial.println(buffr);  // 828 micros
    }
    else Serial.write((uint8_t*)&tele_msg, sizeof(tele_msg));
  }
  else if (tele_msg.type == 1){  // sensor data
    if (HUMAN_READABLE){
        sensor_struct s_data = tele_msg.data.sensor_data;
        Serial.println("Telemetry: " + String(s_data.battery) + " " + String(s_data.height));
    }
    else Serial.write((uint8_t*)&tele_msg, sizeof(tele_msg));
  }
  else if (tele_msg.type == 2){  // ekf data
    if (HUMAN_READABLE){
        char buffr[160];
        ekf_struct ekf_data = tele_msg.data.ekf_data; 
        format_ekf(ekf_data, buffr, sizeof(buffr));  
        Serial.println(buffr);  
    }
    else Serial.write((uint8_t*)&tele_msg, sizeof(tele_msg));
  }
}

bool parse_serial(msg_t& msg){
  if (!(Serial.available() > 0)) return 0;  // no message
  String serial_line = Serial.readStringUntil('\n');  // serial line
  uint8_t space_loc = serial_line.indexOf(' ');  // first space
  String msg_type = serial_line.substring(0, space_loc);
  if (!(msg_type == "roll" || msg_type == "pitch" || msg_type == "yaw")) return 0;  // no message
  config_msg_t new_config = config_from_line(serial_line);

  if (msg_type == "roll"){
    new_config.axis = 0;
    if (DEBUG) Serial.print("Sending Roll config: ");
  }
  else if (msg_type == "pitch"){
    new_config.axis = 1;
    if (DEBUG) Serial.print("Sending Pitch config: ");
  }
  else if (msg_type == "yaw"){
    new_config.axis = 2;
    if (DEBUG) Serial.print("Sending Yaw config: ");
  }
  else if (msg_type == "alt"){
    new_config.axis = 3;
    if (DEBUG) Serial.print("Sending Alt config: ");
  }
  if (DEBUG){
    String text = "P " + String(new_config.P, 2) + ", I " + String(new_config.I, 2) + 
    ", D " + String(new_config.D, 2) + ", sat " + String(new_config.sat, 2) + ", LPc " + String(new_config.LPc, 2);
    Serial.println(text);
  }

  // sent config message
  msg.type = 1;
  msg.data.config_data = new_config;
  return 1;
}

config_msg_t config_from_line(String serial_line){
  uint8_t space_loc = serial_line.indexOf(' ');  // first space
  float serial_reading[5] = {0,0,0,0,0};

  for (int i = 0; i < 5; i++){
    int next_space = serial_line.indexOf(' ', space_loc + 1);
    if (next_space == -1) next_space = serial_line.length() - 1;
    String numero = serial_line.substring(space_loc, next_space);
    serial_reading[i] = numero.toFloat();
    space_loc = next_space;
  } 

  float P = serial_reading[0];
  float I = serial_reading[1];
  float D = serial_reading[2];
  float sat = serial_reading[3];
  float LPc = serial_reading[4];
  // put into config message and print possibly
  config_msg_t config = {P, I, D, sat, LPc, 0};
  return config;
}

void print_commands(ctrl_msg_t ctrl_msg){
  Serial.print(ctrl_msg.roll);
  Serial.print(" ");
  Serial.print(ctrl_msg.pitch);
  Serial.print(" ");
  Serial.print(ctrl_msg.yaw_diff);
  Serial.print(" ");
  Serial.print(ctrl_msg.throttle);
  Serial.print(" ");
  Serial.print(ctrl_msg.motors_on);
  Serial.println();
}