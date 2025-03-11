#include "NRF24.h"

Communication::Communication(){
  radio = RF24(17, 20);
  comm_timed_out = 1;
  new_roll_config = 0;
  new_pitch_config = 0;
  new_yaw_config = 0;
}

void Communication::setup_nrf(){
  Serial.println("Setting up communication module");

  if (!radio.begin(&SPI, 17, 20)) { // blocking 
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {} 
  }
  
  radio.setPALevel(RF24_PA_HIGH);
  radio.setChannel(3);
  radio.setAddressWidth(3);
  radio.setDataRate(RF24_2MBPS);
  radio.setAutoAck(0);
  radio.openWritingPipe(STATADD);
  radio.openReadingPipe(1, CMDADD); 
  radio.startListening();  
  last_ctrl_msg = 0;  
}

Control Communication::update_commands(float initial_yaw){  // receive latest command message with lost signal handling
  ctrl_msg_t new_ctrl_msg;
  static uint32_t ctrl_sequence_num = 0;
  
  bool new_command = false;
  msg_t msg;
  while (radio.available()) { // read latest 32 bytes
    radio.read(&msg, sizeof(msg_t)); 
    if (msg.type == 0){ // control message received
      // if (DEBUG) Serial.println("control received");
      ctrl_msg_count ++;
      last_ctrl_msg = millis(); 
      new_command = true;
      new_ctrl_msg = msg.data.ctrl_data; 
    }
    else if (msg.type == 1){ // config message received
      // if (DEBUG) Serial.println("config received");
      config_msg_t new_config = msg.data.config_data;
      if (new_config.axis == 0){  // roll
        roll_config.set(new_config.P, new_config.I, new_config.D, new_config.sat, new_config.LPc);
        new_roll_config = 1;
      }
      else if (new_config.axis == 1){  // pitch
        pitch_config.set(new_config.P, new_config.I, new_config.D, new_config.sat, new_config.LPc);
        new_pitch_config = 1;
      }
      else if (new_config.axis == 2){  // yaw
        yaw_config.set(new_config.P, new_config.I, new_config.D, new_config.sat, new_config.LPc);
        new_yaw_config = 1;
      }
    }
   
  }
  
  unsigned long no_command_period = millis() - last_ctrl_msg;
  if (no_command_period > 1000){  // if no ctrl message received in last 1 second, try to hover
    comm_timed_out = 1;
    latest_control.roll = 0;  // hover  TODO: calibrate for sensor not being perfectly aligned?
    latest_control.pitch = 0;  // hover  TODO: calibrate for sensor not being perfectly aligned?
    latest_control.throttle = max(0, latest_control.throttle - 0.0001);  // reduce throttle slowly
//    if(SAFETY && (no_command_period >= COMMAND_TIMEOUT_MS)) setpoint.alt = current_state.alt - 0.1;  // start landing slowly after COMMAND_TIMEOUT_MS without signal
  }
  else if (new_command){  // build setpoint from message
    comm_timed_out = 0;
    latest_control.roll = new_ctrl_msg.roll;
    latest_control.pitch = new_ctrl_msg.pitch;
    latest_control.yaw = constrain_angle(initial_yaw + new_ctrl_msg.yaw_diff);
    latest_control.throttle = new_ctrl_msg.throttle;
    latest_control.motors_on = new_ctrl_msg.motors_on;
    // check sequence number on the packet and compare to last seen
    if (++ctrl_sequence_num != new_ctrl_msg.sequence) {
      // if (DEBUG){
      //   Serial.print("Sequence mismatch: ");
      //   Serial.print(ctrl_sequence_num);
      //   Serial.print(" x ");
      //   Serial.println(new_ctrl_msg.sequence);
      // }
      ctrl_sequence_num = new_ctrl_msg.sequence;
    }

  }
  return latest_control;
};

telemetry_msg_t Communication::create_state_telemetry(State state, Vector4 control, float init_yaw, Vector3 PID_outputs){
  state_struct data;
  data.ms = millis();
  data.roll = angle_to_int(state.roll);
  data.pitch = angle_to_int(state.pitch);
  // data.yaw = current_state.yaw;
  data.yaw = angle_to_int(constrain_angle(state.yaw - init_yaw));  // send yaw difference instead of yaw
  
  data.motors[0] = action_to_int(control(0));
  data.motors[1] = action_to_int(control(1));
  data.motors[2] = action_to_int(control(2));
  data.motors[3] = action_to_int(control(3));

  data.PID_outputs[0] = force_to_int(PID_outputs(0));
  data.PID_outputs[1] = force_to_int(PID_outputs(1));
  data.PID_outputs[2] = force_to_int(PID_outputs(2));

  telemetry_msg_t msg;
  msg.data.state_data = data;
  msg.type = 0;
  return msg;
}

telemetry_msg_t Communication::create_batt_telemetry(State state, Measurements m){
  sensor_struct data;
  data.height = state.alt;
  data.battery = m.battery;
  telemetry_msg_t msg;
  msg.type = 1;
  msg.data.sensor_data = data;
  return msg;
}

telemetry_msg_t Communication::create_sensor_telemetry(State state, float init_yaw, Measurements measured_values){
  ekf_struct data;
  data.ms = millis();
  data.roll = angle_to_int(state.roll);
  data.pitch = angle_to_int(state.pitch);
  // data.yaw = current_state.yaw;
  data.yaw = angle_to_int(constrain_angle(state.yaw - init_yaw));  // send yaw difference instead of yaw
  
  data.acc[0] = acc_to_int(measured_values.acc_vec(0));
  data.acc[1] = acc_to_int(measured_values.acc_vec(1));
  data.acc[2] = acc_to_int(measured_values.acc_vec(2));

  data.mag[0] = mag_to_int(measured_values.mag_vec(0));
  data.mag[1] = mag_to_int(measured_values.mag_vec(1));
  data.mag[2] = mag_to_int(measured_values.mag_vec(2));

  data.gyro[0] = gyro_to_int(measured_values.gyro_vec(0));
  data.gyro[1] = gyro_to_int(measured_values.gyro_vec(1));
  data.gyro[2] = gyro_to_int(measured_values.gyro_vec(2));

  telemetry_msg_t msg;
  msg.data.ekf_data = data;
  msg.type = 2;
  return msg;
}

void Communication::send_telemetry(telemetry_msg_t msg){
  radio.stopListening(); 
  // radio.flush_tx();
  // bool report = radio.startWrite(&msg, sizeof(msg), 0); 
  // delayMicroseconds(500);
  // radio.startListening();
  // radio.flush_rx();
  bool report = radio.write(&msg, sizeof(msg)); 
  if (report & (msg.type == 1)) ctrl_msg_count = 0;  // reset counter on success, else try again next loop
  radio.startListening(); 
}
