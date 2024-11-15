#include "NRF24.h"

Communication::Communication(){
  radio = RF24(17, 20);
  if (!radio.begin(&SPI, 17, 20)) { // blocking 
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {} 
  }
  comm_timed_out = 0;
  new_roll_config = 0;
  new_pitch_config = 0;
  new_yaw_config = 0;
}

void Communication::setup_nrf(){
  Serial.println("Setting up communication module");
  radio.setPALevel(RF24_PA_HIGH);
  radio.setChannel(3);
  radio.setAddressWidth(3);
  radio.setDataRate(RF24_2MBPS);
  radio.setAutoAck(0);
  radio.openWritingPipe(STATADD);
  radio.openReadingPipe(1, CMDADD); 
  radio.startListening();  
  last_ctrl_msg = millis();  
}

Control Communication::update_commands(float initial_yaw){  // receive latest command message with lost signal handling
  ctrl_msg_t new_ctrl_msg;
  
  bool new_command = false;
  msg_t msg;
  while (radio.available()) { // read latest 32 bytes
    radio.read(&msg, sizeof(msg_t)); 
    if (msg.type == 0){ // control message received
      ctrl_msg_count ++;
      last_ctrl_msg = millis(); 
      new_command = true;
      new_ctrl_msg = msg.data.ctrl_data; 
    }
    else if (msg.type == 1){ // config message received
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
  }
  return latest_control;
};

void Communication::send_telemetry(telemetry_msg_t msg){
  radio.stopListening(); 
  bool report = radio.write(&msg, sizeof(telemetry_msg_t));  // send message
  if (report & (msg.type == 1)) ctrl_msg_count = 0;  // reset counter on success, else try again next loop
  radio.startListening(); 
}
