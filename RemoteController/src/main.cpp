#include <Arduino.h>
#include <FastLED.h>

#include "comms.h"
#include "serial_comm.h"
#include "comm_structs.h"
#include "config.h"
#include "peripherals.h"

CRGB leds[1];
Inputs input_manager = Inputs();
Communication Comms = Communication();

void setup() {
  
  FastLED.addLeds<WS2812B, WSLED_PIN, GRB>(leds, 1);  // GRB ordering is typical
  Serial.begin(500000);
  while (!Serial){}
  Comms.setup_nrf();

  Serial.println("Setup done");
}

uint8_t motors_mode = 1;
uint8_t motors_on = 0;
Joysticks smooth_joys = {0,0,0,0};
float yaw_diff_setpoint = 0;

uint32_t last_cmd_t = 0;
uint32_t last_telem_t = 0;
uint32_t sequence_num = 0;

void loop() {  

  input_manager.update_readings();
  if (input_manager.button2_rising) motors_on = !motors_on;
  if (input_manager.button1_rising) motors_mode = 1 + (motors_mode) % 5;
  Joysticks joys = input_manager.current_joy;
  Joysticks mapped_joys = input_manager.map_joysticks(joys);

  // leds control
  int32_t telem_delay = millis() - last_telem_t;
  float led_brightness = min(1, max(500 - telem_delay, 0)/500.0f);
  analogWrite(LED1_PIN, int(led_brightness * 255));  // green led indicates signal quality
  leds[0] = CHSV(51*(motors_mode-1), 255, int(motors_on*(50 + 80*(input_manager.pot_reading))));
  FastLED.show();
  
  if ((millis() - last_cmd_t) >= 50){
    smooth_joys = input_manager.smooth_joysticks(mapped_joys, 0.7);
    if (abs(smooth_joys.X) < 1) smooth_joys.X = 0;  // discard small angles
    yaw_diff_setpoint -= smooth_joys.X;
    float alt_diff = smooth_joys.Y/100.0f;
    // if (motors_mode == 2) {
      // smooth_joys.X2 = 30.0f * smooth_joys.X2;
      // smooth_joys.Y2 = 30.0f * smooth_joys.Y2;
    // }
    ctrl_msg_t ctrl_msg = { smooth_joys.X2, smooth_joys.Y2, 
                            int16_t(yaw_diff_setpoint*32.0f), int16_t(input_manager.pot_reading*1024.0f), 
                            alt_diff, uint8_t(motors_on*motors_mode), ++sequence_num};
    last_cmd_t = millis();
    bool report = Comms.send_ctrl(ctrl_msg);
    if (DEBUG) Serial.println("Sending commands"); 
    if (PRINT_COMMANDS) print_commands(ctrl_msg);
  }

  // parse config settings from serial and send
  msg_t msg; 
  if ((millis() - last_cmd_t) >= 40){
    if (parse_serial(msg)){
      bool report = Comms.send_config(msg);
      if (report && DEBUG) Serial.println("Config sent");
    }
  }

  // receive message from drone and print to serial 
  telemetry_msg_t received_msg;
  if (Comms.receive_msg(received_msg)){
    last_telem_t = millis();
    if (DEBUG) Serial.println("Received message");
    if (STATE2SERIAL) print_message_to_serial(received_msg);  // send to processing for rendering
  }

}