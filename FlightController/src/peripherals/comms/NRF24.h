#include <RF24.h>
#include <nRF24L01.h> 
#include "algebra.h"
#include "config.h"
#include <SPI.h>
#include "comm_structs.h"

const byte CMDADD[4] = "DRR";  // receive address
const byte STATADD[4] = "DRT";  // transmit address

class Communication{
  private:
    RF24 radio;
  public:
    long last_ctrl_ms;  // last time we received control
    bool comm_timed_out;  // ctrl delay exceeded timeout threshold
    int16_t batt_telem_countdown;
    PID_config roll_config;  // newest received config
    PID_config pitch_config;  // newest received config
    PID_config yaw_config;  // newest received config
    PID_config alt_config;  // newest received config
    bool new_roll_config;  // new config arrived flag
    bool new_pitch_config;  // new config arrived flag
    bool new_yaw_config;  // new config arrived flag
    bool new_alt_config;  // new config arrived flag
    Control latest_control;

    Communication();

    void setup_nrf();
    void push_config(config_msg_t new_config);
    PID_config pop_config(uint8_t axis);
    Control update_commands(float initial_yaw);
    telemetry_msg_t create_state_telemetry(State state, Vector4 control, float init_yaw, Vector3 PID_outputs, Control ref);
    telemetry_msg_t create_sensor_telemetry(State state, float init_yaw, Measurements measured_values);
    telemetry_msg_t create_batt_telemetry(State state, Measurements m);
    void send_telemetry(telemetry_msg_t msg);

};