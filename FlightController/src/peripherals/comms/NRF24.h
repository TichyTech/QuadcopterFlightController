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
    PID_config roll_config;
    PID_config pitch_config;
    PID_config yaw_config;
    bool new_roll_config;
    bool new_pitch_config;
    bool new_yaw_config;
    Control latest_control;

    Communication();

    void setup_nrf();
    Control update_commands(float initial_yaw);
    telemetry_msg_t create_state_telemetry(State state, Vector4 control, float init_yaw, Vector3 PID_outputs, Control ref);
    telemetry_msg_t create_sensor_telemetry(State state, float init_yaw, Measurements measured_values);
    telemetry_msg_t create_batt_telemetry(State state, Measurements m);
    void send_telemetry(telemetry_msg_t msg);

};