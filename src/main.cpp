#include <Arduino.h>
#include "pico/stdlib.h"
#include <math.h>

#include <Wire.h>
#include <SPI.h>

#include "config.h"
#include "definitions.h"
#include "debug/debugging.h"
#include "peripherals/sensors.h"
#include "peripherals/comms/NRF24.h"

#include "control/controller.h"
#include "control/filter.h"
#include "peripherals/motors/motors.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////

Measurements measured_values;  // struct to store all measured values
Matrix3 estimated_DCM;  // matrix to store current estimated attitude
State current_state;
Control commands;
Vector4 control_action;

unsigned long start_time = 0;
float initial_yaw = 0;
uint32_t motors_on = 0;

Sensors sensors = Sensors();
Communication comm = Communication();
Controller controller = Controller();

void setup() {
  // Start serial
  Serial.begin(115200);
  Serial.println("Starting up");
  if (DEBUG){  // wait until serial is open
    while(!Serial){}
  }

  // Start Wire for gyro, altimeter, accelerometer and magnetometer
  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();
  Wire.setClock(400000);

  // Start SPI for NRF24
  SPI.setRX(16);
  SPI.setSCK(18);
  SPI.setTX(19);
  SPI.begin();

  sensors.setup();  // configure all peripherals
  comm.setup_nrf();  // set up nrf24

  mount_motors();  // mount motor pins
  setup_motor_pio();  // program and initialize pio state machines for motor control

  pinMode(REDLED_PIN, OUTPUT);
  pinMode(BLUELED_PIN, OUTPUT);
  pinMode(GREENLED_PIN, OUTPUT);
  pinMode(YELLOWLED_PIN, OUTPUT);

  arm_motors();  // send non-zero throttle to the motors to arm them
  measured_values = sensors.get_measurements_filtered();
  
  if (SAFETY){
    while (sensors.imu_timed_out){
      Serial.println("IMU timeout");
    }
  }

  // start of the control loop
  estimated_DCM = acc_mag2DCM(measured_values); // init DCM
  current_state = compute_state(estimated_DCM, measured_values);  // initial state for reference
  initial_yaw = current_state.yaw;  // store initial yaw for control
  start_time = millis();
}

void setup1(){
  while(!start_time){  // wait for setup on the first core to finish
      delay(200);
  }
}

unsigned long last_ctrl_msg;  // for lost signal detection

void loop() {  // approxx 1.5 ms per loop 

  while(digitalRead(SWITCH_PIN)) {  // stop motors and blink red LED if switch is on
    signal_motors(zero_4vector);  
    digitalWrite(REDLED_PIN, 1);  
    for (int i = 0; i < 100; i++){
      signal_motors(zero_4vector);  
      delay(2);
    }
    digitalWrite(REDLED_PIN, 0);  
    for (int i = 0; i < 100; i++){
      signal_motors(zero_4vector); 
      delay(2);
    }  
  }

  Control ctrl_commands = commands;
  motors_on = ctrl_commands.motors_on;
  
  measured_values = sensors.get_measurements_filtered();  // update measurements from gyro, acc, mag and alt (and integration_period time constant)

  if (sensors.imu_timed_out || sensors.alt_timed_out || comm.comm_timed_out) digitalWrite(REDLED_PIN, HIGH); // indicate failure
  else digitalWrite(REDLED_PIN, LOW);  
  if (SAFETY){
    if (sensors.imu_timed_out || comm.comm_timed_out) motors_on = 0;
  }

  if (motors_on > 0) digitalWrite(GREENLED_PIN, HIGH);  // light up green if motors run
  else digitalWrite(GREENLED_PIN, LOW);

  estimated_DCM = update_DCM(estimated_DCM, measured_values);  // update DCM matrix
  controller.update_DCM(estimated_DCM);
  control_action = controller.update_motor_percentages(ctrl_commands, measured_values);

  if (motors_on == 0) signal_motors(zero_4vector); 
  else signal_motors(control_action);
}

void loop1() {
  if ((millis() - last_ctrl_msg) >= 90){  // more than 90 milliseconds from last command
    Control received_commands = comm.update_commands(initial_yaw);  // receive new command 
    commands = received_commands;
  }
  else{  // less than 90 milliseconds from last command ==> we can send telemetry
    current_state = compute_state(estimated_DCM, measured_values);  // initial state for reference
    if (TELEMETRY && (comm.ctrl_msg_count > 50)) {
      sensor_struct data;
      data.height = current_state.alt;
      data.battery = measured_values.battery;
      telemetry_msg_t msg;
      msg.type = 1;
      msg.data.sensor_data = data;
      comm.send_telemetry(msg);  // send some data back to controller every 50 ctrl cycles 
    }

    state_struct data;
    data.ms = millis();
    data.roll = current_state.roll;
    data.pitch = current_state.pitch;
    // data.yaw = current_state.yaw;
    data.yaw = constrain_angle(current_state.yaw - initial_yaw);  // send yaw difference instead of yaw
    
    data.motors[0] = uint16_t(control_action(0)*65535);
    data.motors[1] = uint16_t(control_action(1)*65535);
    data.motors[2] = uint16_t(control_action(2)*65535);
    data.motors[3] = uint16_t(control_action(3)*65535);

    telemetry_msg_t msg;
    msg.data.state_data = data;
    msg.type = 0;
    comm.send_telemetry(msg);
  }

}