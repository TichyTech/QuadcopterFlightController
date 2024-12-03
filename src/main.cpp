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
  if (DEBUG){  // wait until serial is open
    while(!Serial){}
  }
  Serial.println("Starting up");

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

  Serial.println("Setup finished!");

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

uint16_t loop_counter = 0;

void loop() {  // approxx 0.85 ms per loop 
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
  
  measured_values = sensors.get_measurements_filtered();  // update measurements from gyro, acc, mag and alt (and dt)

  if (sensors.imu_timed_out || sensors.alt_timed_out || comm.comm_timed_out) digitalWrite(REDLED_PIN, HIGH); // indicate failure
  else digitalWrite(REDLED_PIN, LOW);

  if (SAFETY){
    if (sensors.imu_timed_out || comm.comm_timed_out) motors_on = 0;
  }

  if (motors_on > 0) digitalWrite(GREENLED_PIN, HIGH);  // light up green if motors run
  else digitalWrite(GREENLED_PIN, LOW);

  // estimated_DCM = acc_mag2DCM(measured_values); // init DCM
  estimated_DCM = update_DCM_rejection(estimated_DCM, measured_values);  // update DCM matrix
  controller.update_DCM(estimated_DCM);
  control_action = controller.update_motor_percentages(ctrl_commands, measured_values);

  if (motors_on == 0) signal_motors(zero_4vector); 
  else signal_motors(control_action);
}

void loop1() {
  if ((millis() - comm.last_ctrl_msg) >= 90){  // more than 90 milliseconds from last command
    Control received_commands = comm.update_commands(initial_yaw);  // receive new command 
    commands = received_commands;
  }
  else{  // less than 90 milliseconds from last command ==> we can send telemetry
    current_state = compute_state(estimated_DCM, measured_values);  // initial state for reference
    if (TELEMETRY && (comm.ctrl_msg_count > 50)) {  // send some data back to controller every 50 ctrl cycles 
      telemetry_msg_t msg = comm.create_batt_telemetry(current_state, measured_values);
      comm.send_telemetry(msg);  
    }
    // the following telemetry takes about 0.9 ms
    telemetry_msg_t msg = comm.create_state_telemetry(current_state, control_action, initial_yaw, controller.last_PID_outputs);
    comm.send_telemetry(msg);
  }

}