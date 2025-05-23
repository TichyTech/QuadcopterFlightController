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
#include "control/kalman_filter/kalman.h"
#include "peripherals/motors/motors.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////

Measurements measured_values;  // struct to store all measured values
Matrix3 estimated_DCM;  // matrix to store current estimated attitude
State current_state;
Vector4 q;  // current attitude quaternion
Control commands;
Vector4 control_action = {0,0,0,0}; 

uint32_t start_time = 0;
float initial_yaw = 0;
uint32_t motors_on = 0;

Sensors sensors = Sensors();
Communication comm = Communication(); 
Controller controller = Controller();
KalmanFilter k_filter = KalmanFilter();

/////////////////////////////////////////////////////////////////////////////////////////////////////////

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
  Serial.println("Init measurements: Gyro ");
  printVec3(measured_values.gyro_vec, 2);
  Serial.println("ACC ");
  printVec3(measured_values.acc_vec, 2);
  Serial.println("MAG ");
  printVec3(measured_values.mag_vec, 2);
  Serial.println();

  if (SAFETY){
    while (sensors.imu_timed_out){
      Serial.println("IMU timeout");
    }
  }

  Serial.println("Setup finished!");

  // start of the control loop
  estimated_DCM = acc_mag2DCM(measured_values); // init DCM
  k_filter.init_quat(estimated_DCM);  // correct initialization of quaternion from DCM matrix
  current_state = compute_state(estimated_DCM, measured_values);  // initial state for reference
  initial_yaw = current_state.yaw;  // store initial yaw for control
  start_time = millis();
}

void setup1(){
  while(!start_time){  // wait for setup on the first core to finish
      delay(200);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

bool loop_telemetry_sent = false;

float throttle = 0;
void loop() {
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

  // handle new PID config sent from controller
  if (comm.new_roll_config) controller.update_PID_params(0, comm.pop_config(0));
  if (comm.new_pitch_config) controller.update_PID_params(1, comm.pop_config(1));
  if (comm.new_yaw_config) controller.update_PID_params(2, comm.pop_config(2));
  if (comm.new_alt_config) controller.update_PID_params(3, comm.pop_config(3));

  // estimated_DCM = acc_mag2DCM(measured_values); // init DCM
  // estimated_DCM = update_DCM_rejection(estimated_DCM, measured_values);  // update DCM matrix
  // controller.update_DCM(estimated_DCM);
  q = k_filter.predict(measured_values.gyro_vec, measured_values.integration_period);  // model prediction using gyro
  k_filter.track_acc(measured_values.acc_vec, measured_values.integration_period);  // track accelerometer magnitude

  // fuse acceleration using EKF
  Vector3 unit_acc = normalize(measured_values.acc_vec);
  q = k_filter.fuse_acc(unit_acc);  // fuse accelerometer data

  // fuse magnetometer using EKF
  Vector3 unit_mag;
  if (measured_values.battery > 10) unit_mag = normalize(k_filter.remove_mag_bias(measured_values.mag_vec, motors_on*0.25f*sum(control_action)));
  else unit_mag = normalize(measured_values.mag_vec);  
  Vector3 est_up = estimated_DCM.Column(2);  // best guess of UP direction
  Vector3 perp_mag = unit_mag - dot(unit_mag, est_up)*est_up;  // horizontal component of magnetic vector
  q = k_filter.fuse_mag(normalize(perp_mag));  // fuse mag

  estimated_DCM = k_filter.quat2R(q);  // quat to DCM conversion
  float max_val = k_filter.clamp_variance();  // reduce variance if too big

  // guarding numerical stability of EKF
  if (isinf(q(0)) && isinf(q(1)) && isinf(q(2)) && isinf(q(3))){
    while(1){ 
      signal_motors(zero_4vector);
      Serial.println("inf q");
    }
  }
  if (max_val > 10.0f){  // crash the drone :(
    while(1){ 
      signal_motors(zero_4vector);
      Serial.println("P out of bounds");
    }
  }

  controller.update_DCM(estimated_DCM);  
  control_action = controller.update_motor_percentages(ctrl_commands, measured_values); 
  if (motors_on == 0) signal_motors(zero_4vector); 
  else signal_motors(control_action);

  loop_telemetry_sent = false;  // set flag for sending telemetry
}

void loop1() {
  commands = comm.update_commands(initial_yaw);  // poll for new commands

  if ((millis() - comm.last_ctrl_ms) <= 90) // less than 90 milliseconds from last command ==> we can send telemetry
  {
    current_state = compute_state(estimated_DCM, measured_values);  // initial state for reference
    if (TELEMETRY && (comm.batt_telem_countdown <= 0)) {  // send some data back to controller 
      telemetry_msg_t msg = comm.create_batt_telemetry(current_state, measured_values);
      comm.send_telemetry(msg);  
      comm.batt_telem_countdown = 10;  // reset counter
    }
    
    // the nrf24 telemetry takes about 0.9 ms per message
    if (!loop_telemetry_sent){
      telemetry_msg_t msg = comm.create_state_telemetry(current_state, control_action, initial_yaw, controller.last_PID_outputs, controller.last_reference);
      comm.send_telemetry(msg);
      msg = comm.create_sensor_telemetry(current_state, initial_yaw, measured_values);
      comm.send_telemetry(msg);
      loop_telemetry_sent = true;  // only send once per control loop
    }
  }

}