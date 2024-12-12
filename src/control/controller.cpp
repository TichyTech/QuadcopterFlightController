#include "controller.h"

// MOTOR NUMBERING DIAGRAM
// TOP VIEW
// FRONT
//---------
//        X
//        ^
//        |
//
//     1     2 
//      \ | /
// <-Y   |||    
//      / | \ 
//     4     3 

// useful definitions
const Vector4 roll_action = {1,-1,-1,1};
const Vector4 pitch_action = {-1,-1,1,1};
const Vector4 yaw_action = {1,-1,1,-1};
const Vector4 thrust_action = {1,1,1,1};

Controller::Controller(){
  roll_rate_PID = PID(2, 0, 0.005, 1, 0.02, 40, "roll");
  pitch_rate_PID = PID(2, 0, 0.005, 1, 0.02, 40, "pitch");
  // yaw_rate_PID = PID(35, 5, 0, 1, 0, 0, "yaw"); 
  // yaw_rate_PID = PID(35, 0, 0.01, 0, 0.02, 40, "yaw"); 
  yaw_rate_PID = PID(2, 0, 0.01, 0, 0.02, 40, "yaw"); 
  alt_PID = PID(50, 5, 30, 2, 0.5, 0, "alt");

  // For a linear system, this would yield a time constant tau = 1/P
  roll_P = 3;
  pitch_P = 3;
  yaw_P = 0.2;

  motor_percentages = {0,0,0,0};
  last_PID_outputs = {0,0,0};
}

void Controller::update_DCM(Matrix3 newDCM){
  DCM = newDCM;
}

/**
 * update the PID parameters using a new config struct
 */
void Controller::update_PID_params(int axis, PID_config cfg){
  switch (axis){
    case 0:
      roll_rate_PID.set_PID_params(cfg.P, cfg.I, cfg.D, cfg.sat, cfg.LPc);
      break;
    case 1:
      pitch_rate_PID.set_PID_params(cfg.P, cfg.I, cfg.D, cfg.sat, cfg.LPc);
      break;
    case 2:
      yaw_rate_PID.set_PID_params(cfg.P, cfg.I, cfg.D, cfg.sat, cfg.LPc);
      break;
    case 3:
      alt_PID.set_PID_params(cfg.P, cfg.I, cfg.D, cfg.sat, cfg.LPc);
      break;
    default: 
      break;
  }
}

Vector4 Controller::update_motor_percentages(Control commands, Measurements m){
  // using the current state and setpoint state, compute the PID action
  State current_state = compute_state(DCM, m);  // compute current state (rpy, alt)
  Vector3 forces = {0,0,0};  // rpy forces to apply 
  Vector3 des_rates = {0,0,0};
  float dt = m.integration_period;

  // Proportional controller on RPY
  des_rates(0) += roll_P*(commands.roll - current_state.roll);  
  des_rates(1) += pitch_P*(commands.pitch - current_state.pitch);
  des_rates(2) += yaw_P*constrain_angle(commands.yaw - current_state.yaw);

  des_rates(0) = constrain(des_rates(0), -150, 150);
  des_rates(1) = constrain(des_rates(1), -150, 150);
  des_rates(2) = constrain(des_rates(2), -90, 90);

  forces(0) = roll_rate_PID.process( des_rates(0), m.gyro_vec(0), dt);
  forces(1) = pitch_rate_PID.process(des_rates(1), m.gyro_vec(1), dt);
  forces(2) = yaw_rate_PID.process(  des_rates(2), m.gyro_vec(2), dt);

  last_PID_outputs = forces;  // for telemetry

  forces(0) = constrain(forces(0), -40, 40);
  forces(1) = constrain(forces(1), -40, 40);
  forces(2) = constrain(forces(2), -60, 60);

  Vector4 new_percentages = mix_motors(forces, DCM, commands.throttle, m.battery);
  // motor_percentages = new_percentages*MOTOR_LPF + motor_percentages*(1-MOTOR_LPF);

  // Jerk clamping 
  Vector4 d_p = (new_percentages - motor_percentages)/dt;  //normalize by time constant
  // clamp to about +-20 full-scale per second (or 2% per millisecond)
  const float change_limit = 20;
  d_p(0) = constrain(d_p(0), -change_limit, change_limit);
  d_p(1) = constrain(d_p(1), -change_limit, change_limit);
  d_p(2) = constrain(d_p(2), -change_limit, change_limit);
  d_p(3) = constrain(d_p(3), -change_limit, change_limit);
  motor_percentages = motor_percentages + d_p*dt;

  switch(commands.motors_on){ // enable turning on one motor at a time for debbuging purposes
    case 2:
      motor_percentages = {commands.throttle,0,0,0};
      break;
    case 3:
      motor_percentages = {0,commands.throttle,0,0};
      break;
    case 4:
      motor_percentages = {0,0,commands.throttle,0};
      break;
    case 5:
      motor_percentages = {0,0,0,commands.throttle};
      break;
    default:
      break;
  }
  
  return motor_percentages;
}

Vector4 Controller::mix_motors(Vector3 forces, Matrix3 DCM, float throttle, float battery){  
  //TODO: include battery charge compensation, YAW compensation
  Vector4 motor_percentages = {0, 0, 0, 0};
  if (DCM(2,2) > 0){  // facing up
    // motor_percentages += thrust_action * throttle / DCM(2,2);  // compensate gravity
    motor_percentages += thrust_action * throttle;  // compensate gravity
  }
  // float t = throttle / DCM(2,2);
  // if (t <= 0.2){  // for small throttle, keep values
    motor_percentages += roll_action * (forces(0) / MOTOR_FORCE);  
    motor_percentages += pitch_action * (forces(1) / MOTOR_FORCE); 
  // }
  // else {
  //   motor_percentages += roll_action * (forces(0) / (MOTOR_FORCE * 5 * t));  //scale down with increasing throttle
  //   motor_percentages += pitch_action * (forces(1) / (MOTOR_FORCE * 5 * t)); 
  // }
  motor_percentages += yaw_action*forces(2) / (MOTOR_FORCE);

  if (battery > 10) // should always be the case, if battery is connected
  motor_percentages *= (IDEAL_VOLTAGE*IDEAL_VOLTAGE)/(battery*battery);  // battery charge compensation

  motor_percentages(0) = constrain(motor_percentages(0), 0, 1);
  motor_percentages(1) = constrain(motor_percentages(1), 0, 1);
  motor_percentages(2) = constrain(motor_percentages(2), 0, 1);
  motor_percentages(3) = constrain(motor_percentages(3), 0, 1);

  return motor_percentages;
}