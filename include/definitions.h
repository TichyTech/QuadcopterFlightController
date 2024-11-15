#ifndef DEFS
#define DEFS 

#include "BasicLinearAlgebraFix.h"
using namespace BLA;

// helpful commands
#define NOP __asm__("nop");  // 0.0108 us (at pico base frequency)

template <typename T>
inline int sgn(T x){
  return x < 0 ? -1 : (x > 0 ? 1 : 0);
}

inline float max(float a, float b){
  return (a < b ? b : a);
}

// helpful constants
#define TO_DEG 57.29578
#define TO_RAD 0.01745329251

// helpful types
typedef Matrix<3> Vector3;
typedef Matrix<4> Vector4;
typedef Matrix<3, 3> Matrix3;

const Vector4 zero_4vector = {0,0,0,0};
const Vector3 zero_3vector = {0,0,0};

// Flight controller-used structures definitions

typedef struct Control{
  float roll = 0;
  float pitch = 0;
  float yaw = 0;
  float throttle = 0;
  uint32_t motors_on = 0;
} Control;

typedef struct State{
  float roll = 0;
  float pitch = 0;
  float yaw = 0;
  float alt = 0;
} State;

typedef struct Measurements{
  Vector3 acc_vec;
  Vector3 mag_vec;
  Vector3 gyro_vec;
  float altitude;
  float battery = 12;
  float integration_period;
} Measurements;   // struct to hold all measured values

struct PID_config{
  float P;
  float I;
  float D;
  float sat;
  float LPc;

  void set(float newP, float newI, float newD, float newsat, float newLPc){
    P = newP;
    I = newI; 
    D = newD;
    sat = newsat;
    LPc = newLPc;
  }
};

// Communication message definitions

typedef struct ctrl_msg_t{
  float roll;
  float pitch;
  float yaw_diff;
  float throttle;
  uint32_t motors_on;
} ctrl_msg_t;  // received command message

typedef struct config_msg_t{
  float P;
  float I;
  float D;
  float sat;
  float LPc;
  uint32_t axis = 0;  // roll, pitch, yaw
} config_msg_t;  // received command message

typedef struct msg_t{
  uint32_t type;
  union U{
    U(){};
    byte bytes [24];
    ctrl_msg_t ctrl_data;
    config_msg_t config_data; 
  } data;
} msg_t;

typedef struct state_struct{  // 24 bytes
  uint32_t ms;
  float roll;
  float pitch;
  float yaw;
  uint16_t motors[4];
} state_struct;  // received command message

typedef struct sensor_struct{
  float battery;
  float height;
} sensor_struct;  // received command message

typedef struct telemetry_msg_t{
  uint32_t type;  // state message, sensor_message
  union U{
    byte bytes [28];
    sensor_struct sensor_data;
    state_struct state_data; 
  } data;
} telemetry_msg_t;

#endif