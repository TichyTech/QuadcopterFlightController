#ifndef COMMSTR
#define COMMSTR

#include <Arduino.h>

/// Data transformations for transmission purposes
#define ANGLE_FS 180.0f
#define FORCE_FS 200.0f
#define GYRO_TELEM_FS 720.0f
#define ACC_TELEM_FS 2.0f
#define MAG_TELEM_FS 1.3f

#define MAX_15BIT 32767.0f
#define MAX_16BIT 65535.0f

// State struct conversions

/* Transform a float angle [-180, 180] to 16 bit int representation for transmitting */
inline int16_t angle_to_int(float angle){
  return int16_t(angle * (MAX_15BIT / ANGLE_FS));  // * (2^15 - 1) / 180 ... rescaling to int16_t full scale
}

/* Transform an int angle [-(2^15 - 1), 2^15 - 1] to float angle [-180, 180] */
inline float angle_to_float(int16_t angle){
  return float(angle) * (ANGLE_FS / MAX_15BIT);  // * 180 / (2^15 - 1) ... rescaling to float again
}

/* Transform motor setting from [0,1] float to int [0, 2^15 - 1] */
inline uint16_t action_to_int(float action){
  return uint16_t(action * MAX_16BIT);  // multiply by 2^15 - 1
}

/* Transform motor setting from int [0, 2^15 - 1] to float [0, 1] */
inline float action_to_float(uint16_t action){
  return float(action) * (1 / MAX_16BIT);  // divide by 2^15 - 1
}

/* Transform PID output from float [-FS, FS] to int [0, 2^15 - 1] */
inline int16_t force_to_int(float force){
  return int16_t (force * (MAX_15BIT / FORCE_FS));  // * (2^15 - 1) / FS
}

/* Transform PID output from int [0, 2^15 - 1] to float [-FS, FS]*/
inline float force_to_float(int16_t force){
  return float(force) * (FORCE_FS / MAX_15BIT);  // * FS / (2^15 - 1)
}

// sensor struct conversions

inline int16_t gyro_to_int(float gyro){
  return int16_t(gyro * (MAX_15BIT / GYRO_TELEM_FS));
}

inline float gyro_to_float(int16_t gyro){
  return float(gyro) * (GYRO_TELEM_FS / MAX_15BIT);
}

inline int16_t acc_to_int(float acc){
  return int16_t(acc * (MAX_15BIT / ACC_TELEM_FS));
}

inline float acc_to_float(int16_t acc){
  return float(acc) * (ACC_TELEM_FS / MAX_15BIT);
}

inline int16_t mag_to_int(float mag){
  return int16_t(mag * (MAX_15BIT / MAG_TELEM_FS));
}

inline float mag_to_float(int16_t mag){
  return float(mag) * (MAG_TELEM_FS / MAX_15BIT);
}

// Data structs for communication

typedef struct ctrl_msg_t{
  float roll;
  float pitch;
  float yaw_diff;
  float throttle;
  uint32_t motors_on;
  uint32_t sequence;
} ctrl_msg_t;  // received command message

typedef struct config_msg_t{
  float P;
  float I;
  float D;
  float sat;
  float LPc;
  uint32_t axis;  // roll, pitch, yaw
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

typedef struct state_struct{  // 28 bytes
  uint32_t ms;
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  int16_t PID_outputs[3];  // RPY PID outputs before clamping
  uint16_t motors[4];
  int16_t ref[2];
} state_struct;  // received command message

typedef struct sensor_struct{  // 8 bytes
  float battery;
  float height;
} sensor_struct;  // received command message

typedef struct ekf_struct{  // 28 bytes
  uint32_t ms;
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  int16_t acc[3];
  int16_t mag[3];
  int16_t gyro[3];
} ekf_struct;  // received command message

typedef struct telemetry_msg_t{  // 32 bytes
  uint32_t type;  // state message, sensor_message
  union U{
    byte bytes [28];
    sensor_struct sensor_data;
    state_struct state_data; 
    ekf_struct ekf_data;
  } data;
} telemetry_msg_t;

#endif
