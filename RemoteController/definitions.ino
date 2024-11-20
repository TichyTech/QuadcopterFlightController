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
  uint32_t axis;  // roll, pitch, yaw
} config_msg_t;  // received command message

typedef struct msg_t{
  uint32_t type;  // control message, config message
  union U{
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
