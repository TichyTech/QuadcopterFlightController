#include "debugging.h"

void printVec3(Vector3 v, uint8_t precision){
  Serial.print(v(0), precision);
  Serial.print(' ');
  Serial.print(v(1), precision);
  Serial.print(' ');
  Serial.print(v(2), precision);
  Serial.print(' ');
}

void printVec4(Vector4 v, uint8_t precision){
  Serial.print(v(0), precision);
  Serial.print(' ');
  Serial.print(v(1), precision);
  Serial.print(' ');
  Serial.print(v(2), precision);
  Serial.print(' ');
  Serial.print(v(3), precision);
  Serial.print(' ');
}

void printState(State state, uint8_t precision){
  Serial.print(state.roll, precision);
  Serial.print(' ');
  Serial.print(state.pitch, precision);
  Serial.print(' ');
  Serial.print(state.yaw, precision);
  Serial.print(' ');
  Serial.print(state.alt, precision);
  Serial.print(' ');
}

void printMeasurement(Measurements m, uint8_t precision){
  Serial.print("gyro: ");
  printVec3(m.gyro_vec, precision);
  Serial.print("acc: ");
  printVec3(m.acc_vec, precision);
  Serial.print("mag: ");
  printVec3(m.mag_vec, precision);
}

void printControlMessage(ctrl_msg_t msg, uint8_t precision){
  Serial.print(msg.roll, precision);
  Serial.print(' ');
  Serial.print(msg.pitch, precision);
  Serial.print(' ');
  Serial.print(msg.yaw_diff, precision);
  Serial.print(' ');
  Serial.print(msg.throttle, precision);
  Serial.print(' ');
  Serial.print(msg.motors_on);
  Serial.print(' ');
}

void printR2Processing(Matrix3 R){
  Matrix3 M = R * 1024.0f;
  for (int j = 0; j < 3; j++){
    for (int i = 0; i < 3; i++){
      Serial.print(int(M(i,j)));  // col1, col2, col3
      Serial.print(' ');
    }
  }
  Serial.print('\n');
}

void printV2Processing(Vector3 v){
  for (int i = 0; i < 3; i++){
    Serial.print(int(v(i)));
    Serial.print(' ');
  }
  Serial.print('\n');
}

void printV2ProcessingFloat(Vector3 v){
  for (int i = 0; i < 3; i++){
    Serial.print(v(i), 5);
    Serial.print(' ');
  }
  Serial.print('\n');
}
