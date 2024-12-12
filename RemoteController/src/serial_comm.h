#include "comm_structs.h"

void format_state(state_struct s_data, char* buffr, size_t bsize){

  float r = angle_to_float(s_data.roll);
  float p = angle_to_float(s_data.pitch);
  float y = angle_to_float(s_data.yaw);

  float motors[4] = { action_to_float(s_data.motors[0]),
                      action_to_float(s_data.motors[1]),
                      action_to_float(s_data.motors[2]),
                      action_to_float(s_data.motors[3])};

  float PID_outputs[3] = {force_to_float(s_data.PID_outputs[0]),
                          force_to_float(s_data.PID_outputs[1]),
                          force_to_float(s_data.PID_outputs[2])};

  float all_floats[10] = {r, p, y, motors[0], motors[1], motors[2], motors[3], PID_outputs[0], PID_outputs[1], PID_outputs[2]};

  snprintf(buffr, bsize, "State: %lu ",  s_data.ms);
  char temp_b[7]; 
  int str_len = strlen(buffr);
  for (int i = 0; i < 10; i++){
    dtostrf(all_floats[i], 3, 2, temp_b);
    int float_len = strlen(temp_b);
    for (int j = 0; j < float_len; j++){
      buffr[str_len + j] = temp_b[j];  // copy float to string
    }
    buffr[str_len + float_len] = ' ';  // set a space
    buffr[str_len + float_len + 1] = '\0';  // set a string end char
    str_len += float_len + 1;
//    strcat(buffr, temp_b);
//    strcat(buffr, " ");
  }
}
