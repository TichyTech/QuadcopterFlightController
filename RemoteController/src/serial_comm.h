#include "comm_structs.h"
#include "config.h"

void format_state(state_struct s_data, char *buffr, size_t bsize);

void format_floats_to_buffer(char *buffr, float *all_floats, uint8_t num_floats);

void print_message_to_serial(telemetry_msg_t tele_msg);

bool parse_serial(msg_t &msg);

config_msg_t config_from_line(String serial_line);

void print_commands(ctrl_msg_t msg);
