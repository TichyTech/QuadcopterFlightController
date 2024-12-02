#include "definitions.h"
#include "comm_structs.h"

using namespace BLA;

void printVec3(Vector3 v, uint8_t precision);
void printVec4(Vector4 v, uint8_t precision);
void printState(State state, uint8_t precision);
void printControlMessage(ctrl_msg_t msg, uint8_t precision);
void printR2Processing(Matrix3 R);
void printV2Processing(Vector3 v);
void printV2ProcessingFloat(Vector3 v);