#include "algebra.h"
#include "config.h"


Matrix3 acc_mag2DCM(Measurements m);
Matrix3 update_DCM(Matrix3 current_DCM, Measurements m);
Vector3 get_omega(Vector3 a, Vector3 b);
Vector3 DCM2RPY(Matrix3 D);
State compute_state(Matrix3 D, Measurements m);