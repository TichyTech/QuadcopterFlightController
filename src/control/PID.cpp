#include "PID.h"

PID::PID(){

};

PID::PID(float setP, float setI, float setD, float sat, float setLPc, float set_differr_deadband, String setName){
	float prev_error = 0;
	float prev_diff_error = 0;
	float sum_error = 0;

	P = setP;
	I = setI;
	D = setD;
	saturation = sat;
	LPc = setLPc; // derivative low pass filter coefficient
	differr_deadband = set_differr_deadband;
	nm = setName;
};

void PID::set_PID_params(float setP, float setI, float setD, float sat, float setLPc){
	P = setP;
	I = setI;
	D = setD;
	saturation = sat;
	LPc = setLPc; // derivative low pass filter coefficient
	if (DEBUG)
		Serial.println(nm + " PID parameters set " + String(P, 2) + " " + String(LPc, 2));
}
	
float PID::process(float reference, float measurement, float integration_period){
	float error = reference - measurement;

	float diff_error = (error - prev_error) / integration_period; // derivation
	if (abs(diff_error) <= differr_deadband)
		diff_error = 0;																						 // ignore noise in derivative
	diff_error = LPc * diff_error + (1 - LPc) * prev_diff_error; // Low pass filter on derivative to reduce noise spikes

	prev_diff_error = diff_error;
	prev_error = error;

	sum_error += error * integration_period;									 // integration
	sum_error = constrain(sum_error, -saturation, saturation); // anti windup

	return P * error + I * sum_error + D * diff_error;
}