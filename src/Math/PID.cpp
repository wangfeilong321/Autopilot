#include <PID.h>
#include <cmath>

using namespace std;

PID::PID() {
	Kp = 50.0f;
	Ki = 5.0f;
	Kd = 17.0f;
	Output = 0.0f;
	I_out_total = 0.0f;
	Input_prev = Input_prev2 = 0.0f;
}

void PID::Connect() {}

bool PID::Connected() {
	return true;
}

void PID::SetInput(float val) {
	Input = val;
}

float PID::GetOutput() {
	return Output;
}

bool PID::Run() {
	float Dval = (Input - Input_prev) / dt;
	
	// Do not continue to integrate the input to the integrator if a wind-up
	// condition is sensed - that is, if the property pointed to by the trigger
	// element is non-zero. Reset the integrator to 0.0 if the Trigger value
	// is negative.

 float I_out_delta = 0.0f;
	
	switch (IntType) {
		case eRectEuler:
			I_out_delta = Ki * dt * Input;                         // Normal rectangular integrator
			break;
		case eTrapezoidal:
			I_out_delta = (Ki / 2.0f) * dt * (Input + Input_prev);    // Trapezoidal integrator
			break;
		case eAdamsBashforth2:
			I_out_delta = Ki * dt * (1.5f*Input - 0.5f*Input_prev);  // 2nd order Adams Bashforth integrator
			break;
		case eAdamsBashforth3:                                   // 3rd order Adams Bashforth integrator
			I_out_delta = (Ki / 12.0f) * dt * (23.0f*Input - 16.0f*Input_prev + 5.0f*Input_prev2);
			break;
		case eNone:
			// No integator is defined or used.
			I_out_delta = 0.0f;
			break;
		default: break;
	}
	
	I_out_total += I_out_delta;

	Output = Kp*Input + I_out_total + Kd*Dval;

	Input_prev = Input;
	Input_prev2 = Input_prev;
	Clip();

	return true;
}

void PID::Clip(void) {
	if (Output > CLIP_MAX)      Output = CLIP_MAX;
	else if (Output < CLIP_MIN) Output = CLIP_MIN;
}