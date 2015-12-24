#include <PID.h>
#include <cmath>

using namespace std;

PID::PID() {
	Kp = 50.0;
	Ki = 5.0;
	Kd = 17.0;
	Output = 0.0;
	KpPropertySign = 1.0;
	KiPropertySign = 1.0;
	KdPropertySign = 1.0;
	I_out_total = 0.0;
	Input_prev = Input_prev2 = 0.0;
	IntType = eAdamsBashforth2;
	IsStandard = false;
	clipmin = -1.0;
	clipmax = 1.0;
	dt = 0.00833;
	Trigger = 0.0;
}

void PID::Connect() {}

bool PID::Connected() {
	return true;
}

bool PID::Run() {
	double Dval = (Input - Input_prev) / dt;
	
	// Do not continue to integrate the input to the integrator if a wind-up
	// condition is sensed - that is, if the property pointed to by the trigger
	// element is non-zero. Reset the integrator to 0.0 if the Trigger value
	// is negative.

	double test = 0.0;
	if (Trigger != 0) 
		test = Trigger;
	
	double I_out_delta = 0.0;
	
	if (fabs(test) < 0.000001) {
		switch (IntType) {
		case eRectEuler:
			I_out_delta = Ki * dt * Input;                         // Normal rectangular integrator
			break;
		case eTrapezoidal:
			I_out_delta = (Ki / 2.0) * dt * (Input + Input_prev);    // Trapezoidal integrator
			break;
		case eAdamsBashforth2:
			I_out_delta = Ki * dt * (1.5*Input - 0.5*Input_prev);  // 2nd order Adams Bashforth integrator
			break;
		case eAdamsBashforth3:                                   // 3rd order Adams Bashforth integrator
			I_out_delta = (Ki / 12.0) * dt * (23.0*Input - 16.0*Input_prev + 5.0*Input_prev2);
			break;
		case eNone:
			// No integator is defined or used.
			I_out_delta = 0.0;
			break;
		}
	}
	
	if (test < 0.0) 
		I_out_total = 0.0;  // Reset integrator to 0.0

	I_out_total += I_out_delta;

	if (IsStandard) {
		Output = Kp * (Input + I_out_total + Kd*Dval);
	}
	else {
		Output = Kp*Input + I_out_total + Kd*Dval;
	}

	Input_prev = Input;
	Input_prev2 = Input_prev;
	Clip();

	return true;
}

void PID::SetCurrentInput(double angleRad) { Input = angleRad; }

double PID::GetCurrentOutput() { return -Output; }

void PID::Clip(void) {
	if (Output > clipmax)      Output = clipmax;
	else if (Output < clipmin) Output = clipmin;
}