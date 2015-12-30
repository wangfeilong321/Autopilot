//=====================================================================================================
// StateSpace.h
//=====================================================================================================

/** Class StateSpace holds an arbitrary location XYZ in the Earth centered Earth fixed reference frame (ECEF). Orientation is hold in the Local (North-East-Down, NED) frame.
*/

#ifndef STATE_SPACE_H
#define STATE_SPACE_H

#include <Base.h>
#include <MadgwickAHRS.h>

#include <cmath>

class StateSpace {
public:
	StateSpace() : 
		Roll(0.0),
		Pitch(0.0),
		Yaw(0.0),
		AileronCmd(0.0),
		ElevatorCmd(0.0), 
		RudderCmd(0.0), 
		ThrottleCmd(0.0) {
	}

	void setSensorData(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
		MadgwickAHRSupdate(gx*M_PI / 180.0f, gy*M_PI / 180.0f, gz*M_PI / 180.0f, ax, ay, az, mx, my, mz);
		QuaternionToEuler(&Roll, &Pitch, &Yaw);
	}

	void setGCSData(float aileron, float elevator, float rudder, float throttle) {
		AileronCmd = aileron;
		ElevatorCmd = elevator;
		RudderCmd = rudder;
		ThrottleCmd = throttle;
	}

	float getRoll() {  
		return Roll;
	}

	float getPitch() { 
		return Pitch;
	}

	float getYaw() { 
		return Yaw;
	}

	float getAltitude() {
		return 1000.0f;
	}

	float getAileron() {
		return AileronCmd;
	}

	float getElevator() {
		return ElevatorCmd;
	}

	float getRudder() {
		return RudderCmd;
	}

	float getThrottle() {
		return ThrottleCmd;
	}

private:
	float Roll, Pitch, Yaw;
	float AileronCmd, ElevatorCmd, RudderCmd, ThrottleCmd;
};

#endif