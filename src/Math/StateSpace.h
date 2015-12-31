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
#include <array>

class StateSpace {
public:
	StateSpace() : 
		Roll(0.0),
		Pitch(0.0),
		Yaw(0.0),
		AileronCmd(0.0),
		ElevatorCmd(0.0), 
		RudderCmd(0.0), 
		ThrottleCmd(0.0),
		axd(0.0), ayd(0.0), azd(0.0),
		gxd(0.0), gyd(0.0), gzd(0.0),
		mxd(0.0), myd(0.0), mzd(0.0) {
	}

	void setSensorData(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
		axd = ax; ayd = ay; azd = az;
		gxd = gx; gyd = gy; gzd = gz;
		mxd = mx; myd = my; mzd = mz;
	}

	void setGCSData(float aileron, float elevator, float rudder, float throttle) {
		AileronCmd = aileron;
		ElevatorCmd = elevator;
		RudderCmd = rudder;
		ThrottleCmd = throttle;
	}

	float getRoll() {
		computeAngles();
		return Roll;
	}

	float getPitch() { 
		computeAngles();
		return Pitch;
	}

	float getYaw() { 
		computeAngles();
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
	
	std::array<float, 3> getAngles() {
		computeAngles();
		return std::array<float, 3>{Roll, Pitch, Yaw};
	}

private:
	float axd, ayd, azd;
	float gxd, gyd, gzd;
	float mxd, myd, mzd;

	void computeAngles() {
		MadgwickAHRSupdate(gxd*M_PI / 180.0f, gyd*M_PI / 180.0f, gzd*M_PI / 180.0f, axd, ayd, azd, mxd, myd, mzd);
		QuaternionToEuler(&Roll, &Pitch, &Yaw);
	}

	float Roll, Pitch, Yaw;
	float AileronCmd, ElevatorCmd, RudderCmd, ThrottleCmd;
};

#endif