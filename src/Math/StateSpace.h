//=====================================================================================================
// StateSpace.h
//=====================================================================================================

/** Class StateSpace holds an arbitrary location XYZ in the Earth centered Earth fixed reference frame (ECEF). Orientation is hold in the Local (North-East-Down, NED) frame.
		Class StateSpace implements lazy evaluations. Once cacheValid sets false, we calculate proper data. Later we check new incoming data, and if abs(new-old)<e, we don't compute anything.
*/

#ifndef STATE_SPACE_H
#define STATE_SPACE_H

#include <Base.h>
#include <MadgwickAHRS.h>

#include <atomic>
#include <cmath>

class StateSpace {
public:
	StateSpace() : 
		cacheValid(false), 
		ax(0.0), 
		ay(0.0), 
		az(0.0), 
		gx(0.0), 
		gy(0.0), 
		gz(0.0), 
		mx(0.0), 
		my(0.0),
		mz(0.0), 
		latitude(0.0),
		longitude(0.0),
		altitude(0.0),
		Roll(0.0),
		Pitch(0.0),
		Yaw(0.0),
		X(0.0),
		Y(0.0),
		Z(0.0),
		AileronCmd(0.0),
		ElevatorCmd(0.0), 
		RudderCmd(0.0), 
		ThrottleCmd(0.0) {
		auto m = 0; //test var here
	}

	void setSensorData(float axd, float ayd, float azd, float gxd, float gyd, float gzd, float mxd, float myd, float mzd) {
		if (
			(std::fabs(axd - ax.load()) < e) &&
			(std::fabs(ayd - ay.load()) < e) &&
			(std::fabs(azd - az.load()) < e) &&
			(std::fabs(gxd - gx.load()) < e) &&
			(std::fabs(gyd - gy.load()) < e) &&
			(std::fabs(gzd - gz.load()) < e) &&
			(std::fabs(mxd - mx.load()) < e) &&
			(std::fabs(myd - my.load()) < e) &&
			(std::fabs(mzd - mz.load()) < e)) {
			cacheValid = true;
			return;
		}
		ax = axd;
		ay = ayd;
		az = azd;
		gx = gxd;
		gy = gyd;
		gz = gzd;
		mx = mxd;
		my = myd;
		mz = mzd;
		cacheValid = false;
	}

	void setGPSData(float lat, float lon, float alt) {
		if (
			(std::fabs(lat - latitude.load()) < e) &&
			(std::fabs(lon - longitude.load()) < e) &&
			(std::fabs(alt - altitude.load()) < e)) {
			cacheValid = true;
			return;
		}
		latitude = lat;
		longitude = lon;
		altitude = alt;
		cacheValid = false;
	}

	void setGCSData(float aileron, float elevator, float rudder, float throttle) {
		if (
			(std::fabs(aileron - AileronCmd.load()) < e) &&
			(std::fabs(elevator - ElevatorCmd.load()) < e) &&
			(std::fabs(rudder - RudderCmd.load()) < e) &&
			(std::fabs(throttle - ThrottleCmd.load()) < e) ) {
			cacheValid = true;
			return;
		}
		AileronCmd = aileron;
		ElevatorCmd = elevator;
		RudderCmd = rudder;
		ThrottleCmd = throttle;
		cacheValid = false;
	}

	float getRoll() {  
		if (!cacheValid)
			computeDerived();
		return Roll;
	}

	float getPitch() { 
		if (!cacheValid)
			computeDerived();
		return Pitch;
	}

	float getYaw() { 
		if (!cacheValid)
			computeDerived();
		return Yaw;
	}

	float GetX() {
		if (!cacheValid)
			computeDerived();
		return X;
	}
	
	float getY() {
		if (!cacheValid)
			computeDerived();
		return Y;
	}

	float getZ() {
		if (!cacheValid)
			computeDerived();
		return Z;
	}

	float getAltitude() {
		if (!cacheValid)
			computeDerived();
		return 38.0477f / 0.3028f;
	}

	float getAileron() {
		if (!cacheValid)
			computeDerived();
		return AileronCmd;
	}

	float getElevator() {
		if (!cacheValid)
			computeDerived();
		return ElevatorCmd;
	}

	float getRudder() {
		if (!cacheValid)
			computeDerived();
		return RudderCmd;
	}

	float getThrottle() {
		if (!cacheValid)
			computeDerived();
		return ThrottleCmd;
	}

private:
	void computeDerived() {
		ComputeAngles();
		ComputePosition();
		cacheValid = true;
	}

	void ComputeAngles() {
		MadgwickAHRSupdate(gx*M_PI / 180.0f, gy*M_PI / 180.0f, gz*M_PI / 180.0f, ax, ay, az, mx, my, mz);
		float roll, pitch, yaw;
		QuaternionToEuler(&roll, &pitch, &yaw);
		Roll.store(roll);
		Pitch.store(pitch);
		Yaw.store(yaw);
	}

	void ComputePosition() {}

	bool cacheValid;

	const float e = 1e-9f;

	/* These members hold input data */
	
	// These are params from orientation sensor.
	std::atomic<float> ax;
	std::atomic<float> ay;
	std::atomic<float> az;
	std::atomic<float> gx;
	std::atomic<float> gy;
	std::atomic<float> gz;
	std::atomic<float> mx;
	std::atomic<float> my;
	std::atomic<float> mz;

	// These are lat/lon/alt data from GPS
	std::atomic<float> latitude;
	std::atomic<float> longitude;
	std::atomic<float> altitude;

	/*********************************/

	/* These members hold output data */

	// These are orientation angles in the Local(NED) frame. These data will be send from AP to GCS. Also, these data will be used by FCS of the AP itself.
	std::atomic<float> Roll;
	std::atomic<float> Pitch;
	std::atomic<float> Yaw;
	
	// These are orientation angles in the ECEF frame.  These data will be send from AP to GCS. Also, these data will be used by FCS of the AP itself.
	std::atomic<float> X;
	std::atomic<float> Y;
	std::atomic<float> Z;

	// These are commands from AP to EngineBoard
	std::atomic<float> AileronCmd;
	std::atomic<float> ElevatorCmd;
	std::atomic<float> RudderCmd;
	std::atomic<float> ThrottleCmd;

	/*********************************/
};

#endif
