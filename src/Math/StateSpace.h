#pragma once

#include <Base.h>
#include <MadgwickAHRS.h>
#include <Location.h>
#include <Inertial.h>
#include <Matrix33.h>
#include <Quaternion.h>
#include <ColumnVector3.h>

#include <cmath>
#include <condition_variable>
#include <mutex>
#include <deque>
#include <chrono>

class StateSpace {
public:
	StateSpace();

	void TrimAircraft();

	void InitializeDerivatives();

	bool Run();

	void setSensorData(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

	void setAileron(float aileron);

	void setElevator(float elevator);

	void setRudder(float rudder);

	void setThrottle(float throttle);

	void setEng0RPM(int Rpmd);
	
	void setEng1RPM(int Rpmd);
	
	void setEng2RPM(int Rpmd);

	void setEng3RPM(int Rpmd);

	float getEng0Rpm();

	float getEng1Rpm();

	float getEng2Rpm();

	float getEng3Rpm();

	float getRoll() const;

	float getPitch() const;

	float getYaw() const;

	float getX() const;

	float getY() const;

	float getZ()const;

	float getAileron();

	float getElevator();

	float getRudder();

	float getThrottle();
	
private:

	int timer_sec = 15;
	bool canComputePos = false;

	Location vLocation;
	Inertial vInertial;
	ColumnVector3 vInertialPosition;
	ColumnVector3 vInertialVelocity;
	ColumnVector3 vUVW;
	ColumnVector3 vVel;
	ColumnVector3 vUVWidot;

	float axd, ayd, azd;
	float gxd, gyd, gzd;
	float mxd, myd, mzd;

	float Roll, Pitch, Yaw;
	float AileronCmd, ElevatorCmd, RudderCmd, ThrottleCmd;
	int Rpm0, Rpm1, Rpm2, Rpm3;

	Matrix33 Tec2b;  // ECEF to body frame rotational matrix
	Matrix33 Tb2ec;  // body to ECEF frame rotational matrix 
	Matrix33 Tl2b;   // local to body frame matrix copy for immediate local use
	Matrix33 Tb2l;   // body to local frame matrix copy for immediate local use
	Matrix33 Tl2ec;  // local to ECEF matrix copy for immediate local use
	Matrix33 Tec2l;  // ECEF to local frame matrix copy for immediate local use
	Matrix33 Tec2i;  // ECEF to ECI frame matrix copy for immediate local use
	Matrix33 Ti2ec;  // ECI to ECEF frame matrix copy for immediate local use
	Matrix33 Ti2b;   // ECI to body frame rotation matrix
	Matrix33 Tb2i;   // body to ECI frame rotation matrix
	Matrix33 Ti2l;   // ECI to body frame rotation matrix
	Matrix33 Tl2i;   // local to inertial frame rotation matrix
	const Matrix33 Tap2b = Matrix33(1.0,  0.0,  0.0,
	                                0.0, -1.0,  0.0,
	                                0.0,  0.0, -1.0 );
	Quaternion AttitudeLocal;
	Quaternion AttitudeECI;

	eIntegrateType integrator_translational_rate;
	eIntegrateType integrator_translational_position;

	std::deque <ColumnVector3> dqUVWidot;
	std::deque <ColumnVector3> dqInertialVelocity;

	// Constants for the low-pass filters
	const float timeConstant = 0.18f;

	float filterFactorG;
	float filterFactorA;
	
	float deltatimeG;
	float deltatimeA;

	// Timestamps for the low-pass filters
	std::chrono::high_resolution_clock::time_point timestampG;
	std::chrono::high_resolution_clock::time_point timestampGOld;

	std::chrono::high_resolution_clock::time_point timestampA;
	std::chrono::high_resolution_clock::time_point timestampAOld;
		
	// linear accelerations and smooth accelerations components for the
	// Wikipedia hight pass and low-pass filters
	ColumnVector3 linearAcceleration; //high-pass
	float axdPrev; //high-pass
	float aydPrev; //high-pass
	float azdPrev; //high-pass
	ColumnVector3 smoothAcceleration; //low-pass

	void FilterAcceleration();

	void ComputeAngles();

	void ComputePosition();
	
	void Integrate(ColumnVector3& Integrand, ColumnVector3& Val, std::deque<ColumnVector3>& ValDot, double deltat, eIntegrateType integration_type);
	
	void CalculateUVW(void);

	void UpdateLocationMatrices();

	void UpdateBodyMatrices();
};