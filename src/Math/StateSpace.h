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
		mxd(0.0), myd(0.0), mzd(0.0),
		Rpm0(MIN_THROTTLE), Rpm1(MIN_THROTTLE), Rpm2(MIN_THROTTLE), Rpm3(MIN_THROTTLE), 
		AttitudeLocal(0.0, 0.0, 0.0), vUVW(0.0, 0.0, 0.0), vUVWidot(0.0, 0.0, 0.0) {
		
		integrator_translational_rate = eAdamsBashforth2;
		integrator_translational_position = eAdamsBashforth3;

		dqUVWidot.resize(5, ColumnVector3(0.0, 0.0, 0.0));
		dqInertialVelocity.resize(5, ColumnVector3(0.0, 0.0, 0.0));

		vLocation.SetEllipse(vInertial.GetSemiMajor(), vInertial.GetSemiMinor());
		vLocation.SetPositionGeodetic(-71.0602777*degtorad, 42.35832777*degtorad, 38.05*meterstofeet+150*meterstofeet); //Sets the initial position of the aircraft (lon, lat, HAE)

		Ti2ec = vLocation.GetTi2ec();   // ECI to ECEF transform
		Tec2i = Ti2ec.Transposed();    // ECEF to ECI frame transform
		vInertialPosition = Tec2i * vLocation; // Inertial position 
		UpdateLocationMatrices();
		AttitudeECI = Ti2l.GetQuaternion() * AttitudeLocal; //ECI orientation of the aircraft
		UpdateBodyMatrices();
		vVel = Tb2l * vUVW; // Velocity of the body frame wrt ECEF expressed in Local frame 
		vInertialVelocity = Tb2i * vUVW + (vInertial.GetOmegaPlanet() * vInertialPosition); // Inertial velocity
	}

	bool Run() {
		computeAngles();
		computePosition();
		return true;
	}

	void setSensorData(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
		axd = ax; ayd = ay; azd = az;
		gxd = gx; gyd = gy; gzd = gz;
		mxd = mx; myd = my; mzd = mz;
	}

	void setAileron(float aileron) { AileronCmd = aileron; }

	void setElevator(float elevator) { ElevatorCmd = elevator; }

	void setRudder(float rudder) { RudderCmd = rudder; }

	void setThrottle(float throttle) { ThrottleCmd = throttle; }

	void setEng0RPM(int Rpmd) { Rpm0 = Rpmd; }
	
	void setEng1RPM(int Rpmd) { Rpm1 = Rpmd; }
	
	void setEng2RPM(int Rpmd) { Rpm2 = Rpmd; }

	void setEng3RPM(int Rpmd) { Rpm3 = Rpmd; }

	float getEng0Rpm() { return static_cast<float>(((Rpm0 - 1000.0f) / 1000.0f)*MAX_RPM); }

	float getEng1Rpm() { return static_cast<float>(((Rpm1 - 1000.0f) / 1000.0f)*MAX_RPM); }

	float getEng2Rpm() { return static_cast<float>(((Rpm2 - 1000.0f) / 1000.0f)*MAX_RPM); }

	float getEng3Rpm() { return static_cast<float>(((Rpm3 - 1000.0f) / 1000.0f)*MAX_RPM); }

	float getRoll() { return Roll; }

	float getPitch() { return Pitch; }

	float getYaw() { return Yaw; }

	float getAltitude() { return 38.0477f / 0.3028f; }

	float getAileron() { return AileronCmd; }

	float getElevator() { return ElevatorCmd; }

	float getRudder() { return RudderCmd; }

	float getThrottle() { return ThrottleCmd; }
	
	void Wait() {
		std::mutex mut;
		std::unique_lock<std::mutex> lk(mut);
		cv.wait(lk);
	}

	void Release() { cv.notify_one(); }

private:
	std::condition_variable cv;

	enum { eX = 1, eY, eZ };

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

	/// These define the indices use to select the various integrators.
	enum eIntegrateType { eNone = 0, eRectEuler, eTrapezoidal, eAdamsBashforth2, eAdamsBashforth3, eAdamsBashforth4, eBuss1, eBuss2, eLocalLinearization, eAdamsBashforth5 };

	eIntegrateType integrator_translational_rate;
	eIntegrateType integrator_translational_position;

	std::deque <ColumnVector3> dqUVWidot;
	std::deque <ColumnVector3> dqInertialVelocity;

	void computeAngles() {
		MadgwickAHRSupdate(gxd*M_PI / 180.0f, gyd*M_PI / 180.0f, gzd*M_PI / 180.0f, axd, ayd, azd, mxd, myd, mzd);
		QuaternionToEuler(&Roll, &Pitch, &Yaw);
		Roll *= degtorad;
		Pitch *= degtorad;
		Yaw *= degtorad;
		AttitudeLocal = Quaternion(Roll, Pitch, Yaw);
	}

	void computePosition() {
		vUVWidot = Tb2i*Tap2b*ColumnVector3(axd,ayd,azd) + Tec2i * vInertial.GetGravityJ2(vLocation);
		Integrate(vInertialVelocity, vUVWidot, dqUVWidot, dt, integrator_translational_rate);
		Integrate(vInertialPosition, vInertialVelocity, dqInertialVelocity, dt, integrator_translational_position);
		
		// CAUTION : the order of the operations below is very important to get transformation
		// matrices that are consistent with the new state of the vehicle

		// 1. Update the Earth position angle (EPA)
		vLocation.IncrementEarthPositionAngle(vInertial.omega()*dt);

		// 2. Update the Ti2ec and Tec2i transforms from the updated EPA
		Ti2ec = vLocation.GetTi2ec(); // ECI to ECEF transform
		Tec2i = Ti2ec.Transposed();   // ECEF to ECI frame transform

		// 3. Update the location from the updated Ti2ec and inertial position
		vLocation = Ti2ec*vInertialPosition;

		// 4. Update the other "Location-based" transformation matrices from the updated
		//    vLocation vector.
		UpdateLocationMatrices();

		// 5. Update the "Orientation-based" transformation matrices from the updated
		//    orientation quaternion and vLocation vector.
		UpdateBodyMatrices();

		// Translational position derivative (velocities are integrated in the inertial frame)
		CalculateUVW();

		// Compute vehicle velocity wrt ECEF frame, expressed in Local horizontal frame.
		vVel = Tb2l * vUVW;
	}
	
	void Integrate(ColumnVector3& Integrand, ColumnVector3& Val, std::deque<ColumnVector3>& ValDot, double deltat, eIntegrateType integration_type) {
		ValDot.push_front(Val);
		ValDot.pop_back();
		switch (integration_type) {
		case eRectEuler:       Integrand += deltat*ValDot[0];
			break;
		case eTrapezoidal:     Integrand += 0.5*deltat*(ValDot[0] + ValDot[1]);
			break;
		case eAdamsBashforth2: Integrand += deltat*(1.5*ValDot[0] - 0.5*ValDot[1]);
			break;
		case eAdamsBashforth3: Integrand += (1 / 12.0)*deltat*(23.0*ValDot[0] - 16.0*ValDot[1] + 5.0*ValDot[2]);
			break;
		case eAdamsBashforth4: Integrand += (1 / 24.0)*deltat*(55.0*ValDot[0] - 59.0*ValDot[1] + 37.0*ValDot[2] - 9.0*ValDot[3]);
			break;
		case eAdamsBashforth5: Integrand += deltat*((1901. / 720.)*ValDot[0] - (1387. / 360.)*ValDot[1] + (109. / 30.)*ValDot[2] - (637. / 360.)*ValDot[3] + (251. / 720.)*ValDot[4]);
			break;
		case eNone: // do nothing, freeze translational rate
			break;
		case eBuss1:
		case eBuss2:
		case eLocalLinearization:
			throw("Can only use Buss (1 & 2) or local linearization integration methods in for rotational position!");
		default:
			break;
		}
	}
	
	void CalculateUVW(void) {
		vUVW = Ti2b * (vInertialVelocity - (vInertial.GetOmegaPlanet() * vInertialPosition));
	}

	void UpdateLocationMatrices() {
		Tl2ec = vLocation.GetTl2ec(); // local to ECEF transform
		Tec2l = Tl2ec.Transposed();   // ECEF to local frame transform
		Ti2l = vLocation.GetTi2l();   // ECI to local frame transform
		Tl2i = Ti2l.Transposed();     // local to ECI transform
	}

	void UpdateBodyMatrices() {
		Ti2b = AttitudeECI.GetT();         // ECI to body frame transform
		Tb2i = Ti2b.Transposed();          // body to ECI frame transform
		Tl2b = Ti2b * Tl2i;                // local to body frame transform
		Tb2l = Tl2b.Transposed();          // body to local frame transform
		Tec2b = Ti2b * Tec2i;              // ECEF to body frame transform
		Tb2ec = Tec2b.Transposed();        // body to ECEF frame tranform
	}
};