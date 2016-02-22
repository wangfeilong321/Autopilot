#include <StateSpace.h>

using namespace Windows::System;
using namespace Platform;
using namespace Windows::Foundation;
using namespace Windows::System::Threading;

StateSpace::StateSpace() :
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
	Rpm0(MIN_THROTTLE), Rpm1(MIN_THROTTLE), Rpm2(MIN_THROTTLE), Rpm3(MIN_THROTTLE) {

	integrator_translational_rate = eAdamsBashforth2;
	integrator_translational_position = eAdamsBashforth3;

	vLocation.SetEllipse(vInertial.GetSemiMajor(), vInertial.GetSemiMinor());
	vLocation.SetLongitude(-71.0602777*degtorad);
	vLocation.SetLatitude(42.35832777*degtorad);
	vLocation.SetRadius(vInertial.GetRefRadius() + 38.0477f * meterstofeet);
}

void StateSpace::TrimAircraft() {
	TimeSpan period;
	period.Duration = 1 * 10000000; // 10,000,000 ticks per second.
	ThreadPoolTimer^ PeriodicTimer = ThreadPoolTimer::CreatePeriodicTimer(ref new TimerElapsedHandler([this](ThreadPoolTimer^ source) {
		timer_sec--;
		if (timer_sec == 0) {
			InitializeDerivatives();
			canComputePos = true;
			source->Cancel();
		}
	}), period, ref new TimerDestroyedHandler([&](ThreadPoolTimer^ source) {}));
}

void StateSpace::InitializeDerivatives() {
	ColumnVector3 vApAccel = Gftsec2*ColumnVector3(axd, ayd, azd);
	ColumnVector3 vBodyAccel = Tap2b*vApAccel;
	ColumnVector3 vGravAccel = Tec2i*vInertial.GetGravityJ2(vLocation);
	vUVWidot = Tb2i*(vBodyAccel)+vGravAccel;

	dqUVWidot.resize(5, vUVWidot);
	
	vLocation.SetEarthPositionAngle(0.0);

	Ti2ec = vLocation.GetTi2ec();   // ECI to ECEF transform
	Tec2i = Ti2ec.Transposed();    // ECEF to ECI frame transform
	vInertialPosition = Tec2i * vLocation; // Inertial position 
	UpdateLocationMatrices();
	AttitudeECI = Ti2l.GetQuaternion() * AttitudeLocal; //ECI orientation of the aircraft
	UpdateBodyMatrices();
	vUVW = ColumnVector3(0.0, 0.0, 0.0);
	vVel = Tb2l * vUVW; // Velocity of the body frame wrt ECEF frame expressed in Local frame 
	vInertialVelocity = Tb2i * vUVW + (vInertial.GetOmegaPlanet() * vInertialPosition); // Inertial velocity
	dqInertialVelocity.resize(5, vInertialVelocity);
}

bool StateSpace::Run() {
	ComputeAngles();
	if(canComputePos)
		ComputePosition();
	return true;
}

void StateSpace::setSensorData(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
	axd = ax; ayd = ay; azd = az;
	gxd = gx; gyd = gy; gzd = gz;
	mxd = mx; myd = my; mzd = mz;
}

void StateSpace::setAileron(float aileron) { AileronCmd = aileron; }

void StateSpace::setElevator(float elevator) { ElevatorCmd = elevator; }

void StateSpace::setRudder(float rudder) { RudderCmd = rudder; }

void StateSpace::setThrottle(float throttle) { ThrottleCmd = throttle; }

void StateSpace::setEng0RPM(int Rpmd) { Rpm0 = Rpmd; }

void StateSpace::setEng1RPM(int Rpmd) { Rpm1 = Rpmd; }

void StateSpace::setEng2RPM(int Rpmd) { Rpm2 = Rpmd; }

void StateSpace::setEng3RPM(int Rpmd) { Rpm3 = Rpmd; }

float StateSpace::getEng0Rpm() { return static_cast<float>(((Rpm0 - 1000.0f) / 1000.0f)*MAX_RPM); }

float StateSpace::getEng1Rpm() { return static_cast<float>(((Rpm1 - 1000.0f) / 1000.0f)*MAX_RPM); }

float StateSpace::getEng2Rpm() { return static_cast<float>(((Rpm2 - 1000.0f) / 1000.0f)*MAX_RPM); }

float StateSpace::getEng3Rpm() { return static_cast<float>(((Rpm3 - 1000.0f) / 1000.0f)*MAX_RPM); }

float StateSpace::getRoll() { return Roll; }

float StateSpace::getPitch() { return Pitch; }

float StateSpace::getYaw() { return Yaw; }

float StateSpace::getX() { return vLocation(eX); }

float StateSpace::getY() { return vLocation(eY); }

float StateSpace::getZ() { return vLocation(eZ); }

float StateSpace::getAileron() { return AileronCmd; }

float StateSpace::getElevator() { return ElevatorCmd; }

float StateSpace::getRudder() { return RudderCmd; }

float StateSpace::getThrottle() { return ThrottleCmd; }

void StateSpace::Wait() {
	std::mutex mut;
	std::unique_lock<std::mutex> lk(mut);
	cv.wait(lk);
}

void StateSpace::Release() { cv.notify_one(); }

void StateSpace::ComputeAngles() {
	MadgwickAHRSupdate(gxd*M_PI / 180.0f, gyd*M_PI / 180.0f, gzd*M_PI / 180.0f, axd, ayd, azd, mxd, myd, mzd);
	QuaternionToEuler(&Roll, &Pitch, &Yaw);
	Roll *= degtorad;
	Pitch *= degtorad;
	Yaw *= degtorad;
	AttitudeLocal = Quaternion(Roll, Pitch, Yaw);
}

void StateSpace::ComputePosition() {
	ColumnVector3 vApAccel = Gftsec2*ColumnVector3(axd, ayd, azd);
	ColumnVector3 vBodyAccel = Tap2b*vApAccel;
	ColumnVector3 vGravAccel = Tec2i*vInertial.GetGravityJ2(vLocation);
	vUVWidot = Tb2i*(vBodyAccel) + vGravAccel;

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

	// 5. ECI orientation of the aircraft
	AttitudeECI = Ti2l.GetQuaternion() * AttitudeLocal;

	// 6. Update the "Orientation-based" transformation matrices from the updated
	//    orientation quaternion and vLocation vector.
	UpdateBodyMatrices();

	// Translational position derivative (velocities are integrated in the inertial frame)
	CalculateUVW();

	// Compute vehicle velocity wrt ECEF frame, expressed in Local horizontal frame.
	vVel = Tb2l * vUVW;
}

void StateSpace::Integrate(ColumnVector3& Integrand, ColumnVector3& Val, std::deque<ColumnVector3>& ValDot, double deltat, eIntegrateType integration_type) {
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

void StateSpace::CalculateUVW(void) {
	vUVW = Ti2b * (vInertialVelocity - (vInertial.GetOmegaPlanet() * vInertialPosition));
}

void StateSpace::UpdateLocationMatrices() {
	Tl2ec = vLocation.GetTl2ec(); // local to ECEF transform
	Tec2l = Tl2ec.Transposed();   // ECEF to local frame transform
	Ti2l = vLocation.GetTi2l();   // ECI to local frame transform
	Tl2i = Ti2l.Transposed();     // local to ECI transform
}

void StateSpace::UpdateBodyMatrices() {
	Ti2b = AttitudeECI.GetT();         // ECI to body frame transform
	Tb2i = Ti2b.Transposed();          // body to ECI frame transform
	Tl2b = Ti2b * Tl2i;                // local to body frame transform
	Tb2l = Tl2b.Transposed();          // body to local frame transform
	Tec2b = Ti2b * Tec2i;              // ECEF to body frame transform
	Tb2ec = Tec2b.Transposed();        // body to ECEF frame tranform
}