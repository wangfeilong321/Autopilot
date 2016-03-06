#include <StateSpace.h>

using namespace std::chrono;
using namespace Windows::System;
using namespace Platform;
using namespace Windows::Foundation;
using namespace Windows::System::Threading;

using namespace std;

StateSpace::StateSpace() :
	roll(0.0),
	pitch(0.0),
	yaw(0.0),
	aileronCmd(0.0),
	elevatorCmd(0.0),
	rudderCmd(0.0),
	throttleCmd(0.0),
	axd(0.0), ayd(0.0), azd(0.0),
	gxd(0.0), gyd(0.0), gzd(0.0),
	mxd(0.0), myd(0.0), mzd(0.0),
	rpm0(MIN_THROTTLE), rpm1(MIN_THROTTLE), rpm2(MIN_THROTTLE), rpm3(MIN_THROTTLE) {

	integrator_translational_rate = eAdamsBashforth2;
	integrator_translational_position = eAdamsBashforth3;

	vLocation.SetEllipse(vInertial.GetSemiMajor(), vInertial.GetSemiMinor());
	vLocation.SetPositionGeodetic(-71.0602777*degtorad, 42.35832777*degtorad, (38.047786 + 20)*meterstofeet);
	vLocation.SetEarthPositionAngle(0.0);

	accOut.open("C:\\Deploy\\accOut.dat", ofstream::trunc);
	if (!accOut.is_open())
		return;
	linearAccOut.open("C:\\Deploy\\linearAccOut.dat", ofstream::trunc);
	if (!linearAccOut.is_open())
		return;
	smoothAccOut.open("C:\\Deploy\\smoothAccOut.dat", ofstream::trunc);
	if (!smoothAccOut.is_open())
		return;

	//Initial times

	timestamp = high_resolution_clock::now();
	timestampOld = high_resolution_clock::now();

	timestampG = high_resolution_clock::now();
	timestampGOld = high_resolution_clock::now();

	timestampA = high_resolution_clock::now();
	timestampAOld = high_resolution_clock::now();

	t = tA = tG = 0.0f;

	axdPrev = axd;
	aydPrev = ayd;
	azdPrev = azd;
}

void StateSpace::trimAircraft() {
	TimeSpan delay;
	delay.Duration = TRIM_TIMER * 1 * 10000000; // 10,000,000 ticks per second. 15 seconds.
	ThreadPoolTimer ^ delayTimer = ThreadPoolTimer::CreateTimer(
		ref new TimerElapsedHandler([this](ThreadPoolTimer^ source) {
			initializeDerivatives();
			canComputePos = true;
		}), delay);
}

void StateSpace::initializeDerivatives() {
	ColumnVector3 vApAccel = Gftsec2*smoothAcceleration;
	ColumnVector3 vBodyAccel = Tap2b*vApAccel;
	//ColumnVector3 vGravAccel = Tec2i*vInertial.GetGravityJ2(vLocation);
	vUVWidot = Tb2i*(vBodyAccel);// +vGravAccel;

	dqUVWidot.resize(5, vUVWidot);
	
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
	FilterAcceleration();
	if(canComputePos)
		ComputePosition();
	return true;
}

void StateSpace::setSensorData(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
	axd = ax; ayd = ay; azd = az;
	gxd = gx; gyd = gy; gzd = gz;
	mxd = mx; myd = my; mzd = mz;
}

void StateSpace::setAileron(float aileron) { aileronCmd = aileron; }

void StateSpace::setElevator(float elevator) { elevatorCmd = elevator; }

void StateSpace::setRudder(float rudder) { rudderCmd = rudder; }

void StateSpace::setThrottle(float throttle) { throttleCmd = throttle; }

void StateSpace::setEng0RPM(int Rpmd) { rpm0 = Rpmd; }

void StateSpace::setEng1RPM(int Rpmd) { rpm1 = Rpmd; }

void StateSpace::setEng2RPM(int Rpmd) { rpm2 = Rpmd; }

void StateSpace::setEng3RPM(int Rpmd) { rpm3 = Rpmd; }

float StateSpace::getEng0Rpm() { return static_cast<float>(((rpm0 - 1000.0f) / 1000.0f)*MAX_RPM); }

float StateSpace::getEng1Rpm() { return static_cast<float>(((rpm1 - 1000.0f) / 1000.0f)*MAX_RPM); }

float StateSpace::getEng2Rpm() { return static_cast<float>(((rpm2 - 1000.0f) / 1000.0f)*MAX_RPM); }

float StateSpace::getEng3Rpm() { return static_cast<float>(((rpm3 - 1000.0f) / 1000.0f)*MAX_RPM); }

float StateSpace::getRoll() const { return roll; }

float StateSpace::getPitch() const { return pitch; }

float StateSpace::getYaw() const { return yaw; }

float StateSpace::getX() const { return static_cast<float>(vLocation(eX)); }

float StateSpace::getY() const { return static_cast<float>(vLocation(eY)); }

float StateSpace::getZ() const { return static_cast<float>(vLocation(eZ)); }

float StateSpace::getAileron() { return aileronCmd; }

float StateSpace::getElevator() { return elevatorCmd; }

float StateSpace::getRudder() { return rudderCmd; }

float StateSpace::getThrottle() { return throttleCmd; }

void StateSpace::FilterAcceleration() {
	//Current time
	timestamp = high_resolution_clock::now();
	// Find the sample period (between updates)
	// Convert from nanoseconds to seconds
	deltatime = duration_cast<milliseconds>(timestamp - timestampOld).count() * msectosec;
	//Out input axes accelerations 
	accOut << t << "  " << axd << "  " << ayd << "  " << azd << "  " << endl;
	//New previous time value
	timestampOld = timestamp;
	//Update current time
	t += deltatime;

	//HPF
	//Current time
	timestampG = high_resolution_clock::now();
	// Find the sample period (between updates)
	// Convert from nanoseconds to seconds
	deltatimeG = duration_cast<milliseconds>(timestampG - timestampGOld).count() * msectosec;
	//Filter factor for high-pass
	filterFactorG = RC / (RC + deltatimeG);
	//High pass to get linear acceleration
	linearAcceleration(eX) = filterFactorG * linearAcceleration(eX) + filterFactorG * (axd - axdPrev);
	linearAcceleration(eY) = filterFactorG * linearAcceleration(eY) + filterFactorG * (ayd - aydPrev);
	linearAcceleration(eZ) = filterFactorG * linearAcceleration(eZ) + filterFactorG * (azd - azdPrev);
	//Out linear axes accelerations 
	linearAccOut << tG << "  " << linearAcceleration(eX) << "  " << linearAcceleration(eY) << "  " << linearAcceleration(eZ) << "  " << endl;
	//New previous acceleration values
	axdPrev = axd;
	aydPrev = ayd;
	azdPrev = azd;
	//New previous time value
	timestampGOld = timestampG;
	//Update current time
	tG += deltatimeG;

	//LPF
	//Current time
	timestampA = high_resolution_clock::now();
	// Find the sample period (between updates).
	// Convert from nanoseconds to seconds
	deltatimeA = duration_cast<milliseconds>(timestampA - timestampAOld).count() * msectosec;
	//Filter factor for low-pass
	filterFactorA = deltatimeA / (RC + deltatimeA);
	//Low pass to smooth result
	smoothAcceleration(eX) = filterFactorA * linearAcceleration(eX) + (1.0f - filterFactorA) * smoothAcceleration(eX);
	smoothAcceleration(eY) = filterFactorA * linearAcceleration(eY) + (1.0f - filterFactorA) * smoothAcceleration(eY);
	smoothAcceleration(eZ) = filterFactorA * linearAcceleration(eZ) + (1.0f - filterFactorA) * smoothAcceleration(eZ);
	//Out smooth axes accelerations 
	smoothAccOut << tA << "  " << smoothAcceleration(eX) << "  " << smoothAcceleration(eY) << "  " << smoothAcceleration(eZ) << "  " << endl;
	//New previous time value;
	timestampAOld = timestampA;
	//Update current time
	tA += deltatimeA;
}

void StateSpace::ComputeAngles() {
	MadgwickAHRSupdate(gxd*M_PI / 180.0f, gyd*M_PI / 180.0f, gzd*M_PI / 180.0f, axd, ayd, azd, mxd, myd, mzd);
	QuaternionToEuler(&roll, &pitch, &yaw);
	roll *= degtorad;
	pitch *= degtorad;
	yaw *= degtorad;
	AttitudeLocal = Quaternion(roll, pitch, yaw);
}

void StateSpace::ComputePosition() {
	ColumnVector3 vApAccel = Gftsec2*smoothAcceleration;
	ColumnVector3 vBodyAccel = Tap2b*vApAccel;
	//ColumnVector3 vGravAccel = Tec2i*vInertial.GetGravityJ2(vLocation);
	vUVWidot = Tb2i*(vBodyAccel);// +vGravAccel;

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