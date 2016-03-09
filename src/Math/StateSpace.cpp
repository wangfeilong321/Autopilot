#include <StateSpace.h>

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

  //Set integrators
  integrator_translational_rate = eAdamsBashforth2;
  integrator_translational_position = eAdamsBashforth3;
  
  //Set initial geodetic position for the aircraft
  vLocation.SetEllipse(vInertial.GetSemiMajor(), vInertial.GetSemiMinor());
  vLocation.SetPositionGeodetic(-71.0602777*degtorad, 42.35832777*degtorad, (38.047786 + 20)*meterstofeet);
  vLocation.SetEarthPositionAngle(0.0);
  //Set initial conditions for calculation
  initializeDerivatives();
}

void StateSpace::initializeDerivatives() {
  //Calculate just linear acceleration from shock or move (no gravity, no normal, no friction, no centripetal, no Coriolis forces) 
  vUVWidot = ColumnVector3(-0.0032211f, 0.00933534f, -0.0793486f);
  //Setup initial samples
  dqUVWidot.resize(5, vUVWidot);
	
  Ti2ec = vLocation.GetTi2ec();   // ECI to ECEF transform
  Tec2i = Ti2ec.Transposed();    // ECEF to ECI frame transform
  vInertialPosition = Tec2i * vLocation; // Inertial position 
  UpdateLocationMatrices(); // Update the other "Location-based" transformation matrices from the updated vLocation vector.
  attitudeECI = Ti2l.GetQuaternion() * attitudeLocal; //ECI orientation of the aircraft
  UpdateBodyMatrices(); // Update the "Orientation-based" transformation matrices from the updated orientation quaternion and vLocation vector.
  vUVW = ColumnVector3(0.0, 0.0, 0.0);
  vVel = Tb2l * vUVW; // Velocity of the body frame wrt ECEF frame expressed in Local frame 
  vInertialVelocity = Tb2i * vUVW + (vInertial.GetOmegaPlanet() * vInertialPosition); // Inertial velocity
  //Setup initial samples
  dqInertialVelocity.resize(5, vInertialVelocity);
}

bool StateSpace::Run() {
  ComputeAngles();
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

float StateSpace::getX() const { return static_cast<float>(vLocation(eX)) * feettometers; }

float StateSpace::getY() const { return static_cast<float>(vLocation(eY)) * feettometers; }

float StateSpace::getZ() const { return static_cast<float>(vLocation(eZ)) * feettometers; }

float StateSpace::getAileron() { return aileronCmd; }

float StateSpace::getElevator() { return elevatorCmd; }

float StateSpace::getRudder() { return rudderCmd; }

float StateSpace::getThrottle() { return throttleCmd; }

void StateSpace::ComputeAngles() {
  MadgwickAHRSupdate(gxd*degtorad, gyd*degtorad, gzd*degtorad, axd, ayd, azd, mxd, myd, mzd);
  QuaternionToEuler(&roll, &pitch, &yaw);
  attitudeLocal = Quaternion(roll, pitch, yaw);
}

void StateSpace::ComputePosition() {
  //Calculate just linear acceleration from shock or move (no gravity, no normal, no friction, no centripetal, no Coriolis forces) 
  vUVWidot = ColumnVector3(-0.0032211f,  0.00933534f, - 0.0793486f);
	
  //Double integration to yeild ECI position
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
  attitudeECI = Ti2l.GetQuaternion() * attitudeLocal;

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
  Ti2b = attitudeECI.GetT();         // ECI to body frame transform
  Tb2i = Ti2b.Transposed();          // body to ECI frame transform
  Tl2b = Ti2b * Tl2i;                // local to body frame transform
  Tb2l = Tl2b.Transposed();          // body to local frame transform
  Tec2b = Ti2b * Tec2i;              // ECEF to body frame transform
  Tb2ec = Tec2b.Transposed();        // body to ECEF frame tranform
}