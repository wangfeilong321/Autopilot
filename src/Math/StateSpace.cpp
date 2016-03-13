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
  ComputeTemperature();
  ComputePressure();
  ComputeAngles();
  ComputePosition();
  return true;
}

void StateSpace::setSensorData(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, int16_t ut, long up) {
  axd = ax; ayd = ay; azd = az;
  gxd = gx*degtorad; gyd = gy*degtorad; gzd = gz*degtorad;
  mxd = mx; myd = my; mzd = mz;
  utd = ut;
  upd = up;
}

void StateSpace::setBMPCalibrationData(int16_t ac1, int16_t ac2, int16_t ac3, uint16_t ac4, uint16_t ac5, uint16_t ac6, int16_t b1, int16_t b2, int16_t mb, int16_t mc, int16_t md, uint8_t OSS) {
  ac1d = ac1;
  ac2d = ac2;
  ac3d = ac3;
  ac4d = ac4;
  ac5d = ac5;
  ac6d = ac6;
  b1d = b1;
  b2d = b2;
  mbd = mb;
  mcd = mc;
  mdd = md;
  OSSD = OSS;
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

float StateSpace::getTemperature() { return temperature; }

float StateSpace::getPressure() { return pressure; }

void StateSpace::ComputeAngles() {
  madgwickAHRSupdate();
  yaw = atan2f(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
  pitch = asinf(2.0f * (q1 * q3 - q0 * q2));
  roll = atan2f(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
  yaw += 10.266f*degtorad; // Declination at Moscow, Russia
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

void StateSpace::ComputePressure() {
  long b6 = b5 - 4000;
  // Calculate B3
  long x1 = (b2d * (b6 * b6) >> 12) >> 11;
  long x2 = (ac2d * b6) >> 11;
  long x3 = x1 + x2;
  long b3 = (((((long)ac1d) * 4 + x3) << OSSD) + 2) >> 2;
  // Calculate B4
  x1 = (ac3d * b6) >> 13;
  x2 = (b1d * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  unsigned long b4 = (ac4d * (unsigned long)(x3 + 32768)) >> 15;

  unsigned long b7 = ((unsigned long)(upd - b3) * (50000 >> OSSD));
  long p = 0;
  if (b7 < 0x80000000)
    p = (b7 << 1) / b4;
  else
    p = (b7 / b4) << 1;

  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  pressure += (x1 + x2 + 3791) >> 4;
}

void StateSpace::ComputeTemperature() {
  long x1 = (((long)utd - (long)ac6d)*(long)ac5d) >> 15;
  long x2 = ((long)mcd << 11) / (x1 + mdd);
  b5 = x1 + x2;
  temperature = static_cast<float>(((b5 + 8) >> 4)) * 0.1f; //Temperature in degrees Celsius. 
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

void StateSpace::madgwickAHRSupdate() {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  if ((mxd == 0.0f) && (myd == 0.0f) && (mzd == 0.0f)) {
    madgwickAHRSupdateIMU();
    return;
  }

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gxd - q2 * gyd - q3 * gzd);
  qDot2 = 0.5f * (q0 * gxd + q2 * gzd - q3 * gyd);
  qDot3 = 0.5f * (q0 * gyd - q1 * gzd + q3 * gxd);
  qDot4 = 0.5f * (q0 * gzd + q1 * gyd - q2 * gxd);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((axd == 0.0f) && (ayd == 0.0f) && (azd == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(axd * axd + ayd * ayd + azd * azd);
    axd *= recipNorm;
    ayd *= recipNorm;
    azd *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = invSqrt(mxd * mxd + myd * myd + mzd * mzd);
    mxd *= recipNorm;
    myd *= recipNorm;
    mzd *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mxd;
    _2q0my = 2.0f * q0 * myd;
    _2q0mz = 2.0f * q0 * mzd;
    _2q1mx = 2.0f * q1 * mxd;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // Reference direction of Earth's magnetic field
    hx = mxd * q0q0 - _2q0my * q3 + _2q0mz * q2 + mxd * q1q1 + _2q1 * myd * q2 + _2q1 * mzd * q3 - mxd * q2q2 - mxd * q3q3;
    hy = _2q0mx * q3 + myd * q0q0 - _2q0mz * q1 + _2q1mx * q2 - myd * q1q1 + myd * q2q2 + _2q2 * mzd * q3 - myd * q3q3;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mzd * q0q0 + _2q1mx * q3 - mzd * q1q1 + _2q2 * myd * q3 - mzd * q2q2 + mzd * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - axd) + _2q1 * (2.0f * q0q1 + _2q2q3 - ayd) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mxd) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - myd) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mzd);
    s1 =  _2q3 * (2.0f * q1q3 - _2q0q2 - axd) + _2q0 * (2.0f * q0q1 + _2q2q3 - ayd) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - azd) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mxd) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - myd) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mzd);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - axd) + _2q3 * (2.0f * q0q1 + _2q2q3 - ayd) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - azd) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mxd) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - myd) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mzd);
    s3 =  _2q1 * (2.0f * q1q3 - _2q0q2 - axd) + _2q2 * (2.0f * q0q1 + _2q2q3 - ayd) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mxd) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - myd) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mzd);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= BETA * s0;
    qDot2 -= BETA * s1;
    qDot3 -= BETA * s2;
    qDot4 -= BETA * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * (1.0f / SAMPLE_FREQUENCY);
  q1 += qDot2 * (1.0f / SAMPLE_FREQUENCY);
  q2 += qDot3 * (1.0f / SAMPLE_FREQUENCY);
  q3 += qDot4 * (1.0f / SAMPLE_FREQUENCY);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void StateSpace::madgwickAHRSupdateIMU() {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gxd - q2 * gyd - q3 * gzd);
  qDot2 = 0.5f * (q0 * gxd + q2 * gzd - q3 * gyd);
  qDot3 = 0.5f * (q0 * gyd - q1 * gzd + q3 * gxd);
  qDot4 = 0.5f * (q0 * gzd + q1 * gyd - q2 * gxd);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((axd == 0.0f) && (ayd == 0.0f) && (azd == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(axd * axd + ayd * ayd + azd * azd);
    axd *= recipNorm;
    ayd *= recipNorm;
    azd *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * axd + _4q0 * q1q1 - _2q1 * ayd;
    s1 = _4q1 * q3q3 - _2q3 * axd + 4.0f * q0q0 * q1 - _2q0 * ayd - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * azd;
    s2 = 4.0f * q0q0 * q2 + _2q0 * axd + _4q2 * q3q3 - _2q3 * ayd - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * azd;
    s3 = 4.0f * q1q1 * q3 - _2q1 * axd + 4.0f * q2q2 * q3 - _2q2 * ayd;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= BETA * s0;
    qDot2 -= BETA * s1;
    qDot3 -= BETA * s2;
    qDot4 -= BETA * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * (1.0f / SAMPLE_FREQUENCY);
  q1 += qDot2 * (1.0f / SAMPLE_FREQUENCY);
  q2 += qDot3 * (1.0f / SAMPLE_FREQUENCY);
  q3 += qDot4 * (1.0f / SAMPLE_FREQUENCY);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float StateSpace::invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}