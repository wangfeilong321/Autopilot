#pragma once

#include <Base.h>
#include <Location.h>
#include <Inertial.h>
#include <Matrix33.h>
#include <Quaternion.h>
#include <ColumnVector3.h>

#include <cmath>
#include <deque>

class StateSpace {
public:
  StateSpace();
  
  void initializeDerivatives();
  
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
  
  float roll, pitch, yaw;
  float aileronCmd, elevatorCmd, rudderCmd, throttleCmd;
  int rpm0, rpm1, rpm2, rpm3;

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
  
  Quaternion attitudeLocal;
  Quaternion attitudeECI;

  eIntegrateType integrator_translational_rate;
  eIntegrateType integrator_translational_position;
  
  std::deque <ColumnVector3> dqUVWidot;
  std::deque <ColumnVector3> dqInertialVelocity;

	const float SAMPLE_FREQUENCY = 256.0f;         		        // sample frequency in Hz
	const float BETA = 0.2f;								                  // 2 * proportional gain (Kp) == algorithm gain
	float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	        // quaternion of sensor frame relative to auxiliary frame
  
  void ComputeAngles();
  
  void ComputePosition();
  
  void Integrate(ColumnVector3& Integrand, ColumnVector3& Val, std::deque<ColumnVector3>& ValDot, double deltat, eIntegrateType integration_type);
  
  void CalculateUVW(void);
  
  void UpdateLocationMatrices();
  
  void UpdateBodyMatrices();

	void madgwickAHRSupdate();
	void madgwickAHRSupdateIMU();
	
	float invSqrt(float x);
};