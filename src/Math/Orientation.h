#ifndef ORIENTATION_H
#define ORIENTATION_H

#include <chrono>
#include <vector>
#include "Base.h"

class Orientation {
public:
	std::vector<double> GetAngles(double ax, double ay, double az, double gx, double gy, double gz, double mx, double my, double mz);

private:
	void Update(double ax, double ay, double az, double gx, double gy, double gz, double mx, double my, double mz, const double deltat);

private:
	double q[4] = { 1.0f, 0.0f, 0.0f, 0.0f }; // vector to hold quaternion
	double roll, pitch, yaw;
	
	// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
	const double gyroMeasError = M_PI * (40.0f / 180.0f);       // gyroscope measurement error in rads/s (shown as 40 deg/s)
	const double gyroMeasDrift = M_PI * (0.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
	// There is a tradeoff in the beta parameter between accuracy and response speed.
	// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
	// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
	// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
	// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
	// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
	// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
	// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
	const double beta = sqrt(3.0f / 4.0f) * gyroMeasError;   // compute beta
	const double zeta = sqrt(3.0f / 4.0f) * gyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
};

#endif
