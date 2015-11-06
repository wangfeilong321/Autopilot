#include "Orientation.h"
#include "MadgwickAHRS.h"

using namespace std;

// In the Gy-80, all three orientation sensors' x-, y-, and z-axes are aligned;
// the magnetometer z-axis (+ up) is parallel to z-axis (+ up) of accelerometer and gyro!
// We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
// For the GY-80, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
// in the LSM9DS0 sensor. We negate the z-axis magnetic field to conform to AHRS convention of magnetic z-axis down.
// This rotation can be modified to allow any convenient orientation convention.
// This is ok by aircraft orientation standards!  
// Pass gyro rate as rad/s

std::vector<double> Orientation::GetAngles(double ax, double ay, double az, double gx, double gy, double gz, double mx, double my, double mz) {
	static std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start);
	//double deltat = (duration.count() / 1000000.0f); // set integration time by time elapsed since last filter update
	const double deltat = 0.001953;
	start = std::chrono::high_resolution_clock::now();

	MadgwickAHRSupdate(gx*M_PI / 180.0f, gy*M_PI / 180.0f, gz*M_PI / 180.0f, ax, ay, az, mx, my, mz);
	
	QuaternionToEuler(&roll, &pitch, &yaw);

	vector<double> angles;
	angles.push_back(roll);
	angles.push_back(pitch);
	angles.push_back(yaw);

	return angles;
}