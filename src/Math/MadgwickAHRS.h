#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

#if __cplusplus
extern "C" {
#endif

	extern void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
	extern void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
	extern void QuaternionToEuler(float* roll, float* pitch, float* yaw);

#if __cplusplus
}
#endif

#endif