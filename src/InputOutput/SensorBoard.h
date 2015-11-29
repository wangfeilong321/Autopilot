#ifndef SENSOR_BOARD_H
#define SENSOR_BOARD_H

#include <string>
#include <ppltasks.h>
#include <vector>
#include <sstream>
#include <Interface.h>
#include <Base.h>

using namespace Platform;
using namespace Windows::Foundation;
using namespace Windows::Devices::I2c;
using namespace Windows::Devices::Enumeration;

class SensorBoard : public Interface {
public:
	SensorBoard();
	virtual ~SensorBoard() = default;

	virtual bool Run();
	virtual bool Connected();
	virtual void Connect();

	std::vector<float> GetAngles();
	
protected:
	I2cDevice^ MakeDevice(int slaveAddress, _In_opt_ String^ friendlyName);
	
	class wexception {
	public:
		explicit wexception(const std::wstring &msg) : msg_(msg) {}
		virtual const wchar_t *wwhat() const { return msg_.c_str(); }
	private:
		std::wstring msg_;
	};

private:
	bool readBytes(I2cDevice^ Device, uint8_t Register, uint8_t numBytesToRead, uint8_t * DestBuffer);
	uint8_t readByte(I2cDevice^ Device, uint8_t Register);
	bool writeCommand(I2cDevice^ Device, uint8_t Register, uint8_t Command);

	bool readAccelData(int16_t * destination);
	bool readGyroData(int16_t * destination);
	bool readMagnetData(int16_t * destination);
	
private:
	I2cDevice^ Accel;
	I2cDevice^ Gyro;
	I2cDevice^ Magnet;

	bool ifConnected;

	float axd, ayd, azd;
	float gxd, gyd, gzd;
	float mxd, myd, mzd;
	
	float accelXi, accelYi, accelZi;
	float omegaXi, omegaYi, omegaZi;
	
	const byte WHO_AM_I_ADXL345        = 0x00;   // Should return 0xE5
	const byte ADXL345_THRESH_TAP      = 0x1D;   // Tap threshold
	const byte ADXL345_OFSX				     = 0x1E;   // X-axis offset
	const byte ADXL345_OFSY				     = 0x1F;   // Y-axis offset
	const byte ADXL345_OFSZ				     = 0x20;   // Z-axis offset
	const byte ADXL345_DUR				     = 0x21;   // Tap duration
	const byte ADXL345_LATENT			     = 0x22;   // Tap latency
	const byte ADXL345_WINDOW			     = 0x23;   // Tap window
	const byte ADXL345_THRESH_ACT      = 0x24;   // Activity threshold
	const byte ADXL345_THRESH_INACT    = 0x25;   // Inactivity threshold
	const byte ADXL345_TIME_INACT      = 0x26;   // Inactivity time
	const byte ADXL345_ACT_INACT_CTL   = 0x27;   // Axis enable control for activity/inactivity detection
	const byte ADXL345_THRESH_FF       = 0x28;   // Free-fall threshold
	const byte ADXL345_TIME_FF         = 0x29;   // Free-fall time
	const byte ADXL345_TAP_AXES				 = 0x2A;   // Axis control for single/double tap
	const byte ADXL345_ACT_TAP_STATUS  = 0x2B;   // Source of single/double tap
	const byte ADXL345_BW_RATE         = 0x2C;   // Data rate and power mode control
	const byte ADXL345_POWER_CTL       = 0x2D;   // Power-saving features control
	const byte ADXL345_INT_ENABLE      = 0x2E;   // Interrupt enable control
	const byte ADXL345_INT_MAP         = 0x2F;   // Interrupt mapping control
	const byte ADXL345_INT_SOURCE      = 0x30;   // Source of interrupts
	const byte ADXL345_DATA_FORMAT     = 0x31;   // Data format control
	const byte ADXL345_DATAX0          = 0x32;   // X-axis data 0
	const byte ADXL345_DATAX1          = 0x33;   // X-axis data 1
	const byte ADXL345_DATAY0          = 0x34;   // Y-axis data 0
	const byte ADXL345_DATAY1          = 0x35;   // Y-axis data 1
	const byte ADXL345_DATAZ0          = 0x36;   // Z-axis data 0
	const byte ADXL345_DATAZ1          = 0x37;   // Z-axis data 1
	const byte ADXL345_FIFO_CTL        = 0x38;   // FIFO control
	const byte ADXL345_FIFO_STATUS     = 0x39;   // FIFO status
	const byte ADXL345_ADDRESS         = 0x53;   // Device address when ADO = 0 

	const byte WHO_AM_I_L3G4200D       = 0x0F;  // Should return 0xD3
	const byte L3G4200D_CTRL_REG1      = 0x20;
	const byte L3G4200D_CTRL_REG2      = 0x21;
	const byte L3G4200D_CTRL_REG3      = 0x22;
	const byte L3G4200D_CTRL_REG4      = 0x23;
	const byte L3G4200D_CTRL_REG5      = 0x24;
	const byte L3G4200D_REFERENCE      = 0x25;
	const byte L3G4200D_OUT_TEMP       = 0x26;
	const byte L3G4200D_STATUS_REG     = 0x27;
	const byte L3G4200D_OUT_X_L        = 0x28;
	const byte L3G4200D_OUT_X_H        = 0x29;
	const byte L3G4200D_OUT_Y_L        = 0x2A;
	const byte L3G4200D_OUT_Y_H				 = 0x2B;
	const byte L3G4200D_OUT_Z_L        = 0x2C;
	const byte L3G4200D_OUT_Z_H        = 0x2D;
	const byte L3G4200D_FIFO_CTRL_REG  = 0x2E;
	const byte L3G4200D_FIFO_SRC_REG   = 0x2F;
	const byte L3G4200D_INT1_CFG       = 0x30;
	const byte L3G4200D_INT1_SRC       = 0x31;
	const byte L3G4200D_INT1_TSH_XH    = 0x32;
	const byte L3G4200D_INT1_TSH_XL    = 0x33;
	const byte L3G4200D_INT1_TSH_YH    = 0x34;
	const byte L3G4200D_INT1_TSH_YL    = 0x35;
	const byte L3G4200D_INT1_TSH_ZH    = 0x36;
	const byte L3G4200D_INT1_TSH_ZL    = 0x37;
	const byte L3G4200D_INT1_DURATION  = 0x38;
	const byte L3G4200D_ADDRESS        = 0x69;  // Device address when ADO = 0

	const byte HMC5883L_ADDRESS   = 0x1E;
	const byte HMC5883L_CONFIG_A  = 0x00;
	const byte HMC5883L_CONFIG_B  = 0x01;
	const byte HMC5883L_MODE      = 0x02;
	const byte HMC5883L_OUT_X_H   = 0x03;
	const byte HMC5883L_OUT_X_L   = 0x04;
	const byte HMC5883L_OUT_Z_H   = 0x05;
	const byte HMC5883L_OUT_Z_L   = 0x06;
	const byte HMC5883L_OUT_Y_H   = 0x07;
	const byte HMC5883L_OUT_Y_L   = 0x08;
	const byte HMC5883L_STATUS    = 0x09;
	const byte HMC5883L_IDA       = 0x0A;  // should return 0x48
	const byte HMC5883L_IDB       = 0x0B;  // should return 0x34
	const byte HMC5883L_IDC       = 0x0C;  // should return 0x33
		
	enum Ascale {
		AFS_2G = 0,
		AFS_4G,
		AFS_8G,
		AFS_16G
	};

	// Set accelerometer ODR and Bandwidth
	enum Arate {
		ARTBW_010_005 = 0, // 0.1 Hz ODR, 0.05Hz bandwidth
		ARTBW_020_010,
		ARTBW_039_020,
		ARTBW_078_039,
		ARTBW_156_078,
		ARTBW_313_156,
		ARTBW_125_625,
		ARTBW_25_125,
		ARTBW_50_25,
		ARTBW_100_50,
		ARTBW_200_100,
		ARTBW_400_200,
		ARTBW_800_400,
		ARTBW_1600_800,
		ARTBW_3200_1600  // 3200 Hz ODR, 1600 Hz bandwidth
	};
			
	enum Gscale {
		GFS_250DPS = 0,
		GFS_500DPS,
		GFS_1000DPS,
		GFS_2000DPS
	};

	enum Grate { // set gyro ODR and Bandwidth with 4 bits
		GRTBW_100_125 = 0, // 100 Hz ODR, 12.5 Hz bandwidth
		GRTBW_100_25,
		GRTBW_100_25a,
		GRTBW_100_25b,
		GRTBW_200_125,
		GRTBW_200_25,
		GRTBW_200_50,
		GRTBW_200_70,
		GRTBW_400_20,
		GRTBW_400_25,
		GRTBW_400_50,
		GRTBW_400_110,
		GRTBW_800_30,
		GRTBW_800_35,
		GRTBW_800_50,
		GRTBW_800_110  // 800 Hz ODR, 110 Hz bandwidth   
	};
	
	enum Mrate { // set magnetometer ODR
		MRT_0075 = 0, // 0.75 Hz ODR
		MRT_015,      // 1.5 Hz
		MRT_030,      // 3.0 Hz
		MRT_075,      // 7.5 Hz
		MRT_15,       // 15 Hz
		MRT_30,       // 30 Hz
		MRT_75,       // 75 Hz ODR    
	};

	uint8_t Ascale = AFS_16G;
	uint8_t Arate = ARTBW_3200_1600; // 200 Hz ODR, 100 Hz bandwidth
	uint8_t Gscale = GFS_2000DPS;
	uint8_t Grate = GRTBW_800_110;  // 200 Hz ODR,  50 Hz bandwidth
	uint8_t Mrate = MRT_75;        //  75 Hz ODR 
	float aRes, gRes, mRes;       // scale resolutions per LSB for the sensors
	
	int16_t AccelData[3];
	int16_t GyroData[3];
	int16_t MagnetData[3];
	float magbias[3];

	float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f }; // vector to hold quaternion
	float eInt[3] = { 0.0f, 0.0f, 0.0f }; // vector to hold integral error for Mahony method
	float roll, pitch, yaw;

	// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
	const float gyroMeasError = M_PI * (40.0f / 180.0f);       // gyroscope measurement error in rads/s (shown as 40 deg/s)
	const float gyroMeasDrift = M_PI * (0.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
																														// There is a tradeoff in the beta parameter between accuracy and response speed.
																														// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
																														// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
																														// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
																														// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
																														// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
																														// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
																														// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
	const float beta = sqrtf(3.0f / 4.0f) * gyroMeasError;   // compute beta
	const float zeta = sqrtf(3.0f / 4.0f) * gyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

	const float Kp = 2.0f * 5.0f; // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
	const float Ki = 0.0f;
	const float deltat = 0.001953f;
};

#endif