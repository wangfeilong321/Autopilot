#include "InputDevice.h"
#include <iostream>

using namespace std;

InputDevice::InputDevice() {
	String^ friendlyName;
	Accel = MakeDevice(ADXL345_ADDRESS, friendlyName);
	if ( !Accel )
		return;
	Gyro = MakeDevice(L3G4200D_ADDRESS, friendlyName);
	if ( !Gyro )
		return;
	Magnet = MakeDevice(HMC5883L_ADDRESS, friendlyName);
	if ( !Magnet )
		return;

	auto gpio = GpioController::GetDefault();
	if ( !gpio ) {
		return;
	}

	pin = gpio->OpenPin(LED_PIN);
	if ( !pin )
		return;
	pin->Write(pinValue);
	pin->SetDriveMode(GpioPinDriveMode::Output);

	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
	switch (Gscale) {
		case GFS_250DPS:
			gRes = 250.0 / 32768.0;
			break;
		case GFS_500DPS:
			gRes = 500.0 / 32768.0;
			break;
		case GFS_1000DPS:
			gRes = 1000.0 / 32768.0;
			break;
		case GFS_2000DPS:
			gRes = 2000.0 / 32768.0;
			break;
	}

	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
	switch (Ascale) {
		case AFS_2G:
			aRes = 2.0 / (512.*64.);   // 10-bit 2s-complement
			break;
		case AFS_4G:
			aRes = 4.0 / (1024.*32.);  // 11-bit 2s-complement
			break;
		case AFS_8G:
			aRes = 8.0 / (2048.*16.);  // 12-bit 2s-complement
			break;
		case AFS_16G:
			aRes = 16.0 / (4096.*8.);  // 13-bit 2s-complement
			break;
	}
	ax = ay = az = 0.0;
	gx = gy = gz = 0.0;
	mx = my = mz = 0.0;
}

void InputDevice::OnTick() {
	if (pinValue == GpioPinValue::High) {
		pinValue = GpioPinValue::Low;
		pin->Write(pinValue);
	}
	else {
		pinValue = GpioPinValue::High;
		pin->Write(pinValue);
	}
}

InputDevice::~InputDevice() {}

bool InputDevice::Run() { 
	
	bool IfReadAccelOk = readAccelData(AccelData);
		
	axd = (double)AccelData[0] * aRes;  // get actual g value, this depends on scale being set
	ayd = (double)AccelData[1] * aRes;
	azd = (double)AccelData[2] * aRes;

	//const double alpha = 0.5;

	//Low Pass Filter
	//ax = axd * alpha - (ax * (1.0 - alpha));
	//ay = ayd * alpha - (ay * (1.0 - alpha));
	//az = azd * alpha - (az * (1.0 - alpha));

	bool IfReadGyroOk = readGyroData(GyroData);

	gxd = (double)GyroData[0] * gRes;  // get actual gyro value, this depends on scale being set
	gyd = (double)GyroData[1] * gRes;
	gzd = (double)GyroData[2] * gRes;

	//Low Pass Filter
	//gx = gxd * alpha - (gx * (1.0 - alpha));
	//gy = gyd * alpha - (gy * (1.0 - alpha));
	//gz = gzd * alpha - (gz * (1.0 - alpha));

	bool IfReadMagnetOk = readMagnetData(MagnetData);

	mRes = 0.73; 
	// Conversion to milliGauss, 0.73 mG/LSB in hihgest resolution mode
	// So far, magnetometer bias is calculated and subtracted here manually, should construct an algorithm to do it automatically
	// like the gyro and accelerometer biases
	magbias[0] = -30.;  // User environmental x-axis correction in milliGauss
	magbias[1] = +85.;  // User environmental y-axis correction in milliGauss
	magbias[2] = -78.;  // User environmental z-axis correction in milliGauss

	// Calculate the magnetometer values in milliGauss
	// Include factory calibration per data sheet and user environmental corrections
	// get actual magnetometer value, this depends on scale being set
	mxd = (double)MagnetData[0] * mRes - magbias[0];
	myd = (double)MagnetData[1] * mRes - magbias[1];
	mzd = (double)MagnetData[2] * mRes - magbias[2];
	
	//Low Pass Filter
	//mx = mxd * alpha - (mx * (1.0 - alpha));
	//my = myd * alpha - (my * (1.0 - alpha));
	//mz = mzd * alpha - (mz * (1.0 - alpha));

	return IfReadAccelOk && IfReadGyroOk && IfReadMagnetOk;
}

std::vector<double> InputDevice::GetSensorInput() {

	vector<double> sensorInput;

	sensorInput.push_back(axd);
	sensorInput.push_back(ayd);
	sensorInput.push_back(azd);

	sensorInput.push_back(gxd);
	sensorInput.push_back(gyd);
	sensorInput.push_back(gzd);

	sensorInput.push_back(mxd);
	sensorInput.push_back(myd);
	sensorInput.push_back(mzd);

	return sensorInput;
}

void InputDevice::Connect() {

	uint8_t c = readByte(Accel, WHO_AM_I_ADXL345);

	uint8_t d = readByte(Gyro, WHO_AM_I_L3G4200D);

	uint8_t e = readByte(Magnet, HMC5883L_IDA);  // Read WHO_AM_I register A for HMC5883L
	uint8_t f = readByte(Magnet, HMC5883L_IDB);  // Read WHO_AM_I register B for HMC5883L
	uint8_t g = readByte(Magnet, HMC5883L_IDC);  // Read WHO_AM_I register C for HMC5883L

	bool ifWhoAmI = false;

	if (c == 0xE5 && d == 0xD3 && e == 0x48 && f == 0x34 && g == 0x33) {
		ifWhoAmI = true;
	}

	// wake up device
	bool ifOkPowerControl1 = writeCommand(Accel, ADXL345_POWER_CTL, 0x00);             // Put device in standby mode and clear sleep bit 2
	Sleep(100);																																				 // Let device settle down
	bool ifOkPowerControl2 = writeCommand(Accel, ADXL345_POWER_CTL, 0x08);             /* 0x08 puts the accelerometer into measurement (normal) mode */
																																										 // Set accelerometer configuration; interrupt active high, left justify MSB
	bool ifOkDataFormat = writeCommand(Accel, ADXL345_DATA_FORMAT, 0x04 | Ascale);  // Set full scale range for the accelerometer 
																																										 // Choose ODR and bandwidth
	bool ifOkBWRate = writeCommand(Accel, ADXL345_BW_RATE, Arate);              // Select normal power operation, and ODR and bandwidth
	bool ifOkEnableINTA = writeCommand(Accel, ADXL345_INT_ENABLE, 0x80);            // Enable data ready interrupt
	bool ifOkEnableINTMap = writeCommand(Accel, ADXL345_INT_MAP, 0x00);                // Enable data ready interrupt on INT_1
	bool ifOkBypassFIFO = writeCommand(Accel, ADXL345_FIFO_CTL, 0x00);               // Bypass FIFO

	bool ifOkEnableGyro = writeCommand(Gyro, L3G4200D_CTRL_REG1, Grate << 4 | 0x0F);  // Set gyro ODR to 100 Hz and Bandwidth to 25 Hz, enable normal mode
	bool ifOkEnableINTG = writeCommand(Gyro, L3G4200D_CTRL_REG3, 0x08);               // Push/pull, active high interrupt, enable data ready interrupt 
	bool ifOkGyroScale = writeCommand(Gyro, L3G4200D_CTRL_REG4, Gscale << 4);        // set gyro full scale
	bool ifOkEnableHPF = writeCommand(Gyro, L3G4200D_CTRL_REG5, 0x00);               // Disable FIFO

	bool ifOkEnableRate = writeCommand(Magnet, HMC5883L_CONFIG_A, Mrate << 2);
	bool ifOkEnableGain = writeCommand(Magnet, HMC5883L_CONFIG_B, 0x00);       // set gain (bits[7:5]) to maximum resolution of 0.73 mG/LSB
	bool ifOkEnableMode = writeCommand(Magnet, HMC5883L_MODE, 0x80);           // enable continuous data mode

	bool ifOkSelfTest = false;
	//  Perform self-test and calculate temperature compensation bias
	writeCommand(Magnet, HMC5883L_CONFIG_A, 0x71);   // set 8-average, 15 Hz default, positive self-test measurement
	writeCommand(Magnet, HMC5883L_CONFIG_B, 0xA0);   // set gain (bits[7:5]) to 5
	writeCommand(Magnet, HMC5883L_MODE, 0x80);      // enable continuous data mode
	Sleep(150); // wait 150 ms

	uint8_t rawData[6] = { 0, 0, 0, 0, 0, 0 };                        // x/y/z gyro register data stored here
	readBytes(Magnet, HMC5883L_OUT_X_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	int16_t selfTest[3] = { 0, 0, 0 };
	selfTest[0] = ((int16_t)rawData[0] << 8) | rawData[1];          // Turn the MSB and LSB into a signed 16-bit value
	selfTest[1] = ((int16_t)rawData[4] << 8) | rawData[5];
	selfTest[2] = ((int16_t)rawData[2] << 8) | rawData[3];
	writeCommand(Magnet, HMC5883L_CONFIG_A, 0x00);   // exit self test

	if (selfTest[0] < 575 && selfTest[0] > 243 && selfTest[1] < 575 && selfTest[1] > 243 && selfTest[2] < 575 && selfTest[2] > 243) {
		ifOkSelfTest = true;
	}

	ifConnected =
		ifWhoAmI          &&
		ifOkPowerControl1 &&
		ifOkPowerControl2 &&
		ifOkDataFormat    &&
		ifOkBWRate        &&
		ifOkEnableINTA    &&
		ifOkEnableINTMap  &&
		ifOkBypassFIFO    &&
		ifOkEnableGyro    &&
		ifOkEnableINTG    &&
		ifOkGyroScale     &&
		ifOkEnableHPF     &&
		ifOkEnableRate    &&
		ifOkEnableGain    &&
		ifOkEnableMode    &&
		ifOkSelfTest;
}

I2cDevice^ InputDevice::MakeDevice(int slaveAddress, _In_opt_ String^ friendlyName) {
	using namespace Windows::Devices::Enumeration;

	String^ aqs;
	if (friendlyName)
		aqs = I2cDevice::GetDeviceSelector(friendlyName);
	else
		aqs = I2cDevice::GetDeviceSelector();

	auto dis = concurrency::create_task(DeviceInformation::FindAllAsync(aqs)).get();
	if (dis->Size != 1) {
		throw wexception(L"I2C bus not found");
	}

	String^ id = dis->GetAt(0)->Id;
	auto _device = concurrency::create_task(I2cDevice::FromIdAsync(id, ref new I2cConnectionSettings(slaveAddress))).get();

	if (!_device) {
		std::wostringstream msg;
		msg << L"Slave address 0x" << std::hex << slaveAddress << L" on bus " << id->Data() << L" is in use. Please ensure that no other applications are using I2C.";
		throw wexception(msg.str());
	}
	return _device;
}

bool InputDevice::IfGetData() { 
	return true;
}

bool InputDevice::Connected() {
	return ifConnected;
}

bool InputDevice::readAccelData(int16_t * Destination) {
	uint8_t data[6] = { 0, 0, 0, 0, 0, 0 };
	bool IfReadOk = readBytes(Accel, ADXL345_DATAX0, 6, data);

	Destination[0] = ((int16_t)data[1] << 8) | data[0];  // Turn the MSB and LSB into a signed 16-bit value
	Destination[1] = ((int16_t)data[3] << 8) | data[2];
	Destination[2] = ((int16_t)data[5] << 8) | data[4];
	return IfReadOk;
}

bool InputDevice::readGyroData(int16_t * Destination) {
	uint8_t data[6] = { 0, 0, 0, 0, 0, 0 };
	bool IfReadOk = readBytes(Gyro, L3G4200D_OUT_X_L | 0x80, 6, data);

	Destination[0] = ((int16_t)data[1] << 8) | data[0];  // Turn the MSB and LSB into a signed 16-bit value
	Destination[1] = ((int16_t)data[3] << 8) | data[2];
	Destination[2] = ((int16_t)data[5] << 8) | data[4];
	return IfReadOk;
}

bool InputDevice::readMagnetData(int16_t * destination) {
	uint8_t data[6];																		// x/y/z gyro register data stored here
	bool IfReadOk = readBytes(Magnet, HMC5883L_OUT_X_H, 6, data);				// Read the six raw data registers sequentially into data array
	
	destination[0] = ((int16_t)data[0] << 8) | data[1]; // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)data[4] << 8) | data[5];
	destination[2] = ((int16_t)data[2] << 8) | data[3];
	return IfReadOk;
}

bool InputDevice::readBytes(I2cDevice^ Device, uint8_t Register, uint8_t numBytesToRead, uint8_t * DestBuffer) {
	std::vector<BYTE> WriteBuf = { Register };
	auto ReadBuf = ref new Array<BYTE>(numBytesToRead);
	I2cTransferResult result = Device->WriteReadPartial( ArrayReference<BYTE>(WriteBuf.data(), static_cast<unsigned int>(WriteBuf.size()) ), ReadBuf);
	bool ifOk = false;
	switch (result.Status) {
		case I2cTransferStatus::FullTransfer: {
			for (int i = 0; i < numBytesToRead; ++i)
				DestBuffer[i] = ReadBuf[i];
			ifOk = true;
			break;
		}
		case I2cTransferStatus::PartialTransfer: {
			break;
		}
		case I2cTransferStatus::SlaveAddressNotAcknowledged:
			break;
		default:
			throw wexception(L"Invalid transfer status value");
	}
	return ifOk;
}

bool InputDevice::writeCommand(I2cDevice^ Device, uint8_t Register, uint8_t Command) {
	std::vector<BYTE> WriteBuf = { Register, Command };
	I2cTransferResult result = Device->WritePartial( ArrayReference<BYTE>( WriteBuf.data(), static_cast<unsigned int>(WriteBuf.size()) ) );
	bool ifOk = false;
	switch (result.Status) {
		case I2cTransferStatus::FullTransfer: {
			ifOk = true;
			break;
		}
		case I2cTransferStatus::PartialTransfer:
			std::wcout << L"Partial Transfer. Transferred " << result.BytesTransferred << L" bytes\n";
			break;
		case I2cTransferStatus::SlaveAddressNotAcknowledged:
			std::wcout << L"Slave address was not acknowledged\n";
			break;
		default:
			throw wexception(L"Invalid transfer status value");
	}
	return ifOk;
}

uint8_t InputDevice::readByte(I2cDevice^ Device, uint8_t Register) {
	std::vector<BYTE> WriteBuf = { Register };
	auto ReadBuf = ref new Array<BYTE>(1);
	I2cTransferResult result = Device->WriteReadPartial(ArrayReference<BYTE>(WriteBuf.data(), static_cast<unsigned int>(WriteBuf.size())), ReadBuf);
	uint8_t data = 0;
	switch (result.Status) {
		case I2cTransferStatus::FullTransfer: {
			data = ReadBuf[0];
			break;
		}
		case I2cTransferStatus::PartialTransfer: {
			break;
		}
		case I2cTransferStatus::SlaveAddressNotAcknowledged:
			break;
		default:
			throw wexception(L"Invalid transfer status value");
	}
	return data;
}