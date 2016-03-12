#include <SensorBoard.h>

using namespace std;

SensorBoard::SensorBoard(const std::shared_ptr<StateSpace>& ISS) : IState(ISS), ifConnected(false) {
  String^ friendlyName;
    
  Accel = MakeDevice(ADXL345_ADDRESS, friendlyName);
  if (!Accel)
    return;
  
  Gyro = MakeDevice(L3G4200D_ADDRESS, friendlyName);
  if ( !Gyro )
    return;
  
  Magnet = MakeDevice(HMC5883L_ADDRESS, friendlyName);
  if ( !Magnet )
    return;
  
  Baro = MakeDevice(BMP085_ADDRESS, friendlyName);
  if ( !Baro )
    return;
  
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
}

void SensorBoard::Connect() {
  uint8_t c = readByte(Accel, WHO_AM_I_ADXL345);
  uint8_t d = readByte(Gyro, WHO_AM_I_L3G4200D);
  uint8_t e = readByte(Magnet, HMC5883L_IDA);  // Read WHO_AM_I register A for HMC5883L
  uint8_t f = readByte(Magnet, HMC5883L_IDB);  // Read WHO_AM_I register B for HMC5883L
  uint8_t g = readByte(Magnet, HMC5883L_IDC);  // Read WHO_AM_I register C for HMC5883L
  uint8_t h = readByte(Baro, 0xAA);  // Read WHO_AM_I register A for BMP085L
  uint8_t i = readByte(Baro, 0xAB);  // Read WHO_AM_I register B for BMP085L
  uint8_t j = readByte(Baro, 0xAC);  // Read WHO_AM_I register C for BMP085L

  bool ifWhoAmI = false;

  if (c == 0xE5 && d == 0xD3 && e == 0x48 && f == 0x34 && g == 0x33 && h != 0xFF && h != 0x00) {
    ifWhoAmI = true;
  }

  // wake up device
  bool ifOkPowerControl1 = writeCommand(Accel, ADXL345_POWER_CTL, 0x00);             // Put device in standby mode and clear sleep bit 2
  Sleep(100);                                                                         // Let device settle down
  bool ifOkPowerControl2 = writeCommand(Accel, ADXL345_POWER_CTL, 0x08);             /* 0x08 puts the accelerometer into measurement (normal) mode */
                                                                                     // Set accelerometer configuration; interrupt active high, left justify MSB
  bool ifOkDataFormat = writeCommand(Accel, ADXL345_DATA_FORMAT, 0x04 | Ascale);  // Set full scale range for the accelerometer 
                                                                                  // Choose ODR and bandwidth
  bool ifOkBWRate = writeCommand(Accel, ADXL345_BW_RATE, Arate);                  // Select normal power operation, and ODR and bandwidth
  bool ifOkEnableINTA = writeCommand(Accel, ADXL345_INT_ENABLE, 0x80);            // Enable data ready interrupt
  bool ifOkEnableINTMap = writeCommand(Accel, ADXL345_INT_MAP, 0x00);             // Enable data ready interrupt on INT_1
  bool ifOkBypassFIFO = writeCommand(Accel, ADXL345_FIFO_CTL, 0x00);              // Disable FIFO

  bool ifOkEnableGyro = writeCommand(Gyro, L3G4200D_CTRL_REG1, Grate << 4 | 0x0F);  // Set gyro ODR to 100 Hz and Bandwidth to 25 Hz, enable normal mode
  bool ifOkEnableINTG = writeCommand(Gyro, L3G4200D_CTRL_REG3, 0x08);               // Push/pull, active high interrupt, enable data ready interrupt 
  bool ifOkGyroScale = writeCommand(Gyro, L3G4200D_CTRL_REG4, Gscale << 4);         // set gyro full scale
  bool ifOkEnableHPF = writeCommand(Gyro, L3G4200D_CTRL_REG5, 0x00);                // Disable FIFO

  bool ifOkEnableRate = writeCommand(Magnet, HMC5883L_CONFIG_A, Mrate << 2); // set rate 
  bool ifOkEnableGain = writeCommand(Magnet, HMC5883L_CONFIG_B, 0x00);       // set gain (bits[7:5]) to maximum resolution of 0.73 mG/LSB
  bool ifOkEnableMode = writeCommand(Magnet, HMC5883L_MODE, 0x80);           // enable continuous data mode

  bool ifOkSelfTest = false;
  //  Perform self-test and calculate temperature compensation bias
  writeCommand(Magnet, HMC5883L_CONFIG_A, 0x71);   // set 8-average, 15 Hz default, positive self-test measurement
  writeCommand(Magnet, HMC5883L_CONFIG_B, 0xA0);   // set gain (bits[7:5]) to 5
  writeCommand(Magnet, HMC5883L_MODE, 0x80);       // enable continuous data mode
  Sleep(150);                                      // wait 150 ms

  uint8_t rawData[6] = { 0, 0, 0, 0, 0, 0 };                      // x/z/y magnet register data stored here
  readBytes(Magnet, HMC5883L_OUT_X_H, 6, &rawData[0]);            // Read the six raw data registers sequentially into data array
  int16_t selfTest[3] = { 0, 0, 0 };
  selfTest[0] = ((int16_t)rawData[0] << 8) | rawData[1];          // Turn the MSB and LSB into a signed 16-bit value
  selfTest[1] = ((int16_t)rawData[4] << 8) | rawData[5];
  selfTest[2] = ((int16_t)rawData[2] << 8) | rawData[3];
  writeCommand(Magnet, HMC5883L_CONFIG_A, 0x00);                  // exit self test

  if (selfTest[0] < 575 && selfTest[0] > 243 && selfTest[1] < 575 && selfTest[1] > 243 && selfTest[2] < 575 && selfTest[2] > 243) {
    ifOkSelfTest = true;
  }
  
  const int16_t ac1 = readByte(Baro, 0xAA) << 8 | readByte(Baro, 0xAB);
  const int16_t ac2 = readByte(Baro, 0xAC) << 8 | readByte(Baro, 0xAD);
  const int16_t ac3 = readByte(Baro, 0xAE) << 8 | readByte(Baro, 0xAF);
  const uint16_t ac4 = readByte(Baro, 0xB0) << 8 | readByte(Baro, 0xB1);
  const uint16_t ac5 = readByte(Baro, 0xB2) << 8 | readByte(Baro, 0xB3);
  const uint16_t ac6 = readByte(Baro, 0xB4) << 8 | readByte(Baro, 0xB5);
  const int16_t b1 = readByte(Baro, 0xB6) << 8 | readByte(Baro, 0xB7);
  const int16_t b2 = readByte(Baro, 0xB8) << 8 | readByte(Baro, 0xB9);
  const int16_t mb = readByte(Baro, 0xBA) << 8 | readByte(Baro, 0xBB);
  const int16_t mc = readByte(Baro, 0xBC) << 8 | readByte(Baro, 0xBD);
  const int16_t md = readByte(Baro, 0xBE) << 8 | readByte(Baro, 0xBF);
  
  IState->setBMPCalibrationData(ac1, ac2, ac3, ac4, ac5, ac6, b1, b2, mb, mc, md, OSS);

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

bool SensorBoard::Connected() {
  return ifConnected;
}

bool SensorBoard::Run() { 
  
  bool IfReadAccelOk = readAccelData(AccelData);
    
  ax = AccelData[0] * aRes;  // get actual a value, this depends on scale being set
  ay = AccelData[1] * aRes;
  az = AccelData[2] * aRes;

  bool IfReadGyroOk = readGyroData(GyroData);

  gx = GyroData[0] * gRes;  // get actual gyro value, this depends on scale being set
  gy = GyroData[1] * gRes;
  gz = GyroData[2] * gRes;

  bool IfReadMagnetOk = readMagnetData(MagnetData);

  mRes = 2.56f; 
  // Conversion to milliGauss, 2.56 mG/LSB for the value of 5 (101)in the register B resolution mode
  // So far, magnetometer bias is calculated and subtracted here manually, should construct an algorithm to do it automatically
  // like the gyro and accelerometer biases
  magbias[0] = -30.;  // User environmental x-axis correction in milliGauss
  magbias[1] = +85.;  // User environmental y-axis correction in milliGauss
  magbias[2] = -78.;  // User environmental z-axis correction in milliGauss

  // Calculate the magnetometer values in milliGauss
  // Include factory calibration per data sheet and user environmental corrections
  // get actual magnetometer value, this depends on scale being set
  mx = MagnetData[0] * mRes - magbias[0];
  my = MagnetData[1] * mRes - magbias[1];
  mz = MagnetData[2] * mRes - magbias[2];
  
  // Read raw temperature data
  bool ifReadTempOk = readTempData();
  
  //Read raw pressure data
  bool ifReadBaroOk = readBaroData();
  
  if (IfReadAccelOk && IfReadGyroOk && IfReadMagnetOk && ifReadTempOk && ifReadBaroOk) {
    IState->setMARGData(ax, ay, az, gx, gy, gz, mx, my, mz);
    IState->setBMPData(ut, up);
    return true;
  }

  return false;
}

I2cDevice^ SensorBoard::MakeDevice(int slaveAddress, _In_opt_ String^ friendlyName) {
  String^ aqs;
  if (friendlyName)
    aqs = I2cDevice::GetDeviceSelector(friendlyName);
  else
    aqs = I2cDevice::GetDeviceSelector();
  
  auto dis = concurrency::create_task(DeviceInformation::FindAllAsync(aqs)).get();

  if (dis->Size != 1)
    throw exception("I2C bus not found");
  
  String^ id = dis->GetAt(0)->Id;

  auto _device = concurrency::create_task(I2cDevice::FromIdAsync(id, ref new I2cConnectionSettings(slaveAddress))).get();

  if (!_device) {
    std::ostringstream msg;
    msg << "Slave address 0x" << std::hex << slaveAddress << " on bus " << id->Data() << " is in use. Please ensure that no other applications are using I2C.";
    throw exception(msg.str());
  }
  return _device;
}

bool SensorBoard::readAccelData(int16_t* destination) {
  uint8_t data[6] = { 0, 0, 0, 0, 0, 0 };
  if(readBytes(Accel, ADXL345_DATAX0, 6, data)) {
    destination[0] = ((int16_t)data[1] << 8) | data[0];  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)data[3] << 8) | data[2];
    destination[2] = ((int16_t)data[5] << 8) | data[4];
    return true;
  }
  return false;
}

bool SensorBoard::readGyroData(int16_t* destination) {
  uint8_t data[6] = { 0, 0, 0, 0, 0, 0 };
  if(readBytes(Gyro, L3G4200D_OUT_X_L | 0x80, 6, data)) {
    destination[0] = ((int16_t)data[1] << 8) | data[0];  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)data[3] << 8) | data[2];
    destination[2] = ((int16_t)data[5] << 8) | data[4];
    return true;
  }
  return false;
}

bool SensorBoard::readMagnetData(int16_t* destination) {
  uint8_t data[6];
  if(readBytes(Magnet, HMC5883L_OUT_X_H, 6, data)) { 
    destination[0] = ((int16_t)data[0] << 8) | data[1]; // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)data[4] << 8) | data[5];
    destination[2] = ((int16_t)data[2] << 8) | data[3];
    return true;
  }
  return false;
}

bool SensorBoard::readTempData() {
  writeCommand(Baro, 0xF4, 0x2E); // start temperature measurement
  Sleep(5);
  // read raw temperature measurement
  uint8_t data[2];
  if (readBytes(Baro, 0xF6, 2, data)) {
    ut = (int16_t)(((int16_t)data[0] << 8) | data[1]);
    return true;
  }
  else return false;
}

bool SensorBoard::readBaroData() {
  writeCommand(Baro, 0xF4, 0x34 | OSS << 6); // Start pressure measurement
  Sleep(29);
  // read raw pressure measurement of 19 bits
  uint8_t data[3];
  if(readBytes(Baro, 0xF6, 3, data)) {
    up = (((long)data[0] << 16) | ((long)data[1] << 8) | data[2]) >> (8 - OSS);
  } else return false;
  return true;
}

bool SensorBoard::readBytes(I2cDevice^ Device, uint8_t Register, uint8_t numBytesToRead, uint8_t * DestBuffer) {
  vector<BYTE> WriteBuf = { Register };
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
    case I2cTransferStatus::PartialTransfer:
      break;
    case I2cTransferStatus::SlaveAddressNotAcknowledged:
      throw exception("Slave address was not acknowledged");
    default:
      throw exception("Invalid transfer status value");
  }
  return ifOk;
}

bool SensorBoard::writeCommand(I2cDevice^ Device, uint8_t Register, uint8_t Command) {
  vector<BYTE> WriteBuf = { Register, Command };
  I2cTransferResult result = Device->WritePartial( ArrayReference<BYTE>( WriteBuf.data(), static_cast<unsigned int>(WriteBuf.size()) ) );
  bool ifOk = false;
  switch (result.Status) {
    case I2cTransferStatus::FullTransfer: {
      ifOk = true;
      break;
    }
    case I2cTransferStatus::PartialTransfer:
      break;
    case I2cTransferStatus::SlaveAddressNotAcknowledged:
      throw exception("Slave address was not acknowledged");
    default:
      throw exception("Invalid transfer status value");
  }
  return ifOk;
}

uint8_t SensorBoard::readByte(I2cDevice^ Device, uint8_t Register) {
  vector<BYTE> WriteBuf = { Register };
  auto ReadBuf = ref new Array<BYTE>(1);
  I2cTransferResult result = Device->WriteReadPartial(ArrayReference<BYTE>(WriteBuf.data(), static_cast<unsigned int>(WriteBuf.size())), ReadBuf);
  uint8_t data = 0;
  switch (result.Status) {
    case I2cTransferStatus::FullTransfer: {
      data = ReadBuf[0];
      break;
    }
    case I2cTransferStatus::PartialTransfer:
      break;
    case I2cTransferStatus::SlaveAddressNotAcknowledged:
      throw exception("Slave address was not acknowledged");
    default:
      throw exception("Invalid transfer status value");
  }
  return data;
}