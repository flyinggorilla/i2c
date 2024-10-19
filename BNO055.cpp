#include "BNO055.h"
#include "esp_log.h"
#include "esp_task.h"
#include "driver/gpio.h"
#include <math.h>

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

// Register Map
#define BNO055_DEVICE_ADDRESS_7BIT (0x28) //!< I2C address/bits, 0011001x
#define BNO055_DEVICE_FREQ_HZ 10000

static const char *tag = "BNO055";

BNO055::BNO055(I2c &rI2c, gpio_num_t intPin) : I2cDevice(rI2c), mIntPin(intPin)
{
  // Constructor implementation
}

void BNO055::ConfigOperationMode(uint8_t mode)
{
  mOperationMode = mode;
}


/// @brief Operation modes for BNO055 sensor
/// These modes define the functionality and fusion algorithms used by the sensor.
/// Refer to the BNO055 datasheet for more details on each mode.
/// Note: Uncomment the desired mode and use it in the ConfigOperationMode function.
/// Configuration mode: BNO055_OPERATION_MODE_CONFIG
/// Accelerometer only mode: BNO055_OPERATION_MODE_ACCONLY
/// Magnetometer only mode: BNO055_OPERATION_MODE_MAGONLY
/// Gyroscope only mode: BNO055_OPERATION_MODE_GYRONLY
/// Accelerometer and magnetometer fusion mode: BNO055_OPERATION_MODE_ACCMAG
/// Accelerometer and gyroscope fusion mode: BNO055_OPERATION_MODE_ACCGYRO
/// Magnetometer and gyroscope fusion mode: BNO055_OPERATION_MODE_MAGGYRO
/// Accelerometer, magnetometer, and gyroscope fusion mode: BNO055_OPERATION_MODE_AMG
/// IMU mode with fusion of accelerometer, magnetometer, and gyroscope: BNO055_OPERATION_MODE_IMUPLUS
/// Compass mode: BNO055_OPERATION_MODE_COMPASS
/// M4G mode: BNO055_OPERATION_MODE_M4G
/// Nine degrees of freedom mode with fast magnetic calibration: BNO055_OPERATION_MODE_NDOF_FMC_OFF
/// Nine degrees of freedom mode: BNO055_OPERATION_MODE_NDOF
/// @param mode
/// @return
bool BNO055::WriteOperationMode(uint8_t mode)
{
  uint8_t data[1] = {};
  data[0] = mode;
  if (!WriteRegister(BNO055_OPR_MODE_ADDR, (uint8_t *)&data, sizeof(data)))
  {
    ESP_LOGE(tag, "Error writing operation mode");
    return false;
  }

  // switching times in ms as per sensor documentation
  vTaskDelay((mode == BNO055_OPERATION_MODE_CONFIG ? 19 : 7) / portTICK_PERIOD_MS);

  mCurrentMode = mode;
  return true;
}

bool BNO055::SwitchToConfigMode()
{
  if (mCurrentMode != BNO055_OPERATION_MODE_CONFIG)
  {
    mCurrentMode = BNO055_OPERATION_MODE_CONFIG;
    return WriteOperationMode(mCurrentMode);
  }
  return true;
}

bool BNO055::SwitchToOperationMode()
{
  if (mCurrentMode != mOperationMode)
  {
    mCurrentMode = mOperationMode;
    return WriteOperationMode(mCurrentMode);
  }
  return true;
}


bool BNO055::Reset()
{
  SwitchToConfigMode();
  
  uint8_t data = BNO055_SYS_RST_MSK;
  if (!WriteRegister(BNO055_SYS_TRIGGER_ADDR, (uint8_t *)&data, sizeof(data)))
  {
    ESP_LOGE(tag, "Error resetting device");
    return false;
  }
  // as to sensor documentation: wait at least 650ms after POR or RESET
  vTaskDelay(680 / portTICK_PERIOD_MS);
  return true;
}

bool BNO055::WriteMountingOrientation(uint8_t mapConfig, uint8_t mapSign)
{
  SwitchToConfigMode();

  uint8_t data[2] = {};
  data[0] = mapSign;
  data[1] = mapConfig;
  if (!WriteRegister(BNO055_AXIS_MAP_SIGN_ADDR, (uint8_t *)&data, sizeof(data)))
  {
    ESP_LOGE(tag, "Error configuring mounting orientation");
    return false;
  }
  return true;
}

bool BNO055::ConfigUnits()
{
  SwitchToConfigMode();

  uint8_t data = 0;
  data |= BNO055_ACCEL_UNIT_MSQ << BNO055_ACCEL_UNIT_POS;
  data |= BNO055_GYRO_UNIT_DPS << BNO055_GYRO_UNIT_POS;
  data |= BNO055_EULER_UNIT_DEG << BNO055_EULER_UNIT_POS;
  data |= BNO055_TEMP_UNIT_CELSIUS << BNO055_TEMP_UNIT_POS;
  data |= BNO055_DATA_OUTPUT_FORMAT_ANDROID << BNO055_DATA_OUTPUT_FORMAT_POS;
  if (!WriteRegister(BNO055_UNIT_SEL_ADDR, (uint8_t *)&data, sizeof(data)))
  {
    ESP_LOGE(tag, "Error configuring units");
    return false;
  }

  return false;
}


bool BNO055::WriteCalibrationValues(IMUData &calibration)
{
  SwitchToConfigMode();

  uint8_t data[18] = {};
  data[0] = calibration.accX & 0x00FF;
  data[1] = (calibration.accX & 0xFF00) >> 8;
  data[2] = calibration.accY & 0x00FF;
  data[3] = (calibration.accY & 0xFF00) >> 8;
  data[4] = calibration.accZ & 0x00FF;
  data[5] = (calibration.accZ & 0xFF00) >> 8;
  data[6] = calibration.magX & 0x00FF;
  data[7] = (calibration.magX & 0xFF00) >> 8;
  data[8] = calibration.magY & 0x00FF;
  data[9] = (calibration.magY & 0xFF00) >> 8;
  data[10] = calibration.magZ & 0x00FF;
  data[11] = (calibration.magZ & 0xFF00) >> 8;
  data[12] = calibration.gyrX & 0x00FF;
  data[13] = (calibration.gyrX & 0xFF00) >> 8;
  data[14] = calibration.gyrY & 0x00FF;
  data[15] = (calibration.gyrY & 0xFF00) >> 8;
  data[16] = calibration.gyrZ & 0x00FF;
  data[17] = (calibration.gyrZ & 0xFF00) >> 8;
  if (!WriteRegister(BNO055_ACCEL_OFFSET_X_LSB_ADDR, (uint8_t *)&data, sizeof(data)))
  {
    ESP_LOGE(tag, "Error writing magnetometer offset");
    return false;
  }
  mCalibrated = true;
  return false;
}

bool BNO055::ReadCalibrationValues(IMUData &calibration)
{
  SwitchToConfigMode();

  uint8_t data[18] = {};
  if (!ReadRegister(BNO055_ACCEL_OFFSET_X_LSB_ADDR, (uint8_t *)&data, sizeof(data)))
  {
    ESP_LOGE(tag, "Error reading magnetometer offset");
    return false;
  }

  // portENTER_CRITICAL(&mSpinlock);
  calibration.accX = (data[1] << 8) | data[0];
  calibration.accY = (data[3] << 8) | data[2];
  calibration.accZ = (data[5] << 8) | data[4];
  calibration.magX = (data[7] << 8) | data[6];
  calibration.magY = (data[9] << 8) | data[8];
  calibration.magZ = (data[11] << 8) | data[10];
  calibration.gyrX = (data[13] << 8) | data[12];
  calibration.gyrY = (data[15] << 8) | data[14];
  calibration.gyrZ = (data[17] << 8) | data[16];
  // portEXIT_CRITICAL(&mSpinlock);

  return false;
}




// bool BNO055::ConfigMagnetometer(int16_t offsetX, int16_t offsetY, int16_t offsetZ, float declinationAngle)
// {
//   mDeclinationAngle = declinationAngle;

//   // set magnetometer to continuous mode
//   // CFG_REG_A_M (60h)
//   uint8_t data[3] = {};
//   data[0] |= (false << 0); // MD0 = 00 for continuous mode
//   data[0] |= (false << 1); // MD1 = 00 for continuous mode
//   // configure ODR
//   data[0] |= (false << 2); // ODR = 00 for 10 Hz output data rate
//   data[0] |= (false << 3); // ODR = 00 for 10 Hz output data rate
//   data[0] |= (false << 4); // LP = 01 for low power mode
//   data[0] |= (false << 5); // Soft Reset
//   data[0] |= (false << 6); // REBOOT
//   data[0] |= (false << 7); // COMP_TEMP_EN

//   // CFG_REG_B_M (61h)
//   data[1] |= (true << 0);  // Low Pass Filter LPF
//   data[1] |= (true << 1);  // OFF_CANC = 00
//   data[1] |= (false << 2); // SET_FREQ = 00
//   data[1] |= (false << 3); // INT_ON_DATAOFF = 00
//   data[1] |= (false << 4); // OFF_CANC_ONE_SHOT

//   // CFG_REG_C_M (62h)
//   data[2] |= (false << 0); // INT_MAG
//   data[2] |= (false << 1); // SELF_TEST
//   data[2] |= (false << 3); // BLE
//   data[2] |= (true << 4);  // BDU
//   data[2] |= (false << 5); // I2C_DIS
//   data[2] |= (false << 6); // INT_MAG_PIN

//   if (!WriteRegister(BNO055_CFG_REG_A_M, (uint8_t *)&data, sizeof(data)))
//   {
//     ESP_LOGE(tag, "Error writing magnetometer configuration");
//     return false;
//   }

//   uint8_t dataOffset[6] = {};
//   dataOffset[0] = (offsetX & 0x00FF);
//   dataOffset[1] = (offsetX & 0xFF00) >> 8;
//   dataOffset[2] = (offsetY & 0x00FF);
//   dataOffset[3] = (offsetY & 0xFF00) >> 8;
//   dataOffset[4] = (offsetZ & 0x00FF);
//   dataOffset[5] = (offsetZ & 0xFF00) >> 8;

//   if (!WriteRegister(BNO055_X_REG_L_M, (uint8_t *)&dataOffset, sizeof(dataOffset)))
//   {
//     ESP_LOGE(tag, "Error writing magnetometer offset");
//     return false;
//   }

//   return true;
// }

// bool BNO055::RunCalibration()
// {
//   int16_t minX = INT16_MAX;
//   int16_t maxX = INT16_MIN;
//   int16_t minY = INT16_MAX;
//   int16_t maxY = INT16_MIN;
//   int16_t minZ = INT16_MAX;
//   int16_t maxZ = INT16_MIN;

//   unsigned int ticks = 0;
//   while (true)
//   {
//     if (!ReadMagnetometer())
//     {
//       return false;
//     }

//     if (mX < minX)
//     {
//       minX = mX;
//     }
//     if (mX > maxX)
//     {
//       maxX = mX;
//     }
//     if (mY < minY)
//     {
//       minY = mY;
//     }
//     if (mY > maxY)
//     {
//       maxY = mY;
//     }
//     if (mZ < minZ)
//     {
//       minZ = mZ;
//     }
//     if (mZ > maxZ)
//     {
//       maxZ = mZ;
//     }

//     if (ticks++ > 10)
//     {
//       // log all min/max values
//       ESP_LOGI(tag, "minX: %d, maxX: %d, minY: %d, maxY: %d, minZ: %d, maxZ: %d", minX, maxX, minY, maxY, minZ, maxZ);
//     }

//     vTaskDelay(100 / portTICK_PERIOD_MS);
//   }
//   return false;
// }

// bool BNO055::ReadEulerAngles()
// {
//   portENTER_CRITICAL(&mSpinlock);
//   mX = (data[2] << 8) | data[1];
//   mY = (data[4] << 8) | data[3];
//   mZ = (data[6] << 8) | data[5];
//   portEXIT_CRITICAL(&mSpinlock);

//   // log the heading
//   ESP_LOGD(tag, "heading: %f, x: %d, y: %d, z: %d", GetHeading(), mX, mY, mZ);
//   return true;
// }

///  @brief This API reads Euler data hrp values. Thread-Safe!
///  from register 0x1A to 0x1F it is a six byte data
///  @param euler : The Euler hrp data's
///  Parameter |    result
///  --------- | -----------------
///   h        | The Euler h data
///   r        | The Euler r data
///   p        | The Euler p data
///  @return true, for success
bool BNO055::ReadEulerHrpAngles()
{
  SwitchToOperationMode();

  // Array holding the Euler hrp value
  // data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_LSB] - h->LSB
  // data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_MSB] - h->MSB
  // data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_LSB] - r->MSB
  // data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_MSB] - r->MSB
  // data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_LSB] - p->MSB
  // data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_MSB] - p->MSB

  uint8_t data[BNO055_EULER_HRP_DATA_SIZE] = {};
  if (!ReadRegister(BNO055_EULER_H_LSB_VALUEH_REG, (uint8_t *)data, sizeof(data)))
  {
    return false;
  }
  portENTER_CRITICAL(&mSpinlock);
  mEulerHrpAngles.h = (data[1] << 8) | data[0];
  mEulerHrpAngles.r = (data[3] << 8) | data[2];
  mEulerHrpAngles.p = (data[5] << 8) | data[4];
  portEXIT_CRITICAL(&mSpinlock);
  return true;
}

/// @brief Get the status of the IMU. Thread safe.
/// @return
void BNO055::GetStatus(IMUStatus &status)
{
  portENTER_CRITICAL(&mSpinlock);
  status = mStatus;
  portEXIT_CRITICAL(&mSpinlock);
}

void BNO055::GetMagGyrAcc(IMUData &magGyrAcc)
{
  portENTER_CRITICAL(&mSpinlock);
  magGyrAcc = mMagGyrAcc;
  portEXIT_CRITICAL(&mSpinlock);
}

void BNO055::GetCalibration(IMUData &calibration)
{
  portENTER_CRITICAL(&mSpinlock);
  calibration = mCalibration;
  portEXIT_CRITICAL(&mSpinlock);
}

bool BNO055::ReadCalibrationStatus()
{
  SwitchToOperationMode();

  uint8_t data = 0;
  if (!ReadRegister(BNO055_CALIB_STAT_ADDR, (uint8_t *)&data, sizeof(data)))
  {
    ESP_LOGE(tag, "Error reading calibration status");
    return false;
  }
  portENTER_CRITICAL(&mSpinlock);
  mStatus.magCal = data & 0x03;
  mStatus.accCal = (data >> 2) & 0x03;
  mStatus.gyroCal = (data >> 4) & 0x03;
  mStatus.sysCal = (data >> 6) & 0x03;
  portEXIT_CRITICAL(&mSpinlock);
  // log
  ESP_LOGD(tag, "MagCal: %d, AccCal: %d, GyroCal: %d, SysCal: %d", mStatus.magCal, mStatus.accCal, mStatus.gyroCal, mStatus.sysCal);
  return true;
}


bool BNO055::ReadMagGyrAccValues()
{
  SwitchToOperationMode();

  uint8_t data[18] = {};
  if (!ReadRegister(BNO055_ACCEL_DATA_X_LSB_ADDR, (uint8_t *)&data, sizeof(data)))
  {
    ESP_LOGE(tag, "Error reading magnetometer offset");
    return false;
  }

  portENTER_CRITICAL(&mSpinlock);
  mCalibration.accX = (data[1] << 8) | data[0];
  mCalibration.accY = (data[3] << 8) | data[2];
  mCalibration.accZ = (data[5] << 8) | data[4];
  mCalibration.magX = (data[7] << 8) | data[6];
  mCalibration.magY = (data[9] << 8) | data[8];
  mCalibration.magZ = (data[11] << 8) | data[10];
  mCalibration.gyrX = (data[13] << 8) | data[12];
  mCalibration.gyrY = (data[15] << 8) | data[14];
  mCalibration.gyrZ = (data[17] << 8) | data[16];
  mCalibrated = true;
  portEXIT_CRITICAL(&mSpinlock);

  return true;
}

float BNO055::GetHeading()
{
  float heading;
  portENTER_CRITICAL(&mSpinlock);
  heading = mEulerHrpAngles.h / BNO055_EULER_DIV_DEG + mDeclinationAngleDeg;
  if (heading > 360.0)
  {
    heading -= 360.0;
  }
  portEXIT_CRITICAL(&mSpinlock);
  return heading;
}

bool BNO055::Init()
{
  if (!AddDevice(I2C_ADDR_BIT_LEN_7, BNO055_DEVICE_ADDRESS_7BIT, BNO055_DEVICE_FREQ_HZ, "BNO055"))
    return false;
  if (!MasterProbe(BNO055_DEVICE_ADDRESS_7BIT))
    return false;
  return true;
}
