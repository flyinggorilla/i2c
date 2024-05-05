#include "LSM303AGR.h"
#include "esp_log.h"
#include "esp_task.h"
#include "driver/gpio.h"
#include <math.h>

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

// Register Map
#define LSM303AGR_ACCEL_DEVICE_ADDRESS_7BIT (0x32 >> 1) //!< I2C address/bits, 0011001x
#define LSM303AGR_MAG_DEVICE_ADDRESS_7BIT (0x3C >> 1)   //!< I2C address/bits, 0011001x
#define LSM303AGR_DEVICE_FREQ_HZ 10000

static const char *tag = "LSM303AGR";

LSM303AGR::LSM303AGR(I2c &rI2c, gpio_num_t intPin) : I2cDevice(rI2c), mIntPin(intPin)
{
  // Constructor implementation
}
bool LSM303AGR::ConfigMagnetometer(int16_t offsetX, int16_t offsetY, int16_t offsetZ, float declinationAngle)
{
  mDeclinationAngle = declinationAngle;

  // set magnetometer to continuous mode
  // CFG_REG_A_M (60h)
  uint8_t data[3] = {};
  data[0] |= (false << 0); // MD0 = 00 for continuous mode
  data[0] |= (false << 1); // MD1 = 00 for continuous mode
  // configure ODR
  data[0] |= (false << 2); // ODR = 00 for 10 Hz output data rate
  data[0] |= (false << 3); // ODR = 00 for 10 Hz output data rate
  data[0] |= (false << 4); // LP = 01 for low power mode
  data[0] |= (false << 5); // Soft Reset
  data[0] |= (false << 6); // REBOOT
  data[0] |= (false << 7); // COMP_TEMP_EN

  // CFG_REG_B_M (61h)
  data[1] |= (true << 0);  // Low Pass Filter LPF
  data[1] |= (true << 1);  // OFF_CANC = 00
  data[1] |= (false << 2); // SET_FREQ = 00
  data[1] |= (false << 3); // INT_ON_DATAOFF = 00
  data[1] |= (false << 4); // OFF_CANC_ONE_SHOT

  // CFG_REG_C_M (62h)
  data[2] |= (false << 0); // INT_MAG
  data[2] |= (false << 1); // SELF_TEST
  data[2] |= (false << 3); // BLE
  data[2] |= (true << 4);  // BDU
  data[2] |= (false << 5); // I2C_DIS
  data[2] |= (false << 6); // INT_MAG_PIN

  if (!WriteRegister(LSM303AGR_CFG_REG_A_M, (uint8_t *)&data, sizeof(data)))
  {
    ESP_LOGE(tag, "Error writing magnetometer configuration");
    return false;
  }

  uint8_t dataOffset[6] = {};
  dataOffset[0] = (offsetX & 0x00FF);
  dataOffset[1] = (offsetX & 0xFF00) >> 8;
  dataOffset[2] = (offsetY & 0x00FF);
  dataOffset[3] = (offsetY & 0xFF00) >> 8;
  dataOffset[4] = (offsetZ & 0x00FF);
  dataOffset[5] = (offsetZ & 0xFF00) >> 8;

  if (!WriteRegister(LSM303AGR_X_REG_L_M, (uint8_t *)&dataOffset, sizeof(dataOffset)))
  {
    ESP_LOGE(tag, "Error writing magnetometer offset");
    return false;
  }

  return true;
}

bool LSM303AGR::RunCalibration()
{
  int16_t minX = INT16_MAX;
  int16_t maxX = INT16_MIN;
  int16_t minY = INT16_MAX;
  int16_t maxY = INT16_MIN;
  int16_t minZ = INT16_MAX;
  int16_t maxZ = INT16_MIN;

  unsigned int ticks = 0;
  while (true)
  {
    if (!ReadMagnetometer())
    {
      return false;
    }

    if (mX < minX)
    {
      minX = mX;
    }
    if (mX > maxX)
    {
      maxX = mX;
    }
    if (mY < minY)
    {
      minY = mY;
    }
    if (mY > maxY)
    {
      maxY = mY;
    }
    if (mZ < minZ)
    {
      minZ = mZ;
    }
    if (mZ > maxZ)
    {
      maxZ = mZ;
    }

    if (ticks++ > 10)
    {
      // log all min/max values
      ESP_LOGI(tag, "minX: %d, maxX: %d, minY: %d, maxY: %d, minZ: %d, maxZ: %d", minX, maxX, minY, maxY, minZ, maxZ);
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  return false;
}

bool LSM303AGR::ReadMagnetometer()
{
  // read magnetometer data
  uint8_t data[7] = {};
  if (!ReadRegister(LSM303AGR_STATUS_REG_M, (uint8_t *)data, sizeof(data)))
  {
    return false;
  }

  uint8_t status = data[0];
  if (status)
  {
    ESP_LOGV(tag, "Magnetometer overrun happened: %02x", status);
  }

  portENTER_CRITICAL(&mSpinlock);
  mX = (data[2] << 8) | data[1];
  mY = (data[4] << 8) | data[3];
  mZ = (data[6] << 8) | data[5];
  portEXIT_CRITICAL(&mSpinlock);

  // log the heading
  ESP_LOGD(tag, "heading: %f, x: %d, y: %d, z: %d", GetHeading(), mX, mY, mZ);
  return true;
}

float LSM303AGR::GetHeading()
{
  float heading;
  portENTER_CRITICAL(&mSpinlock);
  // convert the raw x, y and z data to compass direction in degrees towards north
  heading = -(atan2(mY, mX) * 180) / M_PI - mDeclinationAngle;
  if (heading < 0)
  {
    heading += 360;
  }
  portEXIT_CRITICAL(&mSpinlock);
  return heading;
}

bool LSM303AGR::Init()
{
  if (!AddDevice(I2C_ADDR_BIT_LEN_7, LSM303AGR_MAG_DEVICE_ADDRESS_7BIT, LSM303AGR_DEVICE_FREQ_HZ, "LSM303AGR"))
    return false;
  if (!MasterProbe(LSM303AGR_ACCEL_DEVICE_ADDRESS_7BIT))
    return false;

  return true;
}

/*
bool LSM303AGR::ReadSystemVoltageMinimum(uint16_t *vsys_min)
{
  uint8_t data;
  if (!ReadRegister(CHARGER25_REG_MINIMAL_SYSTEM_VOLTAGE, &data, sizeof(data)))
  {
    *vsys_min = 0;
    return false;
  }

  data &= CHARGER25_VSYS_MIN_BIT_MASK;
  *vsys_min = data;
  *vsys_min *= CHARGER25_VSYS_MIN_STEP_SIZE;
  *vsys_min += CHARGER25_VSYS_MIN_FIXED_OFFSET;

  // log vsys_min
  ESP_LOGD(tag, "vsys_min is %u", (unsigned int)*vsys_min);

  return true;
}

bool LSM303AGR::SetSystemVoltageMinimum(uint16_t vsys_min)
{
  EnsureChargerIsDisabled();

  // check input voltage for that range
  if (vsys_min < 2500 || vsys_min > 16000)
    return false;

  uint8_t data = (vsys_min - CHARGER25_VSYS_MIN_FIXED_OFFSET) / CHARGER25_VSYS_MIN_STEP_SIZE;

  return WriteRegister(CHARGER25_REG_MINIMAL_SYSTEM_VOLTAGE, &data, sizeof(data));
}

bool LSM303AGR::GetCellCount()
{
  uint16_t vsys_min;
  if (!ReadSystemVoltageMinimum(&vsys_min))
    return false;

  uint8_t cell_count = (uint8_t)(vsys_min / (CHARGER25_VTG_CELL_COUNT_DIV * 1000));
  // log cell_count
  ESP_LOGD(tag, "cell_count is %u", (unsigned int)cell_count);

  return true;
}

bool LSM303AGR::SetChargeVoltageLimit(uint16_t voltage_mV)
{
  EnsureChargerIsDisabled();

  // check input voltage for that range
  if (voltage_mV < 3000 || voltage_mV > 18800)
    return false;

  // voltage bits are in steps of 10mv = cV
  uint16_t voltage_cV = voltage_mV / 10;
  uint8_t data[2] = {};
  data[0] = (uint8_t)(voltage_cV >> 8);
  data[1] = (uint8_t)(voltage_cV);
  return WriteRegister(CHARGER25_REG_CHARGE_VOLTAGE_LIMIT, (uint8_t *)data, sizeof(data));
}

bool LSM303AGR::ReadChargeVoltageLimit(uint16_t *voltage_mV)
{
  uint8_t data[2];
  if (!ReadRegister(CHARGER25_REG_CHARGE_VOLTAGE_LIMIT, (uint8_t *)data, sizeof(data)))
    return false;

  uint16_t voltage_cV = (data[0] << 8) | data[1];
  *voltage_mV = voltage_cV * 10;
  return true;
}

bool LSM303AGR::SetChargeCurrentLimit(uint16_t current_mA)
{
  EnsureChargerIsDisabled();

  // check input current for that range
  if (current_mA < 50 || current_mA > 5000)
  {
    return false;
  }

  // current bits are in steps of 10mA = mA
  uint16_t current_mA_10 = current_mA / 10;
  uint8_t data[2] = {};
  data[0] = (uint8_t)(current_mA_10 >> 8);
  data[1] = (uint8_t)(current_mA_10 & 0xFF);
  return WriteRegister(CHARGER25_REG_CHARGE_CURRENT_LIMIT, (uint8_t *)data, sizeof(data));
}

bool LSM303AGR::ReadChargeCurrentLimit(uint16_t *current_mA)
{
  uint8_t data[2];
  if (!ReadRegister(CHARGER25_REG_CHARGE_CURRENT_LIMIT, (uint8_t *)data, sizeof(data)))
  {
    *current_mA = 0;
    return false;
  }

  uint16_t current_cA_10 = (data[0] << 8) | data[1];
  *current_mA = current_cA_10 * 10;
  return true;
}

bool LSM303AGR::ReadInputCurrentMinLimit(uint16_t *current_mA)
{
  uint8_t data[2];
  if (!ReadRegister(CHARGER25_REG_INPUT_CURRENT_LIMIT, (uint8_t *)data, sizeof(data)))
  {
    *current_mA = 0;
    return false;
  }

  uint16_t current_mA_10 = ((data[0] & 0x01) << 8) | data[1];
  *current_mA = current_mA_10 * 10;
  return true;
}

bool LSM303AGR::SetInputCurrentMinLimit(uint16_t current_mA)
{
  EnsureChargerIsDisabled();

  // check input current for that range
  if (current_mA < 100 || current_mA > 3300)
    return false;

  // current bits are in steps of 10mA = mA
  uint16_t current_mA_10 = current_mA / 10;
  uint8_t data[2] = {};
  data[0] = (uint8_t)(current_mA_10 >> 8);
  data[1] = (uint8_t)(current_mA_10 & 0xFF);
  return WriteRegister(CHARGER25_REG_INPUT_CURRENT_LIMIT, (uint8_t *)data, sizeof(data));
}

bool LSM303AGR::ReadInputVoltageMinLimit(uint16_t *voltage_mV)
{
  uint8_t data;
  if (!ReadRegister(CHARGER25_REG_INPUT_VOLTAGE_LIMIT, &data, sizeof(data)))
  {
    *voltage_mV = 0;
    return false;
  }

  *voltage_mV = data;
  *voltage_mV *= CHARGER25_INPUT_VTG_CONV_V_MV;
  return true;
}

bool LSM303AGR::ReadInputVoltage(uint16_t *voltage_mV)
{
  uint8_t data[2];
  if (!ReadRegister(CHARGER25_REG_VBUS_ADC, (uint8_t *)data, sizeof(data)))
  {
    *voltage_mV = 0;
    return false;
  }

  *voltage_mV = (data[0] << 8) | data[1];
  return true;
}

bool LSM303AGR::ReadSystemVoltage(uint16_t *voltage_mV)
{
  uint8_t data[2];
  if (!ReadRegister(CHARGER25_REG_VSYS_ADC, (uint8_t *)data, sizeof(data)))
  {
    *voltage_mV = 0;
    return false;
  }

  *voltage_mV = (data[0] << 8) | data[1];
  return true;
}

bool LSM303AGR::SetInputVoltageMinLimit(uint16_t voltage_mV)
{
  EnsureChargerIsDisabled();

  // check input voltage for that range
  if (voltage_mV < 3600 || voltage_mV > 22000)
    return false;

  uint8_t data = voltage_mV / CHARGER25_INPUT_VTG_CONV_V_MV;
  return WriteRegister(CHARGER25_REG_INPUT_VOLTAGE_LIMIT, (uint8_t *)&data, sizeof(data));
}


//  The recommended procedure for enabling detection of HVDCP sources is completed in the following steps:
//  • Before adapter insertion, the charger is configured with AUTO_INDET_EN = 1 and HVDCP_EN = 0. EN_12V
// and EN_9V are set as desired for the system.
//  • When an adapter is inserted, it will be detected as SDP, CDP, DCP or a Non-Standard adapter and an I2C
// interrupt is sent to the host. If any detection other than DCP is made, the host proceeds as usual.
//  • If the adapter is detected as DCP (VBUS_STAT[3:0] = 0011), the host first changes the Input Current Limit
// register to 1.5A and then changes HVDCP_EN = 1.
//  • After HVDCP detection is complete, the host sets HVDCP_EN = 0 to disable HVDCP support in preparation
// for the next input source detection sequence
bool LSM303AGR::SetHvdcp()
{
  EnsureChargerIsDisabled();

  uint8_t data = 0;
  // set bit 7, 6, 5, 4 and 3
  data |= 0 << 7; // FORCE_INDET
  data |= 1 << 6; // AUTO_INDET_EN
  data |= 1 << 5; // EN_12V
  data |= 1 << 4; // EN_9V
  data |= 1 << 3; // HVDCP_EN
  return WriteRegister(CHARGER25_REG_CHARGER_CONTROL_2, (uint8_t *)&data, sizeof(data));
}

bool LSM303AGR::TriggerInputVoltageDetection()
{
  EnsureChargerIsDisabled();

  uint8_t data = 0;
  data |= 0 << 7; // EN_ACDRV2
  data |= 0 << 6; // EN_ACDRV1
  data |= 0 << 5; // PWM_FREQ - Switching frequency selection, this bit POR default value is based on the PROG pin strapping. 0h = 1.5 MHz 1h = 750 kHz
  data |= 0 << 4; // DIS_STAT - disable the stat pin output
  data |= 0 << 3; // DIS_VSYS_SHOR - Disable forward mode VSYS short hiccup protection.
  data |= 0 << 2; // DIS_VOTG_UVP -  Disable OTG mode VOTG UVP hiccup protection.
  data |= 1 << 1; // FORCE_VINDPM_DET - Force VINDPM detection Note: only when VBAT>VSYSMIN, this bit can be set to 1. Once the VINDPM auto detection is done, this bits returns to 0
  data |= 1 << 0; // EN_IBUS_OCP -  Enable IBUS_OCP in forward mode
  return WriteRegister(CHARGER25_REG_CHARGER_CONTROL_4, (uint8_t *)&data, sizeof(data));
}

bool LSM303AGR::ReadBatteryVoltage(uint16_t *vbat_mV)
{
  // voltage bits are in steps of 1mV
  uint8_t data[2];
  if (!ReadRegister(CHARGER25_REG_VBAT_ADC, (uint8_t *)data, sizeof(data)))
  {
    *vbat_mV = 0;
    return false;
  }

  *vbat_mV = (data[0] << 8) | data[1];
  return true;
}

bool LSM303AGR::SetAdcControl(bool enabled, bool runningAverage)
{
  EnsureChargerIsDisabled();

  uint8_t data = 0;
  // set 7th bit to ADC control enabled
  data |= (enabled & 1) << 7;
  // data |= (enabled & 1) << 6;
  // data |= (runningAverage & CHARGER25_SET_ADC_AVG_RUN_AVG) << 3;
  // data |= (runningAverage & CHARGER25_SET_ADC_AVG_INIT_ADC_CNV) << 2;
  WriteRegister(CHARGER25_REG_ADC_CONTROL, (uint8_t *)&data, sizeof(data));


  return true;
}

// IBAT ADC reading
// Reported in 2 's Complement.
// The IBAT ADC reports positive value for the battery charging current,
// and negative value for the battery discharging current if EN_IBAT in
// REG0x14[5] = 1.
// Type : R
// POR: 0mA (0h)
// Range : 0mA-8000mA
// Fixed Offset : 0mA
// Bit Step Size : 1mA
bool LSM303AGR::ReadChargingCurrent(int16_t *current_mA)
{
  uint8_t dataFlags = 0;
  if (!ReadRegister(CHARGER25_REG_CHARGER_CONTROL_5, (uint8_t *)&dataFlags, sizeof(dataFlags)))
  {
    *current_mA = 0;
    return false;
  }

  // check bit 5 to see if battery is discharged when set
  bool negative = dataFlags & 0x20;
  // log negative
  ESP_LOGD(tag, "negative is %d", negative);

  uint8_t data[2] = {};
  if (!ReadRegister(CHARGER25_REG_IBAT_ADC, (uint8_t *)data, sizeof(data)))
  {
    *current_mA = 0;
    return false;
  }

  // log data 0 and data 1 as hex
  ESP_LOGD(tag, "data[0] is %02x data[1] is %02x", data[0], data[1]);

  *current_mA = (data[0] << 8) | data[1];
  // if (negative)
  //   current_mA_10 = -current_mA_10;
  // *current_mA = current_mA;
  return true;
}

bool LSM303AGR::SetChargingCurrentMonitoring(bool enabled)
{
  EnsureChargerIsDisabled();

  uint8_t dataFlags = 0;

  // check bit 5 to see if battery is discharged when set
  dataFlags |= (uint8_t)enabled << 5;
  return WriteRegister(CHARGER25_REG_CHARGER_CONTROL_5, (uint8_t *)&dataFlags, sizeof(dataFlags));
}

bool LSM303AGR::ResetWatchdogTimer()
{
  // this is one byte, where bits 0-2 need to be set to 0x07 and bit 3 to 0x01; please do it in two steps

  uint8_t data = 0x06; // set watchdog timer to 80s
  data |= 1 << 3;      // reset watchdog timer
  return WriteRegister(CHARGER25_REG_CHARGER_CONTROL_1, (uint8_t *)&data, sizeof(data));
}

bool LSM303AGR::ReadChargerStatus(uint8_t *chargeStatus_CHG_STAT, uint8_t *chargeStatus_VBUS_STAT)
{
  uint8_t data[5];
  if (!ReadRegister(CHARGER25_REG_CHARGER_STATUS_0, (uint8_t *)data, sizeof(data)))
  {
    return false;
  }

  // log data 0..4 hex and label it with register name, all in one log line
  ESP_LOGD(tag, "CHARGER_STATUS_0..4: %02x %02x %02x %02x %02x", data[0], data[1], data[2], data[3], data[4]);

  bool watchdogStatus = data[0] & (1 << 5);      // bit 5
  bool poorSourceDetection = data[0] & (1 << 4); // bit 4
  bool powerGoodStatus = data[0] & (1 << 3);     // bit 3
  bool vbusPresentStat = data[0] & (1 << 0);     // bit 0

  // log watchdogStatus, poorSourceDetection, powerGoodStatus
  ESP_LOGD(tag, "WD_STAT: %s POORSRC_STAT: %s PG_STAT: %s, VBUS_PRESENT_STAT: %s",
           watchdogStatus ? "expired" : "ok",
           poorSourceDetection ? "weak adapter" : "ok",
           powerGoodStatus ? "good" : "not good",
           vbusPresentStat ? "present" : "absent");

  // check for bits 7-5
  uint8_t chargeStatus = data[1] >> 5; // bits 5-7

  *chargeStatus_CHG_STAT = chargeStatus;

  if (!IsChargerEnabled()) {
    *chargeStatus_CHG_STAT = 0xFF;
  }

  // log chargeStatus to text
  // 0h = Not Charging
  // 1h = Trickle Charge
  // 2h = Pre-charge
  // 3h = Fast charge (CC mode)
  // 4h = Taper Charge (CV mode)
  // 5h = Reserved
  // 6h = Top-off Timer Active Charging
  // 7h = Charge Termination Done
  // FFh = OFF/Disabled (CE register 0 and/or CE GPIO set to 1)
  switch (chargeStatus)
  {
  case 0:
    ESP_LOGD(tag, "chargeStatus: Not Charging");
    break;
  case 1:
    ESP_LOGD(tag, "chargeStatus: Trickle Charge");
    break;
  case 2:
    ESP_LOGD(tag, "chargeStatus: Pre-charge");
    break;
  case 3:
    ESP_LOGD(tag, "chargeStatus: Fast charge (CC mode)");
    break;
  case 4:
    ESP_LOGD(tag, "chargeStatus: Taper Charge (CV mode)");
    break;
  case 5:
    ESP_LOGD(tag, "chargeStatus: Reserved");
    break;
  case 6:
    ESP_LOGD(tag, "chargeStatus: Top-off Timer Active Charging");
    break;
  case 7:
    ESP_LOGD(tag, "chargeStatus: Charge Termination Done");
    break;
  };



  //   VBUS status bits
  // 0h: No Input or BHOT or BCOLD in OTG mode
  // 1h: USB SDP (500mA)
  // 2h: USB CDP (1.5A)
  // 3h: USB DCP (3.25A)
  // 4h: Adjustable High Voltage DCP (HVDCP) (1.5A)
  // 5h: Unknown adaptor (3A)
  // 6h: Non-Standard Adapter (1A/2A/2.1A/2.4A)
  // 7h: In OTG mode
  // 8h: Not qualified adaptor
  // 9h: Reserved
  // Ah: Reserved
  // Bh: Device directly powered from VBUS
  // Ch: Reserved
  // Dh: Reserved
  // Eh: Reserved
  // Fh: Reserved
  // Type : R
  // POR: 0h
  // create a definition for the bits 4-1
#define CHARGER25_CHARGER_STATUS1_VBUS_STAT 0x1E
  uint8_t vbusStatus = (data[1] & CHARGER25_CHARGER_STATUS1_VBUS_STAT) >> 1; // bits 4-1
  *chargeStatus_VBUS_STAT = vbusStatus;
  // log vbusStatus to text
  switch (vbusStatus)
  {
  case 0:
    ESP_LOGD(tag, "vbusStatus: No Input or BHOT or BCOLD in OTG mode");
    break;
  case 1:
    ESP_LOGD(tag, "vbusStatus: USB SDP (500mA)");
    break;
  case 2:
    ESP_LOGD(tag, "vbusStatus: USB CDP (1.5A)");
    break;
  case 3:
    ESP_LOGD(tag, "vbusStatus: USB DCP (3.25A)");
    break;
  case 4:
    ESP_LOGD(tag, "vbusStatus: Adjustable High Voltage DCP (HVDCP) (1.5A)");
    break;
  case 5:
    ESP_LOGD(tag, "vbusStatus: Unknown adaptor (3A)");
    break;
  case 6:
    ESP_LOGD(tag, "vbusStatus: Non-Standard Adapter (1A/2A/2.1A/2.4A)");
    break;
  case 7:
    ESP_LOGD(tag, "vbusStatus: In OTG mode");
    break;
  case 8:
    ESP_LOGD(tag, "vbusStatus: Not qualified adaptor");
    break;
  case 11:
    ESP_LOGD(tag, "vbusStatus: Device directly powered from VBUS");
    break;
  default:
    ESP_LOGD(tag, "vbusStatus: invalid");
    *chargeStatus_VBUS_STAT = 0xFF;
    break;
  };

  bool vbatPresentStatus = data[2] & (1 << 0); // bit 0
  bool thermalRegulationStatus = data[2] & (1 << 2);
  uint8_t inputCurrentOptimizerStatus = data[1] >> 6; // bits 6-7
  // log vbatPresentStatus, thermalRegulationStatus
  ESP_LOGD(tag, "VBAT_PRESENT_STAT: %s TREG_STAT: %s, ICO_STAT: %d",
           vbatPresentStatus ? "present" : "absent",
           thermalRegulationStatus ? "normal" : "regulated",
           inputCurrentOptimizerStatus);

  bool vsysRegStat = data[3] & (1 << 4);   // bit 4
  bool chgTmrStat = data[3] & (1 << 3);    // bit 3
  bool trichgTmrStat = data[3] & (1 << 2); // bit 2
  bool prechgTmrStat = data[3] & (1 << 1); // bit 1

  // log vsysRegStat, chgTmrStat, trichgTmrStat, prechgTmrStat
  ESP_LOGD(tag, "VSYS_REG_STAT: %s CHG_TMR_STAT: %s, TRICHG_TMR_STAT: %s, PRECHG_TMR_STAT: %s",
           vsysRegStat ? "in VSYSMIN regulation" : "not regulated",
           chgTmrStat ? "safety timer expired" : "normal",
           trichgTmrStat ? "safety timer expired" : "normal",
           prechgTmrStat ? "safety timer expired" : "normal");

  return true;
}

bool LSM303AGR::ReadFaults(uint16_t *faultStatus)
{
  uint8_t data[2] = {};
  if (!ReadRegister(CHARGER25_REG_FAULT_STATUS_0, (uint8_t *)data, sizeof(data)))
    return false;

  // log data 0 and data 1 as hex
  if (data[0] || data[1])
  {
    if (data[0] == 0xFF && data[1] == 0xFF)
    {
      ESP_LOGD(tag, "No battery connected");
    }
    else
    {
      ESP_LOGE(tag, "FAULT_STATUS REG20: %02x, REG21: %02x", data[0], data[1]);
    }
  }
  if (faultStatus) {
    *faultStatus = (data[0] << 8) | data[1];
  }

  return true;
}



bool LSM303AGR::EnableCharger(bool enable, bool inputCurrentOptimizer)
{
  gpio_set_level(mChargeEnablePin, !enable); // pin to low enables the charger

  mChargerEnabled = enable;
  uint8_t data = 0;
  data |= (uint8_t)enable << 5;                // EN_CHG Charger Enable 1 = on
  data |= (uint8_t)inputCurrentOptimizer << 4; // EN_ICO Charge Current Optimizer
  data |= (uint8_t)0 << 1;                     // FORCE_ICO Force Current Optimizer
  data |= (uint8_t)1 << 1;                     // EN_TERM Enable Termination
  return WriteRegister(CHARGER25_REG_CHARGER_CONTROL_0, (uint8_t *)&data, sizeof(data));
}

bool LSM303AGR::ReadChargerEnabled(bool *charger_EN_CHG, bool *charger_EN_TERM)
{
  uint8_t data;
  if (!ReadRegister(CHARGER25_REG_CHARGER_CONTROL_0, &data, sizeof(data)))
    return false;

  *charger_EN_CHG = data & (1 << 5); // bit 5
  bool chgTerm = data & (1 << 1);    // bit 1
  if (charger_EN_TERM)
  {
    *charger_EN_TERM = chgTerm;
  }

  // if the charger CE pin is not connected to a GPIO and kept on GND, then only the register bit matters
  // if the charger CI pin is connected to a GPIO, then BOTH the register bit and the GPIO pin matter
  if ((mChargeEnablePin == GPIO_NUM_NC) || (*charger_EN_CHG == 0))
  {
    mChargerEnabled = *charger_EN_CHG;
  }

  // log chargerStatus
  ESP_LOGW(tag, "Charger_EN_CHG: %d charger_EN_TERM: %d", *charger_EN_CHG, chgTerm);
  return true;
}

void LSM303AGR::EnsureChargerIsDisabled()
{
  if (mChargerEnabled)
  {
    EnableCharger(false);
  }
}

bool LSM303AGR::SetCellCount(uint8_t cellCount)
{
  EnsureChargerIsDisabled();

  if (cellCount < CHARGER25_VTG_CELL_COUNT_MIN || cellCount > CHARGER25_VTG_CELL_COUNT_MAX)
    return false;

  uint8_t data = 0;
  data = (cellCount - 1) << 6;
  return WriteRegister(CHARGER25_REG_RECHARGE_CONTROL, (uint8_t *)&data, sizeof(data));
}

*/