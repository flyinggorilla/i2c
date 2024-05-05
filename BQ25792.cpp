#include "BQ25792.h"
#include "esp_log.h"
#include "esp_task.h"
#include "driver/gpio.h"

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

// Register Map
#define BQ25792_DEVICE_ADDRESS_7BIT 0x6B
#define BQ25792_DEVICE_FREQ_HZ 100000

static const char *tag = "BQ25792";

#define CHARGER25_REG_MINIMAL_SYSTEM_VOLTAGE 0x00
#define CHARGER25_REG_CHARGE_VOLTAGE_LIMIT 0x01
#define CHARGER25_REG_CHARGE_CURRENT_LIMIT 0x03
#define CHARGER25_REG_INPUT_VOLTAGE_LIMIT 0x05
#define CHARGER25_REG_INPUT_CURRENT_LIMIT 0x06
#define CHARGER25_REG_PRECHARGE_CONTROL 0x08
#define CHARGER25_REG_TERMINATION_CONTROL 0x09
#define CHARGER25_REG_RECHARGE_CONTROL 0x0A
#define CHARGER25_REG_VOTG_REGULATION 0x0B
#define CHARGER25_REG_IOTG_REGULATION 0x0D
#define CHARGER25_REG_TIMER_CONTROL 0x0E
#define CHARGER25_REG_CHARGER_CONTROL_0 0x0F
#define CHARGER25_REG_CHARGER_CONTROL_1 0x10
#define CHARGER25_REG_CHARGER_CONTROL_2 0x11
#define CHARGER25_REG_CHARGER_CONTROL_3 0x12
#define CHARGER25_REG_CHARGER_CONTROL_4 0x13
#define CHARGER25_REG_CHARGER_CONTROL_5 0x14
#define CHARGER25_REG_RESERVED 0x15
#define CHARGER25_REG_TEMPERATURE_CONTROL 0x16
#define CHARGER25_REG_NTC_CONTROL_0 0x17
#define CHARGER25_REG_NTC_CONTROL_1 0x18
#define CHARGER25_REG_ICO_CURRENT_LIMIT 0x19
#define CHARGER25_REG_CHARGER_STATUS_0 0x1B
#define CHARGER25_REG_CHARGER_STATUS_1 0x1C
#define CHARGER25_REG_CHARGER_STATUS_2 0x1D
#define CHARGER25_REG_CHARGER_STATUS_3 0x1E
#define CHARGER25_REG_CHARGER_STATUS_4 0x1F
#define CHARGER25_REG_FAULT_STATUS_0 0x20
#define CHARGER25_REG_FAULT_STATUS_1 0x21
#define CHARGER25_REG_CHARGER_FLAG_0 0x22
#define CHARGER25_REG_CHARGER_FLAG_1 0x23
#define CHARGER25_REG_CHARGER_FLAG_2 0x24
#define CHARGER25_REG_CHARGER_FLAG_3 0x25
#define CHARGER25_REG_FAULT_FLAG_0 0x26
#define CHARGER25_REG_FAULT_FLAG_1 0x27
#define CHARGER25_REG_CHARGER_MASK_0 0x28
#define CHARGER25_REG_CHARGER_MASK_1 0x29
#define CHARGER25_REG_CHARGER_MASK_2 0x2A
#define CHARGER25_REG_CHARGER_MASK_3 0x2B
#define CHARGER25_REG_FAULT_MASK_0 0x2C
#define CHARGER25_REG_FAULT_MASK_1 0x2D
#define CHARGER25_REG_ADC_CONTROL 0x2E
#define CHARGER25_REG_ADC_FUNCTION_DISABLE_0 0x2F
#define CHARGER25_REG_ADC_FUNCTION_DISABLE_1 0x30
#define CHARGER25_REG_IBUS_ADC 0x31
#define CHARGER25_REG_IBAT_ADC 0x33
#define CHARGER25_REG_VBUS_ADC 0x35
#define CHARGER25_REG_VAC1_ADC 0x37
#define CHARGER25_REG_VAC2_ADC 0x39
#define CHARGER25_REG_VBAT_ADC 0x3B
#define CHARGER25_REG_VSYS_ADC 0x3D
#define CHARGER25_REG_TS_ADC 0x3F
#define CHARGER25_REG_TDIE_ADC 0x41
#define CHARGER25_REG_D_P_ADC 0x43
#define CHARGER25_REG_D_M_ADC 0x45
#define CHARGER25_REG_DPDM_DRIVER 0x47
#define CHARGER25_REG_PART_INFORMATION 0x48

/**
 * @brief Charger 25 software reset data.
 * @details Specified software reset of Charger 25 Click driver.
 */
#define CHARGER25_SOFT_RESET 0x40

/**
 * @brief Charger 25 software reset data.
 * @details Specified software reset data of Charger 25 Click driver.
 */
#define CHARGER25_VSYS_MIN_BIT_MASK 0x3F
#define CHARGER25_VSYS_MIN_STEP_SIZE 250
#define CHARGER25_VSYS_MIN_FIXED_OFFSET 2500

/**
 * @brief Charger 25 minimal system voltage and battery cell count data.
 * @details Specified minimal system voltage and battery cell count data of Charger 25 Click driver.
 */
#define CHARGER25_VTG_LIM_MSB_BIT_MASK 0x07
#define CHARGER25_VTG_CONV_V_MV 10
#define CHARGER25_VTG_CELL_COUNT_DIV 3
#define CHARGER25_VTG_CELL_SEL_1 1
#define CHARGER25_VTG_CELL_SEL_2 2
#define CHARGER25_VTG_CELL_SEL_3 3
#define CHARGER25_VTG_CELL_SEL_4 4
#define CHARGER25_VTG_CELL_COUNT_MIN 1
#define CHARGER25_VTG_CELL_COUNT_MAX 4
#define CHARGER25_VTG_CELL_COUNT_MUL 7

/**
 * @brief Charger 25 minimal system voltage and battery cell count data.
 * @details Specified minimal system voltage and battery cell count data of Charger 25 Click driver.
 */
#define CHARGER25_CURR_LIM_MSB_BIT_MASK 0x01
#define CHARGER25_CURR_CONV_A_MA 10

/**
 * @brief Charger 25 input voltage convertor data.
 * @details Specified input voltage convertor data of Charger 25 Click driver.
 */
#define CHARGER25_INPUT_VTG_CONV_V_MV 100

/**
 * @brief Charger 25 ADC function and control data.
 * @details Specified ADC function and control data of Charger 25 Click driver.
 */
#define CHARGER25_SET_ADC_FUNC_ENABLE 0x00
#define CHARGER25_SET_ADC_FUNC_DISABLE 0x01
#define CHARGER25_SET_ADC_CTRL_DISABLE 0x00
#define CHARGER25_SET_ADC_CTRL_ENABLE 0x01

/**
 * @brief Charger 25 ADC conversion rate control and sample speed data.
 * @details Specified ADC conversion rate control and sample speed data of Charger 25 Click driver.
 */
#define CHARGER25_SET_ADC_RATE_CONT_CNV 0x00
#define CHARGER25_SET_ADC_ONES_HOT_CNV 0x01
#define CHARGER25_SET_ADC_SAMPLE_15_BIT 0x00
#define CHARGER25_SET_ADC_SAMPLE_14_BIT 0x01
#define CHARGER25_SET_ADC_SAMPLE_13_BIT 0x02
#define CHARGER25_SET_ADC_SAMPLE_12_BIT 0x03

/**
 * @brief Charger 25 ADC average initial value control data.
 * @details Specified ADC average initial value control data of Charger 25 Click driver.
 */
#define CHARGER25_SET_ADC_AVG_SINGLE_VAL 0x00
#define CHARGER25_SET_ADC_AVG_RUN_AVG 0x01
#define CHARGER25_SET_ADC_AVG_INIT_EXI_VAL 0x00
#define CHARGER25_SET_ADC_AVG_INIT_ADC_CNV 0x01

/**
 * @brief Charger 25 charge status data.
 * @details Specified charge status of Charger 25 Click driver.
 */
#define CHARGER25_CH_STAT_NOT_CHARGING 0x00
#define CHARGER25_CH_STAT_TRICKLE_CHARGE 0x01
#define CHARGER25_CH_STAT_PRE_CHARGE 0x02
#define CHARGER25_CH_STAT_FAST_CHARGE 0x03
#define CHARGER25_CH_STAT_TAPER_CHARGE 0x04
#define CHARGER25_CH_STAT_TIMER_ACT_CHARGING 0x06
#define CHARGER25_CH_STAT_CHARGE_TERMIN_DONE 0x07

/**
 * @brief Charger 25 I/O pins state data.
 * @details Specified I/O pins state data of Charger 25 Click driver.
 */
#define CHARGER25_STATE_PIN_QON_LOW 0x00
#define CHARGER25_STATE_PIN_QON_HIGH 0x01

BQ25792::BQ25792(I2c &rI2c, gpio_num_t chargeEnablePin) : I2cDevice(rI2c), mChargeEnablePin(chargeEnablePin)
{
  // Constructor implementation
}

/// @brief The charger will only work on I2C when battery, sys, or USB-C power is attached. 
///        the 3V3 alone is NOT enough for I2c communication.
bool BQ25792::Init()
{
  if (!AddDevice(I2C_ADDR_BIT_LEN_7, BQ25792_DEVICE_ADDRESS_7BIT, BQ25792_DEVICE_FREQ_HZ, "BQ25792"))
    return false;
  if (!MasterProbe(BQ25792_DEVICE_ADDRESS_7BIT))
    return false;

// -------- ONLY FOR TESTING PURPOSES --------
  ReadChargerEnabled(&mChargerEnabled);

  // get GPIO and register into a defined state of "disabled"
  return DisableCharger();
}

bool BQ25792::ReadSystemVoltageMinimum(uint16_t *vsys_min)
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

bool BQ25792::SetSystemVoltageMinimum(uint16_t vsys_min)
{
  EnsureChargerIsDisabled();

  // check input voltage for that range
  if (vsys_min < 2500 || vsys_min > 16000)
    return false;

  uint8_t data = (vsys_min - CHARGER25_VSYS_MIN_FIXED_OFFSET) / CHARGER25_VSYS_MIN_STEP_SIZE;

  return WriteRegister(CHARGER25_REG_MINIMAL_SYSTEM_VOLTAGE, &data, sizeof(data));
}

bool BQ25792::GetCellCount()
{
  uint16_t vsys_min;
  if (!ReadSystemVoltageMinimum(&vsys_min))
    return false;

  uint8_t cell_count = (uint8_t)(vsys_min / (CHARGER25_VTG_CELL_COUNT_DIV * 1000));
  // log cell_count
  ESP_LOGD(tag, "cell_count is %u", (unsigned int)cell_count);

  return true;
}

bool BQ25792::SetChargeVoltageLimit(uint16_t voltage_mV)
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

bool BQ25792::ReadChargeVoltageLimit(uint16_t *voltage_mV)
{
  uint8_t data[2];
  if (!ReadRegister(CHARGER25_REG_CHARGE_VOLTAGE_LIMIT, (uint8_t *)data, sizeof(data)))
    return false;

  uint16_t voltage_cV = (data[0] << 8) | data[1];
  *voltage_mV = voltage_cV * 10;
  return true;
}

bool BQ25792::SetChargeCurrentLimit(uint16_t current_mA)
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

bool BQ25792::ReadChargeCurrentLimit(uint16_t *current_mA)
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

bool BQ25792::ReadInputCurrentMinLimit(uint16_t *current_mA)
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

bool BQ25792::SetInputCurrentMinLimit(uint16_t current_mA)
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

bool BQ25792::ReadInputVoltageMinLimit(uint16_t *voltage_mV)
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

bool BQ25792::ReadInputVoltage(uint16_t *voltage_mV)
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

bool BQ25792::ReadSystemVoltage(uint16_t *voltage_mV)
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

bool BQ25792::SetInputVoltageMinLimit(uint16_t voltage_mV)
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
bool BQ25792::SetHvdcp()
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

bool BQ25792::TriggerInputVoltageDetection()
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

bool BQ25792::ReadBatteryVoltage(uint16_t *vbat_mV)
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

bool BQ25792::SetAdcControl(bool enabled, bool runningAverage)
{
  EnsureChargerIsDisabled();

  uint8_t data = 0;
  // set 7th bit to ADC control enabled
  data |= (enabled & 1) << 7;
  // data |= (enabled & 1) << 6;
  // data |= (runningAverage & CHARGER25_SET_ADC_AVG_RUN_AVG) << 3;
  // data |= (runningAverage & CHARGER25_SET_ADC_AVG_INIT_ADC_CNV) << 2;
  WriteRegister(CHARGER25_REG_ADC_CONTROL, (uint8_t *)&data, sizeof(data));

  /*vTaskDelay(10 / portTICK_PERIOD_MS);
  data = 0xFF;
  data &= ~(1 << 6);
  data &= ~(1 << 4);
  //data |= 1 << 5;
   WriteRegister(CHARGER25_REG_ADC_FUNCTION_DISABLE_0, (uint8_t *)&data, sizeof(data)); */

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
bool BQ25792::ReadChargingCurrent(int16_t *current_mA)
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
  //*current_mA = current_mA;
  return true;
}

bool BQ25792::SetChargingCurrentMonitoring(bool enabled)
{
  EnsureChargerIsDisabled();

  uint8_t dataFlags = 0;

  // check bit 5 to see if battery is discharged when set
  dataFlags |= (uint8_t)enabled << 5;
  return WriteRegister(CHARGER25_REG_CHARGER_CONTROL_5, (uint8_t *)&dataFlags, sizeof(dataFlags));
}

bool BQ25792::ResetWatchdogTimer()
{
  // this is one byte, where bits 0-2 need to be set to 0x07 and bit 3 to 0x01; please do it in two steps

  uint8_t data = 0x06; // set watchdog timer to 80s
  data |= 1 << 3;      // reset watchdog timer
  return WriteRegister(CHARGER25_REG_CHARGER_CONTROL_1, (uint8_t *)&data, sizeof(data));
}

bool BQ25792::ReadChargerStatus(uint8_t *chargeStatus_CHG_STAT, uint8_t *chargeStatus_VBUS_STAT)
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

bool BQ25792::ReadFaults(uint16_t *faultStatus)
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

/*bool BQ25792::SetRechargeThreshold(uint16_t voltage_mV)
{
  // check input voltage for that range
  if (voltage_mV < 3000 || voltage_mV > 18800)
    return false;

  uint16_t voltage_xV = voltage_mV / 50;
  uint8_t data = 0;
  data = (uint8_t)(voltage_xV) & 0x0F;
  return WriteRegister(CHARGER25_REG_RECHARGE_CONTROL, (uint8_t *)data, sizeof(data));
} */

bool BQ25792::EnableCharger(bool enable, bool inputCurrentOptimizer)
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

bool BQ25792::ReadChargerEnabled(bool *charger_EN_CHG, bool *charger_EN_TERM)
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

void BQ25792::EnsureChargerIsDisabled()
{
  if (mChargerEnabled)
  {
    EnableCharger(false);
  }
}

bool BQ25792::SetCellCount(uint8_t cellCount)
{
  EnsureChargerIsDisabled();

  if (cellCount < CHARGER25_VTG_CELL_COUNT_MIN || cellCount > CHARGER25_VTG_CELL_COUNT_MAX)
    return false;

  uint8_t data = 0;
  data = (cellCount - 1) << 6;
  return WriteRegister(CHARGER25_REG_RECHARGE_CONTROL, (uint8_t *)&data, sizeof(data));
}
