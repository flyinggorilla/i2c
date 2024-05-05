#ifndef BQ25792_H_
#define BQ25792_H_

#include "I2c.h"


class BQ25792 : public I2cDevice
{
public:
    BQ25792(I2c &rI2c, gpio_num_t chargeEnablePin);
    virtual ~BQ25792()
    {
        // Destructor implementation goes here
    }
    bool Init();
    bool SetHvdcp();

    // overrides POR setting based on resistors - defines min/max of system voltage and other settings
    bool SetCellCount(uint8_t cellCount);
    // is a calculated value based on the system voltage
    bool GetCellCount(); 

    bool ReadSystemVoltageMinimum(uint16_t *vsys_min);
    bool SetSystemVoltageMinimum(uint16_t vsys_min);

    bool SetChargeVoltageLimit(uint16_t voltage_mV);
    bool ReadChargeVoltageLimit(uint16_t *voltage_mV);
    bool SetChargeCurrentLimit(uint16_t current_mA);
    bool ReadChargeCurrentLimit(uint16_t *current_mA);

    bool ReadBatteryVoltage(uint16_t *vbat_mV);
    bool ReadSystemVoltage(uint16_t *voltage_mV);
    bool ReadInputVoltage(uint16_t *voltage_mV);
    bool ReadChargingCurrent(int16_t *current_mA);

    bool ReadInputVoltageMinLimit(uint16_t *voltage_mV);
    bool SetInputVoltageMinLimit(uint16_t voltage_mV);
    bool ReadInputCurrentMinLimit(uint16_t *current_mA);
    bool SetInputCurrentMinLimit(uint16_t current_mA);

    bool TriggerInputVoltageDetection();

    bool SetAdcControl(bool enabled = true, bool runningAverage = true);
    bool SetChargingCurrentMonitoring(bool enabled);

    // This function must be called at minimum every 30s
    // Default Charger Watchdog timeout is set to 40s
    // REG10_Charger_Control_1 Register
    bool ResetWatchdogTimer();

    bool ReadChargerStatus(uint8_t* chargeStatus_CHG_STAT, uint8_t* chargeStatus_VBUS_STAT);

    #define CHARGER25_FAULT_STATUS_FLAG_IBAT_REG_STAT (0x80 << 8) // bit 7 REG20, 0h = Normal, 1h = Device in battery discharging current regulation
    #define CHARGER25_FAULT_STATUS_FLAG_VBUS_OVP_STAT (0x40 << 8) // bit 6 REG20, 0h = Normal, 1h = Device in over voltage protection
    #define CHARGER25_FAULT_STATUS_FLAG_VBAT_OVP_STAT (0x20 << 8) // bit 5 REG20, 0h = Normal, 1h = Device in over voltage protection
    #define CHARGER25_FAULT_STATUS_FLAG_IBUS_OCP_STAT (0x10 << 8) // bit 4 REG20, 0h = Normal, 1h = Device in over current protection
    #define CHARGER25_FAULT_STATUS_FLAG_IBAT_OCP_STAT (0x08 << 8) // bit 3 REG20, 0h = Normal, 1h = Device in over current protection
    #define CHARGER25_FAULT_STATUS_FLAG_CONV_OCP_STAT (0x04 << 8) // bit 2 REG20, 0h = Normal, 1h = Device in over current protection
    #define CHARGER25_FAULT_STATUS_FLAG_VAC2_OVP_STAT (0x02 << 8) // bit 1 REG20, 0h = Normal, 1h = Device in over voltage protection
    #define CHARGER25_FAULT_STATUS_FLAG_VAC1_OVP_STAT (0x02 << 8) // bit 0 REG20, 0h = Normal, 1h = Device in over voltage protection

    #define CHARGER25_FAULT_STATUS_FLAG_VSYS_SHORT_STAT 0x80 // bit 7 REG21, 0h = Normal, 1h = Device in SYS short circuit protection
    #define CHARGER25_FAULT_STATUS_FLAG_VSYS_OVP_STAT 0x40 // bit 6 REG21, 0h = Normal, 1h = Device in SYS over voltage protection
    #define CHARGER25_FAULT_STATUS_FLAG_OTG_OVP_STAT 0x20 // bit 5 REG21, 0h = Normal, 1h = Device in OTG over-voltage
    #define CHARGER25_FAULT_STATUS_FLAG_OTG_UVP_STAT 0x10 // bit 4 REG21, 0h = Normal, 1h = Device in OTG under-voltage
    #define CHARGER25_FAULT_STATUS_FLAG_TSHUT_STAT 0x04 // bit 2 REG21, 0h = Normal, 1h = Device in thermal shutdown protection

    #define CHARGER25_FAULT_STATUS_FLAG_NOBATTERY_STAT 0xFFFF // all bits set, means no battery/status
    bool ReadFaults(uint16_t* faultsStatus);

    //bool SetRechargeThreshold(uint16_t voltage_mV);

    // The device provides Input Current Optimizer (ICO) to identify maximum power point in order to avoid overloading 
    // the input source. The algorithm automatically identifies maximum input current limit of an unknown power source 
    // and sets the charger IINDPM register properly, in order to prevent from entering the charger input voltage 
    // (VINDPM) regulation. This feature is disabled by default at POR (EN_ICO = 0) and only activates when EN_ICO 
    // bit is set to 1.
    //  After DCP type input source is detected based on the procedures described in Section 9.3.4.5, the algorithm 
    // runs automatically if EN_ICO bit is set. The algorithm can also be forced to execute by setting FORCE_ICO bit 
    // regardless of input source type detected. Please note that EN_ICO = 1 is required for FORCE_ICO to work.
    bool EnableCharger(bool enable = true, bool inputCurrentOptimizer = false);
    bool DisableCharger() { return EnableCharger(false);};

    bool IsChargerEnabled() { return mChargerEnabled; };

    /// @brief Reads charger enable status - note that the charger is only enabled when CE pin low TOO; 
    // use IsEnabled() to check if the charger is enabled
    /// @param charger_EN_CHG 
    /// @param charger_EN_TERM 
    /// @return 
    bool ReadChargerEnabled(bool *charger_EN_CHG, bool *charger_EN_TERM = nullptr);


private:
    gpio_num_t mChargeEnablePin;
    bool mChargerEnabled = true; // assume it is on, as the charer "smokes" if it is enabled and settings are changed!!!!
    void EnsureChargerIsDisabled();
};

#endif