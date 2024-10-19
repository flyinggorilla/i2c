#pragma once

#include "I2c.h"
#include "BNO055Registers.h"
#include <esp_system.h>
#include <esp_task.h>

// https://github.com/boschsensortec/BNO055_SensorAPI
// https://www.bosch-sensortec.com/media/boschsensortec/downloads/application_notes_1/bst-bno055-an007.pdf
// https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
// https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview

typedef struct 
{
    int16_t h; // Euler h data 
    int16_t r; // Euler r data 
    int16_t p; // Euler p data 
} EulerHrpAngles;

typedef struct
{
    uint8_t magCal;
    uint8_t accCal;
    uint8_t gyroCal;
    uint8_t sysCal;
} IMUStatus;

typedef struct
{
    int16_t magX;
    int16_t magY;
    int16_t magZ;
    int16_t magRadius;
    int16_t accX;
    int16_t accY;
    int16_t accZ;
    int16_t accRadius;
    int16_t gyrX;
    int16_t gyrY;
    int16_t gyrZ;
} IMUData;

class BNO055 : public I2cDevice
{
public:
    BNO055(I2c &rI2c, gpio_num_t intPin);
    virtual ~BNO055()
    {
        // Destructor implementation goes here
    }

    bool Reset();
    void ConfigOperationMode(uint8_t mode = BNO055_OPERATION_MODE_NDOF);
    bool WriteOperationMode(uint8_t mode);

    /// @brief Configures the mounting orientation of the Chip
    /// @param mapConfig - bits to remap the 3D orientation of the axis
    /// @param mapSign - bits to remap the +/- direction of the axis
    bool WriteMountingOrientation(uint8_t mapConfig, uint8_t mapSign);

    // bool ConfigMagnetometer(int16_t offsetX, int16_t offsetY, int16_t offsetZ, float declinationAngle);
    bool ConfigUnits();

    // bool RunCalibration();
    // bool ReadEulerAngles();
    float GetHeading();

    void GetStatus(IMUStatus &status);
    void GetMagGyrAcc(IMUData &magGyrAcc);
    void GetCalibration(IMUData &calibration);

    bool ReadEulerHrpAngles();
    bool ReadCalibrationStatus();

    bool ReadCalibrationValues(IMUData &calibration);
    bool WriteCalibrationValues(IMUData &calibration);
    bool ReadMagGyrAccValues();

    bool SwitchToConfigMode();
    bool SwitchToOperationMode();



    /// @brief initialize the device
    /// @return 
    bool Init();

    EulerHrpAngles mEulerHrpAngles = {};
    IMUStatus mStatus = {};
    IMUData mCalibration = {};
    bool mCalibrated = false;
    uint8_t mOperationMode = BNO055_OPERATION_MODE_NDOF;
    uint8_t mCurrentMode = BNO055_OPERATION_MODE_CONFIG;
    IMUData mMagGyrAcc = {};

private:
    gpio_num_t mIntPin;
    portMUX_TYPE mSpinlock = portMUX_INITIALIZER_UNLOCKED;

};
