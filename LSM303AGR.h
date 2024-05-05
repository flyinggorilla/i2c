#ifndef LSM303AGR_H_
#define LSM303AGR_H_

#include "I2c.h"
#include "LSM303AGRRegisters.h"
#include <esp_system.h>
#include <esp_task.h>

// https://github.com/STMicroelectronics/stm32-lsm303agr/blob/main/lsm303agr.h


class LSM303AGR : public I2cDevice
{
public:
    LSM303AGR(I2c &rI2c, gpio_num_t intPin);
    virtual ~LSM303AGR()
    {
        // Destructor implementation goes here
    }

    bool ConfigMagnetometer(int16_t offsetX, int16_t offsetY, int16_t offsetZ, float declinationAngle);

    bool RunCalibration();


    bool ReadMagnetometer();

    float GetHeading();

    /// @brief initialize the device
    /// @return 
    bool Init();

    int16_t mX = 0;
    int16_t mY = 0;
    int16_t mZ = 0;
    float mDeclinationAngle = 0.0;


private:
    gpio_num_t mIntPin;
    portMUX_TYPE mSpinlock = portMUX_INITIALIZER_UNLOCKED;

};


#endif