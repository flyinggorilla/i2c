#ifndef I2C_H_
#define I2C_H_

#include <esp_system.h>
#include "driver/i2c_master.h"

void log_uint32_t_in_binary_format_as_bits(uint32_t num);
void log_uint8_t_in_binary_format_as_bits(uint8_t num);


class I2c
{
public:
    /// @brief Constructs a new synchroneous UART interface
    /// @param uartNo
    I2c();
    virtual ~I2c();

    bool Init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl);

    friend class I2cDevice;

protected:
    //i2c_port_num_t mI2cPort = I2C_NUM_0;
    i2c_master_bus_handle_t mhMasterBusHandle = NULL;
};

class I2cDevice
{
public:
    I2cDevice(I2c &rI2c) : mrI2c(rI2c) { };
    virtual ~I2cDevice()
    {
        // Destructor implementation goes here
    }

    virtual bool Init() = 0; // pure virtual function
    bool Write(uint8_t data);

    /// @brief Write data to the device at the specified register
    /// @param reg 
    /// @param data 
    /// @param dataLength should not exceed 32 bytes for I2C
    /// @return 
    bool WriteRegister(const uint8_t reg, const uint8_t *data, size_t dataLength);

    /// @brief Write data to device at the last register position
    /// @param data 
    /// @param dataLength should not exceed 32 bytes for I2C
    /// @return 
    bool Write(const uint8_t *data, size_t dataLength);

    bool Read(uint8_t *data, size_t dataLength);
    bool ReadRegister(uint8_t reg, uint8_t *data, size_t dataLength);

    /// @brief Note: use pull-up resistor for I2C or very low speeds
    /// high speeds without pullup causes Guru Meditation Error: Core 0 panic'ed (Interrupt wdt timeout on CPU0).
    /// @param addrBitLen 
    /// @param deviceAddress 
    /// @param speedHz 
    /// @param deviceName 
    /// @return 
    bool AddDevice(i2c_addr_bit_len_t addrBitLen, uint16_t deviceAddress, uint32_t speedHz, const char* deviceName);
    bool MasterProbe(uint16_t deviceAddress);


protected:
    //uint8_t mDeviceAddress;
    I2c &mrI2c;
    i2c_master_dev_handle_t mhDevHandle = NULL;
    const char* mcstrDeviceName;
};

#endif
