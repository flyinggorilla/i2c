#include "I2c.h"
#include <hal/i2c_ll.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_task.h>


#define I2C_MASTER_TIMEOUT_MS (50) 


const char tag[] = "I2c";


void log_uint32_t_in_binary_format_as_bits(uint32_t num)
{
  char bits[33];
  for (int i = 0; i < 32; i++)
  {
    bits[i] = (num & (1 << (31 - i))) ? '1' : '0';
  }
  bits[32] = '\0';
  ESP_LOGW(tag, "bits: %s", bits);
}

// log uint8_t in binary format as bits
void log_uint8_t_in_binary_format_as_bits(uint8_t num)
{
  char bits[9];
  for (int i = 0; i < 8; i++)
  {
    bits[i] = (num & (1 << (7 - i))) ? '1' : '0';
  }
  bits[8] = '\0';
  ESP_LOGW(tag, "bits: %s", bits);
}


I2c::I2c()
{
}

I2c::~I2c()
{
}

// add arguments to Init constructor: i2c_port_t i2cPort, gpio_num_t sda_io_num, gpio_num_t scl_io_num, uint32_t clk_speed
bool I2c::Init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl)
{
    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = port,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0, // only for async mode
        .flags = {}, // Initialize flags member with curly braces
    };
    i2c_mst_config.flags.enable_internal_pullup = true;

    if (ESP_OK != i2c_new_master_bus(&i2c_mst_config, &mhMasterBusHandle))
    {
        ESP_LOGE(tag, "i2c_new_master_bus failed");
        return false;
    }
    return true;
}

bool I2cDevice::Write(uint8_t data)
{
    esp_err_t ret = i2c_master_transmit(mhDevHandle, &data, sizeof(data), I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "i2c_master_transmit %s failed with code %x", mcstrDeviceName, ret);
        return false;
    }
    return true;
}

bool I2cDevice::WriteRegister(const uint8_t reg, const uint8_t* data, size_t dataLength)
{
    uint8_t buffer[dataLength + 1];
    buffer[0] = reg;
    memcpy(&buffer[1], data, dataLength);
    esp_err_t ret = i2c_master_transmit(mhDevHandle, buffer, sizeof(buffer), I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "i2c_master_transmit to reg 0x%02x %s failed with code %x", reg, mcstrDeviceName, ret);
        return false;
    }
    return true;
}

bool I2cDevice::Write(const uint8_t *data, size_t dataLength)
{
    esp_err_t ret = i2c_master_transmit(mhDevHandle, data, dataLength, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "i2c_master_transmit %s failed with code %x", mcstrDeviceName, ret);
        return false;
    }
    return true;
}


bool I2cDevice::ReadRegister(uint8_t reg, uint8_t *data, size_t dataLength)
{
    esp_err_t ret = i2c_master_transmit_receive(mhDevHandle, (const uint8_t *)&reg, sizeof(reg), data, dataLength, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "i2c_master_transmit_receive %s failed with code %x", mcstrDeviceName, ret);
        return false;
    }
    return true;
}


bool I2cDevice::Read(uint8_t *data, size_t dataLength)
{
    esp_err_t ret = i2c_master_receive(mhDevHandle, data, dataLength, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "i2c_master_receive %s failed with code %x", mcstrDeviceName, ret);
        return false;
    }
    return true;
}

 
/*
bool I2cDevice::ReadTillTimeout() {
  // call i2c_master_receive in a loop and print the received data as hexadecimal values as well as a string of the individual bits
    uint8_t data;
    uint8_t i = 0;
    while (i2c_master_receive(mhDevHandle, &data, 1, I2C_MASTER_TIMEOUT_MS) == ESP_OK) {
        //ESP_LOGI(tag, "data: %02x", data);
        i++;
        if (i > 10)
            break;
    }
    ESP_LOGI(tag, "ReadTillTimeout completed reading i: %d", i);
    return true;
}
*/

bool I2cDevice::AddDevice(i2c_addr_bit_len_t addrBitLen, uint16_t deviceAddress, uint32_t speedHz, const char* deviceName)
{
    mcstrDeviceName = deviceName;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = addrBitLen,
        .device_address = deviceAddress,
        .scl_speed_hz = speedHz,
    };

    esp_err_t ret = i2c_master_bus_add_device(mrI2c.mhMasterBusHandle, &dev_cfg, &mhDevHandle);
    if (ESP_OK != ret)
    {
        ESP_LOGE(tag, "i2c_master_bus_add_device %s failed with code %x", mcstrDeviceName, ret);
        return false;
    }

    return true;
}

bool I2cDevice::MasterProbe(uint16_t deviceAddress)
{
    esp_err_t ret = i2c_master_probe(mrI2c.mhMasterBusHandle, deviceAddress, I2C_MASTER_TIMEOUT_MS);
    if (ESP_OK != ret)
    {
        ESP_LOGE(tag, "%s device %s!", mcstrDeviceName, ret == ESP_ERR_NOT_FOUND ? "not found" : "timeout");
        return false;
    }
    ESP_LOGI(tag, "%s device detected", mcstrDeviceName);
    return true;
}
