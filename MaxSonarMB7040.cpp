// 


#include "MaxSonarMB7040.h"
#include "esp_log.h"
#include "esp_task.h"

// Register Map
#define MaxSonarMB7040_DEVICE_ADDRESS_7BIT 0x70 // 112 in decimal, 7 bit address required
#define MaxSonarMB7040_DEVICE_FREQ_HZ 10000    




static const char *tag = "MaxSonarMB7040";


// @deviceAddress: Valid IDs are 0x28 (default), 0x29, 0x2A, and 0x2B.
MaxSonarMB7040::MaxSonarMB7040(I2c &rI2c) : I2cDevice(rI2c)
{
}


bool MaxSonarMB7040::Init()
{
  if (!AddDevice(I2C_ADDR_BIT_LEN_7, MaxSonarMB7040_DEVICE_ADDRESS_7BIT, MaxSonarMB7040_DEVICE_FREQ_HZ, "MaxSonarMB7040"))
    return false;
  if(!MasterProbe(MaxSonarMB7040_DEVICE_ADDRESS_7BIT)) 
    return false;

  return true;
}

bool MaxSonarMB7040::TriggerDistanceMeasurement()
{
  // write 0x51 to the device to request a measurement
  if(!Write(0x51)) // 81 = 0x51 is range command
  {
    return false;
  }
  return true;
}

// wait for ~100ms for the measurement to be ready (datasheet)
// alternatively, there is a pin that can be used to signal when the measurement is ready (not implemented here)
bool MaxSonarMB7040::ReadDistance(unsigned short * distanceCm)
{


  uint8_t data[2];
  
  if (!Read(data, sizeof(data))) {
    *distanceCm = 0;
    return false;
  }

  unsigned short distance = (data[0] << 8) | data[1];
  *distanceCm = distance;

  return true;
}


