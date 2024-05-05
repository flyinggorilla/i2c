// https://github.com/usb-c/STUSB4500
// https://www.sparkfun.com/products/15801
// https://www.st.com/resource/en/datasheet/stusb4500.pdf
// https://www.st.com/resource/en/user_manual/um2650-the-stusb4500-software-programing-guide-stmicroelectronics.pdf

#include "StUsb4500.h"
#include "esp_log.h"

// Register Map
#define DEFAULT 0xFF

#define FTP_CUST_PASSWORD_REG 0x95
#define FTP_CUST_PASSWORD 0x47

#define FTP_CTRL_0 0x96
#define FTP_CUST_PWR 0x80
#define FTP_CUST_RST_N 0x40
#define FTP_CUST_REQ 0x10
#define FTP_CUST_SECT 0x07
#define FTP_CTRL_1 0x97
#define FTP_CUST_SER 0xF8
#define FTP_CUST_OPCODE 0x07
#define RW_BUFFER 0x53
#define TX_HEADER_LOW 0x51
#define PD_COMMAND_CTRL 0x1A
#define DPM_PDO_NUMB 0x70

#define READ 0x00
#define WRITE_PL 0x01
#define WRITE_SER 0x02
#define ERASE_SECTOR 0x05
#define PROG_SECTOR 0x06
#define SOFT_PROG_SECTOR 0x07

#define SECTOR_0 0x01
#define SECTOR_1 0x02
#define SECTOR_2 0x04
#define SECTOR_3 0x08
#define SECTOR_4 0x10

#define STUSB4500_DEVICE_ADDRESS_7BIT 0x28
#define STUSB4500_DEVICE_FREQ_HZ 100000    


static const char *tag = "StUsb4500";


// @deviceAddress: Valid IDs are 0x28 (default), 0x29, 0x2A, and 0x2B.
StUsb4500::StUsb4500(I2c &rI2c) : I2cDevice(rI2c)
{
}


bool StUsb4500::Init()
{
  if (!AddDevice(I2C_ADDR_BIT_LEN_7, STUSB4500_DEVICE_ADDRESS_7BIT, STUSB4500_DEVICE_FREQ_HZ, "STUSB4500"))
    return false;
  if(!MasterProbe(STUSB4500_DEVICE_ADDRESS_7BIT)) 
    return false;

  //ReadTillTimeout();

  uint8_t regVal;
  // read all 10 registers from 0x0D to 0x16
  for (uint8_t reg = 0x0D; reg <= 0x16; reg++)
  {
    // log the register address
    ESP_LOGI(tag, "Init Register: %02x", reg);
    I2C_Read_USB_PD(reg, &regVal, sizeof(regVal));
    log_uint8_t_in_binary_format_as_bits(regVal);
  }

  //ReadTillTimeout();

  //uint8_t deviceId;
  I2C_Read_USB_PD(0x2F, &regVal, sizeof(regVal));
  // log dataR which is the device ID 0x25 or 0x21
  ESP_LOGI(tag, "Device ID 0x25 (v1) or 0x21 (v2): %02x", regVal);
  log_uint8_t_in_binary_format_as_bits(regVal);


  return softReset();
}


bool StUsb4500::Read(uint8_t reg, uint8_t *data)
{
  // i2c_master_receive(mhDevHandle, data_rd, DATA_LENGTH, -1);

  uint8_t buf[20] = {0x20};
  uint8_t buffer[2];
  return i2c_master_transmit_receive(mhDevHandle, buf, sizeof(buf), buffer, 2, -1);
}


float StUsb4500::getVoltage(uint8_t pdo_numb)
{
  float voltage = 0;
  uint32_t pdoData = readPDO(pdo_numb);
  // log pdoData to hex and binary
  ESP_LOGI(tag, "pdo%dData: %lx", pdo_numb, pdoData);
  log_uint32_t_in_binary_format_as_bits(pdoData);

  pdoData = (pdoData >> 10) & 0x3FF;
  voltage = pdoData / 20.0;

  return voltage;
}

uint32_t StUsb4500::readPDO(uint8_t pdo_numb)
{
  uint32_t pdoData = 0;
  uint8_t Buffer[4] = {};

  // PDO1:0x85, PDO2:0x89, PDO3:0x8D
  if(!I2C_Read_USB_PD(0x85 + ((pdo_numb - 1) * 4), Buffer, 4))
    return 0;

  // Combine the 4 buffer bytes into one 32-bit integer
  for (uint8_t i = 0; i < 4; i++)
  {
    uint32_t tempData = Buffer[i];
    tempData = (tempData << (i * 8));
    pdoData += tempData;
  }

  return pdoData;
}

bool StUsb4500::writePDO(uint8_t pdo_numb, uint32_t pdoData)
{
  uint8_t Buffer[4];

  Buffer[0] = (pdoData) & 0xFF;
  Buffer[1] = (pdoData >> 8) & 0xFF;
  Buffer[2] = (pdoData >> 16) & 0xFF;
  Buffer[3] = (pdoData >> 24) & 0xFF;

  return I2C_Write_USB_PD(0x85 + ((pdo_numb - 1) * 4), Buffer, 4);
}

uint8_t StUsb4500::getPdoNumber(void)
{
  uint8_t pdoNumber = 0;
  I2C_Read_USB_PD(DPM_PDO_NUMB, &pdoNumber, sizeof(pdoNumber)) ;

  log_uint8_t_in_binary_format_as_bits(pdoNumber);

  // log the pdoNumber
  ESP_LOGI(tag, "PDO Number: %d, %d",pdoNumber, pdoNumber & 0x07);
  // log the numbers now in hex
  ESP_LOGI(tag, "PDO Number hex: %x, %x", pdoNumber, pdoNumber & 0x07);

  return pdoNumber & 0x07;
}

// A function that reverses the bits of a byte
uint8_t msb_to_lsb(uint8_t byte)
{
  uint8_t reversed = 0;
  // Loop through the 8 bits of the byte
  for (int i = 0; i < 8; i++)
  {
    // Shift the reversed byte to the left by one bit
    reversed <<= 1;
    // Copy the least significant bit of the original byte to the reversed byte
    reversed |= byte & 1;
    // Shift the original byte to the right by one bit
    byte >>= 1;
  }
  return reversed;
}

bool StUsb4500::I2C_Read_USB_PD(uint8_t Register, uint8_t *DataR, uint16_t Length)
{

  uint8_t tempData[Length];
  if(ESP_OK != i2c_master_transmit_receive(mhDevHandle, (const uint8_t *)&Register, sizeof(Register), DataR, Length, -1)) {
    ESP_LOGE(tag, "i2c_master_transmit_receive failed");
    return false;
  }

  return true;
}

bool StUsb4500::I2C_Write_USB_PD(uint8_t Register, uint8_t *DataW, uint16_t Length)
{

  /*uint8_t tempData[Length];
  for (uint8_t i = 0; i < Length; i++)
  {
    tempData[i] = (*(DataW + i));
  }*/


// create compiler warning through # warning
#warning "TODO maybe this must be a single transmit for the write to work! with Register + Data as one buffer"



  ////************* TODO THERE MUST BE SOME BYTEORDER ISSUE HERE *************////
  esp_err_t ret = i2c_master_transmit(mhDevHandle, (const uint8_t *)&Register, sizeof(Register), -1);

  ret = i2c_master_transmit(mhDevHandle, (const uint8_t *)&DataW, Length, -1);
  /*

uint8_t StUsb4500::I2C_Write_USB_PD(uint16_t Register, uint8_t *DataW, uint16_t Length)
{
  uint8_t error = 0;
  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(Register);
  for (uint8_t i = 0; i < Length; i++)
  {
    _i2cPort->write(*(DataW + i));
  }
  error = _i2cPort->endTransmission();
  delay(1);

  return error;
}
*/
 if (ret != ESP_OK) {
    ESP_LOGE(tag, "I2C_Write_USB_PD failed");
    return false;
  }

  return true;
}

bool StUsb4500::softReset(void)
{
  uint8_t Buffer[1];

  // Soft Reset
  Buffer[0] = 0x0D; // SOFT_RESET
  I2C_Write_USB_PD(TX_HEADER_LOW, Buffer, 1);

  Buffer[0] = 0x26; // SEND_COMMAND
  return I2C_Write_USB_PD(PD_COMMAND_CTRL, Buffer, 1);
}

bool StUsb4500::setVoltage(uint8_t pdo_numb, float voltage)
{
  if (pdo_numb < 1)
    pdo_numb = 1;
  else if (pdo_numb > 3)
    pdo_numb = 3;

  // Constrain voltage variable to 5-20V
  if (voltage < 5)
    voltage = 5;
  else if (voltage > 20)
    voltage = 20;

  // Load voltage to volatile PDO memory (PDO1 needs to remain at 5V)
  if (pdo_numb == 1)
    voltage = 5;

  voltage *= 20;

  // Replace voltage from bits 10:19 with new voltage
  uint32_t pdoData = readPDO(pdo_numb);

  pdoData &= ~(0xFFC00);
  pdoData |= (uint32_t(voltage) << 10);

  return writePDO(pdo_numb, pdoData);
}
