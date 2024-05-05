#ifndef StUsb4500_H_
#define StUsb4500_H_

#include "I2c.h"


class StUsb4500 : public I2cDevice
{
public:
    StUsb4500(I2c &rI2c);
    virtual ~StUsb4500()
    {
        // Destructor implementation goes here
    }
    bool Init();

    bool softReset(void);


    float getVoltage(uint8_t pdo_numb);
    bool setVoltage(uint8_t pdo_numb, float voltage);


    bool Read(uint8_t reg, uint8_t *data);

    uint32_t readPDO(uint8_t pdo_numb);
    bool writePDO(uint8_t pdo_numb, uint32_t pdoData);


    bool I2C_Read_USB_PD(uint8_t Register, uint8_t *DataR, uint16_t Length);
    bool I2C_Write_USB_PD(uint8_t Register, uint8_t *DataW, uint16_t Length);


    uint8_t getPdoNumber(void);


    // esp_err_t Read(uint8_t reg, uint8_t *data);
    // esp_err_t Write(uint8_t reg, uint8_t data);
private:
    uint8_t sector[5][8];
    uint8_t readSectors = 0;
};

#endif