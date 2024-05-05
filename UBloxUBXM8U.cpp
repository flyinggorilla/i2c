//

#include "UBloxUBX.h"
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task.h"

// Register Map
// #define UBloxUBX_DEVICE_ADDRESS_7BIT 0x42
// #define UBloxUBX_DEVICE_FREQ_HZ 50000

static const char *tag = "UBloxUBX";

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

UBloxUBX::UBloxUBX(I2c &rI2c) : UBloxDDC(rI2c)
{
}

// Given a message and a byte, add to rolling "8-Bit Fletcher" checksum
// This is used when receiving messages from module
//  void UBloxDDC::addToChecksum(uint8_t incoming)
//  {
//    rollingChecksumA += incoming;
//    rollingChecksumB += rollingChecksumA;
//  }

// Returns false if sensor fails to respond to I2C traffic

bool UBloxUBX::SendPreparedUbxPacket()
{
    ubxPacket *pUbxPacket = GetUbxPacket();
    return SendUbxPacket(pUbxPacket->cls, pUbxPacket->id, pUbxPacket->payload, pUbxPacket->len);
}

// Configure a given port to output UBX, NMEA, RTCM3 or a combination thereof
// Port 0=I2c, 1=UART1, 2=UART2, 3=USB, 4=SPI
// Bit:0 = UBX, :1=NMEA, :5=RTCM3
bool UBloxUBX::ConfigPortOutput()
{
    ESP_LOGD(tag, "ConfigPortOutput");
    PrepareUbxPacket(UBX_CLASS_CFG, UBX_CFG_PRT, 20);
    SetUbxU1(4, 0x42 << 1); // I2C Address shifted left by 1
    SetUbxU1(12, 7);        // inUbx, inNmea, inRtcm3
    SetUbxU1(14, 1);        // outUbx
    SendPreparedUbxPacket();
    WaitForAck(UBX_CLASS_CFG, UBX_CFG_PRT);
    return true;
}



// Loads the payloadCfg array with the current protocol bits located the UBX-CFG-PRT register for a given port
bool UBloxUBX::GetPortSettings(uint8_t portID, ubxGetSetCfgPrt *pGetSetCfgPrt)
{
    ESP_LOGI(tag, "GetPortSettings");
    if (!SendUbxPacket(UBX_CLASS_CFG, UBX_CFG_PRT, &portID, 1))
    {
        ESP_LOGE(tag, "GetPortSettings Failed send UBX Command");
        return false;
    }
    return true;
}

// Loads the payloadCfg array with the current protocol bits located the UBX-CFG-PRT register for a given port
bool UBloxUBX::GetAttitude(ubxPollNavAtt *pPollNavAtt)
{
    ESP_LOGI(tag, "GetAttitude");
    if (!SendUbxPacket(UBX_CLASS_NAV, UBX_NAV_ATT, nullptr, 0))
    {
        ESP_LOGE(tag, "GetAttitude Failed send UBX Command");
        return false;
    }

    return true;
}

void UBloxUBX::ProcessUbxPacket(ubxPacket *pUbxPacket)
{

    ESP_LOGV(tag, "ProcessUbxPacket");
    LogUbxPacket(ESP_LOG_VERBOSE, true, pUbxPacket->cls);

    switch (pUbxPacket->cls)
    {
    case UBX_CLASS_ACK:
        ProcessUbxAckPacket(pUbxPacket);
        // ESP_LOGV(tag, "UBX_CLASS_ACK");
        break;
    case UBX_CLASS_CFG:
        ProcessUbxCfgPacket(pUbxPacket);
        // ESP_LOGV(tag, "UBX_CLASS_CFG");
        break;
    case UBX_CLASS_NAV:
        ProcessUbxNavPacket(pUbxPacket);
        // ESP_LOGV(tag, "UBX_CLASS_NAV");
        break;
    }

    return;
}

void UBloxUBX::ProcessUbxAckPacket(ubxPacket *pUbxPacket)
{
    switch (pUbxPacket->id)
    {
    case UBX_ACK_ACK:
        UbxAck(true, pUbxPacket->cls, pUbxPacket->id);
        // ESP_LOGI(tag, "UBX_ACK_ACK");
        break;
    case UBX_ACK_NACK:
        UbxAck(false, pUbxPacket->cls, pUbxPacket->id);
        // ESP_LOGI(tag, "UBX_ACK_NACK");
        break;
    }
}

void UBloxUBX::ProcessUbxCfgPacket(ubxPacket *pUbxPacket)
{
    switch (pUbxPacket->id)
    {
    case UBX_CFG_PRT:
    {
        ubxGetSetCfgPrt getSetCfgPrt = {};
        if (pUbxPacket->len == 20 &&
            pUbxPacket->cls == UBX_CLASS_CFG && pUbxPacket->id == UBX_CFG_PRT)
        {
            getSetCfgPrt.portID = GetUbxU1(0);
            getSetCfgPrt.txReady = GetUbxU2(2);
            getSetCfgPrt.mode = GetUbxU4(4);
            getSetCfgPrt.inProtoMask = GetUbxU2(12);
            getSetCfgPrt.outProtoMask = GetUbxU2(14);
            getSetCfgPrt.flags = GetUbxU2(16);
            UbxCfgPrt(&getSetCfgPrt);
        }
        ESP_LOGI(tag, "UBX_CFG_PRT");
    }
    }
}

void UBloxUBX::ProcessUbxNavPacket(ubxPacket *pUbxPacket)
{
    // log cls and id
    // ESP_LOGI(tag, "ProcessUbxNavPacket cls: %d, id: %d", pUbxPacket->cls, pUbxPacket->id);
    switch (pUbxPacket->id)
    {
    case UBX_NAV_ATT:
    {
        ubxPollNavAtt pollNavAtt = {};
        if (pUbxPacket->len == 20 &&
            pUbxPacket->cls == UBX_CLASS_NAV && pUbxPacket->id == UBX_NAV_ATT)
        {
            pollNavAtt.iTOW = GetUbxI4(0);
            pollNavAtt.version = GetUbxU1(4);
            pollNavAtt.roll = GetUbxI4(8);
            pollNavAtt.pitch = GetUbxI4(12);
            pollNavAtt.heading = GetUbxI4(16);
            pollNavAtt.accRoll = GetUbxI4(20);
            pollNavAtt.accPitch = GetUbxI4(24);
            pollNavAtt.accHeading = GetUbxI4(28);
            UbxNavAtt(&pollNavAtt);
        }
        break;
    }
    case UBX_NAV_PVT:
    {
        ubxPollNavPvt pollNavPvt = {};
        if (pUbxPacket->len == 92 &&
            pUbxPacket->cls == UBX_CLASS_NAV && pUbxPacket->id == UBX_NAV_PVT)
        {
            pollNavPvt.iTOW = GetUbxU4(0);
            pollNavPvt.year = GetUbxU2(4);
            pollNavPvt.month = GetUbxU1(6);
            pollNavPvt.day = GetUbxU1(7);
            pollNavPvt.hour = GetUbxU1(8);
            pollNavPvt.min = GetUbxU1(9);
            pollNavPvt.sec = GetUbxU1(10);
            pollNavPvt.valid = GetUbxX1(11);
            pollNavPvt.tAcc = GetUbxU4(12);
            pollNavPvt.nano = GetUbxI4(16);
            pollNavPvt.fixType = GetUbxU1(20);
            pollNavPvt.flags = GetUbxX1(21);
            pollNavPvt.flags2 = GetUbxX1(22);
            pollNavPvt.numSV = GetUbxU1(23);
            pollNavPvt.lon = GetUbxI4(24);
            pollNavPvt.lat = GetUbxI4(28);
            pollNavPvt.height = GetUbxI4(32);
            pollNavPvt.hMSL = GetUbxI4(36);
            pollNavPvt.hAcc = GetUbxU4(40);
            pollNavPvt.vAcc = GetUbxU4(44);
            pollNavPvt.velN = GetUbxI4(48);
            pollNavPvt.velE = GetUbxI4(52);
            pollNavPvt.velD = GetUbxI4(56);
            pollNavPvt.gSpeed = GetUbxI4(60);
            pollNavPvt.headMot = GetUbxI4(64);
            pollNavPvt.sAcc = GetUbxU4(68);
            pollNavPvt.headAcc = GetUbxU4(72);
            pollNavPvt.pDOP = GetUbxU2(76);
            pollNavPvt.headVeh = GetUbxI4(84);
            pollNavPvt.magDec = GetUbxI2(88);
            pollNavPvt.magAcc = GetUbxU2(90);
            UbxNavPvt(&pollNavPvt);
        }
        else
        {
            ESP_LOGE(tag, "Error reading UBX_NAV_PVT, len: %d, cls: %d, id: %d", pUbxPacket->len, pUbxPacket->cls, pUbxPacket->id);
        }
        break;
    }
    }
}

bool UBloxUBX::WaitForAck(uint8_t cls, uint8_t id, uint16_t timeout)
{
    uint32_t startTime = esp_timer_get_time();
    while (esp_timer_get_time() - startTime < timeout)
    {
        esp_err_t ret = ReadUbxPacket();
        if(ret == ESP_OK)
        {
            ubxPacket *pUbxPacket = GetUbxPacket();
            if (pUbxPacket->cls == UBX_CLASS_ACK)
            {
                if (pUbxPacket->id == UBX_ACK_ACK)
                {
                    ESP_LOGI(tag, "ACK received");
                    return true;
                }
                else if (pUbxPacket->id == UBX_ACK_NACK)
                {
                    ESP_LOGI(tag, "NACK received");
                    return false;
                }
            }
        } else if (ret == ESP_ERR_NOT_FOUND) {
            vTaskDelay(10/portTICK_PERIOD_MS);
        }
    }
    return false;
}

/// @brief resets buffer to zeroes; use SetUbxPacket* calls to populate buffer at desired positions
/// @param cls
/// @param id
/// @param len
/// @return
bool UBloxUBX::PrepareUbxPacket(uint8_t cls, uint8_t id, uint16_t len)
{
    ubxPacket *pUbxPacket = GetUbxPacket();

    if (len > MAX_PAYLOAD_SIZE)
    {
        pUbxPacket->cls = UBX_CLASS_NONE;
        pUbxPacket->id = 0;
        pUbxPacket->len = 0;
        ESP_LOGE(tag, "PrepareUbxPacket: len > MAX_PAYLOAD_SIZE");
        return false;
    }

    pUbxPacket->cls = cls;
    pUbxPacket->id = id;
    pUbxPacket->len = len;
    // memset(pUbxPacket->payload, 0xFF, MAX_PAYLOAD_SIZE);
    memset(pUbxPacket->payload, 0, len);
    return true;
}

bool UBloxUBX::SetUbxU1(uint16_t pos, uint8_t value)
{
    ubxPacket *pUbxPacket = GetUbxPacket();
    if (pos < pUbxPacket->len)
    {
        pUbxPacket->payload[pos] = value;
        return true;
    }
    return false;
}

bool UBloxUBX::SetUbxU2(uint16_t pos, uint16_t value)
{
    ubxPacket *pUbxPacket = GetUbxPacket();
    if (pos < pUbxPacket->len - 1)
    {
        pUbxPacket->payload[pos] = value & 0xFF;
        pUbxPacket->payload[pos + 1] = value >> 8;
        return true;
    }
    return false;
}

bool UBloxUBX::SetUbxU4(uint16_t pos, uint32_t value)
{
    ubxPacket *pUbxPacket = GetUbxPacket();
    if (pos < pUbxPacket->len - 3)
    {
        pUbxPacket->payload[pos] = value & 0xFF;
        pUbxPacket->payload[pos + 1] = value >> 8;
        pUbxPacket->payload[pos + 2] = value >> 16;
        pUbxPacket->payload[pos + 3] = value >> 24;
        return true;
    }
    return false;
}

bool UBloxUBX::SetUbxI2(uint16_t pos, int16_t value)
{
    ubxPacket *pUbxPacket = GetUbxPacket();
    if (pos < pUbxPacket->len - 1)
    {
        pUbxPacket->payload[pos] = value & 0xFF;
        pUbxPacket->payload[pos + 1] = value >> 8;
        return true;
    }
    return false;
}

bool UBloxUBX::SetUbxI4(uint16_t pos, int32_t value)
{
    ubxPacket *pUbxPacket = GetUbxPacket();
    if (pos < pUbxPacket->len - 3)
    {
        pUbxPacket->payload[pos] = value & 0xFF;
        pUbxPacket->payload[pos + 1] = value >> 8;
        pUbxPacket->payload[pos + 2] = value >> 16;
        pUbxPacket->payload[pos + 3] = value >> 24;
        pUbxPacket->len += 4;
        return true;
    }
    return false;
}


bool UBloxUBX::ConfigMessageRate(uint8_t cls, uint8_t id, uint8_t rate)
{
    ESP_LOGD(tag, "ConfigMessageRate %d", rate);
    PrepareUbxPacket(UBX_CLASS_CFG, UBX_CFG_MSG, 3);
    SetUbxU1(0, cls);
    SetUbxU1(1, id);
    SetUbxU1(2, rate);
    SendPreparedUbxPacket();
    WaitForAck(UBX_CLASS_CFG, UBX_CFG_MSG);
    return true;
}

uint8_t UBloxUBX::GetUbxU1(uint16_t pos)
{
    ubxPacket *pUbxPacket = GetUbxPacket();
    if (pos < pUbxPacket->len)
    {
        return pUbxPacket->payload[pos];
    }
    return 0;
}

uint16_t UBloxUBX::GetUbxU2(uint16_t pos)
{
    ubxPacket *pUbxPacket = GetUbxPacket();
    if (pos < pUbxPacket->len - 1)
    {
        return (pUbxPacket->payload[pos + 1] << 8) | pUbxPacket->payload[pos];
    }
    return 0;
}

uint32_t UBloxUBX::GetUbxU4(uint16_t pos)
{
    ubxPacket *pUbxPacket = GetUbxPacket();
    if (pos < pUbxPacket->len - 3)
    {
        return (pUbxPacket->payload[pos + 3] << 24) | (pUbxPacket->payload[pos + 2] << 16) | (pUbxPacket->payload[pos + 1] << 8) | pUbxPacket->payload[pos];
    }
    return 0;
}

int16_t UBloxUBX::GetUbxI2(uint16_t pos)
{
    ubxPacket *pUbxPacket = GetUbxPacket();
    if (pos < pUbxPacket->len - 1)
    {
        return (pUbxPacket->payload[pos + 1] << 8) | pUbxPacket->payload[pos];
    }
    return 0;
}

int32_t UBloxUBX::GetUbxI4(uint16_t pos)
{
    ubxPacket *pUbxPacket = GetUbxPacket();
    if (pos < pUbxPacket->len - 3)
    {
        return (pUbxPacket->payload[pos + 3] << 24) | (pUbxPacket->payload[pos + 2] << 16) | (pUbxPacket->payload[pos + 1] << 8) | pUbxPacket->payload[pos];
    }
    return 0;
}
