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

#define LOG_LOCAL_LEVEL ESP_LOG_INFO

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
    LogUbxPacket(ESP_LOG_DEBUG, false, true);
    bool ret = SendUbxPacket(pUbxPacket->cls, pUbxPacket->id, pUbxPacket->payload, pUbxPacket->len);
    pUbxPacket->len = 0;
    pUbxPacket->cls = UBX_CLASS_NONE;
    pUbxPacket->id = 0;
    return ret;
}

#define CONFIG_VALUE_UBX_LAYERS 0x03 // bit0=RAM, bit1=BBR, bit2=FLASH

bool UBloxUBX::ResetReceiver()
{
    PrepareUbxPacket(UBX_CLASS_CFG, UBX_CFG_RST);

    AddUbxU2(0xFFFF); // BBR sections to clear. 0x0000 Hot start • 0x0001 Warm start • 0xFFFF Cold start
    AddUbxU1(0x0); // Reset Type • 0x00 = Hardware reset (watchdog) immediately • 0x01 = Controlled software reset • 0x02 = Controlled software reset (GNSS only) • 0x04 = Hardware reset (watchdog) after shutdown • 0x08 = Controlled GNSS stop • 0x09 = Controlled GNSS start 
    AddUbxU1(0); //reserved
    SendPreparedUbxPacket();
    // No Ack to be expected!! 
    vTaskDelay(100 / portTICK_PERIOD_MS);
    return true;
}

bool UBloxUBX::ConfigureInterfaces()
{
    PrepareUbxPacket(UBX_CLASS_CFG, UBX_CFG_VALSET);

    // header
    AddUbxU1(0);   // version 0
    AddUbxU1(CONFIG_VALUE_UBX_LAYERS); // bit0=RAM, bit1=BBR, bit2=FLASH
    AddUbxU2(0);   // reserved

    // key value pairs
    AddUbxU4(CFG_I2COUTPROT_NMEA);
    AddUbxU1(0);
    AddUbxU4(CFG_I2COUTPROT_UBX);
    AddUbxU1(1);
    SendPreparedUbxPacket();
    WaitForAck(UBX_CLASS_CFG, UBX_CFG_VALSET);
    return true;
}

bool UBloxUBX::ConfigureOutput()
{
    PrepareUbxPacket(UBX_CLASS_CFG, UBX_CFG_VALSET);

    // header
    AddUbxU1(0);   // version 0
    AddUbxU1(CONFIG_VALUE_UBX_LAYERS); // bit0=RAM, bit1=BBR, bit2=FLASH
    AddUbxU2(0); // reserved

    // key value pairs
    AddUbxU4(CFG_MSGOUT_UBX_NAV_POSLLH_I2C);
    AddUbxU1(1);
    AddUbxU4(CFG_MSGOUT_UBX_NAV_PVT_I2C);
    AddUbxU1(20);
    AddUbxU4(CFG_MSGOUT_UBX_NAV_TIMEUTC_I2C);
    AddUbxU1(20);
    AddUbxU4(CFG_MSGOUT_UBX_NAV_VELNED_I2C);
    AddUbxU1(2);
    AddUbxU4(CFG_NAVSPG_DYNMODEL);
    AddUbxU1(5); // SEA
    AddUbxU4(CFG_RATE_MEAS);
    AddUbxU2(100); // 100ms = 10 Hz
    AddUbxU4(CFG_RATE_NAV);
    AddUbxU2(10); // 1 Hz (10 measurements at 100ms each)
    AddUbxU4(CFG_RATE_TIMEREF);
    AddUbxU1(0); // UTC
    // AddUbxU4(CFG_TXREADY_ENABLED);
    // AddUbxU1(1); // enabled
    // AddUbxU4(CFG_TXREADY_POLARITY);
    // AddUbxU1(0); // high active
    // AddUbxU4(CFG_TXREADY_PIN);
    // AddUbxU1(5); // EXTINT
    // AddUbxU4(CFG_TXREADY_THRESHOLD);
    // AddUbxU2(2); // # 8byte chunks
    // AddUbxU4(CFG_TXREADY_INTERFACE);
    // AddUbxU1(0); // I2C

    SendPreparedUbxPacket();
    WaitForAck(UBX_CLASS_CFG, UBX_CFG_VALSET);
    return true;
}




bool UBloxUBX::ConfigureAntenna()
{
    PrepareUbxPacket(UBX_CLASS_CFG, UBX_CFG_VALSET);
    // header
    AddUbxU1(0);   // version 0
    AddUbxU1(CONFIG_VALUE_UBX_LAYERS); // bit0=RAM, bit1=BBR, bit2=FLASH
    AddUbxU2(0); // reserved
    // key value pairs
    AddUbxU4(CFG_HW_RF_LNA_MODE);
    AddUbxU1(1); // 1 = low-gain (default), 2 = bypass
    SendPreparedUbxPacket();
    WaitForAck(UBX_CLASS_CFG, UBX_CFG_VALSET);
    return true;
}

bool UBloxUBX::ConfigureTxReady()
{
    PrepareUbxPacket(UBX_CLASS_CFG, UBX_CFG_VALSET);

    // header
    AddUbxU1(0);   // version 0
    AddUbxU1(CONFIG_VALUE_UBX_LAYERS); // bit0=RAM, bit1=BBR, bit2=FLASH
    AddUbxU2(0);   // 2 bytes reserved

    // key value pairs
    AddUbxU4(CFG_TXREADY_ENABLED);
    AddUbxU1(1); // enabled
    AddUbxU4(CFG_TXREADY_POLARITY);
    AddUbxU1(0); // high active
    AddUbxU4(CFG_TXREADY_PIN);
    AddUbxU1(4); // timepulse
    AddUbxU4(CFG_TXREADY_THRESHOLD);
    AddUbxU2(2); // # 8byte chunks
    AddUbxU4(CFG_TXREADY_INTERFACE);
    AddUbxU1(0); // I2C

    SendPreparedUbxPacket();
    WaitForAck(UBX_CLASS_CFG, UBX_CFG_VALSET);
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
    // case UBX_CLASS_CFG:
    //     ProcessUbxCfgPacket(pUbxPacket);
    //     // ESP_LOGV(tag, "UBX_CLASS_CFG");
    //     break;
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

// void UBloxUBX::ProcessUbxCfgPacket(ubxPacket *pUbxPacket)
// {
//     switch (pUbxPacket->id)
//     {
//     case UBX_CFG_PRT:
//     {
//         ubxGetSetCfgPrt getSetCfgPrt = {};
//         if (pUbxPacket->len == 20 &&
//             pUbxPacket->cls == UBX_CLASS_CFG && pUbxPacket->id == UBX_CFG_PRT)
//         {
//             getSetCfgPrt.portID = GetUbxU1(0);
//             getSetCfgPrt.txReady = GetUbxU2(2);
//             getSetCfgPrt.mode = GetUbxU4(4);
//             getSetCfgPrt.inProtoMask = GetUbxU2(12);
//             getSetCfgPrt.outProtoMask = GetUbxU2(14);
//             getSetCfgPrt.flags = GetUbxU2(16);
//             UbxCfgPrt(&getSetCfgPrt);
//         }
//         ESP_LOGI(tag, "UBX_CFG_PRT");
//     }
//     }
// }

void UBloxUBX::ProcessUbxNavPacket(ubxPacket *pUbxPacket)
{
    // log cls and id
    // ESP_LOGI(tag, "ProcessUbxNavPacket cls: %d, id: %d", pUbxPacket->cls, pUbxPacket->id);
    switch (pUbxPacket->id)
    {
    case UBX_NAV_PVT:
    {
        ubxNavPvt navPvt = {};
        if (pUbxPacket->len == 92 &&
            pUbxPacket->cls == UBX_CLASS_NAV && pUbxPacket->id == UBX_NAV_PVT)
        {
            navPvt.iTOW = GetUbxU4(0);
            navPvt.year = GetUbxU2(4);
            navPvt.month = GetUbxU1(6);
            navPvt.day = GetUbxU1(7);
            navPvt.hour = GetUbxU1(8);
            navPvt.min = GetUbxU1(9);
            navPvt.sec = GetUbxU1(10);
            navPvt.tAcc = GetUbxU4(12);
            navPvt.nano = GetUbxI4(16);
            navPvt.fixType = GetUbxU1(20);
            navPvt.flags = GetUbxX1(21);
            navPvt.flags2 = GetUbxX1(22);
            navPvt.numSV = GetUbxU1(23);
            navPvt.lon = GetUbxI4(24);
            navPvt.lat = GetUbxI4(28);
            navPvt.height = GetUbxI4(32);
            navPvt.hMSL = GetUbxI4(36);
            navPvt.hAcc = GetUbxU4(40);
            navPvt.vAcc = GetUbxU4(44);
            navPvt.velN = GetUbxI4(48);
            navPvt.velE = GetUbxI4(52);
            navPvt.velD = GetUbxI4(56);
            navPvt.gSpeed = GetUbxI4(60);
            navPvt.headMot = GetUbxI4(64);
            navPvt.sAcc = GetUbxU4(68);
            navPvt.headAcc = GetUbxU4(72);
            navPvt.pDOP = GetUbxU2(76);
            navPvt.headVeh = GetUbxI4(84);
            navPvt.magDec = GetUbxI2(88);
            navPvt.magAcc = GetUbxU2(90);
            UbxNavPvt(&navPvt);
        }
        else
        {
            ESP_LOGE(tag, "Error reading UBX_NAV_PVT, len: %d, cls: %d, id: %d", pUbxPacket->len, pUbxPacket->cls, pUbxPacket->id);
        }
        break;
    }
    case UBX_NAV_POSLLH:
    {
        ubxNavPosllh navPosllh = {};
        if (pUbxPacket->len == 28 &&
            pUbxPacket->cls == UBX_CLASS_NAV && pUbxPacket->id == UBX_NAV_POSLLH)
        {
            navPosllh.iTOW = GetUbxU4(0);
            navPosllh.lon = GetUbxI4(4);
            navPosllh.lat = GetUbxI4(8);
            navPosllh.height = GetUbxI4(12);
            navPosllh.hMSL = GetUbxI4(16);
            navPosllh.hAcc = GetUbxU4(20);
            navPosllh.vAcc = GetUbxU4(24);
            UbxNavPosllh(&navPosllh);
        }
        else
        {
            ESP_LOGE(tag, "Error reading UBX_NAV_POSLLH, len: %d, cls: %d, id: %d", pUbxPacket->len, pUbxPacket->cls, pUbxPacket->id);
        }
        break;
    }
    case UBX_NAV_VELNED:
    {
        ubxNavVelned navVelned = {};
        if (pUbxPacket->len == 36 &&
            pUbxPacket->cls == UBX_CLASS_NAV && pUbxPacket->id == UBX_NAV_VELNED)
        {
            navVelned.iTOW = GetUbxU4(0);
            navVelned.velN = GetUbxI4(4);
            navVelned.velE = GetUbxI4(8);
            navVelned.velD = GetUbxI4(12);
            navVelned.speed = GetUbxU4(16);
            navVelned.gSpeed = GetUbxU4(20);
            navVelned.heading = GetUbxI4(24);
            navVelned.sAcc = GetUbxU4(28);
            navVelned.cAcc = GetUbxU4(32);
            UbxNavVelned(&navVelned);
        }
        else
        {
            ESP_LOGE(tag, "Error reading UBX_NAV_VELNED, len: %d, cls: %d, id: %d", pUbxPacket->len, pUbxPacket->cls, pUbxPacket->id);
        }
    }
    }
}

bool UBloxUBX::WaitForAck(uint8_t cls, uint8_t id, uint16_t timeout)
{
    uint32_t startTime = esp_timer_get_time();
    while (esp_timer_get_time() - startTime < timeout)
    {
        esp_err_t ret = ReadUbxPacket();
        if (ret == ESP_OK)
        {
            ubxPacket *pUbxPacket = GetUbxPacket();
            LogUbxPacket(ESP_LOG_DEBUG, true, true);
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
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
    return false;
}

/// @brief resets buffer to zeroes; use AddUbxPacket* calls to populate buffer at desired positions
/// @param cls
/// @param id
/// @param len
/// @return
bool UBloxUBX::PrepareUbxPacket(uint8_t cls, uint8_t id)
{
    ubxPacket *pUbxPacket = GetUbxPacket();
    pUbxPacket->cls = cls;
    pUbxPacket->id = id;
    pUbxPacket->len = 0;
    memset(pUbxPacket->payload, 0, MAX_PAYLOAD_SIZE);
    return true;
}
bool UBloxUBX::AddUbxU1(uint8_t value)
{
    ubxPacket *pUbxPacket = GetUbxPacket();
    if (pUbxPacket->len < MAX_PAYLOAD_SIZE)
    {
        pUbxPacket->payload[pUbxPacket->len++] = value;
        return true;
    }
    return false;
}

bool UBloxUBX::AddUbxU2(uint16_t value)
{
    ubxPacket *pUbxPacket = GetUbxPacket();
    if (pUbxPacket->len < MAX_PAYLOAD_SIZE - 1)
    {
        pUbxPacket->payload[pUbxPacket->len++] = value & 0xFF;
        pUbxPacket->payload[pUbxPacket->len++] = value >> 8;
        return true;
    }
    return false;
}

bool UBloxUBX::AddUbxU4(uint32_t value)
{
    ubxPacket *pUbxPacket = GetUbxPacket();
    if (pUbxPacket->len < MAX_PAYLOAD_SIZE - 3)
    {
        pUbxPacket->payload[pUbxPacket->len++] = value & 0xFF;
        pUbxPacket->payload[pUbxPacket->len++] = (value >> 8) & 0xFF;
        pUbxPacket->payload[pUbxPacket->len++] = (value >> 16) & 0xFF;
        pUbxPacket->payload[pUbxPacket->len++] = (value >> 24) & 0xFF;
        return true;
    }
    return false;
}

bool UBloxUBX::AddUbxI2(int16_t value)
{
    ubxPacket *pUbxPacket = GetUbxPacket();
    if (pUbxPacket->len < MAX_PAYLOAD_SIZE - 1)
    {
        pUbxPacket->payload[pUbxPacket->len++] = value & 0xFF;
        pUbxPacket->payload[pUbxPacket->len++] = value >> 8;
        return true;
    }
    return false;
}

bool UBloxUBX::AddUbxI4(int32_t value)
{
    ubxPacket *pUbxPacket = GetUbxPacket();
    if (pUbxPacket->len < MAX_PAYLOAD_SIZE - 3)
    {
        pUbxPacket->payload[pUbxPacket->len++] = value & 0xFF;
        pUbxPacket->payload[pUbxPacket->len++] = (value >> 8) & 0xFF;
        pUbxPacket->payload[pUbxPacket->len++] = (value >> 16) & 0xFF;
        pUbxPacket->payload[pUbxPacket->len++] = (value >> 24) & 0xFF;
        return true;
    }
    return false;
}

// bool UBloxUBX::ConfigMessageRate(uint8_t cls, uint8_t id, uint8_t rate)
// {
//     ESP_LOGD(tag, "ConfigMessageRate %d", rate);
//     PrepareUbxPacket(UBX_CLASS_CFG, UBX_CFG_MSG, 3);
//     AddUbxU1(0, cls);
//     AddUbxU1(1, id);
//     AddUbxU1(2, rate);
//     SendPreparedUbxPacket();
//     WaitForAck(UBX_CLASS_CFG, UBX_CFG_MSG);
//     return true;
// }

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

// Pretty prints the current ubxPacket
void UBloxUBX::LogUbxPacket(esp_log_level_t logLevel, bool inbound, bool logPayload)
{
    ubxPacket *ubxPacket = GetUbxPacket();
    // if (cls == UBX_CLASS_ACK)
    // {
    //   ubxPacket = &mUbxPacketAck;
    // }

    const char *inboundTxt = ">>OUT>>";
    if (inbound)
    {
        inboundTxt = "<<IN<<";
    }

    const char *clsTxt = nullptr;
    if (ubxPacket->cls == UBX_CLASS_NAV) // 1
        clsTxt = "NAV";
    else if (ubxPacket->cls == UBX_CLASS_ACK) // 5
        clsTxt = "ACK";
    else if (ubxPacket->cls == UBX_CLASS_CFG) // 6
        clsTxt = "CFG";
    else if (ubxPacket->cls == UBX_CLASS_MON) // 0x0A
        clsTxt = "MON";
    else
    {
        clsTxt = "UNKNOWN";
    }

    const char *idTxt = nullptr;
    if (ubxPacket->cls == UBX_CLASS_NAV && ubxPacket->id == UBX_NAV_PVT)
        idTxt = "PVT";
    else if (ubxPacket->cls == UBX_CLASS_CFG && ubxPacket->id == UBX_CFG_RATE)
        idTxt = "RATE";
    else if (ubxPacket->cls == UBX_CLASS_CFG && ubxPacket->id == UBX_CFG_CFG)
        idTxt = "SAVE";
    else if (ubxPacket->cls == UBX_CLASS_CFG && ubxPacket->id == UBX_CFG_MSG)
        idTxt = "MSG";
    else if (ubxPacket->cls == UBX_CLASS_CFG && ubxPacket->id == UBX_CFG_PRT)
    {
        if (ubxPacket->len < 2)
            idTxt = "POLL CFG-PRT";
        else
            idTxt = "GET/SET CFG-PRT";
    }
    else if (ubxPacket->cls == UBX_CLASS_ACK)
    {
        if (ubxPacket->id == UBX_ACK_NACK)
        {
            idTxt = "NACK";
            logLevel = ESP_LOG_WARN;
        }
        else if (ubxPacket->id == UBX_ACK_ACK)
            idTxt = "ACK";
        else
            idTxt = "UNKNOWN ACK";
    }
    else
    {
        idTxt = "UNKNOWN ID";
    }
    ESP_LOG_LEVEL_LOCAL(logLevel, tag, "%s %s_%s CLS:0x%X ID:0x%X Len:%u", inboundTxt,
                        clsTxt, idTxt, ubxPacket->cls, ubxPacket->id, ubxPacket->len);

    // Only print the payload is ignoreThisPayload is false otherwise
    // we could be printing gibberish from beyond the end of packetBuf
    if (logPayload || logLevel == ESP_LOG_VERBOSE)
    {
        ESP_LOG_BUFFER_HEXDUMP(tag, ubxPacket->payload, ubxPacket->len, logLevel);
    }
}
