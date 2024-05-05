//

#include "UBloxDDC.h"
#include "esp_log.h"
#include "esp_task.h"
#include "string.h"

// Register Map
#define UBloxDDC_DEVICE_ADDRESS_7BIT 0x42
#define UBloxDDC_DEVICE_FREQ_HZ 10000
#define UBloxDDC_DEVICE_REG_BYTESAVAIL 0xFD
#define UBloxDDC_DEVICE_REG_DATASTREAM 0xFF

#define LOG_LOCAL_LEVEL ESP_LOG_INFO

static const char *tag = "UBloxDDC";

//Registers
const uint8_t UBX_SYNCH_1 = 0xB5;
const uint8_t UBX_SYNCH_2 = 0x62;

//The following are UBX Class IDs. Descriptions taken from ZED-F9P Interface Description Document page 32, NEO-M8P Interface Description page 145
const uint8_t UBX_CLASS_NONE = 0x00; // Not a valid Class.
const uint8_t UBX_CLASS_ACK = 0x05;	 //Ack/Nak Messages: Acknowledge or Reject messages to UBX-CFG input messages



UBloxDDC::UBloxDDC(I2c &rI2c) : I2cDevice(rI2c)
{
}

UBloxDDC::~UBloxDDC()
{
  delete[] mpReadData;
  mpReadData = nullptr;
}

bool UBloxDDC::Init()
{
  if (!AddDevice(I2C_ADDR_BIT_LEN_7, UBloxDDC_DEVICE_ADDRESS_7BIT, UBloxDDC_DEVICE_FREQ_HZ, "UBloxDDC"))
    return false;
  if (!MasterProbe(UBloxDDC_DEVICE_ADDRESS_7BIT))
    return false;

  return true;
}

// Given a message, calc and store the two byte "8-Bit Fletcher" checksum over the entirety of the message
// This is called before we send a command message
void UBloxDDC::CalcChecksum(ubxPacket *msg)
{
    msg->checksumA = 0;
    msg->checksumB = 0;

    msg->checksumA += msg->cls;
    msg->checksumB += msg->checksumA;

    msg->checksumA += msg->id;
    msg->checksumB += msg->checksumA;

    msg->checksumA += (msg->len & 0xFF);
    msg->checksumB += msg->checksumA;

    msg->checksumA += (msg->len >> 8);
    msg->checksumB += msg->checksumA;

    for (uint16_t i = 0; i < msg->len; i++)
    {
        msg->checksumA += msg->payload[i];
        msg->checksumB += msg->checksumA;
    }
}


// Pretty prints the current ubxPacket
void UBloxDDC::LogUbxPacket(esp_log_level_t logLevel, bool inbound, bool logPayload)
{
  ubxPacket *ubxPacket = &mUbxPacket;

  char * inboundTxt = ">>OUT>>";
  if (inbound) {
    inboundTxt = "<<IN<<";
  }


  ESP_LOG_LEVEL_LOCAL(logLevel, tag, "%s CLS:0x%X ID:0x%X Len:%u", inboundTxt,
                      ubxPacket->cls, ubxPacket->id, ubxPacket->len);

  // Only print the payload is ignoreThisPayload is false otherwise
  // we could be printing gibberish from beyond the end of packetBuf
  if (logPayload || logLevel == ESP_LOG_VERBOSE)
  {
    ESP_LOG_BUFFER_HEXDUMP(tag, ubxPacket->payload, ubxPacket->len, logLevel);
  }
}

bool UBloxDDC::ReadDdcData()
{
  mReadDataPos = 0;
  mReadDataLen = 0;

  // size_t bytesRead = 0;
  uint8_t data[2];
  if (!ReadRegister(UBloxDDC_DEVICE_REG_BYTESAVAIL, (uint8_t *)data, sizeof(data)))
  {
    ESP_LOGE(tag, "Failed to read bytes available");
    return false;
  }

  uint16_t bytesAvailable = (data[0] << 8) | data[1];
  // log the number of bytes available as uint16 but also log the two bytes separately
  if (bytesAvailable == 0xFFFF)
  {
    ESP_LOGD(tag, "No data available");
    return true;
  }


  if (bytesAvailable > 0)
  {
    ESP_LOGV(tag, "Bytes available: %u, HI0x%02x, LO0x%02x", bytesAvailable, data[0], data[1]);

    delete[] mpReadData;
    mpReadData = new uint8_t[bytesAvailable + 1];
    mpReadData[bytesAvailable] = 0;
    if (!I2cDevice::Read(mpReadData, bytesAvailable))
    {
      delete[] mpReadData;
      mpReadData = nullptr;
      return false;
    }
    // read the data stream byte by byte
    mReadDataLen = bytesAvailable;
    // log using the ESP-IDF hex dump function
    // esp_log_buffer_hex(tag, pData, bytesAvailable);
    ESP_LOG_BUFFER_HEXDUMP(tag, mpReadData, bytesAvailable, ESP_LOG_VERBOSE);
    // ESP_LOGV(tag, "%s", mpReadData);
    // delete[] pData;
  }

  return true;
}

esp_err_t UBloxDDC::ReadUbxPacket()
{
    // ESP_LOGI(tag, "BEGIN ReadUbxPacket - Data in memory: %u, pos: %u", mReadDataLen, mReadDataPos);

    // is there enough data in memory to parse a packet?
    uint16_t len = mReadDataLen - mReadDataPos;
    if(len < 8)
    {
        // log amount of data in memory
        if (len > 0) {
          ESP_LOGW(tag, "ReadUbxPacket too much data in memory: %u, pos: %u", mReadDataLen, mReadDataPos);
        }
      
        if (!ReadDdcData())
        {
            ESP_LOGE(tag, "Failed to read DDC data");
            return ESP_FAIL;
        }
        // ESP_LOGI(tag, "AFTER ReadDdcData() in ReadUbxPacket - Data in memory: %u, pos: %u", mReadDataLen, mReadDataPos);

        len = mReadDataLen - mReadDataPos;
    }

    if (len < 8)
    {
        if (len > 0) ESP_LOGW(tag, "Not enough data in memory to parse a packet - len: %u, pos: %u", mReadDataLen, mReadDataPos);
        return ESP_ERR_NOT_FOUND;
    }


    if(!ParseUbxPacket()) {
        return ESP_FAIL;
    }    
    return ESP_OK;
}

bool UBloxDDC::ParseUbxPacket()
{
  // ensure data is available and there are at least the 6 header bytes + 2 checksum bytes in the buffer
  if (mReadDataLen == 0 || (mReadDataPos >= mReadDataLen + 8))
  {
    mReadDataLen = 0;
    mReadDataPos = 0;
    ESP_LOGE(tag, "No UBX data to parse - len: %u, pos: %u", mReadDataLen, mReadDataPos);
    return false;
  }

  // copy the data into the packet
  uint16_t startPos = mReadDataPos;
  while (mpReadData[mReadDataPos] != 0xB5 || mpReadData[mReadDataPos + 1] != 0x62)
  {
    if (mReadDataPos == startPos) {
      ESP_LOGW(tag, "No UBX header found at begin of packet, continuing search startPos:%u, DataPos:%u, DataLen:%u", startPos, mReadDataPos, mReadDataLen);
      ESP_LOG_BUFFER_HEXDUMP(tag, mpReadData, mReadDataLen, ESP_LOG_VERBOSE);
    }
    mReadDataPos++;
    // check whether there are enough bytes left in the buffer to find the header
    if ((mReadDataLen - mReadDataPos) < 2)
    {
      mReadDataLen = 0;
      mReadDataPos = 0;
      ESP_LOGE(tag, "No UBX header found!");
      return false;
    }
  }
  mReadDataPos += 2;

  uint8_t cls = mpReadData[mReadDataPos++];
  uint8_t id = mpReadData[mReadDataPos++];
  uint16_t len = mpReadData[mReadDataPos++]; 
  len |= mpReadData[mReadDataPos++] << 8;

  ubxPacket *packet = &mUbxPacket;
  uint16_t maxPayloadSize = MAX_PAYLOAD_SIZE;
  if (cls == UBX_CLASS_ACK)
  {
    packet = &mUbxPacketAck;
    maxPayloadSize = 2;
  }

  packet->cls = cls;
  packet->id = id;
  packet->len = len;

  // check if the packet is too long
  if (len > maxPayloadSize)
  {
    ESP_LOGE(tag, "Packet too long");
    packet->valid = SFE_UBLOX_PACKET_VALIDITY_NOT_VALID;
    mReadDataLen = 0;
    mReadDataPos = 0;
    return false;
  }
  memcpy(packet->payload, &mpReadData[mReadDataPos], packet->len);
  mReadDataPos += packet->len;

  packet->checksumA = mpReadData[mReadDataPos++];
  packet->checksumB = mpReadData[mReadDataPos++];
  packet->valid = SFE_UBLOX_PACKET_VALIDITY_VALID;

  // LogUbxPacket(ESP_LOG_VERBOSE, cls, true);

  ProcessUbxPacket(packet);
  return true;
}

bool UBloxDDC::SendUbxPacket(uint8_t cls, uint8_t id, uint8_t *payload, uint16_t len)
{
    ESP_LOGD(tag, "SendUbxPacket");
    ubxPacket *packet = &mUbxPacket;
    if (cls == UBX_CLASS_ACK)
    {
        packet = &mUbxPacketAck;
    }

    packet->cls = cls;
    packet->id = id;
    packet->len = len;


    uint16_t dataLen = 6 + mUbxPacket.len + 2; // Add 6 for header and 2 bytes for checksum
    uint8_t *data = new uint8_t[dataLen];
    data[0] = UBX_SYNCH_1;
    data[1] = UBX_SYNCH_2;
    data[2] = cls;
    data[3] = id;
    data[4] = len & 0xFF; // LSB
    data[5] = len >> 8;   // MSB
    memcpy(data + 6, payload, len);
    CalcChecksum(packet); // Sets checksum A and B bytes of the packet
    data[len + 6] = packet->checksumA;
    data[len + 7] = packet->checksumB;

    LogUbxPacket(ESP_LOG_VERBOSE, false, true);
    if (!WriteRegister(0xFF, data, dataLen))
    {
        delete[] data;
        ESP_LOGE(tag, "Failed to send UBX message");
        return false;
    }

    delete[] data;
    return true;
}


