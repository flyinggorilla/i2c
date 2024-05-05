#ifndef UBloxDDC_H_
#define UBloxDDC_H_

#include "I2c.h"
#include "esp_log.h"


// The Display Data Channel (DDC) bus is a two-wire communication interface compatible with the
//  I²C standard (Inter-Integrated Circuit). See our online product selector matrix for availability.
//  Unlike all other interfaces, the DDC is not able to communicate in full-duplex mode, i.e. TX and RX
//  are mutually exclusive. u-blox receivers act as a slave in the communication setup, therefore they
//  cannot initiate data transfers on their own. The host, which is always master, provides the data
//  clock (SCL), and the clock frequency is therefore not configurable on the slave.
//  The receiver's DDC address is set to 0x42 by default. This address can be changed by setting the
//  mode field in UBX-CFG-PRT for DDC accordingly.
//  As the receiver will be run in slave mode and the DDC physical layer lacks a handshake mechanism
//  to inform the master about data availability, a layer has been inserted between the physical layer
//  and the UBX and NMEA layer. The receiver DDC interface implements a simple streaming
//  interface that allows the constant polling of data, discarding everything that is not parse-able.
//  The receiver returns 0xFF if no data is available. The TX-ready feature can be used to inform the
// master about data availability and can be used as a trigger for data transmission.

//Set the max number of bytes set in a given I2C transaction
#define MAX_I2C_DATASIZE 32
#define MAX_PAYLOAD_SIZE 256

// ubxPacket validity
typedef enum
{
	SFE_UBLOX_PACKET_VALIDITY_NOT_VALID,
	SFE_UBLOX_PACKET_VALIDITY_VALID,
	SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED,
	SFE_UBLOX_PACKET_NOTACKNOWLEDGED // This indicates that we received a NACK
} ubxPacketValidity;


typedef struct
{
	uint8_t cls;
	uint8_t id;
	uint16_t len;		   //Length of the payload. Does not include cls, id, or checksum bytes
	// uint16_t counter;	   //Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
	// uint16_t startingSpot; //The counter value needed to go past before we begin recording into payload array
	uint8_t *payload;
	uint8_t checksumA; //Given to us from module. Checked against the rolling calculated A/B checksums.
	uint8_t checksumB;
	ubxPacketValidity valid;			 //Goes from NOT_DEFINED to VALID or NOT_VALID when checksum is checked
} ubxPacket;



class UBloxDDC : public I2cDevice
{
public:
    UBloxDDC(I2c &rI2c);
    virtual ~UBloxDDC();
    bool Init();

   


    /// @brief Reads next Ubx packet from internal buffer, 
    /// or when buffer is empty, reaches out via I2C to fetch data that is available to read.
    /// @param cls Pointer to store the class of the packet
    /// @param id Pointer to store the id of the packet
    /// @return ESP_OK if successful, ESP_FAIL if failed on an I2C or parsing error, ESP_ERR_NOT_FOUND if no data available
    esp_err_t ReadUbxPacket();         
    // uint8_t GetPacketUbxClass();
    // uint8_t GetPacketUbxId();

protected:
    ubxPacket *GetUbxPacket() { return &mUbxPacket; };

    bool SendUbxPacket(uint8_t cls, uint8_t id, uint8_t *payload, uint16_t len);

    void LogUbxPacket(esp_log_level_t logLevel, bool inbound, bool printPayload = false);

    /// @brief callback method to process incoming UBX packet - override in derived class
    /// @param pUbxPacket 
    virtual void ProcessUbxPacket(ubxPacket *pUbxPacket) = 0;

    
private:
    bool ReadDdcData();
    bool ParseUbxPacket();
    void CalcChecksum(ubxPacket *msg);

    //uint8_t mData[MAX_I2C_DATASIZE] = {0};
    uint16_t mReadDataLen = 0;
    uint16_t mReadDataPos = 0;
    uint8_t *mpReadData = nullptr;

    // The packet buffers
    // These are pointed at from within the ubxPacket
    uint8_t mUbxPayloadAck[2];                // Holds the requested ACK/NACK
    uint8_t mUbxPayloadCfg[MAX_PAYLOAD_SIZE]; // TODO REMOVE !!! Holds the requested data packet
    // uint8_t mUbxPayloadBuf[2];                // Temporary buffer used to screen incoming packets or dump unrequested packets

    // Init the packet structures and init them with pointers to the payloadAck, payloadCfg and payloadBuf arrays
    ubxPacket mUbxPacketAck = {0, 0, 0, mUbxPayloadAck, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
    ubxPacket mUbxPacket = {0, 0, 0, mUbxPayloadCfg, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED };


};

#endif