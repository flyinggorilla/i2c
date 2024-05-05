#ifndef UBloxUBX_H_
#define UBloxUBX_H_

#include "UBloxDDC.h"
#include "UBloxNeoM8URegisters.h"

typedef struct
{
	uint8_t portID; // 0 = DDC port
	uint16_t txReady;
	uint32_t mode; // use LSB and value is shifted left by 1 bit (shift right one bit when reading 0x84 >> 1 = 0x42 I2C address)
	uint16_t inProtoMask;
    uint16_t outProtoMask;
    uint16_t flags;
} ubxGetSetCfgPrt;

typedef struct
{
	uint8_t portID; // 0 = DDC port
} ubxPollCfgPrt;

typedef struct
{
    uint32_t iTOW; // GPS time of week of the navigation epoch
    uint8_t version; // Message version (0x00 for this version)
    int32_t roll; // Roll angle
    int32_t pitch; // Pitch angle
    int32_t heading; // Heading angle
    uint32_t accRoll; // Roll angle accuracy
    uint32_t accPitch; // Pitch angle accuracy
    uint32_t accHeading; // Heading angle accuracy
} ubxPollNavAtt; // 0x01 0x05

//32.17.14 UBX-NAV-PVT (0x01 0x07)
//32.17.14.1 Navigation Position Velocity Time Solution
typedef struct 
{
    uint32_t iTOW; // GPS time of week of the navigation epoch
    uint16_t year; // Year (UTC)
    uint8_t month; // Month, range 1..12 (UTC)
    uint8_t day; // Day of month, range 1..31 (UTC)
    uint8_t hour; // Hour of day, range 0..23 (UTC)
    uint8_t min; // Minute of hour, range 0..59 (UTC)
    uint8_t sec; // Seconds of minute, range 0..60 (UTC)
    uint8_t valid; // Validity flags
    uint32_t tAcc; // Time accuracy estimate
    int32_t nano; // Fraction of second, range -1e9 .. 1e9 (UTC)
    uint8_t fixType; // GNSSfix Type, range 0..5
    uint8_t flags; // Fix Status Flags
    uint8_t flags2; // Additional flags
    uint8_t numSV; // Number of satellites used in Nav Solution
    int32_t lon; // Longitude
    int32_t lat; // Latitude
    int32_t height; // Height above ellipsoid
    int32_t hMSL; // Height above mean sea level
    uint32_t hAcc; // Horizontal accuracy estimate
    uint32_t vAcc; // Vertical accuracy estimate
    int32_t velN; // NED north velocity
    int32_t velE; // NED east velocity
    int32_t velD; // NED down velocity
    int32_t gSpeed; // Ground Speed (2-D)
    int32_t headMot; // Heading of motion
    uint32_t sAcc; // Speed accuracy estimate
    uint32_t headAcc; // Heading accuracy estimate
    uint16_t pDOP; // Position DOP
    int32_t headVeh; // Heading of vehicle (2-D)
    int16_t magDec; // Magnetic declination
    uint16_t magAcc; // Magnetic declination accuracy
} ubxPollNavPvt; // 0x01 0x07


class UBloxUBX : public UBloxDDC
{
public:
    UBloxUBX(I2c &rI2c);
    virtual ~UBloxUBX()
    {
        // Destructor implementation goes here
    }
    //bool Init();

    // configure GPS
    bool ConfigMessageRate(uint8_t cls, uint8_t id, uint8_t rate);
    bool ConfigPortOutput();

    // Poll GPS
    bool GetPortSettings(uint8_t portID, ubxGetSetCfgPrt *pGetSetCfgPrt = nullptr);
    bool GetAttitude(ubxPollNavAtt *pPollNavAtt = nullptr);

    // callback methods to process incoming UBX packets
    virtual void UbxCfgPrt(ubxGetSetCfgPrt *pGetSetCfgPrt) = 0;
    virtual void UbxNavAtt(ubxPollNavAtt *pPollNavAtt) = 0;
    virtual void UbxNavPvt(ubxPollNavPvt *pPollNavPvt) = 0;
    virtual void UbxAck(bool ack, uint8_t cls, uint8_t id) = 0; 


protected:
    bool PrepareUbxPacket(uint8_t cls, uint8_t id, uint16_t len);
    bool SetUbxU1(uint16_t pos, uint8_t value);
    bool SetUbxU2(uint16_t pos, uint16_t value);
    bool SetUbxU4(uint16_t pos, uint32_t value);
    bool SetUbxI2(uint16_t pos, int16_t value);
    bool SetUbxI4(uint16_t pos, int32_t value);
    bool SetUbxX1(uint16_t pos, uint8_t value) { return SetUbxU1(pos, value); };
    bool SendPreparedUbxPacket();

// create the GetUbx* counterparts to SetUbx* methods
    uint8_t GetUbxU1(uint16_t pos);
    uint16_t GetUbxU2(uint16_t pos);
    uint32_t GetUbxU4(uint16_t pos);
    int16_t GetUbxI2(uint16_t pos);
    int32_t GetUbxI4(uint16_t pos);
    uint8_t GetUbxX1(uint16_t pos) { return GetUbxU1(pos); };

    bool WaitForAck(uint8_t cls, uint8_t id, uint16_t timeout = 1000); // 1 second timeout

private:
    virtual void ProcessUbxPacket(ubxPacket *pUbxPacket);
    void ProcessUbxAckPacket(ubxPacket *pUbxPacket);
    void ProcessUbxCfgPacket(ubxPacket *pUbxPacket);
    void ProcessUbxNavPacket(ubxPacket *pUbxPacket);


};

#endif