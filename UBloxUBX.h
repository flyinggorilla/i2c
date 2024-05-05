#ifndef UBloxUBX_H_
#define UBloxUBX_H_

#include "UBloxDDC.h"
#include "UBloxMaxM10SRegisters.h"



//32.17.14 UBX-NAV-PVT (0x01 0x07)
typedef struct
{
    uint32_t iTOW; // GPS time of week of the navigation epoch
    int32_t lon; // Longitude
    int32_t lat; // Latitude
    int32_t height; // Height above ellipsoid
    int32_t hMSL; // Height above mean sea level
    uint32_t hAcc; // Horizontal accuracy estimate
    uint32_t vAcc; // Vertical accuracy estimate
} ubxNavPosllh; // 0x01 0x02

// 3.15.26 UBX-NAV-VELNED (0x01 0x12)
typedef struct
{
    uint32_t iTOW; // GPS time of week of the navigation epoch
    int32_t velN; // NED north velocity
    int32_t velE; // NED east velocity
    int32_t velD; // NED down velocity
    uint32_t speed; // Speed (3-D)
    uint32_t gSpeed; // Ground Speed (2-D)
    int32_t heading; // Heading of motion
    uint32_t sAcc; // Speed accuracy estimate
    uint32_t cAcc; // Course / Heading accuracy estimate
} ubxNavVelned; // 0x01 0x12

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
} ubxNavPvt; // 0x01 0x07


class UBloxUBX : public UBloxDDC
{
public:
    UBloxUBX(I2c &rI2c);
    virtual ~UBloxUBX()
    {
        // Destructor implementation goes here
    }

    bool ResetReceiver();
    bool ConfigureInterfaces();
    bool ConfigureOutput();
    bool ConfigureAntenna();
    bool ConfigureTxReady();


    //bool Init();

    // configure GPS
    // bool ConfigMessageRate(uint8_t cls, uint8_t id, uint8_t rate);
    // bool ConfigPortOutput();

    // Poll GPS
    // bool GetPortSettings(uint8_t portID, ubxGetSetCfgPrt *pGetSetCfgPrt = nullptr);
    // bool GetAttitude(ubxPollNavAtt *pPollNavAtt = nullptr);

    // callback methods to process incoming UBX packets
    // virtual void UbxCfgPrt(ubxGetSetCfgPrt *pGetSetCfgPrt) = 0;
    virtual void UbxNavPosllh(ubxNavPosllh *pNavPosllh) = 0;
    virtual void UbxNavPvt(ubxNavPvt *pNavPvt) = 0;
    virtual void UbxNavVelned(ubxNavVelned *pNavVelned) = 0;
    virtual void UbxAck(bool ack, uint8_t cls, uint8_t id) = 0; 


protected:
    /// @brief Prepare a UBX packet for sending
    /// @param cls 
    /// @param id 
    /// @param len omit this parameter to automatically calculate the length
    /// @return 
    bool PrepareUbxPacket(uint8_t cls, uint8_t id);
    bool AddUbxU1(uint8_t value);
    bool AddUbxU2(uint16_t value);
    bool AddUbxU4(uint32_t value);
    bool AddUbxI2(int16_t value);
    bool AddUbxI4(int32_t value);
    bool AddUbxX1(uint8_t value) { return AddUbxU1(value); };
    bool SendPreparedUbxPacket();
// create the GetUbx* counterparts to SetUbx* methods
    uint8_t GetUbxU1(uint16_t pos);
    uint16_t GetUbxU2(uint16_t pos);
    uint32_t GetUbxU4(uint16_t pos);
    int16_t GetUbxI2(uint16_t pos);
    int32_t GetUbxI4(uint16_t pos);
    uint8_t GetUbxX1(uint16_t pos) { return GetUbxU1(pos); };

    bool WaitForAck(uint8_t cls, uint8_t id, uint16_t timeout = 1000); // 1 second timeout

    void LogUbxPacket(esp_log_level_t logLevel, bool inbound, bool logPayload);


private:
    virtual void ProcessUbxPacket(ubxPacket *pUbxPacket);
    void ProcessUbxAckPacket(ubxPacket *pUbxPacket);
    void ProcessUbxCfgPacket(ubxPacket *pUbxPacket);
    void ProcessUbxNavPacket(ubxPacket *pUbxPacket);
};

#endif