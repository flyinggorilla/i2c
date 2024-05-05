#ifndef MaxSonarMB7040_H_
#define MaxSonarMB7040_H_

#include "I2c.h"


class MaxSonarMB7040 : public I2cDevice
{
public:
    MaxSonarMB7040(I2c &rI2c);
    virtual ~MaxSonarMB7040()
    {
        // Destructor implementation goes here
    }
    bool Init();
    
    bool TriggerDistanceMeasurement();

    // Read the distance in cm approx 70ms after TriggerDistanceMeasurement
    bool ReadDistance(unsigned short *distanceCm);

private:
};

#endif