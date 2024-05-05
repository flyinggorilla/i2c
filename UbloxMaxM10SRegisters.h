#ifndef _UBLOX_MAX_M10S_REGISTERS_H_
#define _UBLOX_MAX_M10S_REGISTERS_H_

#include <esp_system.h>

// https://www.u-blox.com/sites/default/files/u-blox-M10-SPG-5.10_InterfaceDescription_UBX-21035062.pdf
// https://www.u-blox.com/sites/default/files/MAX-M10S_IntegrationManual_UBX-20053088.pdf


#define UBX_CLASS_NONE 0x00 // Not a valid Class.
#define UBX_CLASS_NAV 0x01	// Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
#define UBX_CLASS_ACK 0x05 // Ack/Nak Messages: Acknowledge or Reject messages to UBX-CFG input messages
#define UBX_CLASS_CFG 0x06 // Configuration Input Messages: Configure the receiver.
#define UBX_CLASS_MON 0x0A // Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
#define UBX_CFG_CFG 0x09   // Clear, Save, and Load Configurations. Used to save current configuration
#define UBX_CFG_RST 0x04   // Reset Receiver / Clear Backup Data Structures. Used to reset the receiver to the factory default settings
#define UBX_CFG_MSG 0x01   // Poll a message configuration, or Set Message Rate(s), or Set Message Rate
#define UBX_CFG_PRT 0x00   // Used to configure port specifics. Polls the configuration for one I/O Port, or Port configuration for UART ports, or Port configuration for USB port, or Port configuration for SPI port, or Port configuration for DDC port
#define UBX_CFG_RATE 0x08  // Navigation/Measurement Rate Settings. Used to set port baud rates.
#define UBX_NAV_POSLLH 0x02   // Geodetic Position Solution
#define UBX_NAV_PVT 0x07   // All the things! Position, velocity, time, PDOP, height, h/v accuracies, number of satellites. Navigation Position Velocity Time Solution.
#define UBX_NAV_VELNED 0x12   // Velocity Solution in NED frame

#define UBX_CFG_VALSET 0x8a
#define CFG_I2COUTPROT_UBX 0x10720001             // L
#define CFG_I2COUTPROT_NMEA 0x10720002            // L
#define CFG_MSGOUT_UBX_NAV_POSLLH_I2C 0x20910029  // U1 rate
#define CFG_MSGOUT_UBX_NAV_PVT_I2C 0x20910006     // U1 rate
#define CFG_MSGOUT_UBX_NAV_TIMEUTC_I2C 0x2091005b // U1
#define CFG_MSGOUT_UBX_NAV_VELNED_I2C 0x20910042  // U1
#define CFG_NAVSPG_DYNMODEL 0x20110021            // SEA 5, AUTOMOT 4
#define CFG_RATE_MEAS 0x30210001                  // U2 E.g. 100 ms results in 10 Hz measurement rate, 1000 ms = 1 Hz measurement rate. The minimum value is 25.
#define CFG_RATE_NAV 0x30210002                   // U2 E.g. 5 means five measurements for every navigation solution. The minimum value is 1. The maximum value is 127.
#define CFG_RATE_TIMEREF 0x20210003                  // E1 UTC=0
#define CFG_TXREADY_ENABLED 0x10a20001            // L Flag to indicate if TX ready pin mechanism should be enabled
#define CFG_TXREADY_POLARITY 0x10a20002           // L The polarity of the TX ready pin: false:high active, true:low-active
#define CFG_TXREADY_PIN 0x20a20003                // U1 EXTINT=5, Pin number to use for the TX ready functionality
#define CFG_TXREADY_THRESHOLD 0x30a20004          // U2 Amount of data that should be ready on the interface before triggering the TX ready pin
#define CFG_TXREADY_INTERFACE 0x20a20005          // E1 I2C=0, Interface where the TX ready feature should be linked to
#define CFG_HW_RF_LNA_MODE 0x20a30057             // E1, NORMAL 0 All RFs. Normal, LOWGAIN 1 All RFs. BYPASS 2 All RFs. Bypass LNA

#define UBX_MON_HW3 0x37

#define UBX_ACK_NACK 0x00
#define UBX_ACK_ACK 0x01
#define UBX_ACK_NONE 0x02 // Not a real value

#define COM_PORT_I2C 0
#define COM_PORT_UART1 1
#define COM_PORT_UART2 2
#define COM_PORT_USB 3
#define COM_PORT_SPI 4

#endif