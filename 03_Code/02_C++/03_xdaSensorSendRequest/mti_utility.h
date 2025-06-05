#ifndef MTI_UTILITY_H
#define MTI_UTILITY_H

typedef enum
{
    OPEN_PORT_FAILURE = 100,
    OPEN_PORT_SUCCESS,
    PORT_SET_ATTR_FAILURE,
    PORT_SET_ATTR_SUCCESS,
    PORT_GET_ATTR_FAILURE,
    PORT_GET_ATTR_SUCCESS,
    DEVICE_FOUND_FAILURE,
    DEVICE_FOUND_SUCCESS,
} mtiDecode_enum;

typedef struct
{
    uint8_t preamble;     // Should be 0xFA
    uint8_t bid;          // Bus ID
    uint8_t mid;          // Message ID
    uint8_t len;          // Standard length or 0xFF for extended
    uint16_t ext_len;     // Optional: if len == 0xFF
    uint8_t ind_id;       // Optional: for indication messages
    std::vector<uint8_t> data;     // Payload
    uint8_t checksum;     // Checksum at end of message
} XbusMessage;

typedef enum {
    // Section 8.1 - Control Commands
    WakeUp              = 0x3E, // Wake up the device
    GoToConfig          = 0x30, // Enter config mode
    GoToMeasurement     = 0x10, // Enter measurement mode
    Reset               = 0x40, // Reset the device

    // Section 8.2 - Informational Requests
    ReqDID              = 0x00, // Request device ID
    ReqProductCode      = 0x1C, // Request product code
    ReqHardwareVersion  = 0x1E, // Request hardware version
    ReqFWRev            = 0x12, // Request firmware revision

    // Section 8.3 - Device-specific Commands
    RestoreFactoryDef   = 0x0E, // Restore factory defaults
    ReqBaudrate         = 0x18, // Request baudrate
    SetBaudrate         = 0x18, // Set baudrate
    RunSelftest         = 0x24, // Start self-test
    ReqErrorMode        = 0xDA, // Request error mode
    SetErrorMode        = 0xDA, // Set error mode
    ReqOptionFlags      = 0x48, // Request option flags
    SetOptionFlags      = 0x48, // Set option flags
    ReqLocationID       = 0x84, // Request location ID
    SetLocationID       = 0x84, // Set location ID
    ReqTransmitDelay    = 0xDC, // Request RS485 transmit delay
    SetTransmitDelay    = 0xDC, // Set RS485 transmit delay
    ReqSyncSettings     = 0x2C, // Request synchronization settings
    SetSyncSettings     = 0x2C, // Set synchronization settings

    // Section 8.4 - Configuration
    ReqConfiguration           = 0x0C, // Request config
    ReqPeriod                  = 0x04, // Request sample period
    SetPeriod                  = 0x04, // Set sample period
    ReqExtOutputMode           = 0x86, // Request ext output mode
    SetExtOutputMode           = 0x86, // Set ext output mode
    ReqOutputConfiguration     = 0xC0, // Request output config
    SetOutputConfiguration     = 0xC0, // Set output config
    ReqStringOutputType        = 0x8E, // Request NMEA output types
    SetStringOutputType        = 0x8E, // Set NMEA output types
    ReqAlignmentRotation       = 0xEC, // Request alignment
    SetAlignmentRotation       = 0xEC, // Set alignment

    // Section 8.5 - Filter Profiles and GPS
    ReqAvailableFilterProfiles = 0x62, // Request available filter profiles
    ReqFilterProfile           = 0x64, // Request current filter profile
    SetFilterProfile           = 0x64, // Set filter profile
    ReqGnssPlatform            = 0x76, // Request GNSS platform
    SetGnssPlatform            = 0x76, // Set GNSS platform

    // Section 8.6 - Orientation & Motion
    ResetOrientation           = 0xA4, // Reset orientation
    SetNoRotation              = 0x22, // Start 'no rotation' procedure
    IccCommand                 = 0x74, // Run in-run compass calibration

    // Section 8.7 - Data request
    ReqData                    = 0x34, // Request MTData2
} MTiHostCommandMID_enum;

typedef enum {
    // Section 8.1 - Control Acknowledgements
    WakeUpAck             = 0x3F,
    GoToConfigAck         = 0x31,
    GoToMeasurementAck    = 0x11,
    ResetAck              = 0x41,

    // Section 8.2 - Informational Responses
    DeviceID              = 0x01,
    ProductCode           = 0x1D,
    HardwareVersion       = 0x1F,
    FirmwareRev           = 0x13,
    SelftestResults       = 0x25,
    Error                 = 0x42,

    // Section 8.3 - Device-specific Responses
    SetBaudrateAck        = 0x19,
    ReqBaudrateAck        = 0x19,
    ReqOptionFlagsAck     = 0x49,
    SetOptionFlagsAck     = 0x49,
    ReqLocationIDAck      = 0x85,
    SetLocationIDAck      = 0x85,
    ReqTransmitDelayAck   = 0xDD,
    SetTransmitDelayAck   = 0xDD,
    ReqSyncSettingsAck    = 0x2D,
    SetSyncSettingsAck    = 0x2D,

    // Section 8.4 - Configuration Acknowledgements
    ReqPeriodAck                 = 0x05,
    SetPeriodAck                 = 0x05,
    ExtOutputMode                = 0x87,
    ReqOutputConfigurationAck    = 0xC1,
    SetOutputConfigurationAck    = 0xC1,
    ReqStringOutputTypesAck      = 0x8F,
    SetStringOutputTypesAck      = 0x8F,
    ReqAlignmentRotationAck      = 0xED,
    SetAlignmentRotationAck      = 0xED,

    // Section 8.5 - Filter Profile and GNSS
    AvailableFilterProfiles      = 0x63,
    ReqFilterProfileAck          = 0x65,
    SetFilterProfileAck          = 0x65,
    ReqGnssPlatformAck           = 0x77,
    SetGnssPlatformAck           = 0x77,

    // Section 8.6 - Orientation & Motion
    ResetOrientationAck          = 0xA5,
    SetNoRotationAck             = 0x23,
    IccCommandAck                = 0x75,

    // Section 8.7 - Data Stream
    MTData2                      = 0x36 // Main live data stream
} MTiHostOutputMID_enum;


#endif // MTI_UTILITY_H
