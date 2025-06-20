// xsens_mti710.cpp
#include "xsens_mti710.hpp"
#include <libudev.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#define DEBUG

#ifdef DEBUG
#include <iomanip>
#endif

XsensMti710*  XsensMti710::instance_ = nullptr;
std::string   XsensMti710::devNode_   = "";

XsensMti710::XsensMti710()  = default;
XsensMti710::~XsensMti710() = default;

mtiDecode_enum XsensMti710::findXsensDevice() {
    struct udev* udev = udev_new();
    if (!udev) return DEVICE_FOUND_FAILURE;

    struct udev_enumerate* enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_subsystem(enumerate, "tty");
    udev_enumerate_scan_devices(enumerate);

    struct udev_list_entry* devices = udev_enumerate_get_list_entry(enumerate);
    struct udev_list_entry* entry;

    udev_list_entry_foreach(entry, devices) {
        const char* path = udev_list_entry_get_name(entry);
        struct udev_device* dev = udev_device_new_from_syspath(udev, path);
        struct udev_device* parent = udev_device_get_parent_with_subsystem_devtype(dev, "usb", "usb_device");

        if (parent) {
            const char* vid = udev_device_get_sysattr_value(parent, "idVendor");
            const char* dev_node = udev_device_get_devnode(dev);
            if (vid && dev_node && std::string(vid) == XsensMti710::XSENS_VID) {
                XsensMti710::xsens_device_path = std::string(dev_node);
                std::cout << "✅ Xsens device found: " << XsensMti710::xsens_device_path << "\n";
                udev_device_unref(dev);
                udev_unref(udev);
                return DEVICE_FOUND_SUCCESS;
            }
        }
        udev_device_unref(dev);
    }

    udev_unref(udev);
    std::cerr << "No Xsens device found.\n";
    return DEVICE_FOUND_FAILURE;
}

mtiDecode_enum XsensMti710::openXsensPort() {
    XsensMti710::fd_ = open(XsensMti710::xsens_device_path.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
        std::perror("Failed to open port");
        return OPEN_PORT_FAILURE;
    }

    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) {
        std::perror("Failed to get port attributes");
        close(fd_);
        return PORT_GET_ATTR_FAILURE;
    }

    cfsetispeed(&tty, XsensMti710::BAUDRATE);
    cfsetospeed(&tty, XsensMti710::BAUDRATE);
    tty.c_cflag = CS8 | CLOCAL | CREAD;
    tty.c_iflag = IGNPAR;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 1;

    tcflush(fd_, TCIFLUSH);
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        std::perror("Failed to set port attributes");
        close(fd_);
        return PORT_SET_ATTR_FAILURE;
    }

    std::cout << "Serial port opened successfully.\n";
    return OPEN_PORT_SUCCESS;
}

void XsensMti710::configure() {
    instance_ = this;
    xsens_interface_t iface = XSENS_INTERFACE_RX_TX(&xsens_event_handler, &sendCb);

    xsens_mti_override_id_handler(MT_ACK_GOTOCONFIG,         cbGotoConfig);
    xsens_mti_override_id_handler(MT_ACK_OUTPUTCONFIGURATION,cbOutCfg);
    xsens_mti_override_id_handler(MT_ACK_OPTIONFLAGS,       cbFlags);
    xsens_mti_override_id_handler(MT_ACK_GOTOMEASUREMENT,   cbGotoMeas);

    // A) → Config mode
    ack_ = AckFlag::None;
    xsens_mti_request(&iface, MT_GOTOCONFIG);
    waitFor(AckFlag::GotoConfig);

    // B) → Output config
    static constexpr uint16_t R=100;
    XsensFrequencyConfig_t cfg[] = {
        { .id = XDI_PACKET_COUNTER,     .frequency = 0xFFFF },
        { .id = XDI_SAMPLE_TIME_FINE,   .frequency = 0xFFFF },
        { .id = XDI_DELTA_Q,            .frequency = R     },
        { .id = XDI_RATE_OF_TURN,       .frequency = R     },
        { .id = XDI_DELTA_V,            .frequency = R     },
        { .id = XDI_ACCELERATION,       .frequency = R     },
        { .id = XDI_FREE_ACCELERATION,  .frequency = R     },
        { .id = XDI_MAGNETIC_FIELD,     .frequency = R     },
        { .id = XDI_TEMPERATURE,        .frequency = R     },
        { .id = XDI_STATUS_BYTE,        .frequency = R     },
        { .id = XSENS_IDENTIFIER_FORMAT(
                  XDI_ALTITUDE_ELLIPSOID,
                  XSENS_FLOAT_FIXED1632,
                  XSENS_COORD_ENU),
          .frequency = R 
        },
    };
    ack_ = AckFlag::None;
    xsens_mti_set_configuration(&iface, cfg, sizeof(cfg)/sizeof(cfg[0]));
    waitFor(AckFlag::OutCfg);

    // C) → Option flags
    uint32_t s=0,c=0;
    XSENS_OPTION_FLAG_SET(s,XSENS_OPTFLAG_DISABLE_AUTOSTORE);
    XSENS_OPTION_FLAG_SET(s,XSENS_OPTFLAG_ENABLE_AHS);
    ack_ = AckFlag::None;
    xsens_mti_set_option_flags(&iface,s,c);
    waitFor(AckFlag::Flags);

    // D) → Measurement mode
    ack_ = AckFlag::None;
    xsens_mti_request(&iface, MT_GOTOMEASUREMENT);
    waitFor(AckFlag::GotoMeas);
}

void XsensMti710::streamDataLoop() {
    xsens_interface_t iface = XSENS_INTERFACE_RX(&xsens_event_handler);
    uint8_t buf[256];
    while (true) {
        auto n = ::read(fd_, buf, sizeof(buf));
        if (n<=0) break;
        xsens_mti_parse_buffer(&iface, buf, n);
        // now data_ is up-to-date
    }
}

void XsensMti710::waitFor(AckFlag want) {
    xsens_interface_t iface = XSENS_INTERFACE_RX_TX(&xsens_event_handler, &sendCb);
    uint8_t buf[256];
    while (ack_!=want) {
        auto n = ::read(fd_, buf, sizeof(buf));
        if (n>0) xsens_mti_parse_buffer(&iface, buf, n);
    }
}

// send callbacks & ACK handlers
void XsensMti710::sendCb   (uint8_t* d,uint16_t l){ instance_->sendToDevice(d,l); }
void XsensMti710::sendToDevice(uint8_t* d,uint16_t l){ ::write(fd_,d,l); }
void XsensMti710::cbGotoConfig  (xsens_packet_buffer_t*){ instance_->ack_=AckFlag::GotoConfig; }
void XsensMti710::cbOutCfg      (xsens_packet_buffer_t*){ instance_->ack_=AckFlag::OutCfg; }
void XsensMti710::cbFlags       (xsens_packet_buffer_t*){ instance_->ack_=AckFlag::Flags; }
void XsensMti710::cbGotoMeas    (xsens_packet_buffer_t*){ instance_->ack_=AckFlag::GotoMeas; }

void XsensMti710::xsens_event_handler(XsensEventFlag_t event, XsensEventData_t* mt2data) {
#ifdef DEBUG
    std::cout << std::fixed << std::setprecision(3);
#endif
    switch (event) {
        case XSENS_EVT_EULER:
            if (mt2data->type == XSENS_EVT_TYPE_FLOAT3) {
                xsensData.euler[0] = mt2data->data.f4x3[0];
                xsensData.euler[1] = mt2data->data.f4x3[1];
                xsensData.euler[2] = mt2data->data.f4x3[2];
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_EULER] Roll=" << xsensData.euler[0]
                          << ", Pitch=" << xsensData.euler[1]
                          << ", Yaw=" << xsensData.euler[2]
                          << " (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            } 
            else {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_EULER\n";
                return;
            }
            break;
        case XSENS_EVT_QUATERNION:
            if (mt2data->type == XSENS_EVT_TYPE_FLOAT4) {
                xsensData.quaternion[0] = mt2data->data.f4x4[0];
                xsensData.quaternion[1] = mt2data->data.f4x4[1];
                xsensData.quaternion[2] = mt2data->data.f4x4[2];
                xsensData.quaternion[3] = mt2data->data.f4x4[3];
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_QUATERNION] [" 
                          << xsensData.quaternion[0] << ", "
                          << xsensData.quaternion[1] << ", "
                          << xsensData.quaternion[2] << ", "
                          << xsensData.quaternion[3]
                          << "] (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            } 
            else {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_QUATERNION\n";
                return;
            }
            break;
        case XSENS_EVT_DELTA_V:
            if (mt2data->type == XSENS_EVT_TYPE_FLOAT3) {
                xsensData.delta_v[0] = mt2data->data.f4x3[0];
                xsensData.delta_v[1] = mt2data->data.f4x3[1];
                xsensData.delta_v[2] = mt2data->data.f4x3[2];
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_DELTA_V] X=" << xsensData.delta_v[0]
                          << ", Y=" << xsensData.delta_v[1]
                          << ", Z=" << xsensData.delta_v[2]
                          << " (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            } 
            else {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_DELTA_V\n";
                return;
            }
            break;
        case XSENS_EVT_DELTA_Q:
            if (mt2data->type == XSENS_EVT_TYPE_FLOAT4) {
                xsensData.delta_q[0] = mt2data->data.f4x4[0];
                xsensData.delta_q[1] = mt2data->data.f4x4[1];
                xsensData.delta_q[2] = mt2data->data.f4x4[2];
                xsensData.delta_q[3] = mt2data->data.f4x4[3];
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_DELTA_Q] [" 
                          << xsensData.delta_q[0] << ", "
                          << xsensData.delta_q[1] << ", "
                          << xsensData.delta_q[2] << ", "
                          << xsensData.delta_q[3]
                          << "] (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            } 
            else {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_DELTA_Q\n";
                return;
            }
            break;
        case XSENS_EVT_ACCELERATION:
            if (mt2data->type == XSENS_EVT_TYPE_FLOAT3) {
                xsensData.acceleration[0] = mt2data->data.f4x3[0];
                xsensData.acceleration[1] = mt2data->data.f4x3[1];
                xsensData.acceleration[2] = mt2data->data.f4x3[2];
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_ACCELERATION] X=" << xsensData.acceleration[0]
                          << ", Y=" << xsensData.acceleration[1]
                          << ", Z=" << xsensData.acceleration[2]
                          << " (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            } 
            else {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_ACCELERATION\n";
                return;
            }
            break;
        case XSENS_EVT_FREE_ACCELERATION:
            if (mt2data->type == XSENS_EVT_TYPE_FLOAT3) {
                xsensData.free_acceleration[0] = mt2data->data.f4x3[0];
                xsensData.free_acceleration[1] = mt2data->data.f4x3[1];
                xsensData.free_acceleration[2] = mt2data->data.f4x3[2];
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_FREE_ACCELERATION] X=" << xsensData.free_acceleration[0]
                          << ", Y=" << xsensData.free_acceleration[1]
                          << ", Z=" << xsensData.free_acceleration[2]
                          << " (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            } 
            else {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_FREE_ACCELERATION\n";
                return;
            }
            break;
        case XSENS_EVT_RATE_OF_TURN:
            if (mt2data->type == XSENS_EVT_TYPE_FLOAT3) {
                xsensData.rate_of_turn[0] = mt2data->data.f4x3[0];
                xsensData.rate_of_turn[1] = mt2data->data.f4x3[1];
                xsensData.rate_of_turn[2] = mt2data->data.f4x3[2];
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_RATE_OF_TURN] X=" << xsensData.rate_of_turn[0]
                          << ", Y=" << xsensData.rate_of_turn[1]
                          << ", Z=" << xsensData.rate_of_turn[2]
                          << " (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            } 
            else {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_RATE_OF_TURN\n";
                return;
            }
            break;
        case XSENS_EVT_MAGNETIC:
            if (mt2data->type == XSENS_EVT_TYPE_FLOAT3) {
                xsensData.magnetic[0] = mt2data->data.f4x3[0];
                xsensData.magnetic[1] = mt2data->data.f4x3[1];
                xsensData.magnetic[2] = mt2data->data.f4x3[2];
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_MAGNETIC] X=" << xsensData.magnetic[0]
                          << ", Y=" << xsensData.magnetic[1]
                          << ", Z=" << xsensData.magnetic[2]
                          << " (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            } 
            else {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_MAGNETIC\n";
                return;
            }
            break;
        case XSENS_EVT_LAT_LON:
            if (mt2data->type == XSENS_EVT_TYPE_FLOAT2) {
                xsensData.latitude  = mt2data->data.f8x2[0];
                xsensData.longitude = mt2data->data.f8x2[1];
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_LAT_LON] Lat=" << xsensData.latitude
                          << ", Lon=" << xsensData.longitude
                          << " (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            } 
            else {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_LAT_LON\n";
                return;
            }
            break;
        case XSENS_EVT_ALTITUDE_ELLIPSOID:
            if (mt2data->type == XSENS_EVT_TYPE_DOUBLE) {
                xsensData.altitude = mt2data->data.f8;
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_ALTITUDE_ELLIPSOID] Altitude=" << xsensData.altitude
                          << " (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            } 
            else {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_ALTITUDE_ELLIPSOID\n";
                return;
            }
            break;
        case XSENS_EVT_VELOCITY_XYZ:
            if (mt2data->type == XSENS_EVT_TYPE_FLOAT3) {
                xsensData.velocity[0] = mt2data->data.f4x3[0];
                xsensData.velocity[1] = mt2data->data.f4x3[1];
                xsensData.velocity[2] = mt2data->data.f4x3[2];
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_VELOCITY_XYZ] X=" << xsensData.velocity[0]
                          << ", Y=" << xsensData.velocity[1]
                          << ", Z=" << xsensData.velocity[2]
                          << " (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            } 
            else {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_VELOCITY_XYZ\n";
                return;
            }
            break;
        case XSENS_EVT_STATUS_BYTE:
            if (mt2data->type == XSENS_EVT_TYPE_U8) {
                xsensData.status_byte = mt2data->data.u1;
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_STATUS_BYTE] Status Byte=0x"
                          << std::hex << static_cast<int>(xsensData.status_byte) << std::dec
                          << " (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            } 
            else {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_STATUS_BYTE\n";
                return;
            }
            break;
        case XSENS_EVT_STATUS_WORD:
            if (mt2data->type == XSENS_EVT_TYPE_U32) {
                xsensData.status_word = mt2data->data.u4;
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_STATUS_WORD] Word=0x"
                          << std::hex << xsensData.status_word << std::dec
                          << " (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            } 
            else {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_STATUS_WORD\n";
                return;
            }
            break;
        case XSENS_EVT_PRESSURE:
            if (mt2data->type == XSENS_EVT_TYPE_FLOAT) {
                xsensData.baro_pressure = mt2data->data.f4;
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_PRESSURE] Pressure=" << xsensData.baro_pressure
                          << " (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            } 
            else {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_PRESSURE\n";
                return;
            }
            break;
        case XSENS_EVT_TEMPERATURE:
            if (mt2data->type == XSENS_EVT_TYPE_FLOAT) {
                xsensData.temperature = mt2data->data.f4;
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_TEMPERATURE] Temperature=" << xsensData.temperature
                          << " (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            } 
            else {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_TEMPERATURE\n";
                return;
            }
            break;
        case XSENS_EVT_PACKET_COUNT:
            if (mt2data->type == XSENS_EVT_TYPE_U16) {
                xsensData.packet_counter = mt2data->data.u2;
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_PACKET_COUNT] Count=" << xsensData.packet_counter
                          << " (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            } 
            else {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_PACKET_COUNT\n";
                return;
            }
            break;
        case XSENS_EVT_TIME_FINE:
            if (mt2data->type == XSENS_EVT_TYPE_U32) {
                xsensData.time_fine = mt2data->data.u4;
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_TIME_FINE] Time Fine=" << xsensData.time_fine
                          << " (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            } 
            else {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_TIME_FINE\n";
                return;
            }
            break;
        case XSENS_EVT_TIME_COARSE:
            if (mt2data->type == XSENS_EVT_TYPE_U32) 
            {
                xsensData.time_coarse = mt2data->data.u4;
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_TIME_COARSE] Time Coarse=" << xsensData.time_coarse
                          << " (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            } 
            else {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_TIME_COARSE\n";
                return;
            }
            break;
        case XSENS_EVT_UTC_TIME:
            if (mt2data->type == XSENS_EVT_TYPE_DOUBLE2) 
            {
                xsensData.utc_time = mt2data->data.f8x2[0] + mt2data->data.f8x2[1];
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_UTC_TIME] UTC=" << xsensData.utc_time
                          << " (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            } 
            else if (mt2data->type == XSENS_EVT_TYPE_DOUBLE) {
                xsensData.utc_time = mt2data->data.f8;
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_UTC_TIME] UTC=" << xsensData.utc_time
                          << " (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            } 
            else {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_UTC_TIME\n";
                return;
            }
            break;
        default:
            std::cerr << "[ERROR] Unrecognized event: " << static_cast<int>(event) << "\n";
            return;
    }
}