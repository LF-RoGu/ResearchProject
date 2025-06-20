// xsens_mti710.cpp
#include "xsens_mti710.hpp"
#include <libudev.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>

XsensMti710*  XsensMti710::instance_ = nullptr;
std::string   XsensMti710::devNode_   = "";

XsensMti710::XsensMti710()  = default;
XsensMti710::~XsensMti710() = default;

mtiDecode_enum XsensMti710::findXsensDevice() {
    struct udev* u = udev_new();
    if (!u) return DEVICE_FOUND_FAILURE;

    auto e = udev_enumerate_new(u);
    udev_enumerate_add_match_subsystem(e, "tty");
    udev_enumerate_scan_devices(e);

    udev_list_entry* devs = udev_enumerate_get_list_entry(e);
    udev_list_entry* ent;
    udev_list_entry_foreach(ent, devs) {
        const char* path = udev_list_entry_get_name(ent);
        auto dev = udev_device_new_from_syspath(u, path);
        auto parent = udev_device_get_parent_with_subsystem_devtype(dev, "usb", "usb_device");
        if (parent) {
            const char* vid = udev_device_get_sysattr_value(parent, "idVendor");
            const char* node = udev_device_get_devnode(dev);
            if (vid && node && std::string(vid)==XSENS_VID) {
                devNode_ = node;
                std::cout<<"✅ Xsens found: "<<devNode_<<"\n";
                udev_device_unref(dev);
                udev_unref(u);
                return DEVICE_FOUND_SUCCESS;
            }
        }
        udev_device_unref(dev);
    }
    udev_unref(u);
    return DEVICE_FOUND_FAILURE;
}

mtiDecode_enum XsensMti710::openXsensPort() {
    fd_ = open(devNode_.c_str(), O_RDWR|O_NOCTTY);
    if (fd_<0) return OPEN_PORT_FAILURE;

    termios t{};
    if (tcgetattr(fd_, &t)!=0) { close(fd_); return PORT_GET_ATTR_FAILURE; }
    cfsetispeed(&t, BAUDRATE);
    cfsetospeed(&t, BAUDRATE);
    t.c_cflag = CS8|CLOCAL|CREAD;
    t.c_iflag = IGNPAR;
    t.c_oflag = 0;
    t.c_lflag = 0;
    t.c_cc[VTIME]=1; t.c_cc[VMIN]=1;
    tcflush(fd_, TCIFLUSH);
    if (tcsetattr(fd_, TCSANOW, &t)!=0) { close(fd_); return PORT_SET_ATTR_FAILURE; }

    std::cout<<"Serial port opened.\n";
    return OPEN_PORT_SUCCESS;
}

void XsensMti710::configure() {
    instance_ = this;
    auto iface = XSENS_INTERFACE_RX_TX(&xsens_event_handler, &sendCb);

    xsens_mti_override_id_handler(MT_ACK_GOTOCONFIG,         cbGotoConfig);
    xsens_mti_override_id_handler(MT_ACK_OUTPUTCONFIGURATION,cbOutCfg);
    xsens_mti_override_id_handler(MT_ACK_OPTIONFLAGS,       cbFlags);
    xsens_mti_override_id_handler(MT_ACK_GOTOMEASUREMENT,   cbGotoMeas);

    // A) → Config mode
    ack_ = AckFlag::None;
    xsens_mti_request(&iface, MT_GOTOCONFIG);
    waitFor(AckFlag::GotoConfig);

    // B) → Output config
    static constexpr uint16_t R=10;
    XsensFrequencyConfig_t cfg[]={
      {XDI_PACKET_COUNTER,0xFFFF},{XDI_SAMPLE_TIME_FINE,0xFFFF},
      {XDI_DELTA_Q,R}, {XDI_RATE_OF_TURN,R}, {XDI_DELTA_V,R},
      {XDI_ACCELERATION,R},{XDI_FREE_ACCELERATION,R},
      {XDI_MAGNETIC_FIELD,R},{XDI_TEMPERATURE,R},{XDI_STATUS_BYTE,R},
      {XSENS_IDENTIFIER_FORMAT(XDI_ALTITUDE_ELLIPSOID,XSENS_FLOAT_FIXED1632,XSENS_COORD_ENU),R}
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
    auto iface = XSENS_INTERFACE_RX(&xsens_event_handler);
    uint8_t buf[256];
    while (true) {
        auto n = ::read(fd_, buf, sizeof(buf));
        if (n<=0) break;
        xsens_mti_parse_buffer(&iface, buf, n);
        // now data_ is up-to-date
    }
}

void XsensMti710::waitFor(AckFlag want) {
    auto iface = XSENS_INTERFACE_RX_TX(&xsens_event_handler, &sendCb);
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
            if(mt2data->type == XSENS_EVT_TYPE_FLOAT3)
            {
                // Assuming mt2data->data.f4x3 is a float array of size 3
                xsensData.euler[0] = mt2data->data.f4x3[0]; // Roll
                xsensData.euler[1] = mt2data->data.f4x3[1]; // Pitch
                xsensData.euler[2] = mt2data->data.f4x3[2]; // Yaw
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_EULER] Roll=" << xsensData.euler[0] << ", Pitch=" << xsensData.euler[1] << ", Yaw=" << xsensData.euler[2]
                        << " (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            }
            else
            {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_EULER\n";
                return;
            }
            break;
        case XSENS_EVT_QUATERNION:
            if(mt2data->type == XSENS_EVT_TYPE_FLOAT4)
            {
                // Assuming mt2data->data.f4x4 is a float array of size 4
                xsensData.quaternion[0] = mt2data->data.f4x4[0]; // q0
                xsensData.quaternion[1] = mt2data->data.f4x4[1]; // q1
                xsensData.quaternion[2] = mt2data->data.f4x4[2]; // q2
                xsensData.quaternion[3] = mt2data->data.f4x4[3]; // q3
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_QUATERNION] [" << xsensData.quaternion[0] << ", " << xsensData.quaternion[1] << ", " << xsensData.quaternion[2] << ", " << xsensData.quaternion[3]
                        << "] (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            }
            else
            {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_QUATERNION\n";
                return;
            }
            break;
        case XSENS_EVT_ACCELERATION:
            if(mt2data->type == XSENS_EVT_TYPE_FLOAT3)
            {
                // Assuming mt2data->data.f4x3 is a float array of size 3
                xsensData.acceleration[0] = mt2data->data.f4x3[0]; // ax
                xsensData.acceleration[1] = mt2data->data.f4x3[1]; // ay
                xsensData.acceleration[2] = mt2data->data.f4x3[2]; // az
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_ACCELERATION] X=" << xsensData.acceleration[0] << ", Y=" << xsensData.acceleration[1] << ", Z=" << xsensData.acceleration[2]
                        << " (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            }
            else
            {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_ACCELERATION\n";
                return;
            }
            break;
        case XSENS_EVT_FREE_ACCELERATION:
            if(mt2data->type == XSENS_EVT_TYPE_FLOAT3)
            {
                // Assuming mt2data->data.f4x3 is a float array of size 3
                xsensData.free_acceleration[0] = mt2data->data.f4x3[0]; // fx
                xsensData.free_acceleration[1] = mt2data->data.f4x3[1]; // fy
                xsensData.free_acceleration[2] = mt2data->data.f4x3[2]; // fz
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_FREE_ACCELERATION] X=" << xsensData.free_acceleration[0] << ", Y=" << xsensData.free_acceleration[1] << ", Z=" << xsensData.free_acceleration[2]
                        << " (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            }
            else
            {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_FREE_ACCELERATION\n";
                return;
            }
            break;
        case XSENS_EVT_RATE_OF_TURN:
        if(mt2data->type == XSENS_EVT_TYPE_FLOAT3)
            {
                // Assuming mt2data->data.f4x3 is a float array of size 3
                xsensData.rate_of_turn[0] = mt2data->data.f4x3[0]; // wx
                xsensData.rate_of_turn[1] = mt2data->data.f4x3[1]; // wy
                xsensData.rate_of_turn[2] = mt2data->data.f4x3[2]; // wz
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_RATE_OF_TURN] X=" << xsensData.rate_of_turn[0] << ", Y=" << xsensData.rate_of_turn[1] << ", Z=" << xsensData.rate_of_turn[2]
                        << " (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            }
            else
            {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_RATE_OF_TURN\n";
                return;
            }
            break;
        case XSENS_EVT_MAGNETIC:
        if(mt2data->type == XSENS_EVT_TYPE_FLOAT3)
            {
                // Assuming mt2data->data.f4x3 is a float array of size 3
                xsensData.magnetic[0] = mt2data->data.f4x3[0]; // mx
                xsensData.magnetic[1] = mt2data->data.f4x3[1]; // my
                xsensData.magnetic[2] = mt2data->data.f4x3[2]; // mz
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_MAGNETIC] X=" << xsensData.magnetic[0] << ", Y=" << xsensData.magnetic[1] << ", Z=" << xsensData.magnetic[2]
                        << " (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            }
            else
            {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_MAGNETIC\n";
                return;
            }
            break;
        case XSENS_EVT_LAT_LON:
            if(mt2data->type == XSENS_EVT_TYPE_FLOAT2)
            {
                // Assuming mt2data->data.f8x2 is a double array of size 2
                xsensData.latitude = mt2data->data.f8x2[0]; // Latitude
                xsensData.longitude = mt2data->data.f8x2[1]; // Longitude
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_LAT_LON] Lat=" << xsensData.latitude << ", Lon=" << xsensData.longitude
                        << " (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            }
            else
            {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_LAT_LON\n";
                return;
            }
            break;
        case XSENS_EVT_ALTITUDE_ELLIPSOID:
            if(mt2data->type == XSENS_EVT_TYPE_FLOAT)
            {
                // Assuming mt2data->data.f8 is a double value for altitude
                xsensData.altitude = mt2data->data.f8; // Altitude in meters
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_ALTITUDE] Altitude=" << xsensData.altitude
                        << " m (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            }
            else
            {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_ALTITUDE_ELLIPSOID\n";
                return;
            }
            break;
        case XSENS_EVT_VELOCITY_XYZ:
            if(mt2data->type == XSENS_EVT_TYPE_FLOAT3)
            {
                // Assuming mt2data->data.f4x3 is a float array of size 3
                xsensData.velocity[0] = mt2data->data.f4x3[0]; // vx
                xsensData.velocity[1] = mt2data->data.f4x3[1]; // vy
                xsensData.velocity[2] = mt2data->data.f4x3[2]; // vz
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_VELOCITY] X=" << xsensData.velocity[0] << ", Y=" << xsensData.velocity[1] << ", Z=" << xsensData.velocity[2]
                        << " (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            }
            else
            {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_VELOCITY_XYZ\n";
                return;
            }
            break;
        case XSENS_EVT_STATUS_BYTE:
            if(mt2data->type == XSENS_EVT_TYPE_U8)
            {
                xsensData.status_byte = static_cast<int>(mt2data->data.u1); // Status byte
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_STATUS_BYTE] Status Byte: 0x"
                          << std::hex << xsensData.status_byte << std::dec
                          << " (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            }
            else
            {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_STATUS_BYTE\n";
                return;
            }
            break;
        case XSENS_EVT_PACKET_COUNT:
            if(mt2data->type == XSENS_EVT_TYPE_U16)
            {
                xsensData.packet_counter = mt2data->data.u2; // Packet count
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_PACKET_COUNT] Count: " << xsensData.packet_counter
                          << " (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            }
            else
            {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_PACKET_COUNT\n";
                return;
            }
            break;
        case XSENS_EVT_TEMPERATURE:
            if(mt2data->type == XSENS_EVT_TYPE_FLOAT)
            {
                xsensData.temperature = mt2data->data.f4; // Temperature in Celsius
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_TEMPERATURE] Temperature: " << xsensData.temperature << " °C"
                          << " (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            }
            else
            {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_TEMPERATURE\n";
                return;
            }
            break;
        case XSENS_EVT_TIME_FINE:
            if(mt2data->type == XSENS_EVT_TYPE_U32)
            {
                xsensData.time_fine = mt2data->data.u4; // Fine time in microseconds
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_TIME_FINE] Time Fine: " << xsensData.time_fine
                          << " (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            }
            else
            {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_TIME_FINE\n";
                return;
            }
            break;
        case XSENS_EVT_TIME_COARSE:
            if(mt2data->type == XSENS_EVT_TYPE_U32)
            {
                xsensData.time_coarse = mt2data->data.u4; // Coarse time in seconds
                #ifdef DEBUG
                std::cout << "[DEBUG] [XSENS_EVT_TIME_COARSE] Time Coarse: " << xsensData.time_coarse
                          << " (Event: 0x" << std::hex << static_cast<int>(event) << std::dec << ")\n";
                #endif
            }
            else
            {
                std::cerr << "[ERROR] Invalid data type for XSENS_EVT_TIME_COARSE\n";
                return;
            }
            break;
        default:
            std::cerr << "[ERROR] Unrecognized event: " << static_cast<int>(event) << "\n";
            return;
    }
}
