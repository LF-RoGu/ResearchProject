#include "xsens_mti710.hpp"

XsensMti710::XsensMti710() = default;

mtiDecode_enum XsensMti710::find_xsens_device() {
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

mtiDecode_enum XsensMti710::open_xsens_port() 
{
    XsensMti710::xsens_fd = open(XsensMti710::xsens_device_path.c_str(), O_RDWR | O_NOCTTY);
    if (xsens_fd < 0) {
        std::perror("Failed to open port");
        return OPEN_PORT_FAILURE;
    }

    struct termios tty;
    if (tcgetattr(xsens_fd, &tty) != 0) {
        std::perror("Failed to get port attributes");
        close(xsens_fd);
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

    tcflush(xsens_fd, TCIFLUSH);
    if (tcsetattr(xsens_fd, TCSANOW, &tty) != 0) {
        std::perror("Failed to set port attributes");
        close(xsens_fd);
        return PORT_SET_ATTR_FAILURE;
    }

    std::cout << "Serial port opened successfully.\n";
    return OPEN_PORT_SUCCESS;
}

void XsensMti710::xsens_event_handler(XsensEventFlag_t flag, XsensEventData_t* data) {
    MTiData l_mtiData;
    if (flag & XSENS_EVT_EULER) {
        l_mtiData.euler[0] = data->data.f4x3[0];
        l_mtiData.euler[1] = data->data.f4x3[1];
        l_mtiData.euler[2] = data->data.f4x3[2];

        #ifdef DEBUG
        std::cout << "Euler angles (rad): "
                  << "Roll=" << data->data.f4x3[0]
                  << ", Pitch=" << data->data.f4x3[1]
                  << ", Yaw=" << data->data.f4x3[2] << "\n";
        #endif
    }

    if (flag & XSENS_EVT_QUATERNION) {
        l_mtiData.quaternion[0] = data->data.f4x4[0];
        l_mtiData.quaternion[1] = data->data.f4x4[1];
        l_mtiData.quaternion[2] = data->data.f4x4[2];
        l_mtiData.quaternion[3] = data->data.f4x4[3];
        #ifdef DEBUG
        std::cout << "Quaternion: ["
                  << data->data.f4x4[0] << ", "
                  << data->data.f4x4[1] << ", "
                  << data->data.f4x4[2] << ", "
                  << data->data.f4x4[3] << "]\n";
        #endif
    }

    if (flag & XSENS_EVT_ACCELERATION) {
        l_mtiData.acceleration[0] = data->data.f4x3[0];
        l_mtiData.acceleration[1] = data->data.f4x3[1];
        l_mtiData.acceleration[2] = data->data.f4x3[2];
        #ifdef DEBUG
        std::cout << "Acceleration (m/s²): "
                  << "X=" << data->data.f4x3[0]
                  << ", Y=" << data->data.f4x3[1]
                  << ", Z=" << data->data.f4x3[2] << "\n";
        #endif
    }

    if (flag & XSENS_EVT_FREE_ACCELERATION) {
        l_mtiData.free_acceleration[0] = data->data.f4x3[0];
        l_mtiData.free_acceleration[1] = data->data.f4x3[1];
        l_mtiData.free_acceleration[2] = data->data.f4x3[2];
        #ifdef DEBUG
        std::cout << "Free Acceleration (m/s²): "
                  << "X=" << data->data.f4x3[0]
                  << ", Y=" << data->data.f4x3[1]
                  << ", Z=" << data->data.f4x3[2] << "\n";
        #endif
    }

    if (flag & XSENS_EVT_RATE_OF_TURN) {
        l_mtiData.angular_velocity[0] = data->data.f4x3[0];
        l_mtiData.angular_velocity[1] = data->data.f4x3[1];
        l_mtiData.angular_velocity[2] = data->data.f4x3[2];
        #ifdef DEBUG
        std::cout << "Angular Velocity (rad/s): "
                  << "X=" << data->data.f4x3[0]
                  << ", Y=" << data->data.f4x3[1]
                  << ", Z=" << data->data.f4x3[2] << "\n";
        #endif
    }

    if (flag & XSENS_EVT_MAGNETIC) {
        l_mtiData.magnetic[0] = data->data.f4x3[0];
        l_mtiData.magnetic[1] = data->data.f4x3[1];
        l_mtiData.magnetic[2] = data->data.f4x3[2];
        #ifdef DEBUG
        std::cout << "Magnetic Field (µT): "
                  << "X=" << data->data.f4x3[0]
                  << ", Y=" << data->data.f4x3[1]
                  << ", Z=" << data->data.f4x3[2] << "\n";
        #endif
    }

    if (flag & XSENS_EVT_LAT_LON) {
        l_mtiData.latitude = data->data.f8x2[0];
        l_mtiData.longitude = data->data.f8x2[0];
        #ifdef DEBUG
        std::cout << "Latitude / Longitude: "
                  << "Lat=" << data->data.f8x2[0]
                  << ", Lon=" << data->data.f8x2[1] << "\n";
        #endif
    }

    if (flag & XSENS_EVT_ALTITUDE_ELLIPSOID) {
        l_mtiData.altitude = data->data.f8;
        #ifdef DEBUG
        std::cout << "Altitude (ellipsoidal): " << data->data.f8 << " m\n";
        #endif
    }

    if (flag & XSENS_EVT_VELOCITY_XYZ) {
        l_mtiData.velocity[0] = data->data.f4x3[0];
        l_mtiData.velocity[1] = data->data.f4x3[1];
        l_mtiData.velocity[2] = data->data.f4x3[2];
        #ifdef DEBUG
        std::cout << "Velocity (XYZ m/s): "
                  << "X=" << data->data.f4x3[0]
                  << ", Y=" << data->data.f4x3[1]
                  << ", Z=" << data->data.f4x3[2] << "\n";
        #endif
    }

    if (flag & XSENS_EVT_STATUS_BYTE) {
        l_mtiData.status_byte = static_cast<int>(data->data.u1);
        #ifdef DEBUG
        std::cout << "Status Byte: 0x" << std::hex << static_cast<int>(data->data.u1) << std::dec << "\n";
        #endif
    }

    if (flag & XSENS_EVT_PACKET_COUNT) {
        l_mtiData.packet_counter = data->data.u2;
        #ifdef DEBUG
        std::cout << "Packet Counter: " << data->data.u2 << "\n";
        #endif
    }

    if (flag & XSENS_EVT_TEMPERATURE) {
        l_mtiData.temperature = data->data.f4;
        #ifdef DEBUG
        std::cout << "Temperature: " << data->data.f4 << " °C\n";
        #endif
    }

    if (flag & XSENS_EVT_TIME_FINE) {
        l_mtiData.time_fine = data->data.u4;
        #ifdef DEBUG
        std::cout << "Timestamp (Fine): " << data->data.u4 << " ticks\n";
        #endif
    }

    if (flag & XSENS_EVT_TIME_COARSE) {
        l_mtiData.time_coarse = data->data.u4;
        #ifdef DEBUG
        std::cout << "Timestamp (Coarse): " << data->data.u4 << " ms\n";
        #endif
    }

    if (flag & ~(XSENS_EVT_ALL)) {
        #ifdef DEBUG
        std::cout << "Unknown Event(s) received: 0x" << std::hex << flag << std::dec << "\n";
        #endif
    }
    xsensData = l_mtiData;
}

void XsensMti710::set_xsens_data(const MTiData& data) {
    xsensData = data;
}
MTiData XsensMti710::get_xsens_data()
{
    return xsensData;
}

int XsensMti710::get_fd() const {
    return XsensMti710::xsens_fd;
}
