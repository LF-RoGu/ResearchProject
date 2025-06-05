#include "xda.hpp"

xdaSensorClass::xdaSensorClass() : xsens_device_path(""), newData{} {
    // Constructor implementation
}
xdaSensorClass::~xdaSensorClass() {
    // Destructor implementation
}

bool xdaSensorClass::find_xsens_device() {
    struct udev* udev = udev_new();
    if (!udev) return false;

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
            if (vid && dev_node && std::string(vid) == XSENS_VID) {
                xsens_device_path = std::string(dev_node);
                std::cout << "✅ Xsens device found: " << xsens_device_path << "\n";
                udev_device_unref(dev);
                udev_unref(udev);
                return true;
            }
        }
        udev_device_unref(dev);
    }

    udev_unref(udev);
    std::cerr << "No Xsens device found.\n";
    return false;
}

int xdaSensorClass::open_xsens_port() {
    int fd = open(xsens_device_path.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        std::perror("Failed to open port");
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        std::perror("Failed to get port attributes");
        close(fd);
        return -1;
    }

    cfsetispeed(&tty, BAUDRATE);
    cfsetospeed(&tty, BAUDRATE);
    tty.c_cflag = CS8 | CLOCAL | CREAD;
    tty.c_iflag = IGNPAR;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cc[VTIME] = 1; // 100ms timeout
    tty.c_cc[VMIN] = 1; // Blocking read

    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::perror("Failed to set port attributes");
        close(fd);
        return -1;
    }

    std::cout << "Serial port opened successfully.\n";
    return fd;
}

// Event callback
void xdaSensorClass::xsens_event_handler(XsensEventFlag_t flag, XsensEventData_t* data) {
    MTiData newData{};
    if (!data) {
        std::cerr << "Received null data pointer in event handler.\n";
        return;
    }
    std::cout << "\033[2J\033[H";  // Clear terminal screen
    std::cout << std::fixed << std::setprecision(6);  // Float formatting
    
    if (flag & XSENS_EVT_EULER) {
        newData.euler[0] = data->data.f4x3[0];
        newData.euler[1] = data->data.f4x3[1];
        newData.euler[2] = data->data.f4x3[2];
    }

    if (flag & XSENS_EVT_QUATERNION) {
        newData.quaternion[0] = data->data.f4x4[0];
        newData.quaternion[1] = data->data.f4x4[1];
        newData.quaternion[2] = data->data.f4x4[2];
        newData.quaternion[3] = data->data.f4x4[3];
    }

    if (flag & XSENS_EVT_ACCELERATION) {
        newData.acceleration[0] = data->data.f4x3[0];
        newData.acceleration[1] = data->data.f4x3[1];
        newData.acceleration[2] = data->data.f4x3[2];

    }

    if (flag & XSENS_EVT_FREE_ACCELERATION) {
        newData.free_acceleration[0] = data->data.f4x3[0];
        newData.free_acceleration[1] = data->data.f4x3[1];
        newData.free_acceleration[2] = data->data.f4x3[2];
    }

    if (flag & XSENS_EVT_RATE_OF_TURN) {
        newData.angular_velocity[0] = data->data.f4x3[0];
        newData.angular_velocity[1] = data->data.f4x3[1];
        newData.angular_velocity[2] = data->data.f4x3[2];
    }

    if (flag & XSENS_EVT_MAGNETIC) {
        newData.magnetic[0] = data->data.f4x3[0];
        newData.magnetic[1] = data->data.f4x3[1];
        newData.magnetic[2] = data->data.f4x3[2];
    }

    if (flag & XSENS_EVT_LAT_LON) {
        newData.latitude = data->data.f8x2[0];
        newData.longitude = data->data.f8x2[0];
    }

    if (flag & XSENS_EVT_ALTITUDE_ELLIPSOID) {
        newData.altitude = data->data.f8;
    }

    if (flag & XSENS_EVT_VELOCITY_XYZ) {
        newData.velocity[0] = data->data.f4x3[0];
        newData.velocity[1] = data->data.f4x3[1];
        newData.velocity[2] = data->data.f4x3[2];
    }

    if (flag & XSENS_EVT_STATUS_BYTE) {
        newData.status_byte = static_cast<int>(data->data.u1);
    }

    if (flag & XSENS_EVT_PACKET_COUNT) {
        newData.packet_counter = data->data.u2;
    }

    if (flag & XSENS_EVT_TEMPERATURE) {
        newData.temperature = data->data.f4;
    }

    if (flag & XSENS_EVT_TIME_FINE) {
        newData.time_fine = data->data.u4;
    }

    if (flag & XSENS_EVT_TIME_COARSE) {
        newData.time_coarse = data->data.u4;
    }

    if (flag & ~(XSENS_EVT_ALL)) {
        std::cout << "Unknown Event(s) received: 0x" << std::hex << flag << std::dec << "\n";
    }

    set_mti_data(newData);
    usleep(100000);
}  

void xdaSensorClass::get_mti_data(MTiData latestMTiData) {
    mtiData_g = latestMTiData;
}
void xdaSensorClass::set_mti_data(const MTiData& newData) {
    mtiData_g = newData;
}
// End of xdaSensorClass implementation